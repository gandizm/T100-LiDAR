// ROS2 node for the DJI Agras Mid-360, built on the standalone driver in linux/ (no Livox SDK).
//   /livox/lidar   livox_ros_driver2/CustomMsg  one message per frame (default 0.1 s): FAST-LIO input
//   /livox/imu     sensor_msgs/Imu              200 Hz, gyro rad/s, accel m/s^2
//   /livox/points  sensor_msgs/PointCloud2      the same frames for RViz, only built while someone subscribes
// Stamps: the lidar clock counts from its power-on. It is mapped to host time with one offset taken at the
// first packet (and again if the lidar restarts), so points and IMU stay on one consistent clock.
#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "mid360_agras.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

class Mid360AgrasNode : public rclcpp::Node {
 public:
  Mid360AgrasNode() : Node("mid360_agras") {
    mid360::Config cfg;
    cfg.bind_ip = declare_parameter<std::string>("bind_ip", cfg.bind_ip);
    cfg.lidar_ip = declare_parameter<std::string>("lidar_ip", cfg.lidar_ip);
    cfg.point_port = static_cast<uint16_t>(declare_parameter<int64_t>("point_port", cfg.point_port));
    cfg.imu_port = static_cast<uint16_t>(declare_parameter<int64_t>("imu_port", cfg.imu_port));
    cfg.frame_period_s = declare_parameter<double>("frame_period", cfg.frame_period_s);
    cfg.min_range_m = static_cast<float>(declare_parameter<double>("min_range", cfg.min_range_m));
    cfg.max_range_m = static_cast<float>(declare_parameter<double>("max_range", cfg.max_range_m));
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    const auto reliable = [](size_t depth) { return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable(); };
    // reliable: FAST-LIO and RViz subscribe with the default (reliable) QoS
    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        declare_parameter<std::string>("lidar_topic", "/livox/lidar"), reliable(10));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic", "/livox/imu"),
                                                       reliable(200));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("cloud_topic", "/livox/points"), reliable(5));

    drv_ = std::make_unique<mid360::Driver>(cfg);
    drv_->on_frame([this](const mid360::Frame& f) { publish_frame(f); });
    drv_->on_imu([this](const mid360::Imu& m) { publish_imu(m); });
    if (!drv_->start()) throw std::runtime_error(drv_->error());
    RCLCPP_INFO(get_logger(), "listening for the Agras Mid-360 %s on UDP %u (points) / %u (IMU)",
                cfg.lidar_ip.c_str(), cfg.point_port, cfg.imu_port);
    timer_ = create_wall_timer(std::chrono::seconds(5), [this] { report(); });
  }

  ~Mid360AgrasNode() override {
    if (drv_) drv_->stop();  // join the receive threads before the publishers go away
  }

 private:
  // lidar clock (ns since its power-on) -> host time (ns); called from the point and the IMU thread
  int64_t to_host(uint64_t lidar_ns) {
    std::lock_guard<std::mutex> lk(clock_mu_);
    const int64_t l = static_cast<int64_t>(lidar_ns);
    if (!have_offset_ || l + 1000000000LL < last_lidar_ns_) {  // first packet, or the lidar clock restarted
      if (have_offset_) RCLCPP_WARN(get_logger(), "lidar clock went back (lidar restarted?): new time offset");
      offset_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count() - l;
      have_offset_ = true;
      last_lidar_ns_ = l;
    }
    last_lidar_ns_ = std::max(last_lidar_ns_, l);
    return l + offset_ns_;
  }

  void publish_frame(const mid360::Frame& f) {
    if (f.points.empty() || !rclcpp::ok()) return;
    uint64_t base = f.points.front().timestamp_ns;
    for (const auto& p : f.points) base = std::min(base, p.timestamp_ns);
    const rclcpp::Time stamp(to_host(base), RCL_SYSTEM_TIME);
    auto msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id_;
    msg->timebase = static_cast<uint64_t>(stamp.nanoseconds());
    msg->point_num = static_cast<uint32_t>(f.points.size());
    msg->lidar_id = 0;
    msg->points.resize(f.points.size());
    for (size_t i = 0; i < f.points.size(); ++i) {
      const auto& s = f.points[i];
      auto& d = msg->points[i];
      d.offset_time = static_cast<uint32_t>(s.timestamp_ns - base);  // ns; FAST-LIO reads it as ns / 1e6 = ms
      d.x = s.x;
      d.y = s.y;
      d.z = s.z;
      d.reflectivity = s.reflectivity;
      d.tag = s.tag;    // bits 4-5 are 0 on this firmware: FAST-LIO keeps the point
      d.line = s.line;  // 0..3, the 4 interleaved laser lines (FAST-LIO: scan_line 4)
    }
    try {
      if (cloud_pub_->get_subscription_count() > 0) publish_cloud(f, stamp);
      lidar_pub_->publish(std::move(msg));
    } catch (const std::exception&) {  // the context can shut down while a frame is on its way (Ctrl-C)
    }
  }

  void publish_cloud(const mid360::Frame& f, const rclcpp::Time& stamp) {
    auto c = std::make_unique<sensor_msgs::msg::PointCloud2>();
    c->header.stamp = stamp;
    c->header.frame_id = frame_id_;
    c->height = 1;
    c->width = static_cast<uint32_t>(f.points.size());
    const char* names[4] = {"x", "y", "z", "intensity"};
    for (uint32_t k = 0; k < 4; ++k) {
      sensor_msgs::msg::PointField pf;
      pf.name = names[k];
      pf.offset = 4 * k;
      pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
      pf.count = 1;
      c->fields.push_back(pf);
    }
    c->is_bigendian = false;
    c->point_step = 16;
    c->row_step = c->point_step * c->width;
    c->is_dense = true;
    c->data.resize(c->row_step);
    uint8_t* out = c->data.data();
    for (const auto& p : f.points) {
      const float v[4] = {p.x, p.y, p.z, static_cast<float>(p.reflectivity)};
      std::memcpy(out, v, sizeof(v));
      out += sizeof(v);
    }
    cloud_pub_->publish(std::move(c));
  }

  void publish_imu(const mid360::Imu& m) {
    if (!rclcpp::ok()) return;
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = rclcpp::Time(to_host(m.timestamp_ns), RCL_SYSTEM_TIME);
    msg->header.frame_id = frame_id_;
    msg->orientation_covariance[0] = -1.0;  // the Mid-360 gives no orientation
    msg->angular_velocity.x = m.gyro[0];
    msg->angular_velocity.y = m.gyro[1];
    msg->angular_velocity.z = m.gyro[2];
    msg->linear_acceleration.x = m.accel[0];
    msg->linear_acceleration.y = m.accel[1];
    msg->linear_acceleration.z = m.accel[2];
    try {
      imu_pub_->publish(std::move(msg));
    } catch (const std::exception&) {
    }
  }

  void report() {
    const auto s = drv_->stats();
    const double age = drv_->seconds_since_data();
    if (age < 0 || age > 2) {
      RCLCPP_WARN(get_logger(), "no data from the lidar %s: power, cable, host address 192.168.1.20/24?",
                  age < 0 ? "yet" : "for over 2 s");
    } else if (!reported_ || ++ticks_ % 12 == 0) {  // once when data starts, then every minute
      const double dt = 5.0;
      const double frames = static_cast<double>(s.frames - prev_.frames);
      RCLCPP_INFO(get_logger(), "%.0f packets/s, %.1f frames/s, %.0f points/frame, IMU %.0f Hz, lost packets %llu",
                  (s.point_packets - prev_.point_packets) / dt, frames / dt,
                  frames > 0 ? (s.valid_points - prev_.valid_points) / frames : 0.0,
                  (s.imu_packets - prev_.imu_packets) / dt, static_cast<unsigned long long>(s.lost_packets));
      reported_ = true;
    }
    prev_ = s;
  }

  std::string frame_id_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex clock_mu_;
  bool have_offset_ = false;
  int64_t offset_ns_ = 0, last_lidar_ns_ = 0;
  mid360::Stats prev_{};
  bool reported_ = false;
  unsigned ticks_ = 0;
  std::unique_ptr<mid360::Driver> drv_;  // last member: destroyed first, its threads use everything above
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<Mid360AgrasNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mid360_agras"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
