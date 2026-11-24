// Standalone Linux driver for the DJI Agras Mid-360. See mid360_agras.h.
#include "mid360_agras.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>

namespace mid360 {
namespace {

constexpr float kGravity = 9.80665f;

template <typename T>
T rd(const uint8_t* p) {  // unaligned little-endian read (the Orin / x86 are little endian)
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

int64_t mono_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double wall_s() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

bool parse_header(const uint8_t* d, size_t len, PacketHeader* h) {
  if (len < kHeaderSize) return false;
  h->version = d[0];
  h->length = rd<uint16_t>(d + 1);
  h->time_interval = rd<uint16_t>(d + 3);
  h->dot_num = rd<uint16_t>(d + 5);
  h->udp_cnt = rd<uint16_t>(d + 7);
  h->frame_cnt = d[9];
  h->data_type = d[10];
  h->time_type = d[11];
  h->timestamp_ns = rd<uint64_t>(d + 28);
  return true;
}

int parse_points(const uint8_t* d, size_t len, const Config& cfg, std::vector<Point>* out,
                 PacketHeader* hdr_out) {
  PacketHeader h;
  if (!parse_header(d, len, &h)) return -1;
  if (hdr_out) *hdr_out = h;
  size_t psize;
  float scale;
  if (h.data_type == 1) {
    psize = 14;  // int32 x/y/z mm, refl, tag
    scale = 0.001f;
  } else if (h.data_type == 2) {
    psize = 8;  // int16 x/y/z cm, refl, tag
    scale = 0.01f;
  } else {
    return -1;  // IMU (0) or spherical (3, not sent by this firmware)
  }
  size_t n = (len - kHeaderSize) / psize;
  if (h.dot_num && h.dot_num < n) n = h.dot_num;
  const double dt_ns = n ? h.time_interval * 100.0 / n : 0.0;  // time_interval is in 0.1 us
  const float rmin2 = cfg.min_range_m * cfg.min_range_m, rmax2 = cfg.max_range_m * cfg.max_range_m;
  const uint8_t* p = d + kHeaderSize;
  for (size_t i = 0; i < n; ++i, p += psize) {
    Point pt;
    if (h.data_type == 1) {
      pt.x = rd<int32_t>(p) * scale;
      pt.y = rd<int32_t>(p + 4) * scale;
      pt.z = rd<int32_t>(p + 8) * scale;
      pt.reflectivity = p[12];
      pt.tag = p[13];
    } else {
      pt.x = rd<int16_t>(p) * scale;
      pt.y = rd<int16_t>(p + 2) * scale;
      pt.z = rd<int16_t>(p + 4) * scale;
      pt.reflectivity = p[6];
      pt.tag = p[7];
    }
    if (!cfg.keep_no_return) {
      const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (r2 <= rmin2 || r2 >= rmax2) continue;
    }
    pt.line = static_cast<uint8_t>(i % 4);
    pt.timestamp_ns = h.timestamp_ns + static_cast<uint64_t>(std::llround(i * dt_ns));
    out->push_back(pt);
  }
  return static_cast<int>(n);
}

bool parse_imu(const uint8_t* d, size_t len, Imu* out) {
  PacketHeader h;
  if (!parse_header(d, len, &h) || h.data_type != 0 || len < kHeaderSize + 24) return false;
  out->timestamp_ns = h.timestamp_ns;
  for (int k = 0; k < 3; ++k) {
    out->gyro[k] = rd<float>(d + kHeaderSize + 4 * k);
    out->accel[k] = rd<float>(d + kHeaderSize + 12 + 4 * k) * kGravity;
  }
  return true;
}

// ---------------------------------------------------------------- FrameAssembler

FrameAssembler::FrameAssembler(const Config& cfg) : cfg_(cfg) {}

bool FrameAssembler::close(Frame* done) {
  if (cur_.points.empty() && cur_.packets == 0) return false;
  cur_.seq = ++seq_;
  cur_.host_time = wall_s();
  if (!cur_.points.empty()) {
    cur_.t_begin_ns = cur_.points.front().timestamp_ns;
    cur_.t_end_ns = cur_.points.back().timestamp_ns;
  }
  stats_.frames++;
  *done = std::move(cur_);
  cur_ = Frame();
  cur_.points.reserve(done->points.capacity());
  return true;
}

bool FrameAssembler::feed(const uint8_t* d, size_t len, Frame* done) {
  PacketHeader h;
  if (!parse_header(d, len, &h) || (h.data_type != 1 && h.data_type != 2)) {
    stats_.bad_packets++;
    return false;
  }
  bool closed = false;
  const uint64_t period = static_cast<uint64_t>(cfg_.frame_period_s * 1e9);
  // the firmware's frame_cnt is always 0, so frames are cut on the packet timestamp
  if (!have_start_ || h.timestamp_ns < frame_start_ns_) {  // first packet, or lidar clock reset
    if (have_start_) closed = close(done);
    frame_start_ns_ = h.timestamp_ns;
    have_start_ = true;
  } else if (h.timestamp_ns - frame_start_ns_ >= period) {
    closed = close(done);
    frame_start_ns_ += period * ((h.timestamp_ns - frame_start_ns_) / period);
  }
  if (have_cnt_) {
    const uint16_t gap = static_cast<uint16_t>(h.udp_cnt - last_cnt_ - 1);
    if (gap && gap < 1000) {
      cur_.lost_packets += gap;
      stats_.lost_packets += gap;
    }
  }
  last_cnt_ = h.udp_cnt;
  have_cnt_ = true;
  const size_t before = cur_.points.size();
  const int raw = parse_points(d, len, cfg_, &cur_.points);
  cur_.packets++;
  cur_.raw_points += raw;
  stats_.point_packets++;
  stats_.raw_points += raw;
  stats_.valid_points += cur_.points.size() - before;
  return closed;
}

bool FrameAssembler::flush(Frame* done) { return close(done); }

// ---------------------------------------------------------------- Driver

Driver::Driver(Config cfg) : cfg_(std::move(cfg)) {}

Driver::~Driver() { stop(); }

int Driver::open_socket(uint16_t port) {
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    error_ = std::string("socket: ") + std::strerror(errno);
    return -1;
  }
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &cfg_.rcvbuf_bytes, sizeof(cfg_.rcvbuf_bytes));
  timeval tv{0, 200 * 1000};  // so stop() is noticed within 0.2 s
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  if (::inet_pton(AF_INET, cfg_.bind_ip.c_str(), &a.sin_addr) != 1) {
    error_ = "bad bind_ip " + cfg_.bind_ip;
    ::close(fd);
    return -1;
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) {
    error_ = "bind UDP " + std::to_string(port) + ": " + std::strerror(errno) +
             (errno == EADDRINUSE ? " (stop lidar-web.service / livox_ros_driver2 first)" : "");
    ::close(fd);
    return -1;
  }
  return fd;
}

bool Driver::start() {
  if (running_) return true;
  error_.clear();
  lidar_addr_ = 0;
  if (!cfg_.lidar_ip.empty() && ::inet_pton(AF_INET, cfg_.lidar_ip.c_str(), &lidar_addr_) != 1) {
    error_ = "bad lidar_ip " + cfg_.lidar_ip;
    return false;
  }
  point_fd_ = open_socket(cfg_.point_port);
  if (point_fd_ < 0) return false;
  if (cfg_.enable_imu) {
    imu_fd_ = open_socket(cfg_.imu_port);
    if (imu_fd_ < 0) {
      ::close(point_fd_);
      point_fd_ = -1;
      return false;
    }
  }
  running_ = true;
  point_thread_ = std::thread(&Driver::point_loop, this);
  if (cfg_.enable_imu) imu_thread_ = std::thread(&Driver::imu_loop, this);
  return true;
}

void Driver::stop() {
  running_ = false;
  if (point_thread_.joinable()) point_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
  if (point_fd_ >= 0) ::close(point_fd_);
  if (imu_fd_ >= 0) ::close(imu_fd_);
  point_fd_ = imu_fd_ = -1;
}

Stats Driver::stats() const {
  std::lock_guard<std::mutex> lk(stats_mu_);
  return stats_;
}

double Driver::seconds_since_data() const {
  const int64_t t = last_data_ns_.load();
  return t ? (mono_ns() - t) / 1e9 : -1.0;
}

void Driver::point_loop() {
  FrameAssembler fa(cfg_);
  uint8_t buf[2048];
  Frame done;
  uint64_t other = 0;
  while (running_) {
    sockaddr_in src{};
    socklen_t sl = sizeof(src);
    const ssize_t n = ::recvfrom(point_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &sl);
    if (n < 0) {
      // timeout: close a pending frame if the stream has stopped
      if (seconds_since_data() > cfg_.frame_period_s * 3 && fa.flush(&done) && frame_cb_) frame_cb_(done);
      continue;
    }
    if (lidar_addr_ && src.sin_addr.s_addr != lidar_addr_) {
      other++;
      continue;
    }
    last_data_ns_ = mono_ns();
    const bool closed = fa.feed(buf, static_cast<size_t>(n), &done);
    {
      std::lock_guard<std::mutex> lk(stats_mu_);
      const uint64_t imu = stats_.imu_packets;
      stats_ = fa.stats();
      stats_.imu_packets = imu;
      stats_.other_packets = other;
    }
    if (closed && frame_cb_) frame_cb_(done);
  }
}

void Driver::imu_loop() {
  uint8_t buf[512];
  while (running_) {
    sockaddr_in src{};
    socklen_t sl = sizeof(src);
    const ssize_t n = ::recvfrom(imu_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &sl);
    if (n < 0) continue;
    if (lidar_addr_ && src.sin_addr.s_addr != lidar_addr_) continue;
    Imu imu;
    if (!parse_imu(buf, static_cast<size_t>(n), &imu)) continue;
    {
      std::lock_guard<std::mutex> lk(stats_mu_);
      stats_.imu_packets++;
    }
    if (imu_cb_) imu_cb_(imu);
  }
}

}  // namespace mid360
