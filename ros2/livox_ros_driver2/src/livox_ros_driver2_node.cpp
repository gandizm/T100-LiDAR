#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <functional>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{
constexpr uint8_t kSof = 0x55;
constexpr uint8_t kClientSender = 0x28;
constexpr uint8_t kDeviceReceiver = 0xAD;
constexpr uint8_t kRequestFlag = 0x40;
constexpr uint8_t kResponseFlag = 0x80;
constexpr uint8_t kDataTypeImu = 0;
constexpr uint8_t kDataTypeCartesian = 1;
constexpr size_t kDataHeaderSize = 36;
constexpr size_t kPointRecordSize = 14;
constexpr size_t kImuPacketSize = 60;
constexpr uint64_t kNsPerSecond = 1000000000ULL;
constexpr double kGravity = 9.80665;

uint16_t readU16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int32_t readI32(const uint8_t *p)
{
  return static_cast<int32_t>(readU32(p));
}

uint64_t readU64(const uint8_t *p)
{
  return static_cast<uint64_t>(readU32(p)) |
         (static_cast<uint64_t>(readU32(p + 4)) << 32);
}

float readF32(const uint8_t *p)
{
  const uint32_t bits = readU32(p);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void appendU16(std::vector<uint8_t> &out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void appendU32(std::vector<uint8_t> &out, uint32_t value)
{
  for (unsigned int i = 0; i < 4; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
  }
}

uint8_t crc8(const uint8_t *data, size_t size)
{
  uint8_t value = 0x77;
  for (size_t i = 0; i < size; ++i) {
    value ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      value = static_cast<uint8_t>((value >> 1) ^ ((value & 1U) ? 0x8C : 0));
    }
  }
  return value;
}

uint16_t crc16(const uint8_t *data, size_t size)
{
  uint16_t value = 0x3692;
  for (size_t i = 0; i < size; ++i) {
    value ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      value = static_cast<uint16_t>((value >> 1) ^ ((value & 1U) ? 0x8408 : 0));
    }
  }
  return value;
}

uint32_t crc32Ieee(const uint8_t *data, size_t size)
{
  uint32_t value = 0xFFFFFFFFU;
  for (size_t i = 0; i < size; ++i) {
    value ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      value = (value >> 1) ^ ((value & 1U) ? 0xEDB88320U : 0U);
    }
  }
  return value ^ 0xFFFFFFFFU;
}

std::vector<uint8_t> makeControlFrame(
  uint8_t command_set, uint8_t command_id, uint16_t sequence,
  const std::vector<uint8_t> &payload)
{
  const size_t frame_size = 13 + payload.size();
  if (frame_size > 1023) {
    throw std::runtime_error("T100 control frame exceeds protocol limit");
  }

  std::vector<uint8_t> frame(frame_size, 0);
  frame[0] = kSof;
  frame[1] = static_cast<uint8_t>(frame_size & 0xFF);
  frame[2] = static_cast<uint8_t>(4 | ((frame_size >> 8) & 0x03));
  frame[3] = crc8(frame.data(), 3);
  frame[4] = kClientSender;
  frame[5] = kDeviceReceiver;
  frame[6] = static_cast<uint8_t>(sequence & 0xFF);
  frame[7] = static_cast<uint8_t>((sequence >> 8) & 0xFF);
  frame[8] = kRequestFlag;
  frame[9] = command_set;
  frame[10] = command_id;
  std::copy(payload.begin(), payload.end(), frame.begin() + 11);
  const uint16_t checksum = crc16(frame.data(), frame_size - 2);
  frame[frame_size - 2] = static_cast<uint8_t>(checksum & 0xFF);
  frame[frame_size - 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
  return frame;
}

struct ControlReply
{
  uint8_t command_set{0};
  uint8_t command_id{0};
  uint16_t sequence{0};
  uint8_t sender{0};
  uint8_t receiver{0};
  uint8_t flags{0};
  std::vector<uint8_t> payload;
};

bool decodeControlFrame(const uint8_t *data, size_t size, ControlReply &reply)
{
  if (size < 13 || data[0] != kSof) {
    return false;
  }
  const size_t declared = static_cast<size_t>(data[1]) |
    (static_cast<size_t>(data[2] & 0x03U) << 8);
  if (declared != size || (data[2] >> 2) != 1) {
    return false;
  }
  if (crc8(data, 3) != data[3]) {
    return false;
  }
  if (crc16(data, size - 2) != readU16(data + size - 2)) {
    return false;
  }

  reply.sender = data[4];
  reply.receiver = data[5];
  reply.sequence = readU16(data + 6);
  reply.flags = data[8];
  reply.command_set = data[9];
  reply.command_id = data[10];
  reply.payload.assign(data + 11, data + size - 2);
  return true;
}

int64_t systemNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time toRosTime(uint64_t timestamp_ns)
{
  const int64_t signed_ns = static_cast<int64_t>(timestamp_ns);
  return rclcpp::Time(signed_ns, RCL_SYSTEM_TIME).to_msg();
}

bool encodeIpv4LittleEndian(const std::string &ip, std::vector<uint8_t> &bytes)
{
  in_addr address{};
  if (inet_pton(AF_INET, ip.c_str(), &address) != 1) {
    return false;
  }
  const auto *network_bytes = reinterpret_cast<const uint8_t *>(&address.s_addr);
  bytes = {network_bytes[3], network_bytes[2], network_bytes[1], network_bytes[0]};
  return true;
}

std::string routeLocalIpv4(const std::string &device_ip)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("cannot create route-probe UDP socket");
  }

  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_port = htons(60000);
  if (inet_pton(AF_INET, device_ip.c_str(), &peer.sin_addr) != 1) {
    close(fd);
    throw std::runtime_error("device_ip is not a valid IPv4 address");
  }

  if (connect(fd, reinterpret_cast<sockaddr *>(&peer), sizeof(peer)) != 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("cannot determine route to T100: " + error);
  }

  sockaddr_in local{};
  socklen_t local_size = sizeof(local);
  if (getsockname(fd, reinterpret_cast<sockaddr *>(&local), &local_size) != 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("getsockname failed: " + error);
  }
  close(fd);

  char text[INET_ADDRSTRLEN]{};
  if (inet_ntop(AF_INET, &local.sin_addr, text, sizeof(text)) == nullptr) {
    throw std::runtime_error("cannot format selected local IPv4 address");
  }
  return text;
}

struct TimedPoint
{
  uint64_t timestamp_ns{0};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint8_t reflectivity{0};
  uint8_t tag{0};
  uint8_t line{0};
};

class DeviceClockMapper
{
public:
  uint64_t map(uint64_t device_ns, uint8_t time_type)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (time_type == 0) {
      if (!anchored_) {
        offset_ns_ = systemNowNs() - static_cast<int64_t>(device_ns);
        anchored_ = true;
      }
      const __int128 mapped = static_cast<__int128>(device_ns) + offset_ns_;
      if (mapped < 0 || mapped > std::numeric_limits<uint64_t>::max()) {
        throw std::runtime_error("mapped T100 timestamp is outside uint64 range");
      }
      return static_cast<uint64_t>(mapped);
    }
    // PTP/gPTP and GPS timestamps are already device-synchronized time values.
    return device_ns;
  }

private:
  std::mutex mutex_;
  bool anchored_{false};
  int64_t offset_ns_{0};
};

struct SequenceStats
{
  bool initialized{false};
  uint16_t last{0};
  uint64_t missing{0};
  uint64_t duplicates{0};
  uint64_t out_of_order{0};

  void update(uint16_t value)
  {
    if (!initialized) {
      initialized = true;
      last = value;
      return;
    }
    const uint16_t delta = static_cast<uint16_t>(value - last);
    if (delta == 0) {
      ++duplicates;
    } else if (delta >= 32768) {
      ++out_of_order;
    } else {
      missing += static_cast<uint64_t>(delta - 1);
      last = value;
    }
  }
};
}  // namespace

class T100LivoxCompatNode : public rclcpp::Node
{
public:
  T100LivoxCompatNode()
  : Node("livox_lidar_publisher")
  {
    device_ip_ = declare_parameter<std::string>("device_ip", "192.168.1.12");
    bind_ip_ = declare_parameter<std::string>("bind_ip", "0.0.0.0");
    host_ip_ = declare_parameter<std::string>("host_ip", "");
    control_port_ = declare_parameter<int>("control_port", 60000);
    point_port_ = declare_parameter<int>("point_port", 60001);
    imu_port_ = declare_parameter<int>("imu_port", 60003);
    publish_freq_ = declare_parameter<double>("publish_freq", 10.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    xfer_format_ = declare_parameter<int>("xfer_format", 1);
    auto_configure_destinations_ =
      declare_parameter<bool>("auto_configure_destinations", true);
    restore_destinations_on_exit_ =
      declare_parameter<bool>("restore_destinations_on_exit", true);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    stop_on_exit_ = declare_parameter<bool>("stop_on_exit", true);
    control_timeout_ms_ = declare_parameter<int>("control_timeout_ms", 800);
    point_interval_fallback_ns_ =
      declare_parameter<int64_t>("point_interval_fallback_ns", 0);
    max_buffer_points_ =
      declare_parameter<int>("max_buffer_points", 500000);

    if (device_ip_.empty()) {
      throw std::runtime_error("set device_ip to the T100 IPv4 address");
    }
    if (publish_freq_ <= 0.0 || publish_freq_ > 100.0) {
      throw std::runtime_error("publish_freq must be in (0, 100]");
    }
    if (xfer_format_ != 0 && xfer_format_ != 1) {
      throw std::runtime_error("xfer_format must be 1 (Livox CustomMsg) or 0 (PointCloud2)");
    }
    if (control_port_ < 1 || control_port_ > 65535 ||
        point_port_ < 1 || point_port_ > 65535 ||
        imu_port_ < 1 || imu_port_ > 65535) {
      throw std::runtime_error("UDP ports must be in 1..65535");
    }
    if (control_timeout_ms_ < 50 || max_buffer_points_ < 1000 ||
        point_interval_fallback_ns_ < 0) {
      throw std::runtime_error(
        "invalid control_timeout_ms, max_buffer_points, or point_interval_fallback_ns");
    }
    if (point_port_ == imu_port_) {
      throw std::runtime_error("point_port and imu_port must be different");
    }

    if (auto_configure_destinations_ && host_ip_.empty()) {
      host_ip_ = routeLocalIpv4(device_ip_);
    }

    if (xfer_format_ == 1) {
      custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    } else {
      pointcloud2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    try {
      openSockets();
      if (auto_configure_destinations_) {
        configureDestinations();
      }
      if (auto_start_) {
        sampling_start_attempted_ = true;
        startSampling();
        sampling_started_ = true;
      }

      running_.store(true);
      point_thread_ = std::thread(&T100LivoxCompatNode::pointReceiveLoop, this);
      imu_thread_ = std::thread(&T100LivoxCompatNode::imuReceiveLoop, this);
      const auto period = std::chrono::duration<double>(1.0 / publish_freq_);
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&T100LivoxCompatNode::publishPointFrame, this));

      RCLCPP_INFO(
        get_logger(),
        "T100 MID-360-compatible driver started: device=%s host=%s point=%d imu=%d topic=%s format=%s",
        device_ip_.c_str(), host_ip_.c_str(), point_port_, imu_port_,
        lidar_topic_.c_str(), xfer_format_ == 1 ? "CustomMsg" : "PointCloud2");
      RCLCPP_INFO(
        get_logger(),
        "Publishing Livox-compatible CustomMsg/IMU. T100 time_interval is converted with "
        "the MID-360 driver formula: point_interval_ns = time_interval * 100 / dot_num.");
    } catch (...) {
      running_.store(false);
      if (sampling_start_attempted_) {
        stopSamplingNoThrow();
      }
      if (point_thread_.joinable()) {
        point_thread_.join();
      }
      if (imu_thread_.joinable()) {
        imu_thread_.join();
      }
      restoreDestinationsNoThrow(true);
      closeSockets();
      throw;
    }
  }

  ~T100LivoxCompatNode() override
  {
    shutdownDriver();
  }

private:
  int createBoundSocket(int port, int receive_buffer_bytes)
  {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer_bytes, sizeof(receive_buffer_bytes));

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_ip_.c_str(), &local.sin_addr) != 1) {
      close(fd);
      throw std::runtime_error("bind_ip is not a valid IPv4 address");
    }
    if (bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
      const std::string error = std::strerror(errno);
      close(fd);
      throw std::runtime_error("UDP bind failed on port " + std::to_string(port) + ": " + error);
    }
    return fd;
  }

  void openSockets()
  {
    control_fd_ = createBoundSocket(0, 256 * 1024);
    point_fd_ = createBoundSocket(point_port_, 8 * 1024 * 1024);
    imu_fd_ = createBoundSocket(imu_port_, 2 * 1024 * 1024);

    device_addr_ = {};
    device_addr_.sin_family = AF_INET;
    device_addr_.sin_port = htons(static_cast<uint16_t>(control_port_));
    if (inet_pton(AF_INET, device_ip_.c_str(), &device_addr_.sin_addr) != 1) {
      throw std::runtime_error("device_ip is not a valid IPv4 address");
    }
  }

  void closeSockets()
  {
    for (int *fd : {&control_fd_, &point_fd_, &imu_fd_}) {
      if (*fd >= 0) {
        close(*fd);
        *fd = -1;
      }
    }
  }

  std::vector<uint8_t> controlRequest(
    uint8_t command_set, uint8_t command_id, const std::vector<uint8_t> &payload)
  {
    if (control_fd_ < 0) {
      throw std::runtime_error("control socket is not open");
    }
    const uint16_t sequence = next_control_sequence_++;
    const auto frame = makeControlFrame(command_set, command_id, sequence, payload);
    const ssize_t sent = sendto(
      control_fd_, frame.data(), frame.size(), 0,
      reinterpret_cast<sockaddr *>(&device_addr_), sizeof(device_addr_));
    if (sent != static_cast<ssize_t>(frame.size())) {
      throw std::runtime_error("failed to send T100 control request");
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(control_timeout_ms_);
    std::array<uint8_t, 2048> buffer{};
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) {
        break;
      }
      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(control_fd_, &read_set);
      timeval timeout{};
      timeout.tv_sec = static_cast<time_t>(remaining.count() / 1000000);
      timeout.tv_usec = static_cast<suseconds_t>(remaining.count() % 1000000);
      const int ready = select(control_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
      if (ready <= 0) {
        continue;
      }

      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t received = recvfrom(
        control_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&source), &source_size);
      if (received < 0) {
        continue;
      }
      if (source.sin_addr.s_addr != device_addr_.sin_addr.s_addr ||
          ntohs(source.sin_port) != static_cast<uint16_t>(control_port_)) {
        continue;
      }

      ControlReply reply;
      if (!decodeControlFrame(buffer.data(), static_cast<size_t>(received), reply)) {
        continue;
      }
      if (reply.sender != kDeviceReceiver || reply.receiver != kClientSender ||
          reply.sequence != sequence || reply.command_set != command_set ||
          reply.command_id != command_id || (reply.flags & kResponseFlag) == 0) {
        // Ignore asynchronous status pushes and unrelated responses.
        continue;
      }
      return reply.payload;
    }
    throw std::runtime_error(
      "T100 control timeout for command " + std::to_string(command_set) +
      "/" + std::to_string(command_id));
  }

  std::vector<uint8_t> readParameter(uint32_t uid, uint32_t expected_size)
  {
    std::vector<uint8_t> payload;
    appendU32(payload, uid);
    appendU32(payload, expected_size);
    const auto response = controlRequest(0x32, 0x12, payload);
    if (response.size() < 9 || response[0] != 0) {
      throw std::runtime_error("T100 parameter read failed for UID " + std::to_string(uid));
    }
    const uint32_t returned_uid = readU32(response.data() + 1);
    const uint16_t returned_size = readU16(response.data() + 5);
    if (returned_uid != uid || returned_size != expected_size ||
        response.size() != static_cast<size_t>(9 + returned_size)) {
      throw std::runtime_error("T100 parameter readback format mismatch");
    }
    return std::vector<uint8_t>(response.begin() + 9, response.end());
  }

  void writeParameter(uint32_t uid, const std::vector<uint8_t> &value)
  {
    std::vector<uint8_t> payload;
    appendU32(payload, uid);
    appendU32(payload, static_cast<uint32_t>(value.size()));
    payload.insert(payload.end(), value.begin(), value.end());
    const auto response = controlRequest(0x32, 0x13, payload);
    if (response.size() != 1 || response[0] != 0) {
      throw std::runtime_error("T100 parameter write failed for UID " + std::to_string(uid));
    }
    const auto readback = readParameter(uid, static_cast<uint32_t>(value.size()));
    if (readback != value) {
      throw std::runtime_error("T100 parameter readback mismatch for UID " + std::to_string(uid));
    }
  }

  void configureDestinations()
  {
    std::vector<uint8_t> ip_bytes;
    if (!encodeIpv4LittleEndian(host_ip_, ip_bytes)) {
      throw std::runtime_error("host_ip is not a valid IPv4 address");
    }

    const std::array<std::pair<uint32_t, uint32_t>, 4> settings{{
      {0x0001001B, 4},  // host_pcl_ip
      {0x0001001C, 2},  // host_pcl_port
      {0x0001001D, 4},  // host_imu_ip
      {0x0001001E, 2}   // host_imu_port
    }};

    for (const auto &setting : settings) {
      original_destinations_[setting.first] = readParameter(setting.first, setting.second);
    }

    try {
      writeParameter(0x0001001B, ip_bytes);
      std::vector<uint8_t> port;
      appendU16(port, static_cast<uint16_t>(point_port_));
      writeParameter(0x0001001C, port);

      writeParameter(0x0001001D, ip_bytes);
      port.clear();
      appendU16(port, static_cast<uint16_t>(imu_port_));
      writeParameter(0x0001001E, port);
    } catch (...) {
      restoreDestinationsNoThrow(true);
      throw;
    }
  }

  void restoreDestinationsNoThrow(bool force = false)
  {
    if ((!restore_destinations_on_exit_ && !force) ||
        original_destinations_.empty() || control_fd_ < 0) {
      return;
    }
    for (const uint32_t uid : {0x0001001E, 0x0001001D, 0x0001001C, 0x0001001B}) {
      const auto found = original_destinations_.find(uid);
      if (found == original_destinations_.end()) {
        continue;
      }
      try {
        writeParameter(uid, found->second);
      } catch (const std::exception &error) {
        RCLCPP_WARN(
          get_logger(), "could not restore T100 parameter UID 0x%08X: %s",
          uid, error.what());
      }
    }
    original_destinations_.clear();
  }

  void setTargetState(uint8_t state)
  {
    std::vector<uint8_t> payload{1, 0, 3, 0, 0, 0, 1, 0, state};
    const auto response = controlRequest(0x32, 0x50, payload);
    if (response.size() != 5 ||
        std::any_of(response.begin(), response.end(), [](uint8_t value) { return value != 0; })) {
      throw std::runtime_error("T100 rejected target-state request");
    }
  }

  void setSamplingEnable(bool enabled)
  {
    const auto response = controlRequest(0x32, 0x14, {static_cast<uint8_t>(enabled ? 1 : 0)});
    if (response.size() != 1 || response[0] != 0) {
      throw std::runtime_error("T100 rejected sampling-enable request");
    }
  }

  void startSampling()
  {
    setTargetState(1);
    setSamplingEnable(true);
    const auto response = controlRequest(0x32, 0x51, {1, 0, 3, 0, 0, 0});
    if (response.size() != 14 || response[0] != 0 || response[13] != 1) {
      throw std::runtime_error("T100 did not confirm sampling target state");
    }
  }

  void stopSamplingNoThrow()
  {
    try {
      setSamplingEnable(false);
      setTargetState(2);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "could not stop T100 sampling cleanly: %s", error.what());
    }
    sampling_started_ = false;
  }

  void shutdownDriver()
  {
    if (shutdown_started_.exchange(true)) {
      return;
    }
    if (sampling_started_ && stop_on_exit_) {
      stopSamplingNoThrow();
    }
    running_.store(false);
    if (point_thread_.joinable()) {
      point_thread_.join();
    }
    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
    restoreDestinationsNoThrow();
    closeSockets();
  }

  void pointReceiveLoop()
  {
    std::array<uint8_t, 65536> buffer{};
    while (running_.load()) {
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t received = recvfrom(
        point_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&source), &source_size);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "point UDP recvfrom failed: %s", std::strerror(errno));
        }
        continue;
      }
      if (source.sin_addr.s_addr != device_addr_.sin_addr.s_addr) {
        continue;
      }
      processPointPacket(buffer.data(), static_cast<size_t>(received));
    }
  }

  void imuReceiveLoop()
  {
    std::array<uint8_t, 2048> buffer{};
    while (running_.load()) {
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t received = recvfrom(
        imu_fd_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&source), &source_size);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "IMU UDP recvfrom failed: %s", std::strerror(errno));
        }
        continue;
      }
      if (source.sin_addr.s_addr != device_addr_.sin_addr.s_addr) {
        continue;
      }
      processImuPacket(buffer.data(), static_cast<size_t>(received));
    }
  }

  bool acceptTimeType(uint8_t time_type)
  {
    bool log_mismatch = false;
    uint8_t expected = time_type;
    {
      std::lock_guard<std::mutex> lock(time_type_mutex_);
      if (time_type_seen_ < 0) {
        time_type_seen_ = time_type;
        return true;
      }
      expected = static_cast<uint8_t>(time_type_seen_);
      if (time_type == expected) {
        return true;
      }
      ++time_type_mismatches_;
      if (!time_type_mismatch_logged_) {
        time_type_mismatch_logged_ = true;
        log_mismatch = true;
      }
    }
    if (log_mismatch) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting T100 packet with time_type=%u; the first stream established time_type=%u. "
        "Point and IMU must share one time domain. Check synchronization settings and restart.",
        static_cast<unsigned int>(time_type), static_cast<unsigned int>(expected));
    }
    return false;
  }

  void processPointPacket(const uint8_t *data, size_t size)
  {
    if (size < kDataHeaderSize || data[0] != 1) {
      ++bad_packets_;
      return;
    }
    const uint16_t declared_size = readU16(data + 1);
    const uint16_t time_interval = readU16(data + 3);
    const uint16_t point_count = readU16(data + 5);
    const uint16_t sequence = readU16(data + 7);
    const uint8_t data_type = data[10];
    const uint8_t time_type = data[11];
    const uint64_t device_timestamp = readU64(data + 28);

    if (declared_size != size || data_type != kDataTypeCartesian ||
        point_count == 0 ||
        size != kDataHeaderSize + static_cast<size_t>(point_count) * kPointRecordSize ||
        time_type > 2) {
      ++bad_packets_;
      return;
    }

    if (!acceptTimeType(time_type)) {
      ++bad_packets_;
      return;
    }
    const uint64_t packet_base_ns = clock_mapper_.map(device_timestamp, time_type);
    uint64_t point_interval_ns =
      (static_cast<uint64_t>(time_interval) * 100ULL) / point_count;
    if (point_interval_ns == 0 && point_interval_fallback_ns_ > 0) {
      point_interval_ns = static_cast<uint64_t>(point_interval_fallback_ns_);
    }
    if (point_interval_ns == 0 && !warned_zero_interval_) {
      warned_zero_interval_ = true;
      RCLCPP_WARN(
        get_logger(),
        "T100 time_interval is zero/too small; publishing zero per-point offsets. "
        "FAST-LIO deskew quality is not validated until this field is understood.");
    }

    std::vector<TimedPoint> decoded;
    decoded.reserve(point_count);
    for (uint16_t i = 0; i < point_count; ++i) {
      const uint8_t *record = data + kDataHeaderSize + static_cast<size_t>(i) * kPointRecordSize;
      TimedPoint point;
      point.timestamp_ns = packet_base_ns + static_cast<uint64_t>(i) * point_interval_ns;
      point.x = static_cast<float>(readI32(record)) / 1000.0F;
      point.y = static_cast<float>(readI32(record + 4)) / 1000.0F;
      point.z = static_cast<float>(readI32(record + 8)) / 1000.0F;
      point.reflectivity = record[12];
      point.tag = record[13];
      // Livox ROS Driver2 assigns MID-360 point lines as i % 4.
      point.line = static_cast<uint8_t>(i % 4);
      decoded.push_back(point);
    }

    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      for (const auto &point : decoded) {
        pending_points_.push_back(point);
      }
      while (pending_points_.size() > static_cast<size_t>(max_buffer_points_)) {
        pending_points_.pop_front();
        ++buffer_overruns_;
      }
    }
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      point_sequence_.update(sequence);
    }
    ++point_packets_;
  }

  void processImuPacket(const uint8_t *data, size_t size)
  {
    if (size != kImuPacketSize || data[0] != 1 ||
        readU16(data + 1) != kImuPacketSize ||
        readU16(data + 5) != 1 || data[10] != kDataTypeImu || data[11] > 2) {
      ++bad_packets_;
      return;
    }

    const uint32_t expected_crc = readU32(data + 24);
    const uint32_t actual_crc = crc32Ieee(data + 28, size - 28);
    if (expected_crc != actual_crc) {
      ++bad_packets_;
      return;
    }

    if (!acceptTimeType(data[11])) {
      ++bad_packets_;
      return;
    }
    const uint64_t timestamp_ns = clock_mapper_.map(readU64(data + 28), data[11]);
    const float gx = readF32(data + 36);
    const float gy = readF32(data + 40);
    const float gz = readF32(data + 44);
    const float ax_g = readF32(data + 48);
    const float ay_g = readF32(data + 52);
    const float az_g = readF32(data + 56);
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
        !std::isfinite(ax_g) || !std::isfinite(ay_g) || !std::isfinite(az_g)) {
      ++bad_packets_;
      return;
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = toRosTime(timestamp_ns);
    message.header.frame_id = frame_id_;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity.x = gx;
    message.angular_velocity.y = gy;
    message.angular_velocity.z = gz;
    // T100 protocol documents accelerometer values in g; ROS uses m/s^2.
    message.linear_acceleration.x = static_cast<double>(ax_g) * kGravity;
    message.linear_acceleration.y = static_cast<double>(ay_g) * kGravity;
    message.linear_acceleration.z = static_cast<double>(az_g) * kGravity;
    imu_pub_->publish(message);

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      imu_sequence_.update(readU16(data + 7));
    }
    ++imu_packets_;
  }

  void publishPointFrame()
  {
    std::vector<TimedPoint> points;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      if (pending_points_.empty()) {
        return;
      }
      points.assign(pending_points_.begin(), pending_points_.end());
      pending_points_.clear();
    }

    std::stable_sort(
      points.begin(), points.end(),
      [](const TimedPoint &a, const TimedPoint &b) {
        return a.timestamp_ns < b.timestamp_ns;
      });

    const uint64_t base_time = points.front().timestamp_ns;
    const uint64_t max_offset = std::numeric_limits<uint32_t>::max();
    points.erase(
      std::remove_if(
        points.begin(), points.end(),
        [base_time, max_offset](const TimedPoint &point) {
          return point.timestamp_ns < base_time ||
                 point.timestamp_ns - base_time > max_offset;
        }),
      points.end());
    if (points.empty()) {
      return;
    }

    if (xfer_format_ == 1) {
      livox_ros_driver2::msg::CustomMsg message;
      message.header.stamp = toRosTime(base_time);
      message.header.frame_id = frame_id_;
      message.timebase = base_time;
      message.point_num = static_cast<uint32_t>(points.size());
      message.lidar_id = 0;
      message.rsvd = {0, 0, 0};
      message.points.reserve(points.size());
      for (const auto &source : points) {
        livox_ros_driver2::msg::CustomPoint point;
        point.offset_time = static_cast<uint32_t>(source.timestamp_ns - base_time);
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.reflectivity = source.reflectivity;
        point.tag = source.tag;
        point.line = source.line;
        message.points.push_back(point);
      }
      custom_pub_->publish(message);
    } else {
      publishPointCloud2(points, base_time);
    }

    ++published_frames_;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats_log_ >= std::chrono::seconds(5)) {
      uint64_t missing_points = 0;
      uint64_t missing_imu = 0;
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        missing_points = point_sequence_.missing;
        missing_imu = imu_sequence_.missing;
      }
      RCLCPP_INFO(
        get_logger(),
        "T100 RX: point_packets=%llu imu_packets=%llu bad=%llu seq_missing(point/imu)=%llu/%llu "
        "buffer_overruns=%llu published_frames=%llu time_type_mismatches=%llu",
        static_cast<unsigned long long>(point_packets_.load()),
        static_cast<unsigned long long>(imu_packets_.load()),
        static_cast<unsigned long long>(bad_packets_.load()),
        static_cast<unsigned long long>(missing_points),
        static_cast<unsigned long long>(missing_imu),
        static_cast<unsigned long long>(buffer_overruns_.load()),
        static_cast<unsigned long long>(published_frames_.load()),
        static_cast<unsigned long long>(time_type_mismatches_.load()));
      last_stats_log_ = now;
    }
  }

  void publishPointCloud2(const std::vector<TimedPoint> &points, uint64_t base_time)
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = toRosTime(base_time);
    message.header.frame_id = frame_id_;
    message.height = 1;
    message.width = static_cast<uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = false;
    message.fields.resize(7);

    const std::array<std::string, 7> names{
      "x", "y", "z", "intensity", "tag", "line", "timestamp"};
    const std::array<uint32_t, 7> offsets{0, 4, 8, 12, 16, 17, 18};
    const std::array<uint8_t, 7> types{
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::FLOAT32,
      sensor_msgs::msg::PointField::UINT8,
      sensor_msgs::msg::PointField::UINT8,
      sensor_msgs::msg::PointField::FLOAT64};
    for (size_t i = 0; i < message.fields.size(); ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = offsets[i];
      message.fields[i].datatype = types[i];
      message.fields[i].count = 1;
    }

    // Same packed PointXYZRTLT layout used by Livox ROS Driver2 (26 bytes).
    message.point_step = 26;
    message.row_step = message.point_step * message.width;
    message.data.resize(message.row_step, 0);
    for (size_t i = 0; i < points.size(); ++i) {
      uint8_t *record = message.data.data() + i * message.point_step;
      const float intensity = static_cast<float>(points[i].reflectivity);
      const double timestamp = static_cast<double>(points[i].timestamp_ns);
      std::memcpy(record, &points[i].x, sizeof(float));
      std::memcpy(record + 4, &points[i].y, sizeof(float));
      std::memcpy(record + 8, &points[i].z, sizeof(float));
      std::memcpy(record + 12, &intensity, sizeof(float));
      record[16] = points[i].tag;
      record[17] = points[i].line;
      std::memcpy(record + 18, &timestamp, sizeof(double));
    }
    pointcloud2_pub_->publish(message);
  }

  std::string device_ip_;
  std::string bind_ip_;
  std::string host_ip_;
  std::string frame_id_;
  std::string lidar_topic_;
  std::string imu_topic_;
  int control_port_{60000};
  int point_port_{60001};
  int imu_port_{60003};
  int xfer_format_{1};
  int control_timeout_ms_{800};
  int max_buffer_points_{500000};
  double publish_freq_{10.0};
  int64_t point_interval_fallback_ns_{0};
  bool auto_configure_destinations_{true};
  bool restore_destinations_on_exit_{true};
  bool auto_start_{true};
  bool stop_on_exit_{true};

  int control_fd_{-1};
  int point_fd_{-1};
  int imu_fd_{-1};
  sockaddr_in device_addr_{};
  uint16_t next_control_sequence_{0x1000};
  bool sampling_started_{false};
  bool sampling_start_attempted_{false};
  bool warned_zero_interval_{false};

  std::atomic<bool> running_{false};
  std::atomic<bool> shutdown_started_{false};
  std::atomic<uint64_t> point_packets_{0};
  std::atomic<uint64_t> imu_packets_{0};
  std::atomic<uint64_t> bad_packets_{0};
  std::atomic<uint64_t> buffer_overruns_{0};
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> time_type_mismatches_{0};

  std::thread point_thread_;
  std::thread imu_thread_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::mutex point_mutex_;
  std::deque<TimedPoint> pending_points_;
  std::mutex stats_mutex_;
  std::mutex time_type_mutex_;
  int time_type_seen_{-1};
  bool time_type_mismatch_logged_{false};
  SequenceStats point_sequence_;
  SequenceStats imu_sequence_;
  DeviceClockMapper clock_mapper_;
  std::map<uint32_t, std::vector<uint8_t>> original_destinations_;
  std::chrono::steady_clock::time_point last_stats_log_{std::chrono::steady_clock::now()};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<T100LivoxCompatNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("livox_ros_driver2_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
