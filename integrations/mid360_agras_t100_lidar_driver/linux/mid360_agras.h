// Standalone Linux driver for the DJI Agras Mid-360 ("MID360-agras") lidar.
//
// The Agras firmware streams as soon as it has power and an Ethernet link; no host command is
// needed. Fixed addresses: lidar 192.168.1.10, host must be 192.168.1.20/24.
//   points  UDP 60001  ~3100 packets/s x 96 points (Livox SDK2 point packet, version byte 1)
//   IMU     UDP 60003  200 Hz, gyro rad/s + accel in g
// Packet header (36 B, little endian): version u8, length u16, time_interval u16 (0.1 us),
// dot_num u16, udp_cnt u16, frame_cnt u8 (always 0 on this firmware), data_type u8,
// time_type u8, reserved[12], crc32 u32, timestamp u64 (ns).
//
// No dependencies besides POSIX sockets and C++17. Callbacks run on the driver's own threads.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mid360 {

struct Point {
  float x, y, z;          // m, Livox frame: x forward, y left, z up
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;           // slot % 4: the 4 laser lines are interleaved inside each packet
  uint64_t timestamp_ns;  // lidar clock: packet timestamp + slot * time_interval / dot_num
};

struct Frame {
  uint64_t seq = 0;
  uint64_t t_begin_ns = 0, t_end_ns = 0;  // lidar clock, first / last point
  double host_time = 0;                    // CLOCK_REALTIME seconds when the frame was closed
  uint32_t packets = 0;
  uint32_t lost_packets = 0;               // udp_cnt gaps inside this frame
  uint32_t raw_points = 0;                 // points in the packets, incl. no-return slots
  std::vector<Point> points;               // valid points only (unless keep_no_return)
};

struct Imu {
  uint64_t timestamp_ns;
  float gyro[3];   // rad/s
  float accel[3];  // m/s^2 (the firmware sends g; converted with 9.80665)
};

struct Stats {
  uint64_t point_packets = 0, imu_packets = 0, other_packets = 0, bad_packets = 0;
  uint64_t lost_packets = 0, frames = 0, raw_points = 0, valid_points = 0;
};

struct Config {
  std::string bind_ip = "0.0.0.0";       // local address to bind (0.0.0.0 = any)
  std::string lidar_ip = "192.168.1.10"; // accept packets only from this source ("" = any)
  uint16_t point_port = 60001;
  uint16_t imu_port = 60003;
  bool enable_imu = true;
  double frame_period_s = 0.1;           // frames are cut on the lidar clock
  float min_range_m = 0.05f;             // drops the all-zero "no return" slots
  float max_range_m = 100.0f;            // drops the 200 m "no return" markers
  bool keep_no_return = false;           // true: keep every slot as received
  int rcvbuf_bytes = 8 << 20;            // raise net.core.rmem_max if the kernel caps it
};

// ---- packet level (no sockets): usable for pcap replay or your own receive loop ----

struct PacketHeader {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;  // 0.1 us, whole packet
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;       // 0 IMU, 1 int32 mm xyz, 2 int16 cm xyz, 3 spherical
  uint8_t time_type;
  uint64_t timestamp_ns;
};

constexpr size_t kHeaderSize = 36;

bool parse_header(const uint8_t* data, size_t len, PacketHeader* hdr);

// Appends the packet's points to *out. Returns the number of slots in the packet (raw count),
// or -1 if this is not a point packet.
int parse_points(const uint8_t* data, size_t len, const Config& cfg, std::vector<Point>* out,
                 PacketHeader* hdr = nullptr);

bool parse_imu(const uint8_t* data, size_t len, Imu* out);

// Cuts the point stream into frames of cfg.frame_period_s on the lidar clock.
class FrameAssembler {
 public:
  explicit FrameAssembler(const Config& cfg);
  // Feeds one point packet. Returns true and fills *done when a frame was closed (the packet
  // that closed it starts the next frame).
  bool feed(const uint8_t* data, size_t len, Frame* done);
  // Closes the current frame if it has points (call when the stream stops).
  bool flush(Frame* done);
  const Stats& stats() const { return stats_; }

 private:
  bool close(Frame* done);
  Config cfg_;
  Frame cur_;
  Stats stats_;
  uint64_t seq_ = 0;
  uint64_t frame_start_ns_ = 0;
  bool have_start_ = false;
  bool have_cnt_ = false;
  uint16_t last_cnt_ = 0;
};

// ---- live driver: owns the UDP sockets and the receive threads ----

class Driver {
 public:
  using FrameCallback = std::function<void(const Frame&)>;
  using ImuCallback = std::function<void(const Imu&)>;

  explicit Driver(Config cfg = Config());
  ~Driver();
  Driver(const Driver&) = delete;
  Driver& operator=(const Driver&) = delete;

  // Set before start().
  void on_frame(FrameCallback cb) { frame_cb_ = std::move(cb); }
  void on_imu(ImuCallback cb) { imu_cb_ = std::move(cb); }

  // Binds the sockets and starts the threads. Returns false and sets error() on failure;
  // the usual cause is another program holding UDP 60001 (lidar-web.service, livox_ros_driver2).
  bool start();
  void stop();
  bool running() const { return running_; }
  const std::string& error() const { return error_; }

  Stats stats() const;                 // thread-safe copy of the counters
  double seconds_since_data() const;   // since the last point packet; < 0 if none yet

 private:
  void point_loop();
  void imu_loop();
  int open_socket(uint16_t port);

  Config cfg_;
  FrameCallback frame_cb_;
  ImuCallback imu_cb_;
  std::atomic<bool> running_{false};
  int point_fd_ = -1, imu_fd_ = -1;
  std::thread point_thread_, imu_thread_;
  std::string error_;
  mutable std::mutex stats_mu_;
  Stats stats_;
  uint64_t imu_packets_ = 0;
  std::atomic<int64_t> last_data_ns_{0};
  uint32_t lidar_addr_ = 0;  // network byte order, 0 = accept any source
};

}  // namespace mid360
