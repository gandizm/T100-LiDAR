// mid360_agras_demo: live statistics, optional PCD dump, or pcap replay through the same parser.
//
//   ./mid360_agras_demo                      live: print one line per second
//   ./mid360_agras_demo --pcd out.pcd [s]    live: accumulate s seconds (default 2) into an ASCII PCD
//   ./mid360_agras_demo --pcap file.pcap     offline: parse a capture (tcpdump / record.py / Wireshark
//                                            "pcap" format; pcapng is not supported) and print frames
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include "mid360_agras.h"

namespace {
volatile std::sig_atomic_t g_stop = 0;

int replay_pcap(const char* path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", path);
    return 1;
  }
  std::vector<uint8_t> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (raw.size() < 24 || (std::memcmp(raw.data(), "\xd4\xc3\xb2\xa1", 4) != 0 &&
                          std::memcmp(raw.data(), "\x4d\x3c\xb2\xa1", 4) != 0)) {
    std::fprintf(stderr, "%s: not a little-endian pcap file (pcapng: convert with editcap -F pcap)\n", path);
    return 1;
  }
  mid360::Config cfg;
  mid360::FrameAssembler fa(cfg);
  mid360::Frame fr;
  size_t o = 24, imu = 0;
  auto show = [](const mid360::Frame& f) {
    std::printf("frame %4llu: %3u pkts, %5u slots -> %6zu points, lost %u, span %.1f ms\n",
                (unsigned long long)f.seq, f.packets, f.raw_points, f.points.size(), f.lost_packets,
                (f.t_end_ns - f.t_begin_ns) / 1e6);
  };
  while (o + 16 <= raw.size()) {
    uint32_t cl;
    std::memcpy(&cl, &raw[o + 8], 4);
    const uint8_t* pk = &raw[o + 16];
    o += 16 + cl;
    if (o > raw.size() || cl < 42 || pk[12] != 0x08 || pk[13] != 0x00) continue;  // IPv4 only
    const size_t ihl = (pk[14] & 15) * 4;
    if (pk[14 + 9] != 17) continue;                                              // UDP only
    const uint8_t* udp = pk + 14 + ihl;
    const uint16_t dport = (udp[2] << 8) | udp[3];
    const uint8_t* pl = udp + 8;
    const size_t plen = cl - (pl - pk);
    if (dport == cfg.point_port) {
      if (fa.feed(pl, plen, &fr)) show(fr);
    } else if (dport == cfg.imu_port) {
      mid360::Imu m;
      imu += mid360::parse_imu(pl, plen, &m);
    }
  }
  if (fa.flush(&fr)) show(fr);
  const auto& s = fa.stats();
  std::printf("total: %llu point packets, %llu frames, %llu slots, %llu valid points (%.1f%%), "
              "%llu lost packets, %zu IMU samples\n",
              (unsigned long long)s.point_packets, (unsigned long long)s.frames,
              (unsigned long long)s.raw_points, (unsigned long long)s.valid_points,
              s.raw_points ? 100.0 * s.valid_points / s.raw_points : 0.0, (unsigned long long)s.lost_packets, imu);
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc >= 3 && std::strcmp(argv[1], "--pcap") == 0) return replay_pcap(argv[2]);
  const char* pcd = nullptr;
  double pcd_secs = 2.0;
  if (argc >= 3 && std::strcmp(argv[1], "--pcd") == 0) {
    pcd = argv[2];
    if (argc >= 4) pcd_secs = std::atof(argv[3]);
  }
  std::signal(SIGINT, [](int) { g_stop = 1; });
  std::signal(SIGTERM, [](int) { g_stop = 1; });

  mid360::Driver drv;
  std::mutex mu;
  std::vector<mid360::Point> acc;
  size_t last_n = 0;
  float last_acc[3] = {0, 0, 0};
  drv.on_frame([&](const mid360::Frame& f) {
    std::lock_guard<std::mutex> lk(mu);
    last_n = f.points.size();
    if (pcd) acc.insert(acc.end(), f.points.begin(), f.points.end());
  });
  drv.on_imu([&](const mid360::Imu& m) {
    std::lock_guard<std::mutex> lk(mu);
    std::memcpy(last_acc, m.accel, sizeof(last_acc));
  });
  if (!drv.start()) {
    std::fprintf(stderr, "start failed: %s\n", drv.error().c_str());
    return 1;
  }
  std::printf("listening on UDP 60001/60003 (lidar 192.168.1.10, host must be 192.168.1.20/24)\n");
  const auto t0 = std::chrono::steady_clock::now();
  mid360::Stats prev{};
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto s = drv.stats();
    const double age = drv.seconds_since_data();
    {
      std::lock_guard<std::mutex> lk(mu);
      std::printf("%5llu pkt/s  %3llu frames/s  last frame %6zu pts  lost %llu  IMU %3llu/s acc [%.2f %.2f %.2f] m/s2  %s\n",
                  (unsigned long long)(s.point_packets - prev.point_packets),
                  (unsigned long long)(s.frames - prev.frames), last_n,
                  (unsigned long long)s.lost_packets, (unsigned long long)(s.imu_packets - prev.imu_packets),
                  last_acc[0], last_acc[1], last_acc[2],
                  age < 0 ? "(no data yet: check cable, power, 192.168.1.20/24)" : age > 1 ? "(stream stopped)" : "");
    }
    std::fflush(stdout);
    prev = s;
    if (pcd && std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >= pcd_secs) break;
  }
  drv.stop();
  if (pcd) {
    std::lock_guard<std::mutex> lk(mu);
    std::ofstream o(pcd);
    o << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity tag line\nSIZE 4 4 4 4 1 1\nTYPE F F F F U U\n"
         "COUNT 1 1 1 1 1 1\nWIDTH " << acc.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
      << acc.size() << "\nDATA ascii\n";
    for (const auto& p : acc)
      o << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << ' '
        << int(p.line) << '\n';
    std::printf("wrote %zu points to %s\n", acc.size(), pcd);
  }
  return 0;
}
