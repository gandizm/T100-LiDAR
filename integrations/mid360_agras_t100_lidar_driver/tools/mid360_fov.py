#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MID360-agras 视场（FOV / ROI）配置工具 —— 打开固件屏蔽的方位缺口。

裸板出厂时点云方位角有约 65° 的缺口（右后方），是固件在开机时把 FPGA 的
work_mode1 bit8 设成 3（roi[0x3]）造成的。本工具用雷达的原生配置协议在运行时
把它改回 0（roi[0x0] = 全周），实测缺口归零（gap 0°，约 1 万包/帧）。

纯标准库，跨平台（Windows / Linux / macOS）。不改固件、不动 flash、可逆：
断电重启后固件又按开机逻辑设回缺口，重跑本工具即可。

用法:
    python mid360_fov.py --full                 # 打开全周（最常用）
    python mid360_fov.py --read                 # 只读，打印关键参数核对
    python mid360_fov.py --roi 0 180            # 只保留 0..180° 扇区
    python mid360_fov.py --restore              # 点云流没回来时，把目的地址写回本机
    python mid360_fov.py --full --host 192.168.1.20 --device 192.168.1.10

协议要点（逆向自可用上位机，已对 oracle 值验证）:
    寻址 sender=0x28 receiver=0xAD, UDP 60000, cmd_set 0x32, flag 0x40
    读 = 0x12  payload <uid:u32><len:u32>   回复 status(1)+uid(4)+len(4)+value
    写 = 0x13  payload <uid:u32><len:u32>+value   回复 status，0=成功
    坑①: 写 host_pcl_port 会触发 nml_xfer 重定向点云流 -> 流可能改道收不到
    坑②: 写 IP 参数的 4 字节要【反序】: 192.168.1.20 发 14 01 a8 c0
"""
import argparse
import socket
import struct
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    pass

SOF = 0x55
HOST_ADDR = 0x28
DEV_ADDR = 0xAD
CMD_SET = 0x32
CMD_READ = 0x12
CMD_WRITE = 0x13
CMD_SAMPLE = 0x14
CMD_SET_STATE = 0x50
CMD_READ_STATE = 0x51

# user 模块参数 UID（见 param 表）
UID = {
    "roi_en": 0x00010023,
    "roi_yaw0": 0x00010024, "roi_yaw1": 0x00010025,
    "roi_pitch0": 0x00010026, "roi_pitch1": 0x00010027,
    "roi_yaw0_1": 0x00010029, "roi_yaw1_1": 0x0001002A,
    "roi_pitch0_1": 0x0001002B, "roi_pitch1_1": 0x0001002C,
    "host_pcl_ip": 0x0001001B, "host_pcl_port": 0x0001001C,
    "host_imu_ip": 0x0001001D, "host_imu_port": 0x0001001E,
    "dev_type": 0x00000003, "line_num": 0x00000006,
}


def crc8(data):
    c = 0x77
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
    return c


def crc16(data):
    c = 0x3692
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c


def encode(cmd_id, payload=b"", seq=0, flags=0x40):
    n = 13 + len(payload)
    head = bytearray([SOF, n & 0xFF, ((n >> 8) & 0x03) | 0x04])
    head.append(crc8(head))
    head += bytes([HOST_ADDR, DEV_ADDR]) + struct.pack("<H", seq)
    head += bytes([flags, CMD_SET, cmd_id])
    frame = bytes(head) + payload
    return frame + struct.pack("<H", crc16(frame))


def decode(buf):
    if len(buf) < 13 or buf[0] != SOF:
        return None
    n = buf[1] | ((buf[2] & 0x03) << 8)
    if (buf[2] >> 2) != 1 or n != len(buf):
        return None
    if crc8(buf[:3]) != buf[3]:
        return None
    if crc16(buf[:-2]) != struct.unpack_from("<H", buf, len(buf) - 2)[0]:
        return None
    return {"seq": struct.unpack_from("<H", buf, 6)[0], "ack": bool(buf[8] & 0x80),
            "cmd_set": buf[9], "cmd_id": buf[10], "payload": buf[11:-2]}


def ip_reversed(text):
    """固件按反序读 IP：192.168.1.20 -> b'\\x14\\x01\\xa8\\xc0'。"""
    octets = [int(x) for x in text.split(".")]
    if len(octets) != 4 or any(not 0 <= o <= 255 for o in octets):
        raise ValueError("IPv4 格式错误: " + text)
    return bytes(reversed(octets))


class Lidar:
    def __init__(self, device, port=60000, timeout=0.8):
        self.addr = (device, port)
        self.timeout = timeout
        self.seq = 0x100
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(timeout)

    def close(self):
        self.sock.close()

    def _txn(self, cmd_id, payload):
        self.seq = (self.seq + 1) & 0xFFFF
        seq = self.seq
        self.sock.sendto(encode(cmd_id, payload, seq), self.addr)
        end = time.time() + self.timeout
        while time.time() < end:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (socket.timeout, TimeoutError, OSError):
                break
            f = decode(data)
            if f and f["cmd_set"] == CMD_SET and f["cmd_id"] == cmd_id and f["ack"]:
                return f["payload"]
        return None

    def read(self, uid, length):
        """返回 int 值；读失败返回 None。"""
        p = self._txn(CMD_READ, struct.pack("<II", uid, length))
        if not p or len(p) < 9 + length or p[0] != 0:
            return None
        return int.from_bytes(p[9:9 + length], "little")

    def write(self, uid, value):
        """value 是 bytes。返回 status（0=成功），无应答返回 None。"""
        p = self._txn(CMD_WRITE, struct.pack("<II", uid, len(value)) + value)
        return p[0] if p else None

    def sample(self, on):
        p = self._txn(CMD_SAMPLE, bytes([1 if on else 0]))
        return p[0] if p else None

    def set_state(self, value):
        """设置目标工作状态 32/50。样机: 1=采样(电机转), 2=待机。回 status，0=成功。"""
        pl = struct.pack("<H", 1) + struct.pack("<IHB", 3, 1, value & 0xFF)
        p = self._txn(CMD_SET_STATE, pl)
        return p[0] if p else None

    def read_state(self):
        """读目标工作状态 32/51。返回目标值，失败 None。"""
        p = self._txn(CMD_READ_STATE, struct.pack("<H", 1) + struct.pack("<I", 3))
        if not p or len(p) < 1 or p[0] != 0:
            return None
        return p[-1]


def do_read(lid):
    print("关键参数核对:")
    checks = [("dev_type", UID["dev_type"], 1, 10),
              ("line_num", UID["line_num"], 1, 4),
              ("roi_en", UID["roi_en"], 1, None),
              ("host_pcl_port", UID["host_pcl_port"], 2, None)]
    ok = True
    for name, uid, ln, exp in checks:
        v = lid.read(uid, ln)
        tag = ""
        if v is None:
            tag = "  <- 读失败"
            ok = False
        elif exp is not None:
            tag = "  OK" if v == exp else "  <- 期望 %d" % exp
        print("  %-14s = %s%s" % (name, v, tag))
    return ok


def restore_stream(lid, host, pcl_port, imu_port):
    """把点云/IMU 目的地址写回本机（IP 反序），再开采样。"""
    ipb = ip_reversed(host)
    steps = [("host_pcl_ip", UID["host_pcl_ip"], ipb),
             ("host_pcl_port", UID["host_pcl_port"], struct.pack("<H", pcl_port)),
             ("host_imu_ip", UID["host_imu_ip"], ipb),
             ("host_imu_port", UID["host_imu_port"], struct.pack("<H", imu_port))]
    allok = True
    for name, uid, val in steps:
        st = lid.write(uid, val)
        allok = allok and st == 0
        print("  写 %-14s <- %-10s status=%s" % (name, val.hex(), st))
    st = lid.sample(True)
    print("  采样启动 status=%s" % st)
    print("  点云目的地址已指向 %s:%d" % (host, pcl_port))
    return allok


def open_full(lid):
    """roi_en=0 -> FPGA work_mode1 bit8=0 -> roi[0x0] 全周。"""
    st = lid.write(UID["roi_en"], b"\x00")
    print("  写 roi_en <- 00 (全周) status=%s" % st)
    return st == 0


def unlock_performance(lid):
    """解锁性能：打开全周 FOV + set_state 1（电机采样态）。可逆，断电即还原。

    返回 (ok, detail)。两步都走已验证命令：roi_en=0（全周）、32/50 目标=1。
    """
    detail = {}
    detail["roi_en"] = lid.write(UID["roi_en"], b"\x00")
    detail["set_state"] = lid.set_state(1)
    ok = detail["roi_en"] == 0 and detail["set_state"] == 0
    return ok, detail


def set_roi(lid, start, stop):
    """保留 start..stop 扇区（镜像参考上位机 apply_roi；跨 0° 用双窗口）。"""
    start, stop = int(start), int(stop)
    if not (0 <= start <= 360 and 0 <= stop <= 360):
        print("  角度须在 0..360")
        return False
    p0 = lid.read(UID["roi_pitch0"], 4)
    p1 = lid.read(UID["roi_pitch1"], 4)
    # read() 返回无符号数，俯仰起点是负数（-7 读成 4294967289），按无符号原样写回
    pb0 = struct.pack("<I", p0) if p0 is not None else struct.pack("<i", -7)
    pb1 = struct.pack("<I", p1) if p1 is not None else struct.pack("<i", 52)
    results = []
    if start > stop:  # 跨 0°：cfg0 = start..360, cfg1 = 0..stop
        results.append(lid.write(UID["roi_yaw0"], struct.pack("<i", start)))
        results.append(lid.write(UID["roi_yaw1"], struct.pack("<i", 360)))
        results.append(lid.write(UID["roi_yaw0_1"], struct.pack("<i", 0)))
        results.append(lid.write(UID["roi_yaw1_1"], struct.pack("<i", stop)))
        results.append(lid.write(UID["roi_pitch0_1"], pb0))
        results.append(lid.write(UID["roi_pitch1_1"], pb1))
        results.append(lid.write(UID["roi_en"], b"\x03"))
    else:
        results.append(lid.write(UID["roi_yaw0"], struct.pack("<i", start)))
        results.append(lid.write(UID["roi_yaw1"], struct.pack("<i", stop)))
        results.append(lid.write(UID["roi_en"], b"\x01"))
    ok = all(r == 0 for r in results)
    print("  ROI %d..%d -> %s (status %s)" % (start, stop, "成功" if ok else "失败", results))
    return ok


def main():
    ap = argparse.ArgumentParser(description="MID360-agras 视场配置（打开全周 FOV）")
    ap.add_argument("--device", default="192.168.1.10", help="雷达 IP（默认 192.168.1.10）")
    ap.add_argument("--host", default="192.168.1.20", help="本机 IP，点云目的地址（默认 192.168.1.20）")
    ap.add_argument("--pcl-port", type=int, default=60001)
    ap.add_argument("--imu-port", type=int, default=60003)
    ap.add_argument("--full", action="store_true", help="打开全周（去掉缺口）")
    ap.add_argument("--unlock", action="store_true", help="解锁性能：全周 FOV + set_state 1（电机采样态）")
    ap.add_argument("--roi", nargs=2, type=int, metavar=("START", "STOP"), help="只保留 START..STOP 度扇区")
    ap.add_argument("--read", action="store_true", help="只读核对参数")
    ap.add_argument("--restore", action="store_true", help="把点云/IMU 目的地址写回本机")
    ap.add_argument("--no-restore", action="store_true", help="--full/--roi 后不自动恢复流地址")
    args = ap.parse_args()

    if not (args.full or args.unlock or args.roi or args.read or args.restore):
        ap.error("至少选一个动作：--full / --unlock / --roi / --read / --restore")

    lid = Lidar(args.device, timeout=0.8)
    try:
        if lid.read(UID["dev_type"], 1) is None:
            print("连不上雷达 %s:60000。确认本机是 %s/24、雷达已上电。" % (args.device, args.host))
            return 2

        if args.read:
            do_read(lid)

        changed = False
        unlock_failed = False
        if args.unlock:
            print("解锁性能：全周 FOV + 电机采样态...")
            ok, detail = unlock_performance(lid)
            print("  写 roi_en <- 00 (全周) status=%s" % detail["roi_en"])
            print("  set_state <- 1 (电机采样) status=%s" % detail["set_state"])
            changed = ok or changed
            unlock_failed = not ok
        elif args.full:
            print("打开全周 FOV...")
            changed = open_full(lid) or changed
        elif args.roi:
            print("设置 ROI 扇区...")
            changed = set_roi(lid, args.roi[0], args.roi[1]) or changed

        # 改过参数后点云流可能被 nml_xfer 改道，默认把目的地址写回本机
        if (changed or args.restore) and not args.no_restore:
            print("恢复点云流到本机...")
            restore_stream(lid, args.host, args.pcl_port, args.imu_port)

        if changed:
            print("完成。用查看器或 pcl_measure 确认缺口已消失。")
            print("注意：断电重启后缺口会回来，重跑本工具即可。")
        return 1 if unlock_failed else 0
    finally:
        lid.close()


if __name__ == "__main__":
    sys.exit(main())
