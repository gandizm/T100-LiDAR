"""ROS 2 bridge that presents a T100 as a Livox MID360-style ROS source.

It publishes the same message types and default topic names as Livox ROS Driver 2.
It does not emulate the Livox SDK2 device-discovery/control protocol.
"""

import select
import socket
import struct
from dataclasses import replace
import threading
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField

from t100_lidar.protocol import ProtocolError, decode_data
from t100_lidar.ros2_compat import (
    acceleration_g_to_mps2,
    packet_to_livox_points,
    pack_pointcloud2_data,
    split_livox_batches,
)


NSEC_PER_SEC = 1_000_000_000

# FAST-LIO2 uses reliable subscriptions for Livox CustomMsg and IMU.
LIVOX_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
)


def set_ros_stamp(stamp, timestamp_ns):
    stamp.sec = int(timestamp_ns // NSEC_PER_SEC)
    stamp.nanosec = int(timestamp_ns % NSEC_PER_SEC)


class T100LivoxCompatNode(Node):
    def __init__(self):
        super().__init__("t100_livox_compat")

        self.declare_parameter("device_ip", "")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("point_port", 60001)
        self.declare_parameter("imu_port", 60003)
        self.declare_parameter("publish_freq", 10.0)
        self.declare_parameter("xfer_format", 1)
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("imu_frame_id", "livox_frame")
        self.declare_parameter("topic_prefix", "livox")
        self.declare_parameter("line_count", 4)
        self.declare_parameter("lidar_id", 0)
        self.declare_parameter("max_pending_points", 300000)
        self.declare_parameter("auto_start", False)

        self.device_ip = str(self.get_parameter("device_ip").value).strip()
        self.bind_ip = str(self.get_parameter("bind_ip").value).strip()
        self.point_port = int(self.get_parameter("point_port").value)
        self.imu_port = int(self.get_parameter("imu_port").value)
        self.publish_freq = float(self.get_parameter("publish_freq").value)
        self.xfer_format = int(self.get_parameter("xfer_format").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.topic_prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self.line_count = int(self.get_parameter("line_count").value)
        self.lidar_id = int(self.get_parameter("lidar_id").value)
        self.max_pending_points = int(self.get_parameter("max_pending_points").value)

        if not self.device_ip:
            raise ValueError("set the device_ip launch parameter to the T100 IPv4 address")
        if not self.bind_ip:
            raise ValueError("bind_ip must not be empty")
        if not (1 <= self.point_port <= 65535 and 1 <= self.imu_port <= 65535):
            raise ValueError("UDP ports must be in the range 1..65535")
        if self.point_port == self.imu_port:
            raise ValueError("point_port and imu_port must be different")
        if self.publish_freq <= 0:
            raise ValueError("publish_freq must be positive")
        if self.xfer_format not in (0, 1):
            raise ValueError("xfer_format must be 0 (PointCloud2) or 1 (Livox CustomMsg)")
        if not 1 <= self.line_count <= 255:
            raise ValueError("line_count must be in the range 1..255")
        if not 0 <= self.lidar_id <= 255:
            raise ValueError("lidar_id must fit uint8")
        if self.max_pending_points < 96:
            raise ValueError("max_pending_points must be at least one 96-point packet")

        prefix = self.topic_prefix
        lidar_topic = f"{prefix}/lidar" if prefix else "lidar"
        imu_topic = f"{prefix}/imu" if prefix else "imu"
        lidar_message_type = CustomMsg if self.xfer_format == 1 else PointCloud2
        self.lidar_pub = self.create_publisher(lidar_message_type, lidar_topic, LIVOX_QOS)
        self.imu_pub = self.create_publisher(Imu, imu_topic, LIVOX_QOS)

        self._lock = threading.Lock()
        self._pending_points = []
        self._stop_event = threading.Event()
        self._sockets = []
        self._counters = {
            "point_packets": 0,
            "imu_packets": 0,
            "published_clouds": 0,
            "published_imus": 0,
            "bad_packets": 0,
            "wrong_source": 0,
            "time_type_mismatch": 0,
            "dropped_points": 0,
            "bytes": 0,
        }
        self._last_stats = time.monotonic()
        self._time_type = None
        self._device_to_host_offset_ns = None
        self._time_type_mismatch_logged = False

        self._open_socket(self.point_port)
        self._open_socket(self.imu_port)

        if bool(self.get_parameter("auto_start").value):
            try:
                self._start_device_sampling()
            except Exception:
                for sock in self._sockets:
                    sock.close()
                raise

        self._receiver_thread = threading.Thread(
            target=self._receive_loop, name="t100-udp-receiver", daemon=True
        )
        self._receiver_thread.start()

        self._cloud_timer = self.create_timer(1.0 / self.publish_freq, self._publish_cloud)
        self._stats_timer = self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"T100 Livox-compatible bridge listening on {self.bind_ip}:"
            f"{self.point_port} (points), {self.bind_ip}:{self.imu_port} (IMU); "
            f"source={self.device_ip}; xfer_format={self.xfer_format}; topics={lidar_topic}, {imu_topic}"
        )

    def _open_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        sock.bind((self.bind_ip, port))
        sock.setblocking(False)
        self._sockets.append(sock)

    def _start_device_sampling(self):
        # Explicit opt-in: the default is passive receive-only operation.
        from t100_lidar.client import Client

        with Client(self.device_ip, port=60000, bind=self.bind_ip, timeout=1.0) as client:
            state = client.start()
        self.get_logger().info(f"T100 sampling start requested; target state readback={state}")

    def _receive_loop(self):
        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select(self._sockets, [], [], 0.2)
            except (OSError, ValueError):
                if self._stop_event.is_set():
                    break
                continue

            for sock in readable:
                try:
                    data, source = sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    continue

                if source[0] != self.device_ip:
                    with self._lock:
                        self._counters["wrong_source"] += 1
                    continue

                with self._lock:
                    self._counters["bytes"] += len(data)

                try:
                    packet = decode_data(data)
                    if not self._accept_time_type(packet.time_type):
                        continue
                    packet = self._map_packet_timestamp(packet)
                    if packet.kind == "points":
                        points = packet_to_livox_points(packet, self.line_count)
                        with self._lock:
                            self._counters["point_packets"] += 1
                            overflow = max(
                                0,
                                len(self._pending_points) + len(points) - self.max_pending_points,
                            )
                            if overflow:
                                del self._pending_points[:overflow]
                                self._counters["dropped_points"] += overflow
                            self._pending_points.extend(points)
                    elif packet.kind == "imu":
                        self._publish_imu(packet)
                except (ProtocolError, ValueError, struct.error):
                    with self._lock:
                        self._counters["bad_packets"] += 1

    def _accept_time_type(self, packet_time_type):
        should_log = False
        with self._lock:
            if self._time_type is None:
                self._time_type = packet_time_type
                return True
            if packet_time_type == self._time_type:
                return True
            self._counters["time_type_mismatch"] += 1
            if not self._time_type_mismatch_logged:
                self._time_type_mismatch_logged = True
                should_log = True
            expected = self._time_type
        if should_log:
            self.get_logger().error(
                f"Rejecting packets with time_type={packet_time_type}; "
                f"the first stream established time_type={expected}. "
                "Point-cloud and IMU timestamps must share one time domain; "
                "check device synchronization settings and restart the bridge."
            )
        return False

    def _map_packet_timestamp(self, packet):
        """Map unsynchronized device-uptime stamps to one host epoch for both streams."""
        if packet.time_type != 0:
            return packet
        if self._device_to_host_offset_ns is None:
            self._device_to_host_offset_ns = time.time_ns() - packet.timestamp_ns
        return replace(
            packet,
            timestamp_ns=packet.timestamp_ns + self._device_to_host_offset_ns,
        )

    def _publish_imu(self, packet):
        msg = Imu()
        msg.header.frame_id = self.imu_frame_id
        set_ros_stamp(msg.header.stamp, packet.timestamp_ns)

        msg.angular_velocity.x = float(packet.gyro[0])
        msg.angular_velocity.y = float(packet.gyro[1])
        msg.angular_velocity.z = float(packet.gyro[2])

        acceleration = acceleration_g_to_mps2(packet.acceleration)
        msg.linear_acceleration.x = acceleration[0]
        msg.linear_acceleration.y = acceleration[1]
        msg.linear_acceleration.z = acceleration[2]

        # T100 supplies angular velocity and acceleration, not orientation.
        msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)
        with self._lock:
            self._counters["imu_packets"] += 1
            self._counters["published_imus"] += 1

    def _publish_cloud(self):
        with self._lock:
            points = self._pending_points
            self._pending_points = []
        if not points:
            return

        for timebase_ns, batch in split_livox_batches(points):
            if self.xfer_format == 1:
                msg = CustomMsg()
                msg.header.frame_id = self.frame_id
                set_ros_stamp(msg.header.stamp, timebase_ns)
                msg.timebase = int(timebase_ns)
                msg.point_num = len(batch)
                msg.lidar_id = self.lidar_id

                for offset_ns, point in batch:
                    custom_point = CustomPoint()
                    custom_point.offset_time = int(offset_ns)
                    custom_point.x = float(point.x)
                    custom_point.y = float(point.y)
                    custom_point.z = float(point.z)
                    custom_point.reflectivity = int(point.reflectivity)
                    custom_point.tag = int(point.tag)
                    custom_point.line = int(point.line)
                    msg.points.append(custom_point)
            else:
                msg = PointCloud2()
                msg.header.frame_id = self.frame_id
                set_ros_stamp(msg.header.stamp, timebase_ns)
                msg.height = 1
                msg.width = len(batch)
                msg.fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
                    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
                    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
                ]
                msg.is_bigendian = False
                msg.point_step = 26
                msg.row_step = msg.point_step * msg.width
                msg.is_dense = False
                msg.data = pack_pointcloud2_data(batch)

            self.lidar_pub.publish(msg)
            with self._lock:
                self._counters["published_clouds"] += 1

    def _log_stats(self):
        now = time.monotonic()
        with self._lock:
            snapshot = dict(self._counters)
            pending = len(self._pending_points)
            for key in self._counters:
                self._counters[key] = 0
        elapsed = max(now - self._last_stats, 1e-6)
        self._last_stats = now
        point_rate = snapshot["point_packets"] * 96.0 / elapsed
        self.get_logger().info(
            "T100 bridge: "
            f"point_packets={snapshot['point_packets']}, "
            f"approx_rx_points_s={point_rate:.0f}, "
            f"imu_packets={snapshot['imu_packets']}, "
            f"clouds={snapshot['published_clouds']}, "
            f"bad={snapshot['bad_packets']}, "
            f"wrong_source={snapshot['wrong_source']}, "
            f"time_type_mismatch={snapshot['time_type_mismatch']}, "
            f"dropped_points={snapshot['dropped_points']}, "
            f"pending_points={pending}"
        )

    def destroy_node(self):
        self._stop_event.set()
        for sock in self._sockets:
            try:
                sock.close()
            except OSError:
                pass
        if hasattr(self, "_receiver_thread"):
            self._receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = T100LivoxCompatNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
