import struct
import unittest

from t100_lidar.protocol import DataPacket, Point
from t100_lidar.ros2_compat import (
    acceleration_g_to_mps2,
    pack_pointcloud2_data,
    packet_to_livox_points,
    split_livox_batches,
)


def packet(points, interval=9600, timestamp=1_000_000_000):
    return DataPacket(
        kind="points",
        sequence=7,
        timestamp_ns=timestamp,
        time_type=0,
        points=tuple(points),
        time_interval_0_1us=interval,
    )


class LivoxCompatibilityTests(unittest.TestCase):
    def test_time_interval_matches_livox_sdk2_formula(self):
        points = [Point(1000 + i, 2000, 3000, 80, 0x10) for i in range(96)]
        converted = packet_to_livox_points(packet(points))
        # Livox ROS Driver 2 PubHandler: time_interval * 100 / dot_num ns.
        self.assertEqual(converted[0].timestamp_ns, 1_000_000_000)
        self.assertEqual(converted[1].timestamp_ns - converted[0].timestamp_ns, 10_000)
        self.assertEqual(converted[-1].timestamp_ns - converted[0].timestamp_ns, 950_000)

    def test_mid360_compat_line_assignment_is_configurable(self):
        points = [Point(1000 + i, 0, 0, 10, 0) for i in range(8)]
        self.assertEqual(
            [p.line for p in packet_to_livox_points(packet(points), line_count=4)],
            [0, 1, 2, 3, 0, 1, 2, 3],
        )
        self.assertEqual(
            [p.line for p in packet_to_livox_points(packet(points), line_count=1)],
            [0] * 8,
        )

    def test_invalid_and_no_return_points_are_filtered_without_retiming(self):
        points = [
            Point(1000, 0, 0, 0, 0),
            Point(0, 0, 0, 255, 0),
            Point(200_000, 0, 0, 255, 0),
            Point(3000, 0, 0, 20, 0),
        ]
        converted = packet_to_livox_points(packet(points, interval=400), max_range_m=150)
        self.assertEqual(len(converted), 2)
        self.assertEqual(converted[0].timestamp_ns, 1_000_000_000)
        # Index 3 keeps its original packet timing after indices 1 and 2 are filtered.
        self.assertEqual(converted[1].timestamp_ns, 1_000_000_000 + 3 * 10_000)

    def test_pointcloud2_packed_layout_is_26_bytes(self):
        points = [Point(1000, -2000, 3000, 42, 0x11)]
        converted = packet_to_livox_points(packet(points))
        batch = [(0, converted[0])]
        data = pack_pointcloud2_data(batch)
        self.assertEqual(len(data), 26)
        x, y, z, intensity, tag, line, timestamp = struct.unpack("<ffffBBd", data)
        self.assertEqual((x, y, z, intensity, tag, line),
                         (1.0, -2.0, 3.0, 42.0, 0x11, 0))
        self.assertEqual(timestamp, 1_000_000_000.0)

    def test_custom_msg_batches_use_uint32_relative_offsets(self):
        points = [
            Point(1000, 0, 0, 10, 0),
            Point(2000, 0, 0, 20, 0),
        ]
        converted = packet_to_livox_points(packet(points, interval=9600))
        batches = split_livox_batches(converted, max_offset_ns=5000)
        self.assertEqual(len(batches), 2)
        self.assertEqual(batches[0][0], converted[0].timestamp_ns)
        self.assertEqual(batches[0][1][0][0], 0)
        self.assertEqual(batches[1][1][0][0], 0)

    def test_acceleration_g_to_mps2(self):
        ax, ay, az = acceleration_g_to_mps2((0.0, 0.0, 1.0))
        self.assertAlmostEqual(ax, 0.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(az, 9.80665)


if __name__ == "__main__":
    unittest.main()
