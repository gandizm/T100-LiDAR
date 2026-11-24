import struct
import unittest

from t100_lidar.protocol import DataPacket, Point
from t100_lidar.ros2_compat import (
    acceleration_g_to_mps2,
    pack_pointcloud2_data,
    packet_to_livox_points,
    split_livox_batches,
)


class LivoxCompatibilityTests(unittest.TestCase):
    def make_packet(self, time_interval=4000):
        points = tuple(
            Point(x=1000 + i, y=2000 + i, z=3000 + i, reflectivity=80 + i, tag=0x10)
            for i in range(4)
        )
        return DataPacket(
            kind="points",
            sequence=7,
            timestamp_ns=1_000_000,
            time_type=0,
            points=points,
            time_interval_0_1us=time_interval,
        )

    def test_point_interval_matches_livox_mid360_formula(self):
        points = packet_to_livox_points(self.make_packet(), line_count=4)
        self.assertEqual([p.timestamp_ns for p in points], [
            1_000_000, 1_100_000, 1_200_000, 1_300_000
        ])
        self.assertEqual([p.line for p in points], [0, 1, 2, 3])
        self.assertAlmostEqual(points[0].x, 1.0)
        self.assertEqual(points[0].reflectivity, 80)
        self.assertEqual(points[0].tag, 0x10)

    def test_custom_batches_use_uint32_relative_offsets(self):
        points = packet_to_livox_points(self.make_packet())
        batches = split_livox_batches(points, max_offset_ns=150_000)
        self.assertEqual(len(batches), 2)
        self.assertEqual(batches[0][0], 1_000_000)
        self.assertEqual([entry[0] for entry in batches[0][1]], [0, 100_000])
        self.assertEqual(batches[1][0], 1_200_000)
        self.assertEqual([entry[0] for entry in batches[1][1]], [0, 100_000])

    def test_pointcloud2_matches_packed_26_byte_livox_layout(self):
        points = packet_to_livox_points(self.make_packet())
        batch = split_livox_batches(points)[0][1]
        packed = pack_pointcloud2_data(batch)
        self.assertEqual(len(packed), 4 * 26)
        x, y, z, intensity, tag, line, timestamp = struct.unpack_from("<ffffBBd", packed, 0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(z, 3.0)
        self.assertEqual(intensity, 80.0)
        self.assertEqual(tag, 0x10)
        self.assertEqual(line, 0)
        self.assertEqual(timestamp, 1_000_000.0)

    def test_zero_point_interval_is_not_silently_invented(self):
        with self.assertRaises(ValueError):
            packet_to_livox_points(self.make_packet(time_interval=0))

    def test_acceleration_g_to_si(self):
        values = acceleration_g_to_mps2((0.0, 0.5, -1.0))
        self.assertEqual(values[0], 0.0)
        self.assertAlmostEqual(values[1], 4.903325)
        self.assertAlmostEqual(values[2], -9.80665)


if __name__ == "__main__":
    unittest.main()
