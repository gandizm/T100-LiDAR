"""Conversion helpers for the Livox ROS Driver 2 CustomMsg contract.

The point timing formula intentionally matches Livox ROS Driver 2's
PubHandler::OnLivoxLidarPointCloudCallback implementation.
"""

import struct
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from .protocol import DataPacket


UINT32_MAX = 0xFFFFFFFF
STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class LivoxCompatPoint:
    timestamp_ns: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


def packet_to_livox_points(packet: DataPacket, line_count: int = 4) -> List[LivoxCompatPoint]:
    """Convert one T100 Cartesian-high packet to timestamped Livox points.

    T100 time_interval is expressed in 0.1 microsecond units. Livox SDK2
    computes the point interval as time_interval * 100 / dot_num (ns).
    """
    if packet.kind != "points":
        raise ValueError("expected a point-cloud packet")
    if not packet.points:
        return []
    if not 1 <= int(line_count) <= 255:
        raise ValueError("line_count must be in the range 1..255")

    point_interval_ns = packet.time_interval_0_1us * 100 // len(packet.points)
    if point_interval_ns <= 0:
        raise ValueError("point packet time_interval is too small to recover per-point timing")
    return [
        LivoxCompatPoint(
            timestamp_ns=packet.timestamp_ns + index * point_interval_ns,
            x=point.x / 1000.0,
            y=point.y / 1000.0,
            z=point.z / 1000.0,
            reflectivity=point.reflectivity,
            tag=point.tag,
            line=index % line_count,
        )
        for index, point in enumerate(packet.points)
    ]


def split_livox_batches(
    points: Iterable[LivoxCompatPoint],
    max_offset_ns: int = UINT32_MAX,
) -> List[Tuple[int, Tuple[Tuple[int, LivoxCompatPoint], ...]]]:
    """Sort points and split them into CustomMsg batches with uint32 offsets.

    Each returned tuple contains (timebase_ns, ((offset_ns, point), ...)).
    """
    ordered = sorted(points, key=lambda point: point.timestamp_ns)
    if not ordered:
        return []
    if not 0 <= max_offset_ns <= UINT32_MAX:
        raise ValueError("max_offset_ns must fit uint32")

    batches = []
    base_time = ordered[0].timestamp_ns
    current = []
    for point in ordered:
        offset = point.timestamp_ns - base_time
        if offset < 0:
            raise ValueError("point timestamps must be sortable")
        if current and offset > max_offset_ns:
            batches.append((base_time, tuple(current)))
            base_time = point.timestamp_ns
            current = []
            offset = 0
        current.append((offset, point))
    if current:
        batches.append((base_time, tuple(current)))
    return batches


def acceleration_g_to_mps2(values: Sequence[float]) -> Tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError("acceleration must contain exactly three axes")
    return tuple(float(value) * STANDARD_GRAVITY for value in values)


POINTCLOUD2_POINT_STEP = 26


def pack_pointcloud2_data(batch) -> bytes:
    """Pack the official Livox ROS Driver 2 PointXYZRTLT layout.

    Fields: x/y/z/intensity float32, tag/line uint8, timestamp float64.
    The timestamp field carries the absolute per-point timestamp in ns,
    matching the official driver implementation.
    """
    data = bytearray(len(batch) * POINTCLOUD2_POINT_STEP)
    for index, (_offset_ns, point) in enumerate(batch):
        struct.pack_into(
            "<ffffBBd",
            data,
            index * POINTCLOUD2_POINT_STEP,
            float(point.x),
            float(point.y),
            float(point.z),
            float(point.reflectivity),
            int(point.tag),
            int(point.line),
            float(point.timestamp_ns),
        )
    return bytes(data)
