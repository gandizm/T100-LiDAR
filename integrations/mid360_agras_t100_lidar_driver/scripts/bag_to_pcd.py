#!/usr/bin/env python3
"""Export a map and a trajectory from a FAST-LIO walk bag (recorded by fastlio_walk.sh).

  /cloud_registered (world frame, one message per scan) -> binary PCD x y z intensity t, one point per voxel
                                                           (t: seconds since the first scan when the voxel was
                                                           first seen; the header comment "# t0 = ..." is t = 0)
  /Odometry                                            -> CSV: t, x, y, z, qx, qy, qz, qw

Usage: bag_to_pcd.py BAG_DIR OUT.pcd [--voxel 0.1] [--traj trajectory.csv] [--start S] [--end S]
       [--cloud-topic /cloud_registered] [--odom-topic /Odometry]
--start/--end: keep only scans in that window (seconds since the first scan), e.g. cut off a diverged tail.
If the walk ended with a power loss the bag has no metadata.yaml: run `ros2 bag reindex BAG_DIR` first.
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PF = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}  # PointField datatype
BITS = 21  # per axis in the voxel key: +-2^20 voxels (+-104 km at 0.1 m)


def xyzi(msg):
    f = {p.name: p for p in msg.fields}
    names = [n for n in ("x", "y", "z", "intensity") if n in f]
    dt = np.dtype({"names": names, "formats": [PF[f[n].datatype] for n in names],
                   "offsets": [f[n].offset for n in names], "itemsize": msg.point_step})
    try:
        buf = memoryview(msg.data)
    except TypeError:  # some rclpy versions give a list
        buf = bytes(msg.data)
    a = np.frombuffer(buf, dt, msg.width * msg.height)
    out = np.zeros((len(a), 4), np.float32)
    for i, n in enumerate(names):
        out[:, i] = a[n]
    return out[np.isfinite(out[:, :3]).all(1)]


def voxel_unique(pts, voxel):
    q = np.clip(np.floor(pts[:, :3] / voxel).astype(np.int64) + (1 << (BITS - 1)), 0, (1 << BITS) - 1)
    key = (q[:, 0] << (2 * BITS)) | (q[:, 1] << BITS) | q[:, 2]
    _, first = np.unique(key, return_index=True)
    return pts[np.sort(first)]


def write_pcd(path, pts, t0):
    n = len(pts)
    hdr = (f"# .PCD v0.7 - Point Cloud Data file format\n# t0 = {t0:.6f}\nVERSION 0.7\nFIELDS x y z intensity t\n"
           f"SIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(np.ascontiguousarray(pts, np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser(description="FAST-LIO walk bag -> map.pcd + trajectory.csv")
    ap.add_argument("bag")
    ap.add_argument("pcd")
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--traj")
    ap.add_argument("--start", type=float, help="seconds since the first scan")
    ap.add_argument("--end", type=float, help="seconds since the first scan")
    ap.add_argument("--cloud-topic", default="/cloud_registered")
    ap.add_argument("--odom-topic", default="/Odometry")
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if a.cloud_topic not in types:
        sys.exit(f"no {a.cloud_topic} in {a.bag} (topics: {sorted(types)})")
    want = [t for t in (a.cloud_topic, a.odom_topic) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}

    chunks, pending, compact_at, scans, raw, traj, t0 = [], 0, 5_000_000, 0, 0, [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t0 = st if t0 is None else t0
        rel = st - t0
        if a.end is not None and rel > a.end + 1:
            break
        if (a.start is not None and rel < a.start) or (a.end is not None and rel > a.end):
            continue
        if topic == a.odom_topic:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            traj.append((st, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            continue
        pts = xyzi(msg)
        pts = np.column_stack([pts, np.full(len(pts), rel, np.float32)])
        scans += 1
        raw += len(pts)
        chunks.append(pts)
        pending += len(pts)
        if pending > compact_at:  # keep memory bounded: merge into one voxel-filtered array
            chunks = [voxel_unique(np.concatenate(chunks), a.voxel)]
            pending = len(chunks[0])
            compact_at = max(5_000_000, 2 * pending)

    pts = voxel_unique(np.concatenate(chunks), a.voxel) if chunks else np.zeros((0, 5), np.float32)
    write_pcd(a.pcd, pts, t0 or 0.0)
    print(f"scans {scans}, points {raw} -> map {len(pts)} points ({a.voxel} m voxel) -> {a.pcd}")
    if a.traj and traj:
        T = np.array(traj)
        np.savetxt(a.traj, T, fmt="%.6f", delimiter=",", header="t,x,y,z,qx,qy,qz,qw", comments="")
        length = np.linalg.norm(np.diff(T[:, 1:4], axis=0), axis=1).sum()
        print(f"trajectory {len(T)} poses, {T[-1, 0] - T[0, 0]:.0f} s, path length {length:.1f} m -> {a.traj}")


if __name__ == "__main__":
    main()
