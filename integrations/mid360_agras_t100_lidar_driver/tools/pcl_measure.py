"""Passive point-cloud census on UDP 60001. Receives only; sends nothing.
Reports packet count, valid-point rate, elevation span, and the azimuth gap
(5-degree bins with zero points) to confirm the ~65-deg blackout."""
import socket, struct, time, math, collections, json
from pathlib import Path

SECS = 6
pkts = 0
raw_points = 0
valid_points = 0
data_types = collections.Counter()
az_hist = collections.Counter()
el_min = 99.0
el_max = -99.0
first_ts = last_ts = None

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    try:
        s.bind(("0.0.0.0", 60001))
    except OSError as e:
        print("BIND_FAIL", e, "- another program likely holds UDP 60001")
        raise SystemExit(1)
    end = time.monotonic() + SECS
    while time.monotonic() < end:
        s.settimeout(max(.001, end - time.monotonic()))
        try:
            data, peer = s.recvfrom(65535)
        except socket.timeout:
            break
        if peer[0] != "192.168.1.10" or len(data) < 36:
            continue
        ver, length, tint, dot_num, udp_cnt, frame_cnt, dtype, ttype = struct.unpack_from("<BHHHHBBB", data, 0)
        ts = struct.unpack_from("<Q", data, 28)[0]
        pkts += 1
        data_types[dtype] += 1
        if first_ts is None:
            first_ts = ts
        last_ts = ts
        body = data[36:]
        if dtype == 1:
            psize = 14
            n = len(body) // psize
            raw_points += n
            for i in range(n):
                x, y, z = struct.unpack_from("<iii", body, i * psize)
                if x == 0 and y == 0 and z == 0:
                    continue
                valid_points += 1
                az = math.degrees(math.atan2(y, x))
                az_hist[int(az // 5) * 5] += 1
                r_h = math.hypot(x, y)
                if r_h > 0:
                    el = math.degrees(math.atan2(z, r_h))
                    el_min = min(el_min, el)
                    el_max = max(el_max, el)

dur = (last_ts - first_ts) / 1e9 if first_ts and last_ts and last_ts > first_ts else SECS
rate = valid_points / dur if dur > 0 else 0
covered = sorted(az_hist)
gap = [b for b in range(-180, 180, 5) if az_hist.get(b, 0) == 0]
print(f"packets={pkts}  raw_points={raw_points}  valid_points={valid_points}  dur={dur:.2f}s")
print(f"valid point rate = {rate:,.0f} points/s")
print(f"data_types = {dict(data_types)}  (1=int32mm 2=int16cm 3=spherical)")
print(f"elevation span = {el_min:.1f} .. {el_max:.1f} deg")
print(f"azimuth bins with points: {covered[0] if covered else '-'} .. {covered[-1] if covered else '-'}")
print(f"azimuth GAP (5-deg bins, ZERO points): {gap}")
print(f"  gap width ~= {len(gap)*5} deg")
