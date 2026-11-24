"""MID360-agras 点云查看器（Windows 演示程序）

数据源:
  实时  - 监听 UDP 60001（雷达上电即推流，无需发命令；本机网口需 192.168.1.20/24）
  回放  - .pcapng / .pcap 录包（tshark/tcpdump 抓的），按原始时间循环播放
启动时先等实时数据；3 s 内没收到就自动回放 ../captures 里最新的录包，
之后一旦收到实时数据会自动切回实时。也可以把录包拖到窗口或 .bat 上。
"""
import os
import sys
import glob
import time
import socket
import struct
import threading
import traceback
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.normpath(os.path.join(HERE, 'captures'))
POINT_PORT = 60001
AUTO_REPLAY_AFTER = 3.0  # s without live data before falling back to a recording


def fatal(msg):
    try:
        with open(os.path.join(HERE, 'viewer_error.log'), 'w', encoding='utf-8') as f:
            f.write(msg)
    except OSError:
        pass
    if os.name == 'nt':
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, msg[-2000:], 'MID360 点云查看器', 0x10)
    else:
        print(msg, file=sys.stderr)
    sys.exit(1)


try:
    import numpy as np
    from PyQt5 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
except ImportError as e:
    fatal(f'缺少依赖: {e}\n\n请运行:\n  python -m pip install numpy PyQt5 pyqtgraph PyOpenGL')

# 复用已验证的配置工具（同仓库 tools/mid360_fov.py），避免协议重复实现
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'tools')))
try:
    import mid360_fov
except ImportError:
    mid360_fov = None


# ---------------------------------------------------------------- Livox packets
# SDK2 point packet header, 36 B:
# version u8, length u16, time_interval u16, dot_num u16, udp_cnt u16,
# frame_cnt u8, data_type u8, time_type u8, rsvd[12], crc32 u32, timestamp u64
HDR_LEN = 36
PT_TYPES = {
    1: (np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'), ('r', 'u1'), ('tag', 'u1')]), 0.001),  # mm
    2: (np.dtype([('x', '<i2'), ('y', '<i2'), ('z', '<i2'), ('r', 'u1'), ('tag', 'u1')]), 0.01),   # cm
}


def parse_points(payload):
    """-> (frame_cnt, xyz float32 (n,3) m, refl uint8 (n,)) or None. Drops no-return points."""
    if len(payload) < HDR_LEN:
        return None
    dot_num, = struct.unpack_from('<H', payload, 5)
    frame_cnt, data_type = payload[9], payload[10]
    t = PT_TYPES.get(data_type)
    if t is None:
        return None
    dt, scale = t
    n = min(dot_num, (len(payload) - HDR_LEN) // dt.itemsize)
    a = np.frombuffer(payload, dt, n, HDR_LEN)
    xyz = np.empty((n, 3), np.float32)
    xyz[:, 0] = a['x']
    xyz[:, 1] = a['y']
    xyz[:, 2] = a['z']
    xyz *= scale
    d2 = np.einsum('ij,ij->i', xyz, xyz)
    keep = (d2 > 0.01) & (d2 < 199.0 ** 2)  # 0 = no data, 200 m = no return (Agras firmware)
    return frame_cnt, xyz[keep], a['r'][keep].copy()


# ---------------------------------------------------------------- pcap / pcapng
def _iter_pcapng(f):
    resol = {}
    iface = 0
    while True:
        head = f.read(8)
        if len(head) < 8:
            return
        bt, bl = struct.unpack('<II', head)
        body = f.read(bl - 8)
        if bt == 0x0A0D0D0A:
            if body[:4] != b'\x4d\x3c\x2b\x1a':
                raise ValueError('不支持大端 pcapng')
            resol.clear()
            iface = 0
        elif bt == 1:  # IDB
            link, = struct.unpack_from('<H', body, 0)
            res = 1e-6
            off = 8
            while off + 4 <= len(body) - 4:
                code, ln = struct.unpack_from('<HH', body, off)
                if code == 0:
                    break
                if code == 9 and ln >= 1:
                    v = body[off + 4]
                    res = 2.0 ** -(v & 0x7F) if v & 0x80 else 10.0 ** -v
                off += 4 + ((ln + 3) & ~3)
            resol[iface] = (link, res)
            iface += 1
        elif bt == 6:  # EPB
            iid, th, tl, cap = struct.unpack_from('<IIII', body, 0)
            link, res = resol.get(iid, (1, 1e-6))
            yield ((th << 32) | tl) * res, link, body[20:20 + cap]


def _iter_pcap(f, hdr):
    magic = hdr[:4]
    if magic == b'\xd4\xc3\xb2\xa1':
        frac = 1e-6
    elif magic == b'\x4d\x3c\xb2\xa1':
        frac = 1e-9
    else:
        raise ValueError('不是 pcap/pcapng 文件')
    link, = struct.unpack_from('<I', hdr, 20)
    while True:
        rh = f.read(16)
        if len(rh) < 16:
            return
        sec, sub, incl, _ = struct.unpack('<IIII', rh)
        yield sec + sub * frac, link, f.read(incl)


def iter_udp(path):
    """Yield (timestamp_s, dst_port, payload) for every IPv4 UDP packet in a capture."""
    with open(path, 'rb') as f:
        hdr = f.read(24)
        if hdr[:4] == b'\x0a\x0d\x0d\x0a':
            f.seek(0)
            frames = _iter_pcapng(f)
        else:
            frames = _iter_pcap(f, hdr)
        for ts, link, fr in frames:
            if link == 1:  # Ethernet (+ optional VLAN tags)
                off, et = 14, fr[12:14]
                while et == b'\x81\x00' and len(fr) >= off + 4:
                    et, off = fr[off + 2:off + 4], off + 4
                if et != b'\x08\x00':
                    continue
            elif link == 113:  # Linux cooked (tcpdump -i any)
                if fr[14:16] != b'\x08\x00':
                    continue
                off = 16
            elif link == 276:  # Linux cooked v2
                if fr[0:2] != b'\x08\x00':
                    continue
                off = 20
            elif link in (12, 101):  # raw IP
                off = 0
            else:
                continue
            if len(fr) < off + 28 or fr[off] >> 4 != 4 or fr[off + 9] != 17:
                continue
            if struct.unpack_from('>H', fr, off + 6)[0] & 0x3FFF:  # fragment
                continue
            u = off + (fr[off] & 15) * 4
            dport, ulen = struct.unpack_from('>HH', fr, u + 2)
            yield ts, dport, fr[u + 8:u + ulen]


class Recording:
    """All point packets of a capture, concatenated, with per-packet time offsets."""

    def __init__(self, path):
        self.path = path
        times, xyzs, refls = [], [], []
        for ts, dport, payload in iter_udp(path):
            if dport != POINT_PORT:
                continue
            p = parse_points(payload)
            if p is None:
                continue
            times.append(ts)
            xyzs.append(p[1])
            refls.append(p[2])
        if not times:
            raise ValueError(f'录包里没有 UDP {POINT_PORT} 点云数据')
        t = np.asarray(times)
        order = np.argsort(t, kind='stable')
        self.t = t[order] - t[order[0]]
        counts = np.array([len(xyzs[i]) for i in order])
        self.off = np.concatenate([[0], np.cumsum(counts)])
        self.xyz = np.concatenate([xyzs[i] for i in order])
        self.refl = np.concatenate([refls[i] for i in order])
        self.duration = float(self.t[-1])
        self.pts_per_s = len(self.refl) / max(self.duration, 1e-3)

    def window(self, now, win):
        i0 = np.searchsorted(self.t, now - win)
        i1 = np.searchsorted(self.t, now, 'right')
        a, b = self.off[i0], self.off[i1]
        return self.xyz[a:b], self.refl[a:b]


# ---------------------------------------------------------------- live UDP
class LiveSource:
    KEEP = 1.5  # s of packets kept (> max integration time)

    def __init__(self, port=POINT_PORT):
        self.error = None
        self.buf = deque()
        self.lock = threading.Lock()
        self.last_rx = None
        self.pkts = self.pts = 0
        self.running = True
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
            self.sock.bind(('0.0.0.0', port))
            self.sock.settimeout(0.3)
        except OSError as e:
            self.error = f'无法监听 UDP {port}: {e}（是否有 Livox Viewer 等程序占用？）'
            self.running = False
            return
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            try:
                data = self.sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                if self.running:
                    continue
                return
            p = parse_points(data)
            if p is None:
                continue
            now = time.monotonic()
            _, xyz, refl = p
            with self.lock:
                self.buf.append((now, xyz, refl))
                while self.buf and self.buf[0][0] < now - self.KEEP:
                    self.buf.popleft()
                self.last_rx = now
                self.pkts += 1
                self.pts += len(refl)

    def take_counters(self):
        with self.lock:
            c = (self.pkts, self.pts)
            self.pkts = self.pts = 0
        return c

    def window(self, win):
        now = time.monotonic()
        with self.lock:
            items = [it for it in reversed(self.buf) if it[0] >= now - win] if self.buf else []
        if not items:
            return np.zeros((0, 3), np.float32), np.zeros(0, np.uint8)
        return np.concatenate([it[1] for it in items]), np.concatenate([it[2] for it in items])

    def close(self):
        self.running = False
        if self.error is None:
            self.sock.close()


# ---------------------------------------------------------------- GUI
LUT = (pg.colormap.get('turbo').getLookupTable(0.0, 1.0, 256, alpha=True) / 255.0).astype(np.float32)
COLOR_MODES = ['反射率', '高度', '距离', '单色']
MAX_DRAW = 300_000


class Loader(QtCore.QObject):
    done = QtCore.pyqtSignal(object, str)

    def load(self, path):
        def run():
            try:
                self.done.emit(Recording(path), '')
            except Exception as e:  # noqa: BLE001 - shown to the user
                self.done.emit(None, f'{os.path.basename(path)}: {e}')
        threading.Thread(target=run, daemon=True).start()


class UnlockWorker(QtCore.QThread):
    """后台跑「解锁性能」：全周 FOV + set_state 1，再把点云流写回本机。

    不在 UI 线程发 UDP，避免网络超时卡住界面。结果通过信号回主线程。
    only_if_needed=True 时先读目标状态（32/51），已经是 1（采样）就什么都不做，
    done 的消息为空串。雷达断电/重启后目标状态回到 2，这时才重新解锁。
    （不用 roi_en 判断：重启后它读回来还是 0，但实际扫描已回到有缺口的开机状态。）
    """
    done = QtCore.pyqtSignal(bool, str)

    def __init__(self, device='192.168.1.10', host='192.168.1.20', only_if_needed=False):
        super().__init__()
        self.device = device
        self.host = host
        self.only_if_needed = only_if_needed

    def run(self):
        if mid360_fov is None:
            self.done.emit(False, '找不到 tools/mid360_fov.py，无法发送配置命令')
            return
        try:
            lid = mid360_fov.Lidar(self.device, timeout=0.8)
        except OSError as e:
            self.done.emit(False, f'打不开控制 socket: {e}')
            return
        try:
            if lid.read(mid360_fov.UID['dev_type'], 1) is None:
                self.done.emit(False,
                    f'连不上雷达 {self.device}:60000。\n确认本机是 {self.host}/24、雷达已上电。')
                return
            if self.only_if_needed and lid.read_state() == 1:
                self.done.emit(True, '')
                return
            ok, detail = mid360_fov.unlock_performance(lid)
            mid360_fov.restore_stream(lid, self.host, 60001, 60003)
            if ok:
                self.done.emit(True,
                    '已解锁 360° + 电机转速。\n断电重启后会还原，重按即可。')
            else:
                self.done.emit(False,
                    f'部分失败：roi_en status={detail["roi_en"]}, '
                    f'set_state status={detail["set_state"]}（0 才是成功）')
        except OSError as e:
            self.done.emit(False, f'通信出错: {e}')
        finally:
            lid.close()


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, open_path=None):
        super().__init__()
        self.setWindowTitle('MID360-agras 点云查看器')
        self.resize(1400, 860)
        self.setAcceptDrops(True)

        self.live = LiveSource()
        self.rec = None
        self.mode = 'wait'          # wait | live | replay | loading
        self.want_auto = open_path is None   # fall back to the newest capture if no live data
        self.auto_replay = False             # current replay was started automatically
        self.replay_t0 = 0.0
        self.paused = False
        self.started = time.monotonic()
        self.loader = Loader()
        self.loader.done.connect(self.on_loaded)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor((12, 14, 18))
        grid = gl.GLGridItem()
        grid.setSize(40, 40)
        grid.setSpacing(1, 1)
        grid.setColor((255, 255, 255, 40))
        self.view.addItem(grid)
        axis = gl.GLAxisItem()
        axis.setSize(1, 1, 1)
        self.view.addItem(axis)
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=2.0, pxMode=True)
        self.scatter.setGLOptions('opaque')
        self.view.addItem(self.scatter)
        self.reset_view()

        self.setCentralWidget(self.view)
        self.build_panel()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.stat_timer = QtCore.QTimer(self)
        self.stat_timer.timeout.connect(self.update_stats)
        self.stat_timer.start(1000)
        self.drawn = 0
        self.pause_at = 0.0
        self.frozen = False      # paused frame already drawn
        self.replay_text = ''

        # 自动解锁性能：启动时做一次，之后每 5 s 看一眼目标状态，
        # 雷达断电/重启回到待机（2）就自动再解锁，不弹窗，结果写在状态栏
        self._unlock_worker = None
        self.unlock_timer = QtCore.QTimer(self)
        self.unlock_timer.timeout.connect(self.auto_unlock)
        if mid360_fov is not None:
            self.statusBar().showMessage('自动解锁性能：连接雷达中…')
            QtCore.QTimer.singleShot(300, self.auto_unlock)
            self.unlock_timer.start(5000)

        if open_path:
            self.open_recording(open_path)
        elif self.live.error:
            QtWidgets.QMessageBox.warning(self, '实时数据不可用', self.live.error)

    # ---- controls
    def build_panel(self):
        dock = QtWidgets.QDockWidget('控制', self)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        w = QtWidgets.QWidget()
        lay = QtWidgets.QFormLayout(w)

        self.src_label = QtWidgets.QLabel()
        self.src_label.setWordWrap(True)
        lay.addRow('数据源', self.src_label)
        b_live = QtWidgets.QPushButton('切到实时 (UDP 60001)')
        b_live.clicked.connect(self.go_live)
        b_open = QtWidgets.QPushButton('打开录包…')
        b_open.clicked.connect(self.pick_recording)
        lay.addRow(b_live)
        lay.addRow(b_open)

        self.b_unlock = QtWidgets.QPushButton('解锁性能')
        self.b_unlock.setToolTip('打开 360° 全周点云 + 电机采样态（set_state 1）。\n'
                                 '启动时已自动解锁，雷达断电/重启后 5 s 内也会自动再解锁；\n'
                                 '这个按钮用于手动强制再来一次。\n'
                                 '需本机 192.168.1.20/24、雷达 192.168.1.10 已上电。')
        self.b_unlock.setStyleSheet(
            'QPushButton{background:#155e5a;color:#eafff9;font-weight:bold;padding:7px;'
            'border:1px solid #2a8a80;border-radius:5px}'
            'QPushButton:hover{background:#1c7a74}'
            'QPushButton:disabled{background:#2a3b3a;color:#7a9b96}')
        self.b_unlock.clicked.connect(self.unlock_performance)
        lay.addRow(self.b_unlock)
        self.pause_box = QtWidgets.QCheckBox('暂停（空格）')
        self.pause_box.toggled.connect(self.set_paused)
        lay.addRow(self.pause_box)

        self.win_spin = self.spin(lay, '积分时间', 0.05, 1.5, 0.3, ' s', 0.05)
        self.win_spin.setToolTip('把最近这段时间的点叠成一帧。MID-360 是非重复扫描，积分越长越密。')
        self.color_box = QtWidgets.QComboBox()
        self.color_box.addItems(COLOR_MODES)
        lay.addRow('着色', self.color_box)
        self.size_spin = self.spin(lay, '点大小', 1, 8, 2, ' px', 0.5)
        self.range_spin = self.spin(lay, '最远距离', 1, 70, 40, ' m')
        self.zmin_spin = self.spin(lay, '高度下限', -20, 20, -5, ' m')
        self.zmax_spin = self.spin(lay, '高度上限', -20, 50, 10, ' m')

        b_reset = QtWidgets.QPushButton('重置视角（R）')
        b_reset.clicked.connect(self.reset_view)
        b_top = QtWidgets.QPushButton('俯视（T）')
        b_top.clicked.connect(self.top_view)
        lay.addRow(b_reset)
        lay.addRow(b_top)

        self.info = QtWidgets.QLabel()
        self.info.setWordWrap(True)
        lay.addRow(self.info)
        help_ = QtWidgets.QLabel(
            '<small>左键拖动 旋转 · 中键/Ctrl+左键 平移 · 滚轮 缩放<br>'
            '可把 .pcapng 拖进窗口回放<br>'
            '实时: 本机网口设 192.168.1.20/24，雷达 192.168.1.10，'
            '防火墙放行 UDP 60001</small>')
        help_.setWordWrap(True)
        lay.addRow(help_)

        dock.setWidget(w)
        dock.setMinimumWidth(260)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

    def unlock_busy(self):
        return self._unlock_worker is not None and self._unlock_worker.isRunning()

    def unlock_performance(self):
        """手动按钮：无条件解锁，结果弹窗。"""
        if mid360_fov is None:
            QtWidgets.QMessageBox.warning(self, '解锁性能',
                '找不到 tools/mid360_fov.py，无法发送配置命令。')
            return
        self.b_unlock.setEnabled(False)
        self.b_unlock.setText('解锁中…')
        self.start_manual_unlock()

    def start_manual_unlock(self):
        if self.unlock_busy():      # 后台自动检查正在跑：等它结束再做，不丢掉这次点击
            self._unlock_worker.finished.connect(self.start_manual_unlock)
            return
        self._unlock_worker = UnlockWorker()
        self._unlock_worker.done.connect(self.on_unlock_done)
        self._unlock_worker.start()

    def auto_unlock(self):
        """定时器：雷达不在采样态才解锁，不弹窗。"""
        if self.unlock_busy():
            return
        self._unlock_worker = UnlockWorker(only_if_needed=True)
        self._unlock_worker.done.connect(self.on_auto_unlock_done)
        self._unlock_worker.start()

    def on_auto_unlock_done(self, ok, msg):
        if ok and not msg:
            return                               # 已经是解锁状态，什么都没做
        if ok:
            self.statusBar().showMessage(
                '已自动解锁 360° + 电机转速（%s）' % time.strftime('%H:%M:%S'))
            # 手动回放录包时不打断；等待中或自动回放时切到实时
            if self.mode == 'wait' or (self.mode == 'replay' and self.auto_replay):
                self.go_live()
        else:
            self.statusBar().showMessage('自动解锁性能：' + msg.replace('\n', ' ') + '（5 s 后重试）')

    def on_unlock_done(self, ok, msg):
        self.b_unlock.setEnabled(True)
        self.b_unlock.setText('解锁性能')
        if ok:
            self.statusBar().showMessage(
                '已解锁 360° + 电机转速（%s）' % time.strftime('%H:%M:%S'))
            # 配置后点云流被写回本机，确保在实时模式看效果
            if self.mode != 'live':
                self.go_live()
            QtWidgets.QMessageBox.information(self, '解锁性能', msg)
        else:
            QtWidgets.QMessageBox.warning(self, '解锁性能', msg)

    @staticmethod
    def spin(lay, label, lo, hi, val, suffix, step=1.0):
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setValue(val)
        s.setSuffix(suffix)
        lay.addRow(label, s)
        return s

    def reset_view(self):
        self.view.opts['center'] = pg.Vector(0, 0, 0.5)
        self.view.setCameraPosition(distance=9, elevation=30, azimuth=200)

    def top_view(self):
        self.view.opts['center'] = pg.Vector(0, 0, 0)
        self.view.setCameraPosition(distance=35, elevation=90, azimuth=180)

    def keyPressEvent(self, e):
        k = e.key()
        if k == QtCore.Qt.Key_R:
            self.reset_view()
        elif k == QtCore.Qt.Key_T:
            self.top_view()
        elif k == QtCore.Qt.Key_Space:
            self.pause_box.toggle()
        else:
            super().keyPressEvent(e)

    # ---- sources
    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.acceptProposedAction()

    def dropEvent(self, e):
        urls = e.mimeData().urls()
        if urls:
            self.open_recording(urls[0].toLocalFile())

    def pick_recording(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, '打开录包', CAPTURE_DIR, '抓包文件 (*.pcapng *.pcap);;所有文件 (*)')
        if path:
            self.open_recording(path)

    def open_recording(self, path, auto=False):
        self.auto_replay = auto  # auto: still switch to live if it shows up
        self.want_auto = False
        self.mode = 'loading'
        self.src_label.setText(f'正在读取 {os.path.basename(path)} …')
        self.loader.load(path)

    def on_loaded(self, rec, err):
        if rec is None:
            self.mode = 'wait'
            self.src_label.setText('读取失败')
            QtWidgets.QMessageBox.warning(self, '读取录包失败', err)
            return
        self.rec = rec
        self.mode = 'replay'
        self.replay_t0 = time.monotonic()
        self.pause_box.setChecked(False)
        self.replay_text = (f'回放 {os.path.basename(rec.path)}<br>{rec.duration:.1f} s，'
                            f'{len(rec.refl) / 1e6:.2f} M 有效点（循环）')

    def go_live(self):
        if self.live.error:
            QtWidgets.QMessageBox.warning(self, '实时数据不可用', self.live.error)
            return
        self.auto_replay = False
        self.mode = 'live'

    def set_paused(self, on):
        if self.mode == 'replay' and self.rec:
            if on:
                self.pause_at = self.replay_pos()
            else:
                self.replay_t0 = time.monotonic() - self.pause_at
        self.paused = on
        self.frozen = False

    def replay_pos(self):
        if self.paused:
            return self.pause_at
        return (time.monotonic() - self.replay_t0) % max(self.rec.duration, 1e-3)

    # ---- per-frame
    def tick(self):
        now = time.monotonic()
        live_ok = self.live.last_rx is not None and now - self.live.last_rx < 1.0
        if live_ok and (self.mode == 'wait' or (self.auto_replay and self.mode in ('replay', 'loading'))):
            self.mode = 'live'  # live data showed up: prefer it over the automatic replay
            self.frozen = False
        if self.mode == 'wait':
            self.src_label.setText('等待雷达数据 (UDP 60001)…')
            if self.want_auto and now - self.started > AUTO_REPLAY_AFTER:
                self.want_auto = False
                caps = sorted(glob.glob(os.path.join(CAPTURE_DIR, '*.pcap*')), key=os.path.getmtime)
                if caps:
                    self.open_recording(caps[-1], auto=True)
            return
        if self.mode == 'loading':
            return
        if self.mode == 'live':
            self.src_label.setText('实时 UDP 60001' if live_ok else '实时 UDP 60001（无数据）')
        else:
            self.src_label.setText(self.replay_text)
        if self.paused and self.frozen:
            return
        win = self.win_spin.value()
        if self.mode == 'live':
            xyz, refl = self.live.window(win)
        else:
            xyz, refl = self.rec.window(self.replay_pos(), win)
        self.draw(xyz, refl)
        self.frozen = self.paused

    def draw(self, xyz, refl):
        if len(xyz):
            r = self.range_spin.value()
            z = xyz[:, 2]
            keep = ((np.einsum('ij,ij->i', xyz, xyz) < r * r)
                    & (z > self.zmin_spin.value()) & (z < self.zmax_spin.value()))
            xyz, refl = xyz[keep], refl[keep]
        if len(xyz) > MAX_DRAW:
            idx = np.random.default_rng().choice(len(xyz), MAX_DRAW, replace=False)
            xyz, refl = xyz[idx], refl[idx]
        self.drawn = len(xyz)
        if not len(xyz):
            self.scatter.setData(pos=np.zeros((0, 3), np.float32))
            return
        mode = self.color_box.currentText()
        if mode == '单色':
            color = (0.55, 0.9, 1.0, 1.0)
        else:
            if mode == '反射率':
                v = np.clip(refl.astype(np.float32) / 150.0, 0, 1)  # 0-150 diffuse, >150 retro-reflective
            elif mode == '高度':
                lo, hi = np.percentile(xyz[:, 2], [2, 98])
                v = np.clip((xyz[:, 2] - lo) / max(hi - lo, 0.1), 0, 1)
            else:
                d = np.sqrt(np.einsum('ij,ij->i', xyz, xyz))
                v = np.clip(d / max(np.percentile(d, 98), 1.0), 0, 1)
            color = LUT[(v * 255).astype(np.int32)]
        self.scatter.setData(pos=xyz, color=color, size=self.size_spin.value())

    def update_stats(self):
        pkts, pts = self.live.take_counters()
        lines = [f'显示点数 {self.drawn:,}']
        if self.live.error:
            lines.append(f'<font color="#e88">{self.live.error}</font>')
        else:
            lines.append(f'实时: {pkts} 包/s，{pts:,} 有效点/s')
        if self.mode == 'replay' and self.rec:
            lines.append(f'回放进度 {self.replay_pos():.1f} / {self.rec.duration:.1f} s')
        self.info.setText('<br>'.join(lines))

    def closeEvent(self, e):
        self.live.close()
        super().closeEvent(e)


def main():
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    path = sys.argv[1] if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]) else None
    win = Viewer(path)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        raise
    except Exception:  # noqa: BLE001 - pythonw has no console
        fatal(traceback.format_exc())
