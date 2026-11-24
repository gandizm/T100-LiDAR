"""FAST-LIO 点云查看器: view a walk map (map_full.pcd from scripts/bag_to_pcd.py) on Windows.

  python pcd_viewer.py [map.pcd]        (or drag a .pcd onto the window / pcd_viewer.bat)
Mouse: left drag rotate, wheel zoom, middle drag (or Ctrl + left drag) pan.
trajectory.csv next to the .pcd is drawn as a line; where it moves faster than 3 m/s (FAST-LIO
diverged) it turns red, and "只看发散前" hides everything mapped from that moment on.
"""
import os
import sys

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl

TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
         ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}
TURBO = np.array([[0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943],
                  [0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604],
                  [0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973]])


def read_pcd(path):
    hdr, t0 = {}, 0.0
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD 文件头里没有 DATA 行")
            s = line.decode("ascii", "replace").strip()
            if s.startswith("# t0 ="):
                t0 = float(s.split("=")[1])
            if s and not s.startswith("#"):
                k, *v = s.split()
                hdr[k] = v
                if k == "DATA":
                    break
        body = f.read()
    names, n = hdr["FIELDS"], int(hdr["POINTS"][0])
    if hdr["DATA"][0] == "binary":
        counts = [int(c) for c in hdr.get("COUNT", ["1"] * len(names))]
        dt = np.dtype([(nm if nm != "_" else f"_pad{i}", TYPES[(ty, int(sz))], (c,) if c > 1 else ())
                       for i, (nm, sz, ty, c) in enumerate(zip(names, hdr["SIZE"], hdr["TYPE"], counts))])
        a = np.frombuffer(body, dt, n)
        col = {nm: a[nm] for nm in ("x", "y", "z", "intensity", "t") if nm in names}
    elif hdr["DATA"][0] == "ascii":
        a = np.loadtxt(body.decode().splitlines(), ndmin=2)
        col = {nm: a[:, i] for i, nm in enumerate(names)}
    else:
        raise ValueError(f"不支持 DATA {hdr['DATA'][0]}（请存成 binary 或 ascii）")
    xyz = np.column_stack([col["x"], col["y"], col["z"]]).astype(np.float32)
    ok = np.isfinite(xyz).all(1)
    t = np.asarray(col.get("t", np.zeros(n)), np.float32)
    inten = np.asarray(col.get("intensity", np.zeros(n)), np.float32)
    return xyz[ok], t[ok], inten[ok], t0


def write_pcd(path, xyz, inten, t, t0):
    n = len(xyz)
    hdr = (f"# .PCD v0.7 - Point Cloud Data file format\n# t0 = {t0:.6f}\nVERSION 0.7\nFIELDS x y z intensity t\n"
           f"SIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(np.column_stack([xyz, inten, t]).astype(np.float32).tobytes())


def read_traj(pcd_path, t0):
    p = os.path.join(os.path.dirname(pcd_path), "trajectory.csv")
    if not os.path.exists(p):
        return None, None
    T = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
    t = T[:, 0] - (t0 if t0 else T[0, 0])
    xyz = T[:, 1:4].astype(np.float32)
    v = np.r_[0, np.linalg.norm(np.diff(xyz, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
    vs = np.convolve(v, np.ones(10) / 10, "same")  # 1 s mean at 10 Hz
    # diverged: faster than 3 m/s for 2 s in a row (walking never is); the cut is the last moment before
    # that at walking pace (< 1.8 m/s), i.e. where the runaway started
    hit = np.flatnonzero(np.convolve((vs > 3.0).astype(int), np.ones(20, int), "valid") == 20)
    if not len(hit):
        return (t, xyz), None
    slow = np.flatnonzero(vs[:hit[0]] < 1.8)
    return (t, xyz), float(t[slow[-1]] if len(slow) else t[hit[0]])


def turbo(v):
    v = np.clip(v, 0, 1)
    out = np.ones((len(v), 4), np.float32)
    for k in range(3):
        acc = np.full(len(v), TURBO[k, 5])
        for j in range(4, -1, -1):
            acc = acc * v + TURBO[k, j]
        out[:, k] = np.clip(acc, 0, 1)
    return out


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, path):
        super().__init__()
        self.setWindowTitle("FAST-LIO 点云查看器")
        self.resize(1500, 950)
        self.setAcceptDrops(True)
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(pg.mkColor(20, 22, 26))
        self.setCentralWidget(self.view)
        tb = self.addToolBar("view")
        btn_open = QtWidgets.QPushButton("打开…")
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["按高度着色", "按时间着色"])
        self.psize = QtWidgets.QDoubleSpinBox()
        self.psize.setRange(0.5, 10)
        self.psize.setSingleStep(0.5)
        self.psize.setValue(1.0)
        self.psize.setPrefix("点大小 ")
        self.bright = QtWidgets.QDoubleSpinBox()
        self.bright.setRange(0.1, 1.0)
        self.bright.setSingleStep(0.05)
        self.bright.setValue(0.55)
        self.bright.setPrefix("亮度 ")
        self.t_from, self.t_to = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        for w, p in ((self.t_from, "从 "), (self.t_to, "到 ")):
            w.setRange(0, 1e6)
            w.setDecimals(1)
            w.setPrefix(p)
            w.setSuffix(" s")
            w.setKeyboardTracking(False)
        self.btn_div, btn_all = QtWidgets.QPushButton("只看发散前"), QtWidgets.QPushButton("全部时间")
        btn_save = QtWidgets.QPushButton("另存显示的点…")
        self.traj_cb = QtWidgets.QCheckBox("轨迹")
        self.traj_cb.setChecked(True)
        self.grid_cb = QtWidgets.QCheckBox("网格")
        for w in (btn_open, self.mode, self.psize, self.bright, self.t_from, self.t_to, self.btn_div, btn_all,
                  self.traj_cb, self.grid_cb, btn_save):
            tb.addWidget(w)
        self.scatter = gl.GLScatterPlotItem(pxMode=True, glOptions="opaque")  # default is additive: dense areas go white
        self.line = gl.GLLinePlotItem(width=2, antialias=True)
        self.grid = gl.GLGridItem()
        self.grid.setSpacing(5, 5)
        for it in (self.grid, self.scatter, self.line):
            self.view.addItem(it)
        self.grid.setVisible(False)
        self.grid_cb.toggled.connect(self.grid.setVisible)
        btn_open.clicked.connect(self.ask_open)
        for sig in (self.mode.currentIndexChanged, self.psize.valueChanged, self.bright.valueChanged,
                    self.t_from.valueChanged, self.t_to.valueChanged, self.traj_cb.toggled):
            sig.connect(self.refresh)
        self.btn_div.clicked.connect(lambda: (self.t_from.setValue(0), self.t_to.setValue(self.t_div)))
        btn_save.clicked.connect(self.save_shown)
        btn_all.clicked.connect(lambda: (self.t_from.setValue(0), self.t_to.setValue(self.t_to.maximum())))
        self.xyz = None
        if path:
            self.load(path)

    def ask_open(self):
        p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "打开点云", "", "点云 (*.pcd)")
        if p:
            self.load(p)

    def save_shown(self):
        if self.xyz is None:
            return
        p, _ = QtWidgets.QFileDialog.getSaveFileName(self, "另存显示的点", os.path.splitext(self.path)[0] + "_cut.pcd",
                                                     "点云 (*.pcd)")
        if p:
            m = self.mask
            write_pcd(p, self.xyz[m], self.inten[m], self.t[m], self.t0)
            self.statusBar().showMessage(f"已保存 {m.sum():,} 点到 {p}")

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.acceptProposedAction()

    def dropEvent(self, e):
        for u in e.mimeData().urls():
            if u.toLocalFile().lower().endswith(".pcd"):
                self.load(u.toLocalFile())
                break

    def load(self, path):
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            xyz, t, inten, t0 = read_pcd(path)
            traj, t_div = read_traj(path, t0)
        except Exception as e:  # noqa: BLE001 - show any parse error to the user
            QtWidgets.QApplication.restoreOverrideCursor()
            QtWidgets.QMessageBox.critical(self, "打不开", f"{path}\n\n{e}")
            return
        QtWidgets.QApplication.restoreOverrideCursor()
        self.path, self.xyz, self.t, self.traj, self.t_div = path, xyz, t, traj, t_div
        self.inten, self.t0 = inten, t0
        ref = xyz if t_div is None or not (t < t_div).any() else xyz[t < t_div]  # diverged points are not the map
        self.zr = np.percentile(ref[:, 2], [2, 98])
        tmax = float(max(t.max(), traj[0][-1] if traj else 0)) + 1
        for w in (self.t_from, self.t_to):
            w.blockSignals(True)
            w.setMaximum(tmax)
        self.t_from.setValue(0)
        self.t_to.setValue(t_div if t_div is not None else tmax)  # the diverged part is hidden by default
        for w in (self.t_from, self.t_to):
            w.blockSignals(False)
        self.btn_div.setEnabled(t_div is not None)
        c = np.median(ref, 0)
        span = float(max((np.percentile(ref, 98, 0) - np.percentile(ref, 2, 0))[:2].max(), 10))
        self.view.setCameraPosition(pos=QtGui.QVector3D(*map(float, c)), distance=span * 1.3, elevation=40, azimuth=-90)
        self.grid.resetTransform()
        g = float(np.ceil(span * 1.5 / 10) * 10)
        self.grid.setSize(g, g)
        self.grid.translate(float(c[0]), float(c[1]), float(np.percentile(ref[:, 2], 2)))
        self.refresh()

    def refresh(self):
        if self.xyz is None:
            return
        lo, hi = self.t_from.value(), self.t_to.value()
        m = (self.t >= lo) & (self.t <= hi)
        self.mask = m
        pts = self.xyz[m]
        if self.mode.currentIndex() == 0:
            v = (pts[:, 2] - self.zr[0]) / max(self.zr[1] - self.zr[0], 1e-3)
        else:
            v = (self.t[m] - lo) / max(hi - lo, 1e-3)
        col = turbo(v)
        col[:, :3] *= self.bright.value()
        self.scatter.setData(pos=pts, color=col, size=self.psize.value())
        k = None
        if self.traj and self.traj_cb.isChecked():
            t, p = self.traj
            k = (t >= lo) & (t <= hi)
        if k is not None and k.sum() > 1:
            col = np.tile(np.array([1, 1, 1, 1], np.float32), (k.sum(), 1))
            if self.t_div is not None:
                col[self.traj[0][k] >= self.t_div] = (1, 0.25, 0.25, 1)
            self.line.setData(pos=self.traj[1][k], color=col, mode="line_strip")
            self.line.setVisible(True)
        else:
            self.line.setVisible(False)
        txt = (f"{os.path.basename(os.path.dirname(self.path))}/{os.path.basename(self.path)}    "
               f"显示 {m.sum():,} / {len(self.xyz):,} 点    时间 {lo:.1f} – {hi:.1f} s")
        if self.t_div is not None:
            txt += f"    FAST-LIO 从 {self.t_div:.1f} s 起发散（红线），之后的点默认不显示"
        self.statusBar().showMessage(txt)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Viewer(sys.argv[1] if len(sys.argv) > 1 else None)
    w.show()
    if w.xyz is None:
        w.ask_open()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
