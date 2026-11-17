"""Ground station v0: serial link, realtime roll/pitch plot, event log, CSV logging of every line.

Run: python gcs/gcs.py   (pip install -r gcs/requirements.txt)
"""
import collections
import csv
import datetime
import pathlib
import queue
import sys
import threading
import time

import numpy as np
# Load PySide6 first so pyqtgraph uses the same Qt binding.
from PySide6 import QtCore, QtGui, QtWidgets

import pyqtgraph as pg
import serial
import serial.tools.list_ports

# Field order of the T line, PLAN §9.1 (angles in deg, lengths in m).
T_FIELDS = [
    "t_ms", "arm", "mode", "fs", "roll", "pitch", "yaw", "p", "q", "r",
    "roll_sp", "pitch_sp", "yawr_sp", "z", "vz", "z_sp", "tof", "thr",
    "m1", "m2", "m3", "m4", "ctrl_us_max", "jitter_us_max", "h_terrain", "baro_rel", "alt_src",
    "p_sp", "q_sp", "acro", "alt_valid",
    "vel_fwd", "vel_right", "pos_x", "pos_y", "flow_q", "flow_valid",
]
IDX_T, IDX_ROLL, IDX_PITCH = T_FIELDS.index("t_ms"), T_FIELDS.index("roll"), T_FIELDS.index("pitch")
FS_TEXT = {0: "", 1: "  FAILSAFE 1 (mất sóng: giữ thăng bằng)", 2: "  FAILSAFE 2"}
# CH6 up = Alt Hold (+ position hold when the flow is valid), middle/down = Angle (Acro removed)
MODE_TEXT = {0: "Angle", 1: "GIỮ ĐỘ CAO", 2: "GIỮ ĐỘ CAO + VỊ TRÍ", 3: "HẠ CÁNH"}
HOLD_MODES = (1, 2, 3)  # modes with an altitude setpoint
# G1 GPS line (App/app/tasks.c send_gps), 5 Hz on USB, 1 Hz on the radio
G_FIELDS = ("age_ms", "fix_type", "fix_ok", "num_sv", "lat_e7", "lon_e7", "hmsl_mm", "h_acc_mm", "v_acc_mm",
            "vel_n", "vel_e", "vel_d", "s_acc", "pdop", "bytes", "pvt_count", "ck_errors", "nmea_lines",
            "cfg_sent", "acks", "naks")
FIX_TEXT = {0: "không fix", 1: "dead reckoning", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "chỉ thời gian"}
EARTH_R = 6371000.0
RATE_KEYS = [("p", "p_sp", "#d62728"), ("q", "q_sp", "#1f77b4"), ("r", "yawr_sp", "#2ca02c")]
# Gains on the quick-tune panel of the "Bay" tab (same names as the firmware parameter table).
TUNE_PARAMS = [("rate_p_rp", "P roll/pitch"), ("rate_i_rp", "I roll/pitch"), ("rate_d_rp", "D roll/pitch"),
               ("rate_p_y", "P yaw"), ("rate_i_y", "I yaw"), ("ang_p_rp", "P góc (Angle)"),
               ("trim_roll", "Trim roll ° (trôi phải: giảm)"), ("trim_pitch", "Trim pitch ° (trôi lùi: giảm)"),
               ("alt_p", "P độ cao"), ("vz_p", "P vận tốc đứng"), ("vz_i", "I vận tốc đứng"),
               ("yaw_hold_p", "P giữ hướng (0 = tắt)"), ("pos_p", "P vị trí (0 = tắt)"),
               ("vel_p", "P vận tốc ngang °/(m/s)"), ("vel_i", "I vận tốc ngang")]
RMS_WINDOW_S = 5.0        # tracking-error window
RMS_MIN_THR = 0.05        # samples count only while armed and above idle throttle
LINK_STALE_S = 0.5        # no T line for this long = data lost (radio link or USB)
OSC_MIN_HZ = 15.0         # capture analysis: strongest peak above this = oscillation / vibration
CAPTURE_REPLY_S = 1.5     # no OK/ERR to a CAPTURE within this time = command lost
# = capture_header() in App/app/capture.c; used when that line itself was lost on the radio link.
CAPTURE_HEADER = ("C,i,dt_us,p,q,r,p_sp,q_sp,r_sp,P_roll,I_roll,D_roll,P_pitch,I_pitch,D_pitch,P_yaw,I_yaw,"
                  "thr,m1,m2,m3,m4")
# Same mapping as the firmware (app/config.h): ESC pulse = 900 + 1100 * m us, 900 us = stop.
ESC_STOP_US, ESC_SPAN_US = 900, 1100
MOTOR_POS = ["(trước-phải)", "(sau-trái)", "(trước-trái)", "(sau-phải)"]  # PLAN §7.6 M1..M4

HISTORY_LEN = 50 * 20     # 20 s of T lines at 50 Hz
REFRESH_MS = 50           # 20 Hz redraw is plenty for a 50 Hz signal
DEFAULT_Y_RANGE = 30.0    # deg

LOG_DIR = pathlib.Path(__file__).resolve().parent / "logs"

# R line (M4 RC test): R,valid,failsafe,age_ms,fps,bad,uart_errs,lost,roll,pitch,yaw,thr,ch1..ch16
R_HEADER_LEN = 11
RC_CHANGE_THRESHOLD = 40   # raw units; sticks jitter a few counts, switches jump hundreds
RC_HIGHLIGHT_S = 1.5


class SerialReader(threading.Thread):
    """Reads lines from the port into a queue; while logging is on (start_log / stop_log, GUI buttons) every line
    also goes to a CSV log."""

    def __init__(self, port_name, out_queue):
        super().__init__(daemon=True)
        # USB CDC ignores the baud rate (STM32 directly, or the ESP-NOW bridge board).
        # DTR on: the STM32 only sends while DTR is set. RTS off: RTS can reset the ESP32-C3's USB port.
        self.ser = serial.Serial(None, 115200, timeout=0.05)
        self.ser.port = port_name
        self.ser.dtr = True
        self.ser.rts = False
        self.ser.open()
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.log_lock = threading.Lock()  # the GUI thread opens/closes, this thread writes
        self.log_file = self.log_writer = self.log_path = None

    def start_log(self):
        """Opens a new CSV (gcs/logs/log_<date>_<time>.csv) and returns its path."""
        LOG_DIR.mkdir(exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = LOG_DIR / f"log_{stamp}.csv"
        f = open(path, "w", newline="", encoding="utf-8")
        w = csv.writer(f)
        w.writerow(["pc_time_s", "line"])
        with self.log_lock:
            self._close_log()
            self.log_file, self.log_writer, self.log_path = f, w, path
        return path

    def stop_log(self):
        """Closes the CSV (everything written to disk); returns its path, or None if logging was off."""
        with self.log_lock:
            path = self.log_path
            self._close_log()
        return path

    def _close_log(self):
        if self.log_file is not None:
            self.log_file.close()
        self.log_file = self.log_writer = self.log_path = None

    def run(self):
        buf = b""
        last_flush = time.monotonic()
        while not self.stop_event.is_set():
            # Push the CSV to disk once a second: a crash or a killed GCS then loses at most ~1 s of log.
            if time.monotonic() - last_flush >= 1.0:
                with self.log_lock:
                    if self.log_file is not None:
                        self.log_file.flush()
                last_flush = time.monotonic()
            try:
                chunk = self.ser.read(4096)
            except serial.SerialException as exc:
                self.out_queue.put(f"[GCS] serial error: {exc}")
                break
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")
            now = f"{time.time():.3f}"
            with self.log_lock:
                for raw in lines:
                    line = raw.decode("ascii", errors="replace").strip()
                    if line:
                        if self.log_writer is not None:
                            self.log_writer.writerow([now, line])
                        self.out_queue.put(line)
        with self.log_lock:
            self._close_log()
        self.ser.close()

    def send(self, text):
        # Leading newline: ends any partial line (noise, a lost byte) left in the drone's command buffer;
        # the drone ignores empty lines.
        self.ser.write(("\n" + text.strip() + "\n").encode("ascii"))

    def stop(self):
        self.stop_event.set()


def load_capture(header, rows):
    """C lines (without the "C," prefix, as text fields) -> dict of numpy arrays in physical units
    (s, deg/s, torque, us). Malformed rows (radio errors) are skipped; with the sample index column "i",
    lost samples are filled by linear interpolation so the time base and the FFT stay uniform."""
    good, bad = [], 0
    for r in rows:
        try:
            if len(r) == len(header):
                good.append([float(x) for x in r])
                continue
        except ValueError:
            pass
        bad += 1
    if len(good) < 16:
        raise ValueError(f"chỉ còn {len(good)} dòng hợp lệ")
    a = np.array(good, dtype=float)
    cols = {name: a[:, i] for i, name in enumerate(header)}
    missing = 0
    if "i" in cols:
        idx, first = np.unique(cols["i"].astype(int), return_index=True)  # drops duplicated lines
        cols = {k: v[first] for k, v in cols.items()}
        consecutive = np.diff(idx) == 1
        dt_us = float(np.median(cols["dt_us"][1:][consecutive])) if consecutive.any() else 1887.0
        grid = np.arange(idx[0], idx[-1] + 1)
        missing = len(grid) - len(idx)
        cols = {k: np.interp(grid, idx, v) for k, v in cols.items()}
        t = (grid - grid[0]) * dt_us / 1e6
    else:
        t = np.cumsum(cols["dt_us"]) / 1e6
    d = {"t": t, "bad_rows": bad, "missing": missing}
    for k in ("p", "q", "r", "p_sp", "q_sp", "r_sp"):
        d[k] = cols[k] / 10.0
    for k in header:
        if k[:2] in ("P_", "I_", "D_"):
            d[k] = cols[k] / 1e4
    d["thr"] = cols["thr"] / 1000.0
    for i in range(1, 5):
        d[f"m{i}"] = cols[f"m{i}"]
    return d


def capture_metrics(t, meas, sp):
    """Tracking numbers for one axis: RMS error, lag, peak ratio, strongest oscillation (FFT)."""
    fs = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0
    out = {"fs": fs, "rms": float(np.sqrt(np.mean((meas - sp) ** 2)))}
    out["lag_ms"] = out["peak_pct"] = out["corr"] = None
    out["lag_note"] = ""
    if np.ptp(sp) > 20.0 and fs > 0:
        a, b = sp - sp.mean(), meas - meas.mean()
        max_lag = min(int(0.1 * fs), len(a) - 1)
        norm = np.sqrt(np.dot(a, a) * np.dot(b, b)) or 1.0
        # Both directions: a best match at lag <= 0 (measurement ahead of the setpoint) is not a real delay.
        lags = range(-max_lag, max_lag + 1)
        xc = [np.dot(a[max(0, -k):len(a) - max(0, k)], b[max(0, k):len(b) - max(0, -k)]) / norm for k in lags]
        best = int(np.argmax(xc))
        lag, out["corr"] = lags[best], float(xc[best])
        if out["corr"] < 0.5:
            out["lag_note"] = "đo ít giống đặt"
        elif lag <= 0:
            out["lag_note"] = "đo đi trước đặt"
        else:
            out["lag_ms"] = 1000.0 * lag / fs
        # Peak of the motion, not of the vibration riding on it: 30 ms moving average first.
        w = max(1, int(0.03 * fs))
        smooth = np.convolve(meas, np.ones(w) / w, mode="same")
        out["peak_pct"] = 100.0 * np.max(np.abs(smooth)) / np.max(np.abs(sp))
    x = (meas - meas.mean()) * np.hanning(len(meas))
    spec = np.abs(np.fft.rfft(x)) * 2.0 / np.sum(np.hanning(len(meas)))
    freq = np.fft.rfftfreq(len(meas), 1.0 / fs) if fs > 0 else np.zeros(len(spec))
    # Stick moves and the response to them sit below ~10 Hz; a peak above OSC_MIN_HZ is a loop oscillation
    # (P/D too high) or vibration.
    band = freq >= OSC_MIN_HZ
    out["osc_hz"] = out["osc_amp"] = None
    if band.any():
        i = int(np.argmax(np.where(band, spec, 0.0)))
        if spec[i] >= 0.5:
            out["osc_hz"], out["osc_amp"] = float(freq[i]), float(spec[i])
    out["freq"], out["spec"] = freq, spec
    return out


class CaptureWindow(QtWidgets.QWidget):
    """High-rate capture (CAPTURE command, ~530 Hz): setpoint vs rate, PID terms, motors, gyro spectrum."""

    AXES = [("Roll (p)", "p", "p_sp", "roll"), ("Pitch (q)", "q", "q_sp", "pitch"), ("Yaw (r)", "r", "r_sp", "yaw")]

    def __init__(self, data, title):
        super().__init__()
        self.setWindowTitle(f"Capture tần số cao - {title}")
        self.resize(1200, 900)
        self.d = data
        lay = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Trục:"))
        self.axis_box = QtWidgets.QComboBox()
        for text, *_ in self.AXES:
            self.axis_box.addItem(text)
        self.axis_box.currentIndexChanged.connect(self._draw)
        top.addWidget(self.axis_box)
        top.addStretch(1)
        lay.addLayout(top)
        self.metrics = QtWidgets.QLabel()
        self.metrics.setStyleSheet("font-family: Consolas, monospace; font-size: 13px;")
        self.metrics.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        lay.addWidget(self.metrics)

        self.p_rate = self._plot("tốc độ góc", "deg/s")
        self.p_terms = self._plot("khâu PID (mômen chuẩn hóa)", "")
        self.p_motor = self._plot("motor", "us")
        for p in (self.p_terms, self.p_motor):
            p.setXLink(self.p_rate)
        self.p_fft = self._plot("biên độ (log)", "deg/s", bottom=("tần số", "Hz"))
        self.p_fft.setLogMode(x=False, y=True)  # small oscillation peaks next to the large stick-move content
        for p, s in ((self.p_rate, 3), (self.p_terms, 2), (self.p_motor, 2), (self.p_fft, 2)):
            lay.addWidget(p, stretch=s)
        self._fill_metrics()
        self._draw()

    @staticmethod
    def _plot(label, units, bottom=("thời gian", "s")):
        p = pg.PlotWidget()
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setLabel("left", label, units=units)
        p.setLabel("bottom", bottom[0], units=bottom[1])
        p.addLegend(offset=(10, 10))
        return p

    def _fill_metrics(self):
        t, lines = self.d["t"], []
        for text, meas, sp, _ in self.AXES:
            m = capture_metrics(t, self.d[meas], self.d[sp])
            if m["lag_ms"] is not None:
                lag = f"{m['lag_ms']:5.1f} ms (khớp {m['corr']:.2f})"
            elif m["lag_note"]:
                lag = f"? ({m['lag_note']}, khớp {m['corr']:.2f})"
            else:
                lag = "  -  "
            peak = "  -  " if m["peak_pct"] is None else f"{m['peak_pct']:5.0f}"
            osc = "-" if m["osc_hz"] is None else f"{m['osc_hz']:5.1f} Hz ({m['osc_amp']:.1f} deg/s)"
            lines.append(f"{text:9s} RMS sai số {m['rms']:6.1f} deg/s | trễ {lag} | đỉnh đo/đặt {peak} % "
                         f"| dao động mạnh nhất > {OSC_MIN_HZ:.0f} Hz: {osc}")
        fs = capture_metrics(t, self.d["p"], self.d["p_sp"])["fs"]
        lost = ""
        if self.d["missing"] or self.d["bad_rows"]:
            lost = (f" | MẤT {self.d['missing']} mẫu trên đường truyền (đã nội suy), "
                    f"{self.d['bad_rows']} dòng hỏng bị bỏ: nhiều thì nên ghi lại")
        lines.append(f"{len(t)} mẫu, {t[-1]:.2f} s, {fs:.0f} Hz{lost}. Trễ / đỉnh chỉ tính khi đặt thay đổi "
                     f"> 20 deg/s (hãy gạt cần dứt khoát lúc ghi). Đỉnh > 100 %: vọt lố.")
        self.metrics.setText("\n".join(lines))

    def _draw(self):
        _, meas, sp, name = self.AXES[self.axis_box.currentIndex()]
        t, d = self.d["t"], self.d
        for p in (self.p_rate, self.p_terms, self.p_motor, self.p_fft):
            p.clear()
        self.p_rate.plot(t, d[sp], name="đặt", pen=pg.mkPen("k", width=2, style=QtCore.Qt.DashLine))
        self.p_rate.plot(t, d[meas], name="đo", pen=pg.mkPen("#d62728", width=2))
        total = np.zeros_like(t)
        for k, color in (("P", "#1f77b4"), ("I", "#2ca02c"), ("D", "#ff7f0e")):
            key = f"{k}_{name}"
            if key in d:
                self.p_terms.plot(t, d[key], name=k, pen=pg.mkPen(color, width=2))
                total = total + d[key]
        self.p_terms.plot(t, total, name="tổng", pen=pg.mkPen("k", width=1))
        for i, color in zip(range(1, 5), ("#d62728", "#1f77b4", "#2ca02c", "#9467bd")):
            self.p_motor.plot(t, d[f"m{i}"], name=f"M{i}{MOTOR_POS[i - 1]}", pen=pg.mkPen(color, width=1))
        self.p_motor.addLine(y=ESC_STOP_US + ESC_SPAN_US, pen=pg.mkPen("r", style=QtCore.Qt.DotLine))
        m = capture_metrics(t, d[meas], d[sp])
        self.p_fft.plot(m["freq"], m["spec"] + 1e-3, name=f"{meas} đo", pen=pg.mkPen("#d62728", width=1))
        e = capture_metrics(t, d[meas] - d[sp], np.zeros_like(t))
        self.p_fft.plot(e["freq"], e["spec"] + 1e-3, name="sai số", pen=pg.mkPen("#7f7f7f", width=1))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone GCS v0 - roll / pitch")
        self.resize(1100, 750)
        self.reader = None
        self.reconnect_port = None
        self.reconnect_tries = 0
        self.rx_queue = queue.Queue()
        self.t_s = collections.deque(maxlen=HISTORY_LEN)
        self.roll = collections.deque(maxlen=HISTORY_LEN)
        self.pitch = collections.deque(maxlen=HISTORY_LEN)
        # Setpoints from the same T line, to compare controller response with the request.
        self.roll_sp = collections.deque(maxlen=HISTORY_LEN)
        self.pitch_sp = collections.deque(maxlen=HISTORY_LEN)
        # Body rates p/q/r and their rate-loop setpoints (deg/s), keyed by T field name.
        self.rates = {k: collections.deque(maxlen=HISTORY_LEN) for m, s, _ in RATE_KEYS for k in (m, s)}
        self.active = collections.deque(maxlen=HISTORY_LEN)  # armed and above idle: counts for RMS
        self.last_t_rx = None  # time.monotonic() of the last T line
        self.dirty = False
        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(REFRESH_MS)

    def _build_ui(self):
        outer_widget = QtWidgets.QWidget()
        self.setCentralWidget(outer_widget)
        outer = QtWidgets.QVBoxLayout(outer_widget)
        self.tabs = QtWidgets.QTabWidget()
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        self.tabs.addTab(central, "Bay")
        self.tabs.addTab(self._build_alt_tab(), "Độ cao")
        self.tabs.addTab(self._build_pos_tab(), "Vị trí")
        self.tabs.addTab(self._build_gps_tab(), "GPS")
        self.tabs.addTab(self._build_param_tab(), "Tham số / Hộp đen")

        bar = QtWidgets.QHBoxLayout()
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(260)
        refresh_btn = QtWidgets.QPushButton("Làm mới")
        refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QtWidgets.QPushButton("Kết nối")
        self.connect_btn.clicked.connect(self._toggle_connect)
        # Log only between these presses (user, 2026-11-10); kept on across an automatic reconnect (new file).
        self.log_btn = QtWidgets.QPushButton("● Bắt đầu ghi log")
        self.log_btn.setCheckable(True)
        self.log_btn.setMinimumWidth(190)
        self.log_btn.toggled.connect(self._toggle_log)
        disarm_btn = QtWidgets.QPushButton("DISARM")
        disarm_btn.setMinimumSize(160, 50)
        disarm_btn.setStyleSheet("QPushButton { background: #d00000; color: white; font-size: 20px; font-weight: bold; }")
        disarm_btn.clicked.connect(lambda: self._send("DISARM"))
        # Controlled landing (0.3 m/s, disarm at touchdown); DISARM stays the emergency cut.
        land_btn = QtWidgets.QPushButton("HẠ CÁNH")
        land_btn.setMinimumSize(140, 50)
        land_btn.setStyleSheet("QPushButton { background: #ef6c00; color: white; font-size: 18px; font-weight: bold; }")
        land_btn.clicked.connect(lambda: self._send("LAND"))
        for w in (QtWidgets.QLabel("Cổng:"), self.port_box, refresh_btn, self.connect_btn, self.log_btn):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(land_btn)
        bar.addWidget(disarm_btn)
        outer.addLayout(bar)
        outer.addWidget(self.tabs, stretch=4)

        nums = QtWidgets.QHBoxLayout()
        big = "font-family: Consolas, monospace; font-size: 34px; font-weight: bold;"
        self.roll_label = QtWidgets.QLabel("Roll   ---")
        self.roll_label.setStyleSheet(big + "color: #d62728;")
        self.pitch_label = QtWidgets.QLabel("Pitch  ---")
        self.pitch_label.setStyleSheet(big + "color: #1f77b4;")
        self.arm_label = QtWidgets.QLabel("DISARMED")
        self.arm_label.setStyleSheet(big + "color: white; background: #2e7d32; padding: 2px 12px;")
        nums.addWidget(self.arm_label)
        nums.addSpacing(30)
        nums.addWidget(self.roll_label)
        nums.addSpacing(60)
        nums.addWidget(self.pitch_label)
        nums.addStretch(1)
        self.link_label = QtWidgets.QLabel("")
        nums.addWidget(self.link_label)
        self.auto_y = QtWidgets.QCheckBox("Tự co trục Y")
        nums.addWidget(self.auto_y)
        # Paused: curves freeze (data keeps coming) and the time axis can be zoomed/panned with the mouse.
        self.pause_btn = QtWidgets.QPushButton("Tạm dừng đồ thị")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._set_paused)
        nums.addWidget(self.pause_btn)
        root.addLayout(nums)

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("left", "góc", units="deg")
        self.plot.setLabel("bottom", "thời gian (0 = mới nhất)", units="s")
        self.plot.addLegend(offset=(10, 10))
        self.plot.setXRange(-HISTORY_LEN / 50.0, 0.0, padding=0)
        self.plot.setYRange(-DEFAULT_Y_RANGE, DEFAULT_Y_RANGE, padding=0)
        self.plot.setMouseEnabled(x=False, y=True)
        self.roll_curve = self.plot.plot(name="roll", pen=pg.mkPen("#d62728", width=2))
        self.pitch_curve = self.plot.plot(name="pitch", pen=pg.mkPen("#1f77b4", width=2))
        self.roll_sp_curve = self.plot.plot(name="roll đặt", pen=pg.mkPen("#d62728", width=2, style=QtCore.Qt.DashLine))
        self.pitch_sp_curve = self.plot.plot(name="pitch đặt", pen=pg.mkPen("#1f77b4", width=2, style=QtCore.Qt.DashLine))

        # Rate loop: measured (solid) vs setpoint (dashed). Click a checkbox to hide an axis.
        self.rate_plot = pg.PlotWidget()
        self.rate_plot.showGrid(x=True, y=True, alpha=0.3)
        self.rate_plot.setLabel("left", "tốc độ góc", units="deg/s")
        self.rate_plot.addLegend(offset=(10, 10))
        self.rate_plot.setXLink(self.plot)
        self.rate_plot.setYRange(-200.0, 200.0, padding=0)
        self.rate_plot.setMouseEnabled(x=False, y=True)
        self.rate_curves = {}
        for meas, sp, color in RATE_KEYS:
            self.rate_curves[meas] = self.rate_plot.plot(name=f"{meas} đo", pen=pg.mkPen(color, width=2))
            self.rate_curves[sp] = self.rate_plot.plot(
                name=f"{meas} đặt", pen=pg.mkPen(color, width=2, style=QtCore.Qt.DashLine))
        for c in (self.roll_curve, self.pitch_curve, self.roll_sp_curve, self.pitch_sp_curve,
                  *self.rate_curves.values()):
            c.setClipToView(True)
            c.setSkipFiniteCheck(True)
        rate_row = QtWidgets.QHBoxLayout()
        rate_row.addWidget(QtWidgets.QLabel("Đồ thị rate hiện:"))
        for meas, sp, _ in RATE_KEYS:
            box = QtWidgets.QCheckBox(meas)
            box.setChecked(True)
            box.toggled.connect(lambda on, m=meas, s=sp: [self.rate_curves[k].setVisible(on) for k in (m, s)])
            rate_row.addWidget(box)
        rate_row.addSpacing(30)
        self.rms_label = QtWidgets.QLabel("")
        self.rms_label.setStyleSheet("font-family: Consolas, monospace; font-size: 14px; font-weight: bold;")
        rate_row.addWidget(self.rms_label)
        rate_row.addStretch(1)
        root.addWidget(self.plot, stretch=2)
        root.addLayout(rate_row)
        root.addWidget(self.rate_plot, stretch=3)
        root.addWidget(self._build_tune_panel())
        self.motor_label = QtWidgets.QLabel("Motor: ---")
        self.motor_label.setStyleSheet("font-family: Consolas, monospace; font-size: 15px; font-weight: bold;")
        root.addWidget(self.motor_label)
        test_row = QtWidgets.QHBoxLayout()
        test_row.addWidget(QtWidgets.QLabel("Thử từng ESC (chỉ khi DISARMED, ĐÃ THÁO CÁNH):"))
        for ch in range(1, 5):
            b = QtWidgets.QPushButton(f"ESC{ch}")
            b.clicked.connect(lambda _=False, c=ch: self._motor_test(c))
            test_row.addWidget(b)
        test_row.addStretch(1)
        # The 16-channel RC panel takes a lot of height; hidden by default while tuning.
        show_rc = QtWidgets.QCheckBox("Hiện bảng tay cầm")
        test_row.addWidget(show_rc)
        root.addLayout(test_row)
        self.props_off_confirmed = False
        rc_panel = self._build_rc_panel()
        rc_panel.setVisible(False)
        show_rc.toggled.connect(rc_panel.setVisible)
        root.addWidget(rc_panel)

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1000)
        self.log_view.setStyleSheet("font-family: Consolas, monospace; font-size: 13px;")
        outer.addWidget(self.log_view, stretch=1)

        cmd_row = QtWidgets.QHBoxLayout()
        self.cmd_edit = QtWidgets.QLineEdit()
        self.cmd_edit.setPlaceholderText("Lệnh gửi xuống drone (ví dụ: trace) rồi Enter")
        self.cmd_edit.returnPressed.connect(self._send_cmd)
        self.show_trc = QtWidgets.QCheckBox("Hiện TRC")
        cmd_row.addWidget(self.cmd_edit)
        cmd_row.addWidget(self.show_trc)
        outer.addLayout(cmd_row)

        self._refresh_ports()

    def _build_alt_tab(self):
        """M9: altitude estimate z vs raw ToF and terrain offset; vz below. Observation only."""
        self.alt = {k: collections.deque(maxlen=HISTORY_LEN)
                    for k in ("z", "z_sp", "tof", "h_terrain", "baro_rel", "vz")}
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        self.alt_label = QtWidgets.QLabel("Chưa có dữ liệu độ cao")
        lay.addWidget(self.alt_label)
        row = QtWidgets.QHBoxLayout()
        raw = QtWidgets.QCheckBox("Ghi dòng thô A (ALTRAW) vào log CSV cho tools/replay_kf.py")
        raw.toggled.connect(lambda on: self._send(f"ALTRAW {1 if on else 0}"))
        row.addWidget(raw)
        flow = QtWidgets.QCheckBox("Ghi optical flow (FLOWLOG, dùng USB) cho tools/flow_cal.py")
        flow.toggled.connect(lambda on: self._send(f"FLOWLOG {1 if on else 0}"))
        row.addWidget(flow)
        row.addStretch(1)
        lay.addLayout(row)
        self.alt_curves = {}
        zp = pg.PlotWidget()
        zp.showGrid(x=True, y=True, alpha=0.3)
        zp.setLabel("left", "độ cao", units="m")
        zp.addLegend(offset=(10, 10))
        zp.setLabel("bottom", "thời gian (0 = mới nhất)", units="s")
        zp.setXRange(-HISTORY_LEN / 50.0, 0.0, padding=0)
        for k, name, color, style in (("z", "z ước lượng", "#d62728", QtCore.Qt.SolidLine),
                                      ("z_sp", "z đặt (Alt Hold)", "#ff7f0e", QtCore.Qt.DashLine),
                                      ("tof", "ToF thô (chưa bù nghiêng)", "#7f7f7f", QtCore.Qt.SolidLine),
                                      ("h_terrain", "h_terrain (vật cản)", "#2ca02c", QtCore.Qt.DashLine),
                                      ("baro_rel", "baro (tham khảo)", "#9467bd", QtCore.Qt.DotLine)):
            self.alt_curves[k] = zp.plot(name=name, pen=pg.mkPen(color, width=2, style=style))
        vp = pg.PlotWidget()
        vp.showGrid(x=True, y=True, alpha=0.3)
        vp.setLabel("left", "vz", units="m/s")
        vp.setXLink(zp)
        self.alt_curves["vz"] = vp.plot(pen=pg.mkPen("#1f77b4", width=2))
        for c in self.alt_curves.values():
            c.setClipToView(True)
        lay.addWidget(zp, stretch=3)
        lay.addWidget(vp, stretch=1)
        return w

    def _build_pos_tab(self):
        """Position hold step 2: optical flow velocity (heading frame) and position (room frame). Observation only."""
        self.vel = {k: collections.deque(maxlen=HISTORY_LEN) for k in ("vel_fwd", "vel_right")}
        self.pos_xy = (collections.deque(maxlen=HISTORY_LEN * 3), collections.deque(maxlen=HISTORY_LEN * 3))
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        top = QtWidgets.QHBoxLayout()
        self.pos_label = QtWidgets.QLabel("Chưa có dữ liệu optical flow")
        top.addWidget(self.pos_label)
        top.addStretch(1)
        zero = QtWidgets.QPushButton("Đặt vị trí về 0 (POSZERO)")
        zero.clicked.connect(lambda: (self._send("POSZERO"), [d.clear() for d in self.pos_xy]))
        top.addWidget(zero)
        lay.addLayout(top)
        row = QtWidgets.QHBoxLayout()
        vp = pg.PlotWidget()
        vp.showGrid(x=True, y=True, alpha=0.3)
        vp.setLabel("left", "vận tốc", units="m/s")
        vp.setLabel("bottom", "thời gian (0 = mới nhất)", units="s")
        vp.addLegend(offset=(10, 10))
        vp.setXRange(-HISTORY_LEN / 50.0, 0.0, padding=0)
        self.vel_curves = {
            "vel_fwd": vp.plot(name="tới (+) / lùi", pen=pg.mkPen("#d62728", width=2)),
            "vel_right": vp.plot(name="phải (+) / trái", pen=pg.mkPen("#1f77b4", width=2)),
        }
        xy = pg.PlotWidget()
        xy.showGrid(x=True, y=True, alpha=0.3)
        xy.setAspectLocked(True)
        xy.setLabel("left", "x: hướng mũi lúc khởi động", units="m")
        xy.setLabel("bottom", "y: bên phải lúc khởi động", units="m")
        self.pos_curve = xy.plot(pen=pg.mkPen("#7f7f7f", width=1))
        self.pos_dot = xy.plot(symbol="o", symbolSize=10, symbolBrush="#d62728", pen=None)
        row.addWidget(vp, stretch=3)
        row.addWidget(xy, stretch=2)
        lay.addLayout(row)
        return w

    def _build_gps_tab(self):
        """G1: GPS status and track in metres north/east of a reference point (the first 3D fix, or 'Đặt gốc')."""
        self.gps_track = (collections.deque(maxlen=3000), collections.deque(maxlen=3000))  # north, east (m)
        self.gps_origin = None  # (lat_deg, lon_deg)
        self.gps_last = None
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        top = QtWidgets.QHBoxLayout()
        self.gps_label = QtWidgets.QLabel("Chưa có dòng GPS (G) từ drone")
        self.gps_label.setStyleSheet("font-family: Consolas, monospace; font-size: 16px; font-weight: bold;")
        top.addWidget(self.gps_label)
        top.addStretch(1)
        origin = QtWidgets.QPushButton("Đặt gốc tại vị trí hiện tại")
        origin.clicked.connect(self._gps_set_origin)
        top.addWidget(origin)
        lay.addLayout(top)
        self.gps_diag = QtWidgets.QLabel("")
        self.gps_diag.setStyleSheet("font-family: Consolas, monospace; color: #555;")
        lay.addWidget(self.gps_diag)
        xy = pg.PlotWidget()
        xy.showGrid(x=True, y=True, alpha=0.3)
        xy.setAspectLocked(True)
        xy.setLabel("left", "Bắc", units="m")
        xy.setLabel("bottom", "Đông", units="m")
        self.gps_curve = xy.plot(pen=pg.mkPen("#7f7f7f", width=1))
        self.gps_dot = xy.plot(symbol="o", symbolSize=10, symbolBrush="#d62728", pen=None)
        lay.addWidget(xy)
        return w

    def _gps_set_origin(self):
        if self.gps_last is not None:
            self.gps_origin = self.gps_last
        for d in self.gps_track:
            d.clear()

    def _handle_gps(self, parts):
        if len(parts) != len(G_FIELDS) + 1:
            return
        try:
            g = {k: int(v) for k, v in zip(G_FIELDS, parts[1:])}
        except ValueError:
            return
        fixed = g["fix_ok"] == 1 and g["fix_type"] >= 3 and g["age_ms"] < 1000
        if fixed:
            lat, lon = g["lat_e7"] * 1e-7, g["lon_e7"] * 1e-7
            self.gps_last = (lat, lon)
            if self.gps_origin is None:
                self.gps_origin = (lat, lon)
            lat0, lon0 = self.gps_origin
            north = np.radians(lat - lat0) * EARTH_R
            east = np.radians(lon - lon0) * EARTH_R * np.cos(np.radians(lat0))
            self.gps_track[0].append(north)
            self.gps_track[1].append(east)
            n, e = np.fromiter(self.gps_track[0], dtype=float), np.fromiter(self.gps_track[1], dtype=float)
            self.gps_curve.setData(e, n)
            self.gps_dot.setData(e[-1:], n[-1:])
            pos = f"   N {north:+.1f} m  E {east:+.1f} m"
        else:
            pos = ""
        if g["pvt_count"] == 0:
            state = "KHÔNG CÓ NAV-PVT" if g["bytes"] else "KHÔNG CÓ DỮ LIỆU"
        else:
            state = FIX_TEXT.get(g["fix_type"], str(g["fix_type"])) + (" OK" if g["fix_ok"] else "")
        self.gps_label.setText(
            f"GPS {state}   {g['num_sv']} vệ tinh   sai số ngang {g['h_acc_mm'] / 1000:.1f} m  dọc "
            f"{g['v_acc_mm'] / 1000:.1f} m   vận tốc N {g['vel_n'] / 1000:+.2f} E {g['vel_e'] / 1000:+.2f} "
            f"xuống {g['vel_d'] / 1000:+.2f} m/s (±{g['s_acc'] / 1000:.2f}){pos}")
        self.gps_label.setStyleSheet("font-family: Consolas, monospace; font-size: 16px; font-weight: bold; "
                                     f"color: {'#2e7d32' if fixed else '#d00000'};")
        self.gps_diag.setText(
            f"tuổi {g['age_ms']} ms   pDOP {g['pdop'] / 100:.2f}   cao MSL {g['hmsl_mm'] / 1000:.1f} m   "
            f"bytes {g['bytes']}  NAV-PVT {g['pvt_count']}  lỗi checksum {g['ck_errors']}  dòng NMEA "
            f"{g['nmea_lines']}  cấu hình gửi {g['cfg_sent']} ack {g['acks']} nak {g['naks']}")

    def _build_tune_panel(self):
        box = QtWidgets.QGroupBox("Chỉnh nhanh (Enter trong ô = gửi ô đó; ô vàng = chưa gửi; chưa lưu flash)")
        outer = QtWidgets.QVBoxLayout(box)  # parameter grid above, buttons and capture row below
        grid = QtWidgets.QGridLayout()
        outer.addLayout(grid)
        lay = QtWidgets.QHBoxLayout()
        outer.addLayout(lay)
        self.tune_edits = {}
        per_row = 6
        for i, (name, text) in enumerate(TUNE_PARAMS):
            r, c = divmod(i, per_row)
            grid.addWidget(QtWidgets.QLabel(text), r, 2 * c)
            edit = QtWidgets.QLineEdit()
            edit.setMaximumWidth(80)
            edit.setToolTip(name)
            edit.textEdited.connect(lambda _t, n=name: self._tune_mark(n))
            edit.returnPressed.connect(lambda n=name: self._tune_send([n]))
            grid.addWidget(edit, r, 2 * c + 1)
            self.tune_edits[name] = edit
        send_btn = QtWidgets.QPushButton("Gửi ô vàng (SET)")
        send_btn.clicked.connect(lambda: self._tune_send(list(self.tune_edits)))
        read_btn = QtWidgets.QPushButton("Đọc (LIST)")
        read_btn.clicked.connect(lambda: self._send("LIST"))
        lay.addWidget(send_btn)
        lay.addWidget(read_btn)
        lay.addStretch(1)
        # CAPTURE n: n control cycles (~1060 Hz) averaged per record, 1280 records.
        self.cap_len = QtWidgets.QComboBox()
        for text, n in (("2,4 s (530 Hz)", 2), ("3,6 s (353 Hz)", 3), ("4,8 s (265 Hz)", 4), ("7,2 s (177 Hz)", 6)):
            self.cap_len.addItem(text, n)
        self.cap_len.setCurrentIndex(2)
        self.cap_len.setToolTip("Dài hơn = tần số mẫu thấp hơn: chỉ thấy dao động tới ~1/2 tần số mẫu")
        cap_btn = QtWidgets.QPushButton("Ghi tần số cao (CAPTURE)")
        cap_btn.setToolTip("Bấm rồi gạt cần dứt khoát vài lần; dữ liệu tự tải về và mở cửa sổ phân tích")
        cap_btn.clicked.connect(self._start_capture)
        lay.addWidget(self.cap_len)
        open_btn = QtWidgets.QPushButton("Mở capture cũ")
        open_btn.clicked.connect(self._open_capture_file)
        lay.addWidget(cap_btn)
        lay.addWidget(open_btn)
        self.capture_lines = None  # list while a capture is being received
        self.capture_windows = []  # keep references so the windows stay open
        self.capture_sent_at = None  # time.monotonic() of a CAPTURE not yet answered
        return box

    def _start_capture(self):
        self._send(f"CAPTURE {self.cap_len.currentData()}")
        self.capture_sent_at = time.monotonic()

    def _check_capture_reply(self):
        if self.capture_sent_at is not None and time.monotonic() - self.capture_sent_at > CAPTURE_REPLY_S:
            self.capture_sent_at = None
            self._log("[GCS] Drone không trả lời lệnh CAPTURE (lệnh có thể mất trên đường truyền): bấm lại")

    def _finish_capture(self):
        lines, self.capture_lines = self.capture_lines, None
        if not lines:
            self._log("[GCS] Capture trống, bỏ qua")
            return
        if not lines[0].startswith(("C,i,", "C,dt_us")):
            lines.insert(0, CAPTURE_HEADER)  # header line lost on the radio link: same order as the firmware
        LOG_DIR.mkdir(exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = LOG_DIR / f"capture_{stamp}.csv"
        with open(path, "w", encoding="utf-8", newline="") as f:
            for line in lines:
                f.write(line[2:] + "\n")
        self._log(f"[GCS] Đã lưu capture: {path} ({len(lines) - 1} mẫu)")
        self._show_capture(path)

    def _open_capture_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Mở capture", str(LOG_DIR), "Capture (capture_*.csv)")
        if path:
            self._show_capture(pathlib.Path(path))

    def _show_capture(self, path):
        try:
            with open(path, encoding="utf-8") as f:
                rows = [line.strip().split(",") for line in f if line.strip()]
            data = load_capture(rows[0], rows[1:])
        except (OSError, ValueError, KeyError, IndexError) as exc:
            self._log(f"[GCS] Không đọc được capture {path}: {exc}")
            return
        win = CaptureWindow(data, path.name)
        self.capture_windows.append(win)
        win.show()

    def _tune_mark(self, name):
        edit = self.tune_edits[name]
        changed = edit.text().strip() != self.param_device.get(name)
        edit.setStyleSheet("background: #fff3b0;" if changed else "")

    def _tune_send(self, names):
        sent = 0
        for name in names:
            text = self.tune_edits[name].text().strip()
            if text and text != self.param_device.get(name):
                self._send(f"SET {name} {text}")
                sent += 1
        if sent == 0:
            self._log("[GCS] Không có ô nào thay đổi")

    def _set_paused(self, paused):
        self.pause_btn.setText("Tiếp tục" if paused else "Tạm dừng đồ thị")
        for p in (self.plot, self.rate_plot):
            p.setMouseEnabled(x=paused, y=True)
        if not paused:
            self.plot.setXRange(-HISTORY_LEN / 50.0, 0.0, padding=0)
            self.dirty = True

    def _update_rms(self, t):
        mask = (t >= -RMS_WINDOW_S) & np.fromiter(self.active, dtype=bool)
        n = int(mask.sum())
        if n == 0:
            self.rms_label.setText(f"Sai số bám RMS {RMS_WINDOW_S:.0f} s: chưa có mẫu (cần ARMED, ga > 5%)")
            return
        parts = []
        for meas, sp, _ in RATE_KEYS:
            err = np.fromiter(self.rates[meas], dtype=float)[mask] - np.fromiter(self.rates[sp], dtype=float)[mask]
            parts.append(f"{meas} {np.sqrt(np.mean(err * err)):5.1f}")
        self.rms_label.setText(f"Sai số bám RMS {RMS_WINDOW_S:.0f} s:  " + "  ".join(parts) + f"  °/s  ({n} mẫu)")

    def _update_link_view(self):
        if self.reader is None:
            self.link_label.setText("")
            return
        age = None if self.last_t_rx is None else time.monotonic() - self.last_t_rx
        style = "font-family: Consolas, monospace; font-size: 16px; font-weight: bold; padding: 2px 8px;"
        if self.capture_lines is not None or self.dump_lines is not None:
            # Telemetry is paused by the drone while it sends a capture / black box.
            n = len(self.capture_lines if self.capture_lines is not None else self.dump_lines)
            self.link_label.setText(f"Đang tải dữ liệu... {n} dòng")
            self.link_label.setStyleSheet(style + "color: #1f77b4;")
        elif age is None or age > LINK_STALE_S:
            self.link_label.setText("MẤT DỮ LIỆU" + ("" if age is None else f" {age:.1f} s"))
            self.link_label.setStyleSheet(style + "color: white; background: #d00000;")
        else:
            self.link_label.setText("Dữ liệu OK")
            self.link_label.setStyleSheet(style + "color: #2e7d32;")

    def _build_rc_panel(self):
        box = QtWidgets.QGroupBox("Tay cầm (SBUS) - kênh vừa đổi giá trị sẽ sáng vàng")
        lay = QtWidgets.QVBoxLayout(box)
        self.rc_status = QtWidgets.QLabel("Chưa có dữ liệu tay cầm")
        self.rc_status.setStyleSheet("font-family: Consolas, monospace; font-size: 14px; font-weight: bold;")
        lay.addWidget(self.rc_status)
        grid = QtWidgets.QGridLayout()
        self.rc_bars = []
        self.rc_last = [None] * 16
        self.rc_changed_at = [0.0] * 16
        names = {0: "roll", 1: "pitch", 2: "ga", 3: "yaw"}
        for i in range(16):
            label = QtWidgets.QLabel(f"CH{i + 1}" + (f" ({names[i]})" if i in names else ""))
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 2047)
            bar.setFormat("%v")
            bar.setTextVisible(True)
            bar.setMaximumHeight(18)
            row, col = i % 8, (i // 8) * 2
            grid.addWidget(label, row, col)
            grid.addWidget(bar, row, col + 1)
            self.rc_bars.append(bar)
        lay.addLayout(grid)
        return box

    def _handle_rc(self, parts):
        try:
            v = [int(x) for x in parts[1:]]
        except ValueError:
            return
        if len(v) != R_HEADER_LEN + 16:
            return
        valid, fs, age, fps, bad, errs, lost = v[:7]
        roll, pitch, yaw, thr = (x / 1000.0 for x in v[7:11])
        state = "OK" if valid else ("FAILSAFE (bộ thu báo)" if fs else "MẤT TÍN HIỆU")
        color = "#008000" if valid else "#d00000"
        self.rc_status.setStyleSheet(
            f"font-family: Consolas, monospace; font-size: 14px; font-weight: bold; color: {color};")
        self.rc_status.setText(
            f"{state}  | {fps} khung/s  tuổi {age} ms  lỗi khung {bad}  lỗi UART {errs}  mất khung {lost}"
            f"  | roll {roll:+.2f}  pitch {pitch:+.2f}  yaw {yaw:+.2f}  ga {thr:.2f}")
        now = time.monotonic()
        for i, raw in enumerate(v[R_HEADER_LEN:]):
            if self.rc_last[i] is not None and abs(raw - self.rc_last[i]) >= RC_CHANGE_THRESHOLD:
                self.rc_changed_at[i] = now
            self.rc_last[i] = raw
            bar = self.rc_bars[i]
            bar.setValue(raw)
            hot = now - self.rc_changed_at[i] < RC_HIGHLIGHT_S
            bar.setStyleSheet("QProgressBar::chunk { background: %s; }" % ("#ffcc00" if hot else "#4a90d9"))

    def _build_param_tab(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        btns = QtWidgets.QHBoxLayout()
        read_btn = QtWidgets.QPushButton("Đọc tham số (LIST)")
        read_btn.clicked.connect(lambda: self._send("LIST"))
        write_btn = QtWidgets.QPushButton("Gửi giá trị đã sửa (SET)")
        write_btn.clicked.connect(self._send_param_edits)
        save_btn = QtWidgets.QPushButton("Lưu vào flash (SAVE)")
        save_btn.clicked.connect(self._save_params)
        dump_btn = QtWidgets.QPushButton("Tải hộp đen (DUMP)")
        dump_btn.clicked.connect(self._start_dump)
        for b in (read_btn, write_btn, save_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        btns.addWidget(dump_btn)
        lay.addLayout(btns)
        lay.addWidget(QtWidgets.QLabel(
            "Sửa cột 'Giá trị' rồi bấm Gửi: có hiệu lực ngay (chưa lưu). Lưu vào flash chỉ được khi DISARMED; "
            "CPU dừng ~1-2 s lúc ghi flash."))
        self.param_table = QtWidgets.QTableWidget(0, 4)
        self.param_table.setHorizontalHeaderLabels(["Tên", "Giá trị", "Min", "Max"])
        header = self.param_table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.param_table.itemChanged.connect(self._param_edited)
        lay.addWidget(self.param_table)
        self.param_rows = {}       # name -> row
        self.param_device = {}     # name -> value text last reported by the drone
        self.dump_lines = None     # list while a DUMP is being received
        return w

    def _set_param_row(self, name, value, lo, hi):
        self.param_table.blockSignals(True)
        row = self.param_rows.get(name)
        if row is None:
            row = self.param_table.rowCount()
            self.param_table.insertRow(row)
            self.param_rows[name] = row
            for col, text in ((0, name), (2, lo), (3, hi)):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.param_table.setItem(row, col, item)
            self.param_table.setItem(row, 1, QtWidgets.QTableWidgetItem(value))
        self.param_table.item(row, 1).setText(value)
        self.param_table.item(row, 1).setBackground(QtGui.QColor("white"))
        self.param_device[name] = value
        self.param_table.blockSignals(False)
        edit = self.tune_edits.get(name)
        if edit is not None:
            edit.setText(value)
            edit.setStyleSheet("")

    def _param_edited(self, item):
        if item.column() != 1:
            return
        name = self.param_table.item(item.row(), 0).text()
        changed = item.text().strip() != self.param_device.get(name)
        item.setBackground(QtGui.QColor("#fff3b0" if changed else "white"))  # yellow = not sent yet

    def _send_param_edits(self):
        sent = 0
        for name, row in self.param_rows.items():
            text = self.param_table.item(row, 1).text().strip()
            if text != self.param_device.get(name):
                self._send(f"SET {name} {text}")
                sent += 1
        if sent == 0:
            self._log("[GCS] Không có giá trị nào thay đổi")

    def _save_params(self):
        answer = QtWidgets.QMessageBox.question(
            self, "Lưu tham số", "Ghi tham số hiện tại vào flash?\n(Chỉ khi DISARMED; CPU dừng ~1-2 s.)")
        if answer == QtWidgets.QMessageBox.Yes:
            self._send("SAVE")

    def _start_dump(self):
        self.dump_lines = []
        self._send("DUMP")

    def _finish_dump(self):
        lines, self.dump_lines = self.dump_lines, None
        if not lines:
            return
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"blackbox_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(path, "w", encoding="utf-8", newline="") as f:
            for line in lines:
                f.write(line[2:] + "\n")  # drop the "L," prefix; the first line is the header
        self._log(f"[GCS] Đã lưu hộp đen: {path} ({len(lines) - 1} bản ghi)")

    def _refresh_ports(self):
        self.port_box.clear()
        for p in serial.tools.list_ports.comports():
            self.port_box.addItem(f"{p.device} - {p.description}", p.device)

    def _connection_lost(self, text):
        # The MCU stalls ~1-2 s while SAVE erases flash; Windows then flags the COM port. Reopen it.
        self._log(text)
        if self.reader is None:
            return
        port = self.reader.ser.port
        self.reader.stop()
        self.reader = None
        self.connect_btn.setText("Kết nối")
        self.reconnect_port = port
        self.reconnect_tries = 10
        self._log(f"[GCS] Mất kết nối, tự kết nối lại {port}...")
        QtCore.QTimer.singleShot(1000, self._try_reconnect)

    def _try_reconnect(self):
        if self.reader is not None or self.reconnect_tries <= 0:
            return
        self.reconnect_tries -= 1
        try:
            self.reader = SerialReader(self.reconnect_port, self.rx_queue)
        except serial.SerialException:
            self.reader = None
            if self.reconnect_tries > 0:
                QtCore.QTimer.singleShot(1000, self._try_reconnect)
            else:
                self._log("[GCS] Không kết nối lại được, hãy bấm Kết nối")
            return
        self.reader.start()
        self.connect_btn.setText("Ngắt kết nối")
        self._log(f"[GCS] Đã kết nối lại {self.reconnect_port}")
        if self.log_btn.isChecked():
            self._log(f"[GCS] Tiếp tục ghi log vào file mới: {self.reader.start_log()}")
        self._on_connected()

    def _toggle_log(self, on):
        if on and self.reader is None:
            self._log("[GCS] Hãy kết nối trước rồi mới bắt đầu ghi log")
            self.log_btn.setChecked(False)
            return
        if on:
            self._log(f"[GCS] Bắt đầu ghi log: {self.reader.start_log()}")
        elif self.reader is not None:
            path = self.reader.stop_log()
            if path is not None:
                self._log(f"[GCS] Đã dừng và lưu log: {path}")
        self.log_btn.setText("■ Dừng ghi log (đang ghi)" if on else "● Bắt đầu ghi log")
        self.log_btn.setStyleSheet("QPushButton { background: #c62828; color: white; font-weight: bold; }"
                                   if on else "")

    def _on_connected(self):
        # Over the radio bridge there is no USB DTR edge, so ask for the banner and the parameters.
        self.last_t_rx = None
        self._send("HELLO")
        self._send("LIST")

    def _toggle_connect(self):
        self.reconnect_tries = 0  # a manual connect/disconnect cancels any pending auto-reconnect
        if self.reader is not None:
            if self.log_btn.isChecked():
                self.log_btn.setChecked(False)  # closes and reports the file before the port goes
            self.reader.stop()
            self.reader = None
            self.connect_btn.setText("Kết nối")
            self._log("[GCS] Đã ngắt kết nối")
            return
        port = self.port_box.currentData()
        if not port:
            return
        try:
            self.reader = SerialReader(port, self.rx_queue)
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.warning(self, "Lỗi", f"Không mở được {port}:\n{exc}")
            self.reader = None
            return
        self.reader.start()
        self.connect_btn.setText("Ngắt kết nối")
        self._log(f"[GCS] Kết nối {port} (chưa ghi log: bấm \"Bắt đầu ghi log\")")
        self._on_connected()

    def _send(self, text):
        if self.reader is None:
            self._log("[GCS] Chưa kết nối")
            return
        self.reader.send(text)
        self._log(f"> {text}")

    def _motor_test(self, ch):
        if not self.props_off_confirmed:
            answer = QtWidgets.QMessageBox.warning(
                self, "Thử motor",
                "Motor sẽ quay khoảng 2 giây.\n\nBạn đã THÁO HẾT CÁNH QUẠT chưa?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                return
            self.props_off_confirmed = True  # asked once per GCS session
        self._send(f"MOTOR_TEST {ch} CONFIRM")

    def _send_cmd(self):
        text = self.cmd_edit.text().strip()
        if text:
            self._send(text)
        self.cmd_edit.clear()

    def _log(self, text):
        self.log_view.appendPlainText(text)

    def _handle_line(self, line):
        if line.startswith("[GCS] serial error"):
            self._connection_lost(line)
            return
        if line.startswith("T,"):
            parts = line.split(",")
            if len(parts) != len(T_FIELDS) + 1:
                return
            try:
                t = float(parts[1 + IDX_T]) / 1000.0
                roll = float(parts[1 + IDX_ROLL])
                pitch = float(parts[1 + IDX_PITCH])
                self.last_t = {name: float(v) for name, v in zip(T_FIELDS, parts[1:])}
            except ValueError:
                return
            self.t_s.append(t)
            self.roll.append(roll)
            self.roll_sp.append(self.last_t["roll_sp"])
            self.pitch_sp.append(self.last_t["pitch_sp"])
            for k, dq in self.rates.items():
                dq.append(self.last_t[k])
            self.active.append(bool(self.last_t["arm"]) and self.last_t["thr"] > RMS_MIN_THR)
            for k, dq in self.alt.items():
                # ToF = -1 when the sensor has no fresh in-range reading: plotted as a gap.
                v = self.last_t[k]
                hold_off = k == "z_sp" and int(self.last_t["mode"]) not in HOLD_MODES  # z_sp only while holding
                dq.append(np.nan if (k == "tof" and v < 0) or hold_off else v)
            flow_ok = int(self.last_t["flow_valid"]) == 1
            for k, dq in self.vel.items():
                dq.append(self.last_t[k] if flow_ok else np.nan)
            self.pos_xy[0].append(self.last_t["pos_x"])
            self.pos_xy[1].append(self.last_t["pos_y"])
            self.last_t_rx = time.monotonic()
            self.pitch.append(pitch)
            self.dirty = True
        elif line.startswith("P,"):
            parts = line.split(",")
            if len(parts) == 5:
                self._set_param_row(parts[1], parts[2], parts[3], parts[4])
        elif line.startswith("L,"):
            if self.dump_lines is not None:
                self.dump_lines.append(line)
        elif line.startswith("C,"):
            if self.capture_lines is not None:
                self.capture_lines.append(line)
        elif line.startswith(("OK,CAPTURE,started", "ERR,CAPTURE")):
            self.capture_sent_at = None
            self._log(line)
        elif line.startswith("MSG,CAPTURE sending"):
            self.capture_lines = []
            self._log(line)
        elif line == "OK,CAPTURE,done":
            self._log(line)
            if self.capture_lines is not None:
                self._finish_capture()
        elif line == "OK,DUMP":
            self._finish_dump()
            self._log(line)
        elif line.startswith("ERR,DUMP"):
            self.dump_lines = None
            self._log(line)
        elif line.startswith("R,"):
            self._handle_rc(line.split(","))
        elif line.startswith("G,"):
            self._handle_gps(line.split(","))
        elif line.startswith("TRC,") and not self.show_trc.isChecked():
            return
        elif line.startswith(("A,", "F,")):
            return  # ALTRAW / FLOWLOG lines: only kept in the CSV log, for tools/replay_kf.py / flow_cal.py
        else:
            self._log(line)  # MSG / OK / ERR events only; T lines never go to the text log

    def _on_timer(self):
        for _ in range(2000):  # bounded per tick so the GUI never stalls on a backlog
            try:
                self._handle_line(self.rx_queue.get_nowait())
            except queue.Empty:
                break
        self._update_link_view()
        self._check_capture_reply()
        if not self.dirty or not self.t_s:
            return
        self.dirty = False
        t = np.fromiter(self.t_s, dtype=float)
        t -= t[-1]
        self.roll_label.setText(f"Roll  {self.roll[-1]:+7.2f}°")
        self.pitch_label.setText(f"Pitch {self.pitch[-1]:+7.2f}°")
        self._update_arm_view()
        self._update_rms(t)
        if self.pause_btn.isChecked():
            return  # curves frozen for inspection; numbers above stay live
        self.roll_curve.setData(t, np.fromiter(self.roll, dtype=float))
        self.pitch_curve.setData(t, np.fromiter(self.pitch, dtype=float))
        self.roll_sp_curve.setData(t, np.fromiter(self.roll_sp, dtype=float))
        self.pitch_sp_curve.setData(t, np.fromiter(self.pitch_sp, dtype=float))
        for k, dq in self.rates.items():
            self.rate_curves[k].setData(t, np.fromiter(dq, dtype=float))
        for k, dq in self.alt.items():
            self.alt_curves[k].setData(t, np.fromiter(dq, dtype=float))
        d = self.last_t
        valid = int(d["alt_valid"])
        src = {0: "ToF", 1: "baro", 2: "chỉ dự báo"}.get(int(d["alt_src"]), "?")
        hold = (f"   {MODE_TEXT[int(d['mode'])]} z đặt {d['z_sp']:.2f} m  ga {d['thr']:.2f}"
                if int(d["mode"]) in HOLD_MODES else "")
        self.alt_label.setText(f"z {d['z']:+.3f} m   vz {d['vz']:+.2f} m/s   h_terrain {d['h_terrain']:+.3f} m   "
                               f"nguồn {src}   {'HỢP LỆ' if valid else 'KHÔNG HỢP LỆ'}{hold}")
        self.alt_label.setStyleSheet("font-family: Consolas, monospace; font-size: 18px; font-weight: bold; "
                                     f"color: {'#2e7d32' if valid else '#d00000'};")
        for k, dq in self.vel.items():
            self.vel_curves[k].setData(t, np.fromiter(dq, dtype=float))
        # x north-like (nose at boot) drawn up, y right drawn right
        px, py = np.fromiter(self.pos_xy[0], dtype=float), np.fromiter(self.pos_xy[1], dtype=float)
        self.pos_curve.setData(py, px)
        self.pos_dot.setData(py[-1:], px[-1:])
        fv = int(d["flow_valid"])
        self.pos_label.setText(f"tới {d['vel_fwd']:+.2f} m/s   phải {d['vel_right']:+.2f} m/s   "
                               f"x {d['pos_x']:+.2f} m   y {d['pos_y']:+.2f} m   quality {int(d['flow_q'])}   "
                               f"{'HỢP LỆ' if fv else 'KHÔNG HỢP LỆ'}")
        self.pos_label.setStyleSheet("font-family: Consolas, monospace; font-size: 18px; font-weight: bold; "
                                     f"color: {'#2e7d32' if fv else '#d00000'};")
        for p in (self.plot, self.rate_plot):
            if self.auto_y.isChecked():
                p.enableAutoRange(axis="y")
            else:
                p.disableAutoRange(axis="y")


    def _update_arm_view(self):
        d = getattr(self, "last_t", None)
        if not d:
            return
        big = "font-family: Consolas, monospace; font-size: 34px; font-weight: bold; color: white; padding: 2px 12px;"
        if int(d["arm"]):
            self.arm_label.setText({1: "ARMED GIỮ ĐỘ CAO", 2: "ARMED GIỮ VỊ TRÍ", 3: "ĐANG HẠ CÁNH"}
                                   .get(int(d["mode"]), "ARMED"))
            self.arm_label.setStyleSheet(big + "background: #c62828;")
        else:
            self.arm_label.setText("DISARMED")
            self.arm_label.setStyleSheet(big + "background: #2e7d32;")
        motors = [d[f"m{i}"] for i in range(1, 5)]
        us = [ESC_STOP_US + round(m * ESC_SPAN_US) for m in motors]
        self.motor_label.setText(
            "Motor (us): " + "  ".join(f"M{i + 1}{MOTOR_POS[i]} {u:4d}" for i, u in enumerate(us))
            + f"   | ga lệnh {d['thr']:.2f}   chế độ {MODE_TEXT.get(int(d['mode']), '?')}"
            + FS_TEXT.get(int(d["fs"]), ""))


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
