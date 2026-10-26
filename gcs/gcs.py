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
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# Field order of the T line, PLAN §9.1 (angles in deg, lengths in m).
T_FIELDS = [
    "t_ms", "arm", "mode", "fs", "roll", "pitch", "yaw", "p", "q", "r",
    "roll_sp", "pitch_sp", "yawr_sp", "z", "vz", "z_sp", "tof", "thr",
    "m1", "m2", "m3", "m4", "ctrl_us_max", "jitter_us_max", "h_terrain", "baro_rel", "alt_src",
]
IDX_T, IDX_ROLL, IDX_PITCH = T_FIELDS.index("t_ms"), T_FIELDS.index("roll"), T_FIELDS.index("pitch")
FS_TEXT = {0: "", 1: "  FAILSAFE 1 (mất sóng: giữ thăng bằng)", 2: "  FAILSAFE 2"}
MODE_TEXT = {0: "Angle", 1: "Alt Hold", 2: "dự phòng"}
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
    """Reads lines from the port into a queue and appends every line to a CSV log."""

    def __init__(self, port_name, out_queue):
        super().__init__(daemon=True)
        # USB CDC ignores the baud rate; 115200 also works for an HC-05 later.
        self.ser = serial.Serial(port_name, 115200, timeout=0.05)
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        LOG_DIR.mkdir(exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = LOG_DIR / f"log_{stamp}.csv"
        self.log_file = open(self.log_path, "w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["pc_time_s", "line"])

    def run(self):
        buf = b""
        while not self.stop_event.is_set():
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
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.log_writer.writerow([now, line])
                    self.out_queue.put(line)
        self.log_file.close()
        self.ser.close()

    def send(self, text):
        self.ser.write((text.strip() + "\n").encode("ascii"))

    def stop(self):
        self.stop_event.set()


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
        self.yaw_rate = collections.deque(maxlen=HISTORY_LEN)
        self.yaw_rate_sp = collections.deque(maxlen=HISTORY_LEN)
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
        self.tabs.addTab(self._build_param_tab(), "Tham số / Hộp đen")

        bar = QtWidgets.QHBoxLayout()
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(260)
        refresh_btn = QtWidgets.QPushButton("Làm mới")
        refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QtWidgets.QPushButton("Kết nối")
        self.connect_btn.clicked.connect(self._toggle_connect)
        disarm_btn = QtWidgets.QPushButton("DISARM")
        disarm_btn.setMinimumSize(160, 50)
        disarm_btn.setStyleSheet("QPushButton { background: #d00000; color: white; font-size: 20px; font-weight: bold; }")
        disarm_btn.clicked.connect(lambda: self._send("DISARM"))
        for w in (QtWidgets.QLabel("Cổng:"), self.port_box, refresh_btn, self.connect_btn):
            bar.addWidget(w)
        bar.addStretch(1)
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
        self.auto_y = QtWidgets.QCheckBox("Tự co trục Y")
        nums.addWidget(self.auto_y)
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

        self.yaw_plot = pg.PlotWidget()
        self.yaw_plot.showGrid(x=True, y=True, alpha=0.3)
        self.yaw_plot.setLabel("left", "tốc độ yaw", units="deg/s")
        self.yaw_plot.addLegend(offset=(10, 10))
        self.yaw_plot.setXLink(self.plot)
        self.yaw_plot.setYRange(-150.0, 150.0, padding=0)
        self.yaw_plot.setMouseEnabled(x=False, y=True)
        self.yaw_curve = self.yaw_plot.plot(name="r đo", pen=pg.mkPen("#2ca02c", width=2))
        self.yaw_sp_curve = self.yaw_plot.plot(name="r đặt", pen=pg.mkPen("#2ca02c", width=2, style=QtCore.Qt.DashLine))
        for c in (self.roll_curve, self.pitch_curve, self.roll_sp_curve, self.pitch_sp_curve,
                  self.yaw_curve, self.yaw_sp_curve):
            c.setClipToView(True)
            c.setSkipFiniteCheck(True)
        root.addWidget(self.plot, stretch=3)
        root.addWidget(self.yaw_plot, stretch=1)
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
        root.addLayout(test_row)
        self.props_off_confirmed = False
        root.addWidget(self._build_rc_panel())

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
        self._log(f"[GCS] Đã kết nối lại {self.reconnect_port}, ghi log: {self.reader.log_path}")

    def _toggle_connect(self):
        self.reconnect_tries = 0  # a manual connect/disconnect cancels any pending auto-reconnect
        if self.reader is not None:
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
        self._log(f"[GCS] Kết nối {port}, ghi log: {self.reader.log_path}")

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
            self.yaw_rate.append(self.last_t["r"])
            self.yaw_rate_sp.append(self.last_t["yawr_sp"])
            self.pitch.append(pitch)
            self.dirty = True
        elif line.startswith("P,"):
            parts = line.split(",")
            if len(parts) == 5:
                self._set_param_row(parts[1], parts[2], parts[3], parts[4])
        elif line.startswith("L,"):
            if self.dump_lines is not None:
                self.dump_lines.append(line)
        elif line == "OK,DUMP":
            self._finish_dump()
            self._log(line)
        elif line.startswith("ERR,DUMP"):
            self.dump_lines = None
            self._log(line)
        elif line.startswith("R,"):
            self._handle_rc(line.split(","))
        elif line.startswith("TRC,") and not self.show_trc.isChecked():
            return
        else:
            self._log(line)  # MSG / OK / ERR events only; T lines never go to the text log

    def _on_timer(self):
        for _ in range(2000):  # bounded per tick so the GUI never stalls on a backlog
            try:
                self._handle_line(self.rx_queue.get_nowait())
            except queue.Empty:
                break
        if not self.dirty:
            return
        self.dirty = False
        t = np.fromiter(self.t_s, dtype=float)
        t -= t[-1]
        self.roll_curve.setData(t, np.fromiter(self.roll, dtype=float))
        self.pitch_curve.setData(t, np.fromiter(self.pitch, dtype=float))
        self.roll_sp_curve.setData(t, np.fromiter(self.roll_sp, dtype=float))
        self.pitch_sp_curve.setData(t, np.fromiter(self.pitch_sp, dtype=float))
        self.yaw_curve.setData(t, np.fromiter(self.yaw_rate, dtype=float))
        self.yaw_sp_curve.setData(t, np.fromiter(self.yaw_rate_sp, dtype=float))
        self.roll_label.setText(f"Roll  {self.roll[-1]:+7.2f}°")
        self.pitch_label.setText(f"Pitch {self.pitch[-1]:+7.2f}°")
        self._update_arm_view()
        if self.auto_y.isChecked():
            self.plot.enableAutoRange(axis="y")
        else:
            self.plot.disableAutoRange(axis="y")


    def _update_arm_view(self):
        d = getattr(self, "last_t", None)
        if not d:
            return
        big = "font-family: Consolas, monospace; font-size: 34px; font-weight: bold; color: white; padding: 2px 12px;"
        if int(d["arm"]):
            self.arm_label.setText("ARMED")
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
