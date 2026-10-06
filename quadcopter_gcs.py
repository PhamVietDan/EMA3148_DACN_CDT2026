"""
Quadcopter Real-Time Ground Control Station (GCS)
Giao diện Giám sát & Cảnh báo Telemetry ESP-NOW thời gian thực
Tính năng:
- Artificial Horizon (Roll / Pitch / Yaw)
- Optical Flow Vector Radar & Drift Tracker
- Motor Outputs PWM (M1, M2, M3, M4)
- GPS Telemetry (Lat, Lon, Alt, Sats, Speed, HDOP)
- ESP-NOW Link Quality: Cột sóng RSSI (dBm), Tỷ lệ Link Quality %, Packet Interval (ms)
- Hệ thống Cảnh báo Nguy hiểm Đa Tầng (Critical Alarm Banner):
    + Cảnh báo Tín hiệu yếu (Weak RSSI Warning: < -75 dBm, Critical: < -85 dBm)
    + Cảnh báo Mất kết nối / Rớt sóng (Telemetry Signal Lost / Failsafe Watchdog)
    + Cảnh báo Tần số truyền thấp (Low Data Rate < 5 Hz)
    + Cảnh báo Góc nghiêng lật nguy hiểm (Excessive Tilt > 45°)
    + Âm thanh còi báo động (Audio Beeper với nút Mute/Unmute)
    + Nút Test Mô Phỏng Tín Hiệu Yếu / Mất Sóng ngay trong Demo Mode
"""

import sys
import math
import time
import json
import random
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

# Hỗ trợ âm thanh cảnh báo còi trên Windows
try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False


# ==========================================
# CẤU HÌNH GIAO DIỆN & MÀU SẮC (DARK HUD THEME)
# ==========================================
COLOR_BG = "#0b0f19"          # Nền cực tối (Dark slate space)
COLOR_PANEL = "#131c2e"       # Khung panel HUD
COLOR_PANEL_BORDER = "#25334d"# Viền panel
COLOR_TEXT_PRIMARY = "#f8fafc"
COLOR_TEXT_MUTED = "#94a3b8"
COLOR_CYAN = "#06b6d4"        # Điểm nhấn neon cyan
COLOR_SKY = "#38bdf8"
COLOR_GREEN = "#22c55e"
COLOR_YELLOW = "#eab308"
COLOR_ORANGE = "#f97316"
COLOR_RED = "#ef4444"
COLOR_CRITICAL_BG = "#7f1d1d" # Nền đỏ cảnh báo
COLOR_HORIZON_SKY = "#0369a1" # Bầu trời horizon
COLOR_HORIZON_GROUND = "#78350f" # Mặt đất horizon


class AttitudeIndicator(tk.Canvas):
    """
    Widget hiển thị Attitude Indicator (Artificial Horizon)
    Vẽ Roll & Pitch chuẩn hàng không + Thước Yaw ở đáy.
    """
    def __init__(self, parent, size=240, **kwargs):
        super().__init__(
            parent,
            width=size,
            height=size,
            bg=COLOR_PANEL,
            highlightthickness=1,
            highlightbackground=COLOR_PANEL_BORDER,
            **kwargs
        )
        self.size = size
        self.cx = size / 2
        self.cy = size / 2
        self.radius = (size / 2) - 10
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.draw()

    def update_attitude(self, roll, pitch, yaw):
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.draw()

    def draw(self):
        self.delete("all")
        cx, cy, r = self.cx, self.cy, self.radius

        # 1. Vẽ vòng tròn clip mask viền ngoài
        self.create_oval(cx - r - 2, cy - r - 2, cx + r + 2, cy + r + 2, fill="#070a12", outline=COLOR_PANEL_BORDER)

        # 2. Mặt phẳng chân trời xoay theo Roll & Pitch
        pitch_pixels = self.pitch * 2.2
        rad_roll = math.radians(self.roll)
        cos_r = math.cos(rad_roll)
        sin_r = math.sin(rad_roll)

        big_box = r * 2.2
        p1 = (-big_box, -big_box + pitch_pixels)
        p2 = (big_box, -big_box + pitch_pixels)
        p3 = (big_box, pitch_pixels)
        p4 = (-big_box, pitch_pixels)
        p5 = (-big_box, big_box + pitch_pixels)
        p6 = (big_box, big_box + pitch_pixels)

        def rotate(pt):
            x, y = pt
            rx = x * cos_r - y * sin_r + cx
            ry = x * sin_r + y * cos_r + cy
            return rx, ry

        # Đất
        g_pts = [rotate(p4), rotate(p3), rotate(p6), rotate(p5)]
        self.create_polygon([c for pt in g_pts for c in pt], fill=COLOR_HORIZON_GROUND, outline="")

        # Trời
        s_pts = [rotate(p1), rotate(p2), rotate(p3), rotate(p4)]
        self.create_polygon([c for pt in s_pts for c in pt], fill=COLOR_HORIZON_SKY, outline="")

        # Đường chân trời trắng
        h1 = rotate((-big_box, pitch_pixels))
        h2 = rotate((big_box, pitch_pixels))
        self.create_line(h1[0], h1[1], h2[0], h2[1], fill="#ffffff", width=2)

        # Thang chia độ Pitch
        for p in [-30, -20, -10, 10, 20, 30]:
            y_offset = pitch_pixels - (p * 2.2)
            w = 24 if p % 20 == 0 else 14
            line_l = rotate((-w, y_offset))
            line_r = rotate((w, y_offset))
            self.create_line(line_l[0], line_l[1], line_r[0], line_r[1], fill="#e2e8f0", width=1.5)
            txt_pt = rotate((w + 12, y_offset))
            self.create_text(txt_pt[0], txt_pt[1], text=f"{abs(p)}", fill="#cbd5e1", font=("Segoe UI", 7))

        # Viền tròn HUD
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=COLOR_CYAN, width=2)

        # 3. Ký hiệu máy bay cố định ở tâm
        wing_len = 34
        self.create_line(cx - wing_len, cy, cx - 10, cy, fill=COLOR_YELLOW, width=3)
        self.create_line(cx - 10, cy, cx - 10, cy + 7, fill=COLOR_YELLOW, width=3)
        self.create_line(cx + 10, cy, cx + wing_len, cy, fill=COLOR_YELLOW, width=3)
        self.create_line(cx + 10, cy, cx + 10, cy + 7, fill=COLOR_YELLOW, width=3)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=COLOR_YELLOW, outline="")

        # 4. Thước đo góc Roll ở đỉnh
        arc_r = r - 15
        self.create_arc(
            cx - arc_r, cy - arc_r, cx + arc_r, cy + arc_r,
            start=45, extent=90, style=tk.ARC, outline="#64748b", width=1.5
        )
        top_roll_rad = math.radians(self.roll - 90)
        tri_x = cx + (arc_r - 2) * math.cos(top_roll_rad)
        tri_y = cy + (arc_r - 2) * math.sin(top_roll_rad)
        self.create_polygon(
            tri_x, tri_y,
            tri_x - 5 * math.sin(top_roll_rad), tri_y + 5 * math.cos(top_roll_rad),
            tri_x + 5 * math.sin(top_roll_rad), tri_y - 5 * math.cos(top_roll_rad),
            fill=COLOR_CYAN, outline=""
        )

        # 5. Thông số số
        self.create_rectangle(cx - 60, self.size - 28, cx + 60, self.size - 6, fill="#0b0f19", outline="#334155")
        self.create_text(
            cx, self.size - 17,
            text=f"R:{self.roll:+.1f}°  P:{self.pitch:+.1f}°",
            fill=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold")
        )

        self.create_rectangle(cx - 40, 6, cx + 40, 24, fill="#0b0f19", outline="#334155")
        self.create_text(
            cx, 15,
            text=f"HDG {self.yaw:05.1f}°",
            fill=COLOR_CYAN, font=("Consolas", 8, "bold")
        )


class OpticalFlowRadar(tk.Canvas):
    """Hiển thị vector dịch chuyển của Optical Flow (vx, vy, pos_x, pos_y)"""
    def __init__(self, parent, size=200, **kwargs):
        super().__init__(
            parent,
            width=size,
            height=size,
            bg=COLOR_PANEL,
            highlightthickness=1,
            highlightbackground=COLOR_PANEL_BORDER,
            **kwargs
        )
        self.size = size
        self.cx = size / 2
        self.cy = size / 2
        self.vx = 0.0
        self.vy = 0.0
        self.quality = 100
        self.history = []
        self.draw()

    def update_flow(self, vx, vy, quality=100):
        self.vx = float(vx)
        self.vy = float(vy)
        self.quality = int(quality)
        
        new_x = self.history[-1][0] + self.vx * 0.05 if self.history else 0
        new_y = self.history[-1][1] + self.vy * 0.05 if self.history else 0
        limit = 45
        new_x = max(-limit, min(limit, new_x))
        new_y = max(-limit, min(limit, new_y))
        
        self.history.append((new_x, new_y))
        if len(self.history) > 30:
            self.history.pop(0)

        self.draw()

    def reset_track(self):
        self.history.clear()
        self.draw()

    def draw(self):
        self.delete("all")
        cx, cy = self.cx, self.cy
        max_r = (self.size / 2) - 18

        for r_factor in [0.33, 0.66, 1.0]:
            r = max_r * r_factor
            self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#25334d", width=1, dash=(2, 2))

        self.create_line(cx, cy - max_r, cx, cy + max_r, fill="#25334d")
        self.create_line(cx - max_r, cy, cx + max_r, cy, fill="#25334d")

        if len(self.history) > 1:
            points = []
            for hx, hy in self.history:
                px = cx + hx * (max_r / 50.0)
                py = cy - hy * (max_r / 50.0)
                points.extend([px, py])
            self.create_line(points, fill="#0284c7", width=1.5, smooth=True)

        scale_v = 14.0
        tip_x = max(cx - max_r, min(cx + max_r, cx + self.vx * scale_v))
        tip_y = max(cy - max_r, min(cy + max_r, cy - self.vy * scale_v))

        self.create_line(cx, cy, tip_x, tip_y, fill=COLOR_CYAN, width=2, arrow=tk.LAST)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=COLOR_CYAN, outline="")

        if self.history:
            cur_x = cx + self.history[-1][0] * (max_r / 50.0)
            cur_y = cy - self.history[-1][1] * (max_r / 50.0)
            self.create_oval(cur_x - 4, cur_y - 4, cur_x + 4, cur_y + 4, fill=COLOR_GREEN, outline="#ffffff")

        self.create_text(8, 12, anchor="w", text=f"Vx: {self.vx:+.2f} m/s", fill=COLOR_TEXT_PRIMARY, font=("Consolas", 8))
        self.create_text(8, 25, anchor="w", text=f"Vy: {self.vy:+.2f} m/s", fill=COLOR_TEXT_PRIMARY, font=("Consolas", 8))
        
        q_color = COLOR_GREEN if self.quality > 60 else (COLOR_YELLOW if self.quality > 30 else COLOR_RED)
        self.create_text(self.size - 8, 12, anchor="e", text=f"Q: {self.quality}%", fill=q_color, font=("Consolas", 8, "bold"))


class RSSILinkWidget(tk.Frame):
    """
    Widget hiển thị chi tiết chất lượng sóng ESP-NOW Telemetry:
    - RSSI (dBm): Thanh đo tín hiệu và vạch sóng
    - Link Quality (%): Ước tính độ tin cậy liên kết
    - Packet Interval / Ping (ms): Độ trễ giữa các gói
    - Cảnh báo trạng thái tín hiệu
    """
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=COLOR_PANEL, bd=1, relief="solid", **kwargs)
        self.rssi = -60
        self.link_quality = 80
        self.packet_interval_ms = 50

        # Header
        hdr = tk.Frame(self, bg=COLOR_PANEL)
        hdr.pack(fill="x", padx=8, pady=(6, 2))
        tk.Label(hdr, text="ESP-NOW LINK TELEMETRY", bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 9, "bold")).pack(side="left")
        
        self.lbl_link_state = tk.Label(hdr, text="GOOD", bg="#14532d", fg="#86efac", font=("Segoe UI", 7, "bold"), padx=6)
        self.lbl_link_state.pack(side="right")

        content = tk.Frame(self, bg=COLOR_PANEL)
        content.pack(fill="both", expand=True, padx=8, pady=4)

        # Cột trái: Cột sóng trực quan Canvas
        self.canvas_bars = tk.Canvas(content, width=64, height=64, bg="#0b0f19", highlightthickness=1, highlightbackground=COLOR_PANEL_BORDER)
        self.canvas_bars.grid(row=0, column=0, rowspan=2, padx=(0, 10), pady=2)

        # Cột phải: Thông số số liệu
        info_frame = tk.Frame(content, bg=COLOR_PANEL)
        info_frame.grid(row=0, column=1, sticky="w")

        # Hàng 1: RSSI dBm
        r_f = tk.Frame(info_frame, bg=COLOR_PANEL)
        r_f.pack(anchor="w")
        tk.Label(r_f, text="Signal RSSI: ", bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side="left")
        self.lbl_rssi = tk.Label(r_f, text="-60 dBm", bg=COLOR_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 10, "bold"))
        self.lbl_rssi.pack(side="left")

        # Hàng 2: Link Quality %
        q_f = tk.Frame(info_frame, bg=COLOR_PANEL)
        q_f.pack(anchor="w")
        tk.Label(q_f, text="Link Quality: ", bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side="left")
        self.lbl_quality = tk.Label(q_f, text="80 %", bg=COLOR_PANEL, fg=COLOR_GREEN, font=("Consolas", 10, "bold"))
        self.lbl_quality.pack(side="left")

        # Hàng 3: Packet Interval
        p_f = tk.Frame(info_frame, bg=COLOR_PANEL)
        p_f.pack(anchor="w")
        tk.Label(p_f, text="Interval / Lag: ", bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(side="left")
        self.lbl_interval = tk.Label(p_f, text="50 ms", bg=COLOR_PANEL, fg=COLOR_SKY, font=("Consolas", 9, "bold"))
        self.lbl_interval.pack(side="left")

        # Thanh tiến trình ngang RSSI dBm (-100 dBm -> -30 dBm)
        self.canvas_progress = tk.Canvas(self, height=10, bg="#0b0f19", highlightthickness=1, highlightbackground=COLOR_PANEL_BORDER)
        self.canvas_progress.pack(fill="x", padx=8, pady=(2, 6))

        self.redraw()

    def update_link(self, rssi, interval_ms=50):
        self.rssi = int(rssi)
        self.packet_interval_ms = int(interval_ms)

        # Tính toán Link Quality % (từ -100 dBm đến -40 dBm)
        # -40 dBm -> 100%, -100 dBm -> 0%
        pct = int(max(0, min(100, (self.rssi + 100) * (100.0 / 60.0))))
        self.link_quality = pct

        self.lbl_rssi.config(text=f"{self.rssi} dBm")
        self.lbl_quality.config(text=f"{self.link_quality} %")
        self.lbl_interval.config(text=f"{self.packet_interval_ms} ms")

        # Đánh giá trạng thái
        if self.rssi >= -68:
            state_text = "EXCELLENT" if self.rssi >= -55 else "GOOD"
            state_bg = "#14532d"
            state_fg = "#86efac"
            self.lbl_quality.config(fg=COLOR_GREEN)
            self.lbl_rssi.config(fg=COLOR_GREEN)
        elif self.rssi >= -80:
            state_text = "MODERATE"
            state_bg = "#713f12"
            state_fg = "#fde047"
            self.lbl_quality.config(fg=COLOR_YELLOW)
            self.lbl_rssi.config(fg=COLOR_YELLOW)
        elif self.rssi >= -88:
            state_text = "WEAK ⚠️"
            state_bg = "#7c2d12"
            state_fg = "#fdba74"
            self.lbl_quality.config(fg=COLOR_ORANGE)
            self.lbl_rssi.config(fg=COLOR_ORANGE)
        else:
            state_text = "CRITICAL 🚨"
            state_bg = "#7f1d1d"
            state_fg = "#fca5a5"
            self.lbl_quality.config(fg=COLOR_RED)
            self.lbl_rssi.config(fg=COLOR_RED)

        self.lbl_link_state.config(text=state_text, bg=state_bg, fg=state_fg)
        self.redraw()

    def redraw(self):
        # 1. Vẽ 5 vạch sóng trên canvas_bars
        c = self.canvas_bars
        c.delete("all")
        w, h = 64, 64
        bar_count = 5
        bar_width = 8
        spacing = 4
        start_x = 4

        # Số vạch sáng tương ứng quality (0-100)
        active_bars = int(math.ceil((self.link_quality / 100.0) * bar_count))
        if self.link_quality <= 5:
            active_bars = 0

        for i in range(bar_count):
            bx = start_x + i * (bar_width + spacing)
            bh = 10 + i * 11
            by = h - 6 - bh

            if i < active_bars:
                if active_bars >= 4:
                    col = COLOR_GREEN
                elif active_bars >= 3:
                    col = COLOR_YELLOW
                elif active_bars >= 2:
                    col = COLOR_ORANGE
                else:
                    col = COLOR_RED
            else:
                col = "#1e293b" # Vạch tắt xám mờ

            c.create_rectangle(bx, by, bx + bar_width, h - 6, fill=col, outline="")

        # 2. Vẽ thanh tiến trình ngang
        cp = self.canvas_progress
        cp.delete("all")
        pw = cp.winfo_width()
        if pw < 10:
            pw = 200
        
        fill_w = (self.link_quality / 100.0) * pw
        fill_col = COLOR_GREEN if self.link_quality > 60 else (COLOR_YELLOW if self.link_quality > 35 else (COLOR_ORANGE if self.link_quality > 15 else COLOR_RED))
        cp.create_rectangle(0, 0, fill_w, 10, fill=fill_col, outline="")


class MotorPWMWidget(tk.Frame):
    """Hiển thị trạng thái 4 động cơ Quadcopter (M1, M2, M3, M4)"""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=COLOR_PANEL, bd=1, relief="solid", **kwargs)
        self.pwms = [1000, 1000, 1000, 1000]
        
        title_lbl = tk.Label(
            self, text="MOTOR OUTPUTS (PWM)",
            bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 9, "bold")
        )
        title_lbl.pack(pady=(6, 4))

        container = tk.Frame(self, bg=COLOR_PANEL)
        container.pack(fill="both", expand=True, padx=8, pady=4)

        self.canvas_bars = []
        self.val_labels = []
        names = ["M1 (FR)", "M2 (RR)", "M3 (RL)", "M4 (FL)"]
        
        for i in range(4):
            col_frame = tk.Frame(container, bg=COLOR_PANEL)
            col_frame.pack(side="left", fill="both", expand=True, padx=3)

            lbl = tk.Label(col_frame, text="1000", bg=COLOR_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 8, "bold"))
            lbl.pack(side="top")
            self.val_labels.append(lbl)

            c = tk.Canvas(col_frame, width=24, height=110, bg="#0b0f19", highlightthickness=1, highlightbackground=COLOR_PANEL_BORDER)
            c.pack(side="top", pady=2)
            self.canvas_bars.append(c)

            name_lbl = tk.Label(col_frame, text=names[i], bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 7))
            name_lbl.pack(side="top")

        self.redraw()

    def update_motors(self, m1, m2, m3, m4):
        self.pwms = [int(m1), int(m2), int(m3), int(m4)]
        self.redraw()

    def redraw(self):
        c_h = 110
        c_w = 24
        min_pwm, max_pwm = 1000, 2000

        for i in range(4):
            pwm = max(min_pwm, min(max_pwm, self.pwms[i]))
            percent = (pwm - min_pwm) / (max_pwm - min_pwm)
            bar_height = percent * (c_h - 4)

            canvas = self.canvas_bars[i]
            canvas.delete("all")

            if percent < 0.5:
                bar_color = COLOR_GREEN
            elif percent < 0.8:
                bar_color = COLOR_YELLOW
            else:
                bar_color = COLOR_RED

            y_top = c_h - 2 - bar_height
            canvas.create_rectangle(2, y_top, c_w - 2, c_h - 2, fill=bar_color, outline="")

            for frac in [0.25, 0.5, 0.75]:
                y_mark = c_h - 2 - (frac * (c_h - 4))
                canvas.create_line(2, y_mark, c_w - 2, y_mark, fill="#334155", width=1)

            self.val_labels[i].config(text=f"{pwm}")


class GPSWidget(tk.Frame):
    """Hiển thị thông tin định vị vệ tinh GPS"""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=COLOR_PANEL, bd=1, relief="solid", **kwargs)
        
        hdr = tk.Frame(self, bg=COLOR_PANEL)
        hdr.pack(fill="x", padx=8, pady=(6, 2))
        tk.Label(hdr, text="GPS TELEMETRY", bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 9, "bold")).pack(side="left")
        self.lbl_fix = tk.Label(hdr, text="NO FIX", bg="#7f1d1d", fg="#fca5a5", font=("Segoe UI", 7, "bold"), padx=4)
        self.lbl_fix.pack(side="right")

        content = tk.Frame(self, bg=COLOR_PANEL)
        content.pack(fill="both", expand=True, padx=8, pady=4)

        self.fields = {}
        items = [
            ("Latitude:", "0.000000°", 0, 0),
            ("Longitude:", "0.000000°", 0, 1),
            ("Altitude:", "0.0 m", 1, 0),
            ("Satellites:", "0", 1, 1),
            ("Ground Speed:", "0.0 m/s", 2, 0),
            ("HDOP:", "99.9", 2, 1)
        ]

        for label_text, default_val, r, c in items:
            f = tk.Frame(content, bg=COLOR_PANEL)
            f.grid(row=r, column=c, sticky="w", padx=6, pady=1)
            tk.Label(f, text=label_text, bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)).pack(anchor="w")
            v_lbl = tk.Label(f, text=default_val, bg=COLOR_PANEL, fg=COLOR_TEXT_PRIMARY, font=("Consolas", 9, "bold"))
            v_lbl.pack(anchor="w")
            self.fields[label_text] = v_lbl

    def update_gps(self, lat, lon, alt, sats, speed=0.0, hdop=1.2, fix="3D FIX"):
        self.fields["Latitude:"].config(text=f"{lat:.6f}°")
        self.fields["Longitude:"].config(text=f"{lon:.6f}°")
        self.fields["Altitude:"].config(text=f"{alt:.1f} m")
        self.fields["Satellites:"].config(text=f"{int(sats)}")
        self.fields["Ground Speed:"].config(text=f"{speed:.1f} m/s")
        self.fields["HDOP:"].config(text=f"{hdop:.1f}")

        if fix == "3D FIX":
            self.lbl_fix.config(text="3D FIX", bg="#14532d", fg="#86efac")
        elif fix == "2D FIX":
            self.lbl_fix.config(text="2D FIX", bg="#713f12", fg="#fde047")
        else:
            self.lbl_fix.config(text="NO FIX", bg="#7f1d1d", fg="#fca5a5")


# ==========================================
# GIAO DIỆN CHÍNH (MAIN APPLICATION)
# ==========================================
class QuadcopterGCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP-NOW Quadcopter Real-Time Ground Control Station")
        self.root.geometry("1100x740")
        self.root.minsize(1020, 680)
        self.root.configure(bg=COLOR_BG)

        # Trạng thái kết nối & Watchdog
        self.serial_inst = None
        self.is_connected = False
        self.is_simulating = False
        self.read_thread = None
        self.stop_event = threading.Event()

        # Dữ liệu gói tin & Tính toán Link
        self.packets_received = 0
        self.last_packet_time = time.time()
        self.data_rate_hz = 0.0
        self.current_rssi = -60
        self.last_packet_arrival = time.time()
        
        # Cảnh báo & Âm thanh
        self.sound_enabled = True
        self.is_blinking = False
        self.alarm_level = "OK"  # "OK", "WARNING", "CRITICAL", "LOST"
        self.alarm_message = "SYSTEM READY - AWAITING TELEMETRY LINK"
        self.last_beep_time = 0

        # Thiết lập các phần tử giao diện
        self._build_top_bar()
        self._build_alarm_banner()  # Banner cảnh báo nguy hiểm trực quan
        self._build_main_dashboard()
        self._build_status_bar()

        # Timer lặp định kỳ (Watchdog kiểm tra mất sóng & cập nhật Hz)
        self._run_watchdog()

    def _build_top_bar(self):
        """Thanh điều khiển kết nối Serial & Demo Mode"""
        top_frame = tk.Frame(self.root, bg=COLOR_PANEL, bd=1, relief="solid")
        top_frame.pack(fill="x", padx=12, pady=(8, 4))

        # Tiêu đề & Logo
        title_box = tk.Frame(top_frame, bg=COLOR_PANEL)
        title_box.pack(side="left", padx=12, pady=6)
        tk.Label(
            title_box, text="QUADCOPTER GCS HUD",
            bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 12, "bold")
        ).pack(anchor="w")
        tk.Label(
            title_box, text="ESP-NOW Real-time Link Monitor",
            bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)
        ).pack(anchor="w")

        # Khối chọn cổng COM & Baudrate
        conn_box = tk.Frame(top_frame, bg=COLOR_PANEL)
        conn_box.pack(side="left", padx=15, pady=6)

        tk.Label(conn_box, text="Port:", bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9)).grid(row=0, column=0, padx=4)
        self.cb_ports = ttk.Combobox(conn_box, width=11, state="readonly")
        self.cb_ports.grid(row=0, column=1, padx=4)
        self._scan_com_ports()

        btn_refresh = tk.Button(conn_box, text="↻", command=self._scan_com_ports, bg="#334155", fg="white", bd=0, padx=6)
        btn_refresh.grid(row=0, column=2, padx=2)

        tk.Label(conn_box, text="Baud:", bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 9)).grid(row=0, column=3, padx=(10, 4))
        self.cb_baud = ttk.Combobox(conn_box, values=["115200", "230400", "460800", "921600"], width=8, state="readonly")
        self.cb_baud.set("115200")
        self.cb_baud.grid(row=0, column=4, padx=4)

        # Nút Kết nối
        self.btn_connect = tk.Button(
            conn_box, text="CONNECT", command=self._toggle_connect,
            bg=COLOR_CYAN, fg="#0f172a", font=("Segoe UI", 9, "bold"),
            bd=0, padx=12, pady=3, activebackground=COLOR_SKY
        )
        self.btn_connect.grid(row=0, column=5, padx=10)

        # Khối Simulation & Âm thanh
        right_box = tk.Frame(top_frame, bg=COLOR_PANEL)
        right_box.pack(side="right", padx=12, pady=6)

        # Nút Mute/Unmute Beeper
        self.btn_sound = tk.Button(
            right_box, text="🔊 ALARM SOUND: ON", command=self._toggle_sound,
            bg="#334155", fg="white", font=("Segoe UI", 8, "bold"),
            bd=0, padx=8, pady=4
        )
        self.btn_sound.pack(side="left", padx=6)

        # Nút Test giả lập Tín hiệu Yếu / Ngắt sóng
        self.btn_weak_test = tk.Button(
            right_box, text="⚡ SIMULATE WEAK SIGNAL", command=self._force_simulate_weak,
            bg="#b45309", fg="white", font=("Segoe UI", 8, "bold"),
            bd=0, padx=8, pady=4
        )
        self.btn_weak_test.pack(side="left", padx=4)

        # Nút Demo Simulation chính
        self.btn_demo = tk.Button(
            right_box, text="▶ START DEMO", command=self._toggle_simulation,
            bg="#0284c7", fg="white", font=("Segoe UI", 9, "bold"),
            bd=0, padx=12, pady=4, activebackground="#0369a1"
        )
        self.btn_demo.pack(side="left", padx=6)

    def _build_alarm_banner(self):
        """
        Banner cảnh báo khẩn cấp nổi bật ở ngay đầu Dashboard:
        Tự động đổi màu và nhấp nháy khi Telemetry yếu hoặc Mất sóng!
        """
        self.banner_frame = tk.Frame(self.root, bg="#1e293b", bd=2, relief="solid")
        self.banner_frame.pack(fill="x", padx=12, pady=(0, 4))

        self.lbl_alarm_icon = tk.Label(
            self.banner_frame, text="🛡️", bg="#1e293b",
            fg=COLOR_GREEN, font=("Segoe UI", 12)
        )
        self.lbl_alarm_icon.pack(side="left", padx=(10, 4), pady=4)

        self.lbl_alarm_text = tk.Label(
            self.banner_frame, text="LINK OK - TELEMETRY NORMAL",
            bg="#1e293b", fg=COLOR_GREEN, font=("Segoe UI", 10, "bold")
        )
        self.lbl_alarm_text.pack(side="left", pady=4)

        self.lbl_alarm_sub = tk.Label(
            self.banner_frame, text="Watchdog Active (Timeout 1.2s)",
            bg="#1e293b", fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)
        )
        self.lbl_alarm_sub.pack(side="right", padx=12, pady=4)

    def _build_main_dashboard(self):
        """Khung chứa các cụm đồng hồ và bảng điều khiển"""
        dash = tk.Frame(self.root, bg=COLOR_BG)
        dash.pack(fill="both", expand=True, padx=12, pady=4)

        # Cột Trái: Attitude Horizon + Optical Flow Radar
        left_col = tk.Frame(dash, bg=COLOR_BG)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # Panel 1: Attitude Horizon
        att_panel = tk.Frame(left_col, bg=COLOR_PANEL, bd=1, relief="solid")
        att_panel.pack(fill="both", expand=True, pady=(0, 5))
        tk.Label(
            att_panel, text="ARTIFICIAL HORIZON / ATTITUDE",
            bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 9, "bold")
        ).pack(anchor="w", padx=10, pady=(4, 2))
        
        self.att_indicator = AttitudeIndicator(att_panel, size=230)
        self.att_indicator.pack(expand=True, pady=4)

        # Panel 2: Optical Flow
        flow_panel = tk.Frame(left_col, bg=COLOR_PANEL, bd=1, relief="solid")
        flow_panel.pack(fill="both", expand=True)
        
        flow_hdr = tk.Frame(flow_panel, bg=COLOR_PANEL)
        flow_hdr.pack(fill="x", padx=10, pady=(4, 2))
        tk.Label(
            flow_hdr, text="OPTICAL FLOW DRIFT & VELOCITY",
            bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 9, "bold")
        ).pack(side="left")
        
        btn_reset_flow = tk.Button(
            flow_hdr, text="Clear Track", command=lambda: self.flow_radar.reset_track(),
            bg="#334155", fg="white", font=("Segoe UI", 7), bd=0, padx=6
        )
        btn_reset_flow.pack(side="right")

        self.flow_radar = OpticalFlowRadar(flow_panel, size=180)
        self.flow_radar.pack(expand=True, pady=3)

        # Cột Phải: RSSI Link Widget + Motor PWMs + GPS + Raw Log
        right_col = tk.Frame(dash, bg=COLOR_BG)
        right_col.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # Panel 3: ESP-NOW RSSI & Link Quality (MỚI)
        self.rssi_widget = RSSILinkWidget(right_col)
        self.rssi_widget.pack(fill="x", pady=(0, 5))

        # Panel 4: Motor PWMs
        self.motor_widget = MotorPWMWidget(right_col)
        self.motor_widget.pack(fill="x", pady=(0, 5))

        # Panel 5: GPS Telemetry
        self.gps_widget = GPSWidget(right_col)
        self.gps_widget.pack(fill="x", pady=(0, 5))

        # Panel 6: Raw Serial Packet Monitor
        log_panel = tk.Frame(right_col, bg=COLOR_PANEL, bd=1, relief="solid")
        log_panel.pack(fill="both", expand=True)

        log_hdr = tk.Frame(log_panel, bg=COLOR_PANEL)
        log_hdr.pack(fill="x", padx=8, pady=(3, 1))
        tk.Label(
            log_hdr, text="INCOMING SERIAL STREAM (TELEMETRY PACKETS)",
            bg=COLOR_PANEL, fg=COLOR_CYAN, font=("Segoe UI", 8, "bold")
        ).pack(side="left")

        self.txt_log = tk.Text(
            log_panel, height=4, bg="#070a12", fg="#a5f3fc",
            font=("Consolas", 8), bd=0, highlightthickness=0
        )
        self.txt_log.pack(fill="both", expand=True, padx=8, pady=(0, 4))

    def _build_status_bar(self):
        """Thanh trạng thái dưới đáy"""
        sb = tk.Frame(self.root, bg=COLOR_PANEL, bd=1, relief="solid")
        sb.pack(fill="x", side="bottom", padx=12, pady=(2, 8))

        self.lbl_status = tk.Label(
            sb, text="● DISCONNECTED", bg=COLOR_PANEL,
            fg=COLOR_RED, font=("Segoe UI", 9, "bold")
        )
        self.lbl_status.pack(side="left", padx=10, pady=3)

        self.lbl_stats = tk.Label(
            sb, text="Packets: 0 | Rate: 0.0 Hz | RSSI: --- | Formats: JSON / $QUAD",
            bg=COLOR_PANEL, fg=COLOR_TEXT_MUTED, font=("Segoe UI", 8)
        )
        self.lbl_stats.pack(side="right", padx=10, pady=3)

    def _scan_com_ports(self):
        """Quét danh sách cổng COM"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.cb_ports['values'] = ports
        if ports:
            self.cb_ports.current(0)
        else:
            self.cb_ports.set("No COM Port")

    def _toggle_sound(self):
        """Bật/tắt còi cảnh báo"""
        self.sound_enabled = not self.sound_enabled
        if self.sound_enabled:
            self.btn_sound.config(text="🔊 ALARM SOUND: ON", bg="#334155")
        else:
            self.btn_sound.config(text="🔇 ALARM SOUND: MUTED", bg="#78350f")

    def _trigger_audio_beep(self, freq=1200, duration_ms=180):
        """Phát âm thanh bíp cảnh báo trong thread nền"""
        if not self.sound_enabled or not HAS_WINSOUND:
            return
        now = time.time()
        # Giới hạn tần suất kêu không bị chồng tiếng (ít nhất 250ms giữa 2 lần kêu)
        if now - self.last_beep_time < 0.25:
            return
        self.last_beep_time = now

        def _beep():
            try:
                winsound.Beep(freq, duration_ms)
            except:
                pass
        threading.Thread(target=_beep, daemon=True).start()

    def _toggle_connect(self):
        """Kết nối / Ngắt kết nối Serial"""
        if self.is_connected:
            self._disconnect()
        else:
            port = self.cb_ports.get()
            if not port or "No COM" in port:
                messagebox.showwarning("Cảnh báo", "Không tìm thấy cổng COM! Hãy cắm ESP32 Gateway hoặc dùng chế độ 'START DEMO'.")
                return
            baud = int(self.cb_baud.get())
            try:
                self.serial_inst = serial.Serial(port, baud, timeout=1.0)
                self.is_connected = True
                self.btn_connect.config(text="DISCONNECT", bg=COLOR_RED, fg="white")
                self.lbl_status.config(text=f"● CONNECTED [{port} @ {baud}]", fg=COLOR_GREEN)
                self.last_packet_arrival = time.time()
                self.stop_event.clear()
                self.read_thread = threading.Thread(target=self._serial_reader_worker, daemon=True)
                self.read_thread.start()
            except Exception as e:
                messagebox.showerror("Lỗi Serial", str(e))

    def _disconnect(self):
        self.is_connected = False
        self.stop_event.set()
        if self.serial_inst and self.serial_inst.is_open:
            try:
                self.serial_inst.close()
            except:
                pass
        self.btn_connect.config(text="CONNECT", bg=COLOR_CYAN, fg="#0f172a")
        self.lbl_status.config(text="● DISCONNECTED", fg=COLOR_RED)
        self._set_alarm_state("OK", "DISCONNECTED - READY TO CONNECT", COLOR_PANEL, COLOR_TEXT_MUTED)

    def _serial_reader_worker(self):
        """Thread đọc Serial"""
        while not self.stop_event.is_set():
            if self.serial_inst and self.serial_inst.is_open:
                try:
                    line = self.serial_inst.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.root.after(0, self._process_incoming_data, line)
                except Exception:
                    break
            else:
                break

    def _process_incoming_data(self, raw_line):
        """Phân tích dữ liệu gói tin nhận được"""
        now = time.time()
        interval_ms = int((now - self.last_packet_arrival) * 1000)
        self.last_packet_arrival = now
        self.packets_received += 1
        
        # Log vào text window
        self.txt_log.insert("end", raw_line + "\n")
        lines_count = int(self.txt_log.index('end-1c').split('.')[0])
        if lines_count > 40:
            self.txt_log.delete("1.0", "2.0")
        self.txt_log.see("end")

        parsed = None
        try:
            if raw_line.startswith("{") and raw_line.endswith("}"):
                parsed = json.loads(raw_line)
            elif raw_line.startswith("$QUAD"):
                parts = raw_line.split(",")
                # Format: $QUAD,roll,pitch,yaw,flow_x,flow_y,flow_q,m1,m2,m3,m4,lat,lon,alt,sats,rssi
                parsed = {
                    "roll": float(parts[1]),
                    "pitch": float(parts[2]),
                    "yaw": float(parts[3]),
                    "flow_x": float(parts[4]),
                    "flow_y": float(parts[5]),
                    "flow_q": int(parts[6]),
                    "m1": int(parts[7]),
                    "m2": int(parts[8]),
                    "m3": int(parts[9]),
                    "m4": int(parts[10]),
                    "lat": float(parts[11]),
                    "lon": float(parts[12]),
                    "alt": float(parts[13]),
                    "sats": int(parts[14]),
                    "rssi": int(parts[15]) if len(parts) > 15 else -60
                }
        except Exception:
            return

        if parsed:
            # 1. Cập nhật Attitude
            roll = parsed.get("roll", 0.0)
            pitch = parsed.get("pitch", 0.0)
            yaw = parsed.get("yaw", 0.0)
            self.att_indicator.update_attitude(roll, pitch, yaw)

            # 2. Cập nhật Optical Flow
            flow_x = parsed.get("flow_x", 0.0)
            flow_y = parsed.get("flow_y", 0.0)
            flow_q = parsed.get("flow_q", 100)
            self.flow_radar.update_flow(flow_x, flow_y, flow_q)

            # 3. Cập nhật Motor PWM
            m1 = parsed.get("m1", 1000)
            m2 = parsed.get("m2", 1000)
            m3 = parsed.get("m3", 1000)
            m4 = parsed.get("m4", 1000)
            self.motor_widget.update_motors(m1, m2, m3, m4)

            # 4. Cập nhật GPS
            lat = parsed.get("lat", 21.028511)
            lon = parsed.get("lon", 105.804817)
            alt = parsed.get("alt", 0.0)
            sats = parsed.get("sats", 0)
            speed = parsed.get("speed", math.sqrt(flow_x**2 + flow_y**2))
            fix_str = "3D FIX" if sats >= 6 else ("2D FIX" if sats >= 4 else "NO FIX")
            self.gps_widget.update_gps(lat, lon, alt, sats, speed=speed, fix=fix_str)

            # 5. Cập nhật RSSI & Link Quality (ESP-NOW)
            rssi = parsed.get("rssi", -60)
            self.current_rssi = rssi
            self.rssi_widget.update_link(rssi, interval_ms=interval_ms)

            # 6. Kiểm tra các cảnh báo nguy hiểm thời gian thực
            self._evaluate_safety_alerts(roll, pitch, rssi)

    def _evaluate_safety_alerts(self, roll, pitch, rssi):
        """Hệ thống đánh giá an toàn đa tầng"""
        # Kiểm tra lật drone nguy hiểm
        if abs(roll) > 50 or abs(pitch) > 50:
            self._set_alarm_state(
                "CRITICAL",
                f"🚨 DANGER: EXCESSIVE TILT ANGLE! (Roll: {roll:+.1f}°, Pitch: {pitch:+.1f}°)",
                COLOR_CRITICAL_BG, COLOR_RED
            )
            self._trigger_audio_beep(freq=1800, duration_ms=250)
            return

        # Kiểm tra RSSI quá thấp (Nguy cơ bay mất tầm kiểm soát)
        if rssi < -86:
            self._set_alarm_state(
                "CRITICAL",
                f"🚨 CRITICAL TELEMETRY: ESP-NOW SIGNAL SEVERELY DEGRADED ({rssi} dBm)! RISK OF FLYAWAY!",
                COLOR_CRITICAL_BG, COLOR_RED
            )
            self._trigger_audio_beep(freq=1500, duration_ms=200)
        elif rssi < -78:
            self._set_alarm_state(
                "WARNING",
                f"⚠️ WARNING: WEAK ESP-NOW LINK ({rssi} dBm) - FLY CLOSER TO GATEWAY",
                "#78350f", COLOR_YELLOW
            )
            self._trigger_audio_beep(freq=900, duration_ms=120)
        else:
            self._set_alarm_state(
                "OK",
                f"✔ LINK HEALTHY - ESP-NOW SIGNAL STRONG ({rssi} dBm)",
                "#14532d", COLOR_GREEN
            )

    def _set_alarm_state(self, level, message, bg_color, fg_color):
        """Cập nhật hiển thị Banner cảnh báo"""
        self.alarm_level = level
        self.alarm_message = message

        icons = {
            "OK": "🛡️",
            "WARNING": "⚠️",
            "CRITICAL": "🚨",
            "LOST": "❌"
        }
        icon = icons.get(level, "⚠️")

        self.banner_frame.config(bg=bg_color)
        self.lbl_alarm_icon.config(text=icon, bg=bg_color, fg=fg_color)
        self.lbl_alarm_text.config(text=message, bg=bg_color, fg=fg_color)
        self.lbl_alarm_sub.config(bg=bg_color, fg=COLOR_TEXT_PRIMARY)

    def _run_watchdog(self):
        """
        Watchdog chạy mỗi 100ms kiểm tra:
        - Mất kết nối (Failsafe Timeout > 1.2 giây không có gói tin)
        - Nhấp nháy banner khi có cảnh báo đỏ
        """
        now = time.time()
        time_since_packet = now - self.last_packet_arrival

        # Nếu đang kết nối hoặc đang chạy demo mà bị đứt gói quá 1.2s -> Báo động mất sóng
        if (self.is_connected or self.is_simulating) and time_since_packet > 1.2:
            self.alarm_level = "LOST"
            msg = f"🚨 FAILSAFE TRIGGERED! SIGNAL LOST! ({time_since_packet:.1f}s NO PACKET)"
            
            # Hiệu ứng nhấp nháy đỏ chói lọi
            self.is_blinking = not self.is_blinking
            bg = COLOR_CRITICAL_BG if self.is_blinking else "#3f0a0a"
            fg = "#ffffff" if self.is_blinking else COLOR_RED
            self._set_alarm_state("LOST", msg, bg, fg)
            
            # Phát còi báo động khẩn cấp
            self._trigger_audio_beep(freq=2000, duration_ms=150)
            self.rssi_widget.update_link(-105, interval_ms=int(time_since_packet * 1000))

        elif self.alarm_level == "CRITICAL":
            # Nhấp nháy khi ở mức CRITICAL
            self.is_blinking = not self.is_blinking
            bg = COLOR_CRITICAL_BG if self.is_blinking else "#450a0a"
            self.banner_frame.config(bg=bg)
            self.lbl_alarm_icon.config(bg=bg)
            self.lbl_alarm_text.config(bg=bg)
            self.lbl_alarm_sub.config(bg=bg)

        # Cập nhật Data rate mỗi giây
        dt = now - self.last_packet_time
        if dt >= 1.0:
            self.data_rate_hz = self.packets_received / dt
            rssi_str = f"{self.current_rssi} dBm" if (self.is_connected or self.is_simulating) else "---"
            self.lbl_stats.config(
                text=f"Packets: {self.packets_received} | Rate: {self.data_rate_hz:.1f} Hz | RSSI: {rssi_str} | Timeout: 1.2s"
            )
            self.packets_received = 0
            self.last_packet_time = now

        self.root.after(100, self._run_watchdog)

    def _toggle_simulation(self):
        """Bật/tắt chế độ Demo giả lập bay"""
        if self.is_simulating:
            self.is_simulating = False
            self.btn_demo.config(text="▶ START DEMO", bg="#0284c7")
            self.lbl_status.config(text="● DISCONNECTED", fg=COLOR_RED)
            self._set_alarm_state("OK", "DEMO STOPPED - STANDBY", COLOR_PANEL, COLOR_TEXT_MUTED)
        else:
            if self.is_connected:
                self._disconnect()
            self.is_simulating = True
            self.sim_weak_mode = False
            self.btn_demo.config(text="■ STOP DEMO", bg=COLOR_RED)
            self.lbl_status.config(text="● SIMULATION RUNNING (DEMO)", fg=COLOR_CYAN)
            self.last_packet_arrival = time.time()
            self._simulation_step()

    def _force_simulate_weak(self):
        """Kích hoạt tình huống giả lập sóng yếu hoặc mất sóng để kiểm tra hệ thống cảnh báo"""
        if not self.is_simulating:
            self._toggle_simulation()
        
        self.sim_weak_mode = not getattr(self, "sim_weak_mode", False)
        if self.sim_weak_mode:
            self.btn_weak_test.config(text="⚡ RESTORE STRONG SIGNAL", bg="#ef4444")
        else:
            self.btn_weak_test.config(text="⚡ SIMULATE WEAK SIGNAL", bg="#b45309")

    def _simulation_step(self):
        """Hàm sinh dữ liệu bay giả lập mượt mà"""
        if not self.is_simulating:
            return

        t = time.time()

        # Kiểm tra chế độ kiểm thử sóng yếu
        if getattr(self, "sim_weak_mode", False):
            # Tín hiệu suy giảm cực mạnh (-88 dBm đến -95 dBm)
            sim_rssi = -89 + int(4 * math.sin(t * 2.0))
        else:
            # Tín hiệu bình thường mạnh (-52 dBm đến -64 dBm)
            sim_rssi = -58 + int(8 * math.sin(t * 0.4))

        sim_roll = 14.0 * math.sin(t * 1.5)
        sim_pitch = 9.0 * math.cos(t * 1.2)
        sim_yaw = (t * 22.0) % 360.0

        sim_vx = 1.1 * math.sin(t * 0.8) + random.uniform(-0.08, 0.08)
        sim_vy = 0.9 * math.cos(t * 0.9) + random.uniform(-0.08, 0.08)
        sim_q = random.randint(86, 99)

        base_throttle = 1450 + int(120 * math.sin(t * 0.5))
        roll_diff = int(sim_roll * 7)
        pitch_diff = int(sim_pitch * 7)

        m1 = base_throttle - roll_diff - pitch_diff
        m2 = base_throttle - roll_diff + pitch_diff
        m3 = base_throttle + roll_diff + pitch_diff
        m4 = base_throttle + roll_diff - pitch_diff

        radius = 0.0003
        sim_lat = 21.028511 + radius * math.sin(t * 0.3)
        sim_lon = 105.804817 + radius * math.cos(t * 0.3)
        sim_alt = 14.0 + 3.0 * math.sin(t * 0.4)
        sim_sats = 14

        sim_data = {
            "roll": round(sim_roll, 2),
            "pitch": round(sim_pitch, 2),
            "yaw": round(sim_yaw, 2),
            "flow_x": round(sim_vx, 2),
            "flow_y": round(sim_vy, 2),
            "flow_q": sim_q,
            "m1": max(1000, min(2000, m1)),
            "m2": max(1000, min(2000, m2)),
            "m3": max(1000, min(2000, m3)),
            "m4": max(1000, min(2000, m4)),
            "lat": round(sim_lat, 6),
            "lon": round(sim_lon, 6),
            "alt": round(sim_alt, 2),
            "sats": sim_sats,
            "rssi": sim_rssi
        }

        self._process_incoming_data(json.dumps(sim_data))

        # Tần số cập nhật 50ms (~20Hz)
        self.root.after(50, self._simulation_step)

    def on_close(self):
        self.is_simulating = False
        self._disconnect()
        self.root.destroy()


# ==========================================
# MAIN ENTRY POINT
# ==========================================
if __name__ == "__main__":
    root = tk.Tk()
    app = QuadcopterGCSApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
