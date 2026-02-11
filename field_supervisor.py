#!/usr/bin/env python3
# =====================================
# Field Supervisor (single always-on manager)
# - QR control + OLED status always active
# - Starts/stops camera + control-cam + IMU logging on QR commands
# =====================================

import os
import sys
import time
import json
import threading
import subprocess
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Optional, List, Tuple

import cv2

# ----------------------------
# Stable device paths (edit to match wiring)
# ----------------------------
QR_CAM_DEVICE = "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0"

# These are the "main" recording cameras (not the QR cam).
# Add a 4th camera later by appending another by-path video-index0.
MAIN_CAM_DEVICES_DEFAULT = [
    "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-video-index0",
    "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0",
]

# ----------------------------
# QR command mapping
# ----------------------------
QR_STOP  = "1"
QR_START = "2"
QR_RESTART = "3" # restart session (stop then start)
QR_REBOOT = "4"  # reboot device

# ----------------------------
# Logging & rates
# ----------------------------
DEFAULT_OUTROOT = "/home/camrig2/cam_logs"
MAIN_FPS = 4.0                 # main cameras
CONTROL_SAVE_EVERY_S = 0.25     # control cam recording (while session active)
IMU_HZ = 50                    # IMU logging rate (100 is too much)

# QR debounce
QR_SCAN_FPS = 6
QR_CONSEC_REQUIRED = 4
QR_COOLDOWN_S = 8.0

# Camera settings
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
PREFER_MJPEG = True
JPEG_QUALITY = 90

# Frozen frame detection (optional)
STALE_THRESHOLD = 8  # consecutive identical fingerprints -> mark inactive


# ============================================================
# Status model (shared by all threads)
# ============================================================

@dataclass
class DeviceStatus:
    ok: bool = False
    msg: str = "INIT"
    last_update_mono: float = 0.0

@dataclass
class Status:
    # global
    state: str = "IDLE"   # IDLE / RECORDING / ERROR
    session_dir: str = ""
    last_qr: str = ""

    # devices
    control_cam: DeviceStatus = field(default_factory=DeviceStatus)
    imu: DeviceStatus = field(default_factory=DeviceStatus)
    cams: Dict[str, DeviceStatus] = field(default_factory=dict)  # key=cam_name e.g. cam00

    # qr
    qr_progress: float = 0.0   # 0..1
    qr_seen: bool = False      # whether any QR is currently visible

    # misc
    error_msg: str = ""


status_lock = threading.Lock()
i2c_lock = threading.Lock()
status = Status()


def set_status(fn):
    """Decorator-like helper to update status under a lock."""
    def wrapper(*args, **kwargs):
        with status_lock:
            return fn(*args, **kwargs)
    return wrapper


# ============================================================
# OLED (SSD1306) - optional, soft dependency
# ============================================================

class OLEDUI:
    """
    Minimal SSD1306 UI. If OLED libraries aren't available, it prints to stdout.
    We'll flesh this out later (bars, icons, etc.).
    """
    def __init__(self):
        self.enabled = False
        self.disp = None
        self.image = None
        self.draw = None
        self.font = None

        # Try to import OLED libs
        try:
            import board
            import busio
            import adafruit_ssd1306
            from PIL import Image, ImageDraw, ImageFont

            i2c = busio.I2C(board.SCL, board.SDA)
            self.disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)
            self.disp.fill(0)
            self.disp.show()

            self.image = Image.new("1", (self.disp.width, self.disp.height))
            self.draw = ImageDraw.Draw(self.image)
            self.font = ImageFont.load_default()
            self.enabled = True
        except Exception as e:
            print(f"[OLED] Disabled (missing libs or init error): {e}")

    def render(self, snap: Status):
        if not self.enabled:
            return

        # ---- layout constants ----
        W, H = self.disp.width, self.disp.height   # 128x32
        top_h = 8                                  # label band
        pad_x = 2
        gap = 1
        cols = 5
        col_w = (W - 2*pad_x - (cols-1)*gap) // cols  # integer fit
        rect_top = top_h + 5
        rect_bot = H - 2

        print("[OLED] state", snap.state, "cams", {k: v.ok for k, v in snap.cams.items()})

        labels = ["C0", "C1", "C2", "IM", "QR"]

        # Blink: 1Hz when recording
        blink_on = True
        if snap.state == "RECORDING":
            # toggle every 0.5s -> 1Hz blink (on/off)
            blink_on = (int(time.monotonic() * 2) % 2) == 0

        def col_x(i):
            return pad_x + i * (col_w + gap)

        def draw_outline(i):
            x0 = col_x(i)
            x1 = x0 + col_w - 1
            self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255, fill=0)

        def draw_filled(i, blink=False):
            x0 = col_x(i)
            x1 = x0 + col_w - 1
            if blink and not blink_on:
                # blink "off" -> outline only
                self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255, fill=0)
            else:
                self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255, fill=255)

        # ---- clear frame ----
        self.draw.rectangle((0, 0, W, H), outline=0, fill=0)

        # ---- labels ----
        for i, lab in enumerate(labels):
            x0 = col_x(i)
            # center-ish label
            self.draw.text((x0 + 6, 0), lab, font=self.font, fill=255)

        # ---- determine per-column "ok now" ----
        # cam0 = control cam
        cam0_ok = bool(snap.control_cam.ok)

        # cam1/cam2 = main cameras cam00/cam01 in current naming
        # If you later add more main cams, we can map first two here.
        cam_keys = sorted(list(snap.cams.keys()))  # e.g. ["cam00","cam01",...]
        cam1_ok = bool(snap.cams.get(cam_keys[0]).ok) if len(cam_keys) >= 1 else False
        cam2_ok = bool(snap.cams.get(cam_keys[1]).ok) if len(cam_keys) >= 2 else False

        imu_ok = bool(snap.imu.ok)

        # QR column: use qr_seen/progress instead of ok flag
        qr_seen = getattr(snap, "qr_seen", False)
        qr_prog = float(getattr(snap, "qr_progress", 0.0))

        # ---- draw cam0/cam1/cam2/imu columns ----
        # Rule:
        # - ok + RECORDING -> blink filled
        # - ok + not recording -> steady filled
        # - not ok -> outline
        def draw_device(i, ok):
            if not ok:
                draw_outline(i)
            else:
                if snap.state == "RECORDING":
                    draw_filled(i, blink=True)
                else:
                    draw_filled(i, blink=False)

        draw_device(0, cam0_ok)
        draw_device(1, cam1_ok)
        draw_device(2, cam2_ok)
        draw_device(3, imu_ok)

        # ---- draw QR column ----
        x0 = col_x(4)
        x1 = x0 + col_w - 1

        # default: outline only
        self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255, fill=0)

        # If QR is currently visible, fill proportionally bottom-up
        if qr_seen:
            qr_prog = max(0.0, min(1.0, qr_prog))
            full_h = rect_bot - rect_top
            fill_h = int(full_h * qr_prog)

            if fill_h > 0:
                y0 = rect_bot - fill_h
                # Fill inside area (don’t overwrite border)
                self.draw.rectangle((x0+1, y0, x1-1, rect_bot-1), outline=0, fill=255)

            # Re-draw border ONLY (no fill argument)
            self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255)

            # Optional: blink when fully satisfied
            if qr_prog >= 1.0 and (int(time.monotonic() * 4) % 2 == 0):
                self.draw.rectangle((x0+2, rect_top+2, x1-2, rect_bot-2), outline=0, fill=255)
                self.draw.rectangle((x0, rect_top, x1, rect_bot), outline=255)


        # ---- push to display (guard with i2c_lock) ----
        with i2c_lock:
            self.disp.image(self.image)
            self.disp.show()


# ============================================================
# IMU logger
# ============================================================

from smbus2 import SMBus

# ----------------------------
# IMU config (ISM330DHCX)
# ----------------------------
IMU_BUS_NUM = 1
IMU_ISM_ADDR = 0x6B  # change to 0x6A if needed

WHO_AM_I   = 0x0F
CTRL1_XL   = 0x10
CTRL2_G    = 0x11
CTRL3_C    = 0x12

OUT_TEMP_L = 0x20
OUTX_L_G   = 0x22
OUTX_L_A   = 0x28

ACCEL_SCALE = 9.80665 / 16384.0
GYRO_SCALE  = (1.0 / 131.072) * (3.141592653589793 / 180.0)

def to_int16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v

def write_reg(bus, reg, val):
    bus.write_byte_data(IMU_ISM_ADDR, reg, val)

def read_reg(bus, reg):
    return bus.read_byte_data(IMU_ISM_ADDR, reg)

def read_block(bus, start_reg, length):
    return bus.read_i2c_block_data(IMU_ISM_ADDR, start_reg, length)

def init_ism330(bus) -> bool:
    # Probe WHO_AM_I
    who = read_reg(bus, WHO_AM_I)
    # Commonly 0x6B for ISM330DHCX (already verified in imu_test)
    if who not in (0x6B, 0x6A):
        return False

    # Soft reset
    write_reg(bus, CTRL3_C, 0x01)
    time.sleep(0.05)

    # BDU=1 + IF_INC=1 => 0x44
    write_reg(bus, CTRL3_C, 0x44)
    time.sleep(0.01)

    # 104 Hz accel, ±2g
    write_reg(bus, CTRL1_XL, 0x40)

    # 104 Hz gyro, ±250 dps
    write_reg(bus, CTRL2_G, 0x40)

    return True

def read_acc_gyro_temp(bus):
    b = read_block(bus, OUT_TEMP_L, 14)

    temp_raw = to_int16(b[0], b[1])

    gx_raw = to_int16(b[2], b[3])
    gy_raw = to_int16(b[4], b[5])
    gz_raw = to_int16(b[6], b[7])

    ax_raw = to_int16(b[8],  b[9])
    ay_raw = to_int16(b[10], b[11])
    az_raw = to_int16(b[12], b[13])

    ax = ax_raw * ACCEL_SCALE
    ay = ay_raw * ACCEL_SCALE
    az = az_raw * ACCEL_SCALE

    gx = gx_raw * GYRO_SCALE
    gy = gy_raw * GYRO_SCALE
    gz = gz_raw * GYRO_SCALE

    temp_c = 25.0 + (temp_raw / 256.0)
    return ax, ay, az, gx, gy, gz, temp_c


class IMULogger(threading.Thread):
    """
    Logs ISM330DHCX accel/gyro/temp at IMU_HZ while session_active is set.
    Graceful behavior:
      - If IMU not present: IMU:NO, no imu.csv created
      - If IMU present: IMU:OK + imu.csv written
    """
    def __init__(self, session_active: threading.Event, stop_all: threading.Event):
        super().__init__(daemon=True)
        self.session_active = session_active
        self.stop_all = stop_all
        self._stop_session = threading.Event()
        self._session_dir = ""
        self._f = None
        self._bus = None
        self._imu_ready = False

    def start_session(self, session_dir: str):
        self._session_dir = session_dir
        self._stop_session.clear()

    def stop_session(self):
        self._stop_session.set()

    def _close(self):
        try:
            if self._f:
                self._f.flush()
                self._f.close()
        except Exception:
            pass
        self._f = None

        try:
            if self._bus:
                self._bus.close()
        except Exception:
            pass
        self._bus = None
        self._imu_ready = False

    def run(self):
        period = 1.0 / float(IMU_HZ)
        next_t = time.monotonic()

        while not self.stop_all.is_set():
            if not self.session_active.is_set():
                # ensure we are closed if not recording
                if self._f or self._bus:
                    self._close()
                time.sleep(0.1)
                continue

            # Handle stop request even while active
            if self._stop_session.is_set():
                self._close()
                with status_lock:
                    status.imu.ok = False
                    status.imu.msg = "STOPPED"
                    status.imu.last_update_mono = time.monotonic()

                self._stop_session.clear()

                # CRITICAL: don't re-open imu.csv in the same session while supervisor is stopping
                while self.session_active.is_set() and not self.stop_all.is_set():
                    time.sleep(0.1)
                continue

            # Init bus + IMU once per session
            if self._bus is None:
                try:
                    self._bus = SMBus(IMU_BUS_NUM)
                    # IMPORTANT: lock the I2C bus because OLED also uses it
                    with i2c_lock:
                        self._imu_ready = init_ism330(self._bus)
                except Exception:
                    self._imu_ready = False

                if not self._imu_ready:
                    # No IMU present: don’t create imu.csv
                    self._close()
                    with status_lock:
                        status.imu.ok = False
                        status.imu.msg = "NO IMU"
                        status.imu.last_update_mono = time.monotonic()
                    time.sleep(0.5)
                    continue

                # Open file only if IMU is ready
                try:
                    path = os.path.join(self._session_dir, "imu.csv")
                    self._f = open(path, "w", buffering=1)
                    self._f.write("t_mono_ns,ax,ay,az,gx,gy,gz,temp_c\n")
                except Exception as e:
                    self._close()
                    with status_lock:
                        status.imu.ok = False
                        status.imu.msg = f"FILE ERR"
                        status.error_msg = "IMU file"
                        status.state = "ERROR"
                    time.sleep(0.5)
                    continue

                with status_lock:
                    status.imu.ok = True
                    status.imu.msg = "LOGGING"
                    status.imu.last_update_mono = time.monotonic()

            # Pace loop
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.01, next_t - now))
                continue
            next_t += period

            # Read + log
            try:
                t_ns = time.monotonic_ns()
                with i2c_lock:
                    ax, ay, az, gx, gy, gz, temp_c = read_acc_gyro_temp(self._bus)

                self._f.write(f"{t_ns},{ax},{ay},{az},{gx},{gy},{gz},{temp_c}\n")

                with status_lock:
                    status.imu.ok = True
                    status.imu.msg = "LOGGING"
                    status.imu.last_update_mono = time.monotonic()

            except Exception:
                # If read fails transiently, mark bad but keep trying
                with status_lock:
                    status.imu.ok = False
                    status.imu.msg = "READ ERR"
                    status.imu.last_update_mono = time.monotonic()
                time.sleep(0.1)



# ============================================================
# Camera recorder (main cams)
# ============================================================

def frame_fingerprint(frame) -> str:
    import hashlib
    small = cv2.resize(frame, (64, 48), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    return hashlib.md5(gray.tobytes()).hexdigest()


class MainCamLogger(threading.Thread):
    """
    Records frames from MAIN_CAM_DEVICES while session_active is set.
    """
    def __init__(self, session_active: threading.Event, stop_all: threading.Event, cam_devs: List[str]):
        super().__init__(daemon=True)
        self.session_active = session_active
        self.stop_all = stop_all
        self.cam_devs = cam_devs

        self._stop_session = threading.Event()
        self._cams: List[Tuple[str, cv2.VideoCapture]] = []
        self._session_dir = ""
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)]
        self._target_period = 1.0 / max(MAIN_FPS, 0.1)
        self._set_count = 0

        # per cam state
        self._stale_fp: Dict[str, Optional[str]] = {}
        self._stale_count: Dict[str, int] = {}

    def start_session(self, session_dir: str):
        self._session_dir = session_dir
        self._stop_session.clear()
        self._close_cams()

    def stop_session(self):
        self._stop_session.set()

    def _open_cams(self) -> bool:
        self._cams = []
        for dev in self.cam_devs:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            if PREFER_MJPEG:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(FRAME_WIDTH))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(FRAME_HEIGHT))
            ok, fr = cap.read()
            if not ok or fr is None:
                cap.release()
                continue
            self._cams.append((dev, cap))

        # init status entries
        with status_lock:
            status.cams = {}
            for i, (dev, _) in enumerate(self._cams):
                name = f"cam{i:02d}"
                status.cams[name] = DeviceStatus(ok=True, msg="READY", last_update_mono=time.monotonic())
        return len(self._cams) > 0

    def _close_cams(self):
        for _, cap in self._cams:
            cap.release()
        self._cams = []
        self._stale_fp.clear()
        self._stale_count.clear()

    def run(self):
        next_tick = time.monotonic()

        while not self.stop_all.is_set():

            # 1) ALWAYS honor stop request, even if session_active already got cleared
            if self._stop_session.is_set():
                self._close_cams()
                with status_lock:
                    for k in list(status.cams.keys()):
                        status.cams[k].ok = False
                        status.cams[k].msg = "IDLE"
                        status.cams[k].last_update_mono = time.monotonic()
                self._stop_session.clear()
                # don't fall through and reopen in same iteration
                time.sleep(0.05)
                continue

            # 2) If not recording, do nothing
            if not self.session_active.is_set():
                time.sleep(0.1)
                continue

            # 3) Open cams if needed
            if not self._cams:
                ok = self._open_cams()
                if not ok:
                    with status_lock:
                        status.error_msg = "No main cams"
                        status.state = "ERROR"
                    time.sleep(0.5)
                    continue

                # create folders + init stale
                for i, (dev, _) in enumerate(self._cams):
                    cam_name = f"cam{i:02d}"
                    os.makedirs(os.path.join(self._session_dir, cam_name), exist_ok=True)
                    self._stale_fp[cam_name] = None
                    self._stale_count[cam_name] = 0

                self._set_count = 0
                next_tick = time.monotonic()

            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.01, next_tick - now))
                continue
            next_tick += self._target_period

            if self._stop_session.is_set():
                self._close_cams()
                with status_lock:
                    for k in status.cams:
                        status.cams[k].ok = False
                        status.cams[k].msg = "STOPPED"
                        status.cams[k].last_update_mono = time.monotonic()
                self._stop_session.clear()
                continue

            # Grab all
            for _, cap in self._cams:
                cap.grab()

            t_wall = time.time()
            t_ns = time.time_ns()

            saved = 0
            for i, (_, cap) in enumerate(self._cams):
                cam_name = f"cam{i:02d}"
                ok, frame = cap.retrieve()
                if not ok or frame is None:
                    with status_lock:
                        status.cams[cam_name].ok = False
                        status.cams[cam_name].msg = "MISSING"
                        status.cams[cam_name].last_update_mono = time.monotonic()
                    continue

                # stale detect
                fp = frame_fingerprint(frame)
                last_fp = self._stale_fp.get(cam_name)
                if last_fp is not None and fp == last_fp:
                    self._stale_count[cam_name] += 1
                else:
                    self._stale_count[cam_name] = 0
                    self._stale_fp[cam_name] = fp

                if self._stale_count[cam_name] >= STALE_THRESHOLD:
                    with status_lock:
                        status.cams[cam_name].ok = False
                        status.cams[cam_name].msg = "FROZEN"
                        status.cams[cam_name].last_update_mono = time.monotonic()
                    continue

                fname = f"frame_{self._set_count:06d}_{t_wall:.6f}_{cam_name}_{t_ns}.jpg"
                outpath = os.path.join(self._session_dir, cam_name, fname)
                if cv2.imwrite(outpath, frame, self._jpeg_params):
                    saved += 1
                    with status_lock:
                        status.cams[cam_name].ok = True
                        status.cams[cam_name].msg = "LOGGING"
                        status.cams[cam_name].last_update_mono = time.monotonic()

            self._set_count += 1


# ============================================================
# Control cam: QR scanning + optional recording while session active
# ============================================================

class QRControl(threading.Thread):
    def __init__(self, session_active: threading.Event, stop_all: threading.Event, supervisor):
        super().__init__(daemon=True)
        self.session_active = session_active
        self.stop_all = stop_all
        self.supervisor = supervisor

        self._cap = None
        self._detector = cv2.QRCodeDetector()
        self._last_code = None
        self._consec = 0
        self._last_exec = 0.0

        # control-cam recording
        self._control_dir = ""
        self._next_save = 0.0
        self._frame_idx = 0
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)]

    def _open(self):
        cap = cv2.VideoCapture(QR_CAM_DEVICE, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if PREFER_MJPEG:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(FRAME_WIDTH))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(FRAME_HEIGHT))
        ok, fr = cap.read()
        if not ok or fr is None:
            cap.release()
            return None
        return cap

    def set_control_record_dir(self, session_dir: str):
        # called by supervisor on session start
        self._control_dir = os.path.join(session_dir, "control_cam")
        os.makedirs(self._control_dir, exist_ok=True)
        self._next_save = time.monotonic()
        self._frame_idx = 0

    def clear_control_record_dir(self):
        self._control_dir = ""

    def _maybe_save_control(self, frame, now_mono: float):
        if not self.session_active.is_set():
            with status_lock:
                status.control_cam.ok = True
                status.control_cam.msg = "QR_SCAN"
                status.control_cam.last_update_mono = time.monotonic()
            return

        # record control cam only while session is active
        if not self._control_dir:
            return

        if now_mono < self._next_save:
            with status_lock:
                status.control_cam.ok = True
                status.control_cam.msg = "LOGGING"
                status.control_cam.last_update_mono = time.monotonic()
            return

        t_wall = time.time()
        t_ns = time.time_ns()
        fname = f"frame_{self._frame_idx:06d}_{t_wall:.6f}_{t_ns}.jpg"
        outpath = os.path.join(self._control_dir, fname)
        cv2.imwrite(outpath, frame, self._jpeg_params)
        self._frame_idx += 1
        self._next_save = now_mono + CONTROL_SAVE_EVERY_S

        with status_lock:
            status.control_cam.ok = True
            status.control_cam.msg = "LOGGING"
            status.control_cam.last_update_mono = time.monotonic()

    def run(self):
        period = 1.0 / float(QR_SCAN_FPS)
        next_t = time.monotonic()

        print("[QR] thread started")

        while not self.stop_all.is_set():
            # Ensure camera is opened (retry if missing)
            if self._cap is None:
                self._cap = self._open()
                if self._cap is None:
                    with status_lock:
                        status.control_cam.ok = False
                        status.control_cam.msg = "QR CAM MISSING"
                        status.control_cam.last_update_mono = time.monotonic()
                    time.sleep(1.0)
                    continue
                else:
                    with status_lock:
                        status.control_cam.ok = True
                        status.control_cam.msg = "QR_SCAN"
                        status.control_cam.last_update_mono = time.monotonic()

            # Pace loop
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.01, next_t - now))
                continue
            next_t += period

            ok, frame = self._cap.read()
            if not ok or frame is None:
                # If camera went away, drop handle and retry opening
                self._cap.release()
                self._cap = None
                with status_lock:
                    status.control_cam.ok = False
                    status.control_cam.msg = "QR CAM LOST"
                    status.control_cam.last_update_mono = time.monotonic()
                continue

            # Optional: record control cam while session active
            self._maybe_save_control(frame, now)

            # QR detection
            data, _, _ = self._detector.detectAndDecode(frame)
            data = (data or "").strip()

            # Update consecutive count FIRST
            if not data:
                self._last_code = None
                self._consec = 0
                with status_lock:
                    status.qr_seen = False
                    status.qr_progress = 0.0
                continue

            if data == self._last_code:
                self._consec += 1
            else:
                self._last_code = data
                self._consec = 1

            # Now compute progress from the UPDATED count
            prog = min(1.0, float(self._consec) / float(QR_CONSEC_REQUIRED))
            with status_lock:
                status.qr_seen = True
                status.qr_progress = prog
                # print(f"[QR] consec={self._consec} prog={prog:.2f} seen=True data={data!r}")

            # Not enough consecutive reads yet
            if self._consec < QR_CONSEC_REQUIRED:
                continue

            # Cooldown gate (prevents repeated triggers)
            if (now - self._last_exec) < QR_COOLDOWN_S:
                continue

            # Commit: we are triggering an action
            with status_lock:
                status.last_qr = data
                status.qr_seen = False
                status.qr_progress = 0.0

            # Dispatch commands
            if data == QR_START:
                self.supervisor.request_start()
            elif data == QR_STOP:
                self.supervisor.request_stop()
            elif data == QR_RESTART:
                self.supervisor.request_stop()
                time.sleep(0.2)
                self.supervisor.request_start()
            elif data == QR_REBOOT:
                self.supervisor.request_reboot()

            self._last_exec = now
            self._last_code = None
            self._consec = 0




# ============================================================
# Supervisor
# ============================================================

class Supervisor:
    def __init__(self, outroot: str, main_cam_devs: List[str]):
        self.outroot = outroot
        self.main_cam_devs = main_cam_devs

        self.stop_all = threading.Event()
        self.session_active = threading.Event()

        self._want_start = threading.Event()
        self._want_stop = threading.Event()
        self._want_reboot = threading.Event()

        self.oled = OLEDUI()

        # workers
        self.qr = QRControl(self.session_active, self.stop_all, supervisor=self)
        self.main_logger = MainCamLogger(self.session_active, self.stop_all, cam_devs=self.main_cam_devs)
        self.imu_logger = IMULogger(self.session_active, self.stop_all)

        self._ui_thread = threading.Thread(target=self._ui_loop, daemon=True)

    def request_start(self):
        self._want_start.set()

    def request_stop(self):
        self._want_stop.set()

    def request_reboot(self):
        self._want_reboot.set()

    def _new_session_dir(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        d = os.path.join(self.outroot, f"session_{ts}")
        os.makedirs(d, exist_ok=True)
        return d

    def _write_meta(self, session_dir: str):
        meta = {
            "created_wall": time.time(),
            "qr_cam_device": QR_CAM_DEVICE,
            "main_cam_devices": self.main_cam_devs,
            "main_fps": MAIN_FPS,
            "control_save_every_s": CONTROL_SAVE_EVERY_S,
            "imu_hz": IMU_HZ,
        }
        with open(os.path.join(session_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

    def _start_session(self):
        session_dir = self._new_session_dir()
        self._write_meta(session_dir)

        with status_lock:
            status.session_dir = session_dir
            status.state = "RECORDING"
            status.error_msg = ""

        self.session_active.set()

        # Tell modules where to write
        self.qr.set_control_record_dir(session_dir)
        self.main_logger.start_session(session_dir)
        self.imu_logger.start_session(session_dir)

        print(f"[SUP] START session: {session_dir}")

    def _stop_session(self):
        print("[SUP] STOP session")

        # Tell modules to stop + close files FIRST (while session_active is still True)
        self.main_logger.stop_session()
        self.imu_logger.stop_session()
        self.qr.clear_control_record_dir()

        # Give worker threads a moment to process stop (optional but helpful)
        time.sleep(0.2)

        # Now clear the global session flag
        self.session_active.clear()

        with status_lock:
            for ds in status.cams.values():
                ds.ok = False
                ds.msg = "IDLE"
                ds.last_update_mono = time.monotonic()
            status.state = "IDLE"
            status.error_msg = ""
        with status_lock:
            print("[SUP] after STOP status.cams:", {k: v.ok for k, v in status.cams.items()})

    def _ui_loop(self):
        while not self.stop_all.is_set():
            with status_lock:
                snap = Status(
                    state=status.state,
                    session_dir=status.session_dir,
                    last_qr=status.last_qr,
                    control_cam=DeviceStatus(
                        ok=status.control_cam.ok,
                        msg=status.control_cam.msg,
                        last_update_mono=status.control_cam.last_update_mono,
                    ),
                    imu=DeviceStatus(
                        ok=status.imu.ok,
                        msg=status.imu.msg,
                        last_update_mono=status.imu.last_update_mono,
                    ),
                    cams={
                        k: DeviceStatus(ok=v.ok, msg=v.msg, last_update_mono=v.last_update_mono)
                        for k, v in status.cams.items()
                    },
                    qr_progress=status.qr_progress,
                    qr_seen=status.qr_seen,
                    error_msg=status.error_msg,
                )
            self.oled.render(snap)
            time.sleep(0.5)

    def run(self):
        os.makedirs(self.outroot, exist_ok=True)

        # start threads
        self.qr.start()
        self.main_logger.start()
        self.imu_logger.start()
        self._ui_thread.start()

        with status_lock:
            status.state = "IDLE"
            status.error_msg = ""
            status.control_cam.ok = True
            status.control_cam.msg = "QR_SCAN"
            status.imu.ok = False
            status.imu.msg = "STOPPED"

        try:
            while True:
                if self._want_reboot.is_set():
                    print("[SUP] REBOOT requested")
                    # Best-effort: stop session first
                    if self.session_active.is_set():
                        self._stop_session()
                        time.sleep(0.3)
                    subprocess.run(["systemctl", "reboot"], check=False)
                    # subprocess.run(["sudo", "/bin/systemctl", "reboot"], check=False)
                    self._want_reboot.clear()

                if self._want_start.is_set():
                    self._want_start.clear()
                    if not self.session_active.is_set():
                        self._start_session()
                    else:
                        print("[SUP] START ignored (already recording)")

                if self._want_stop.is_set():
                    self._want_stop.clear()
                    if self.session_active.is_set():
                        self._stop_session()
                    else:
                        print("[SUP] STOP ignored (already idle)")

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[SUP] Exiting (Ctrl+C)")
        finally:
            self.stop_all.set()
            if self.session_active.is_set():
                self._stop_session()
            time.sleep(0.2)


def parse_args():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--outroot", type=str, default=DEFAULT_OUTROOT)
    p.add_argument("--main-cams", type=str, default=",".join(MAIN_CAM_DEVICES_DEFAULT),
                   help="Comma-separated by-path devices for main recording cameras (exclude QR cam).")
    return p.parse_args()


def main():
    args = parse_args()
    main_cams = [d.strip() for d in args.main_cams.split(",") if d.strip()]
    sup = Supervisor(outroot=args.outroot, main_cam_devs=main_cams)
    sup.run()


if __name__ == "__main__":
    main()
