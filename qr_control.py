#!/usr/bin/env python3
import os
import time
import subprocess
from datetime import datetime

import cv2

# ----------------------------
# Config
# ----------------------------
CONTROL_CAM_DEVICE = "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-video-index0"

SCAN_FPS = 6
WIDTH, HEIGHT = 640, 480
CONSEC_REQUIRED = 5
COMMAND_COOLDOWN_S = 8.0

REC_SERVICE = "multi_cam_logger.service"

QR_ACTIONS = {
    "1": ("STOP_RECORDING",    ["systemctl", "stop", REC_SERVICE]),
    "2": ("START_RECORDING",   ["systemctl", "start", REC_SERVICE]),
    "3": ("RESTART_RECORDING", ["systemctl", "restart", REC_SERVICE]),
    "4": ("REBOOT_PI",         ["systemctl", "reboot"]),
    # "5": ("POWEROFF_PI",       ["systemctl", "poweroff"]),
}

# Control-cam logging (always-on, independent)
ENABLE_CONTROL_LOGGING = True
CONTROL_LOG_ROOT = "/home/camrig2/control_cam_logs"
CONTROL_SAVE_EVERY_S = 1.0      # 1 fps; set to 2.0 for 0.5 fps, etc.
CONTROL_JPEG_QUALITY = 85


def run_cmd(cmd_list):
    subprocess.run(cmd_list, check=False)


def make_control_session_dir(root: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    d = os.path.join(root, f"control_session_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


def main():
    cap = cv2.VideoCapture(CONTROL_CAM_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[QR] ERROR: cannot open control camera: {CONTROL_CAM_DEVICE}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(WIDTH))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(HEIGHT))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    detector = cv2.QRCodeDetector()

    last_code = None
    consec = 0
    last_exec_t = 0.0

    period = 1.0 / max(1.0, SCAN_FPS)
    next_t = time.monotonic()

    # Control logging setup
    control_dir = None
    next_save_t = 0.0
    jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(CONTROL_JPEG_QUALITY)]
    frame_idx = 0

    if ENABLE_CONTROL_LOGGING:
        os.makedirs(CONTROL_LOG_ROOT, exist_ok=True)
        control_dir = make_control_session_dir(CONTROL_LOG_ROOT)
        next_save_t = time.monotonic()
        print(f"[QR] Control-cam logging enabled: {control_dir} (every {CONTROL_SAVE_EVERY_S}s)")

    print("[QR] Running (no arming). Hold QR steady ~1s to trigger.")
    print("[QR] Using camera:", CONTROL_CAM_DEVICE)
    print("[QR] Commands:", {k: v[0] for k, v in QR_ACTIONS.items()})

    while True:
        now = time.monotonic()

        if now < next_t:
            time.sleep(min(0.01, next_t - now))
            continue
        next_t += period

        ok, frame = cap.read()
        if not ok or frame is None:
            continue

        # --- Save control-cam frames at a low rate (optional) ---
        if ENABLE_CONTROL_LOGGING and control_dir is not None and now >= next_save_t:
            t_wall = time.time()
            t_ns = time.time_ns()
            fname = f"frame_{frame_idx:06d}_{t_wall:.6f}_{t_ns}.jpg"
            outpath = os.path.join(control_dir, fname)
            cv2.imwrite(outpath, frame, jpeg_params)
            frame_idx += 1
            next_save_t = now + CONTROL_SAVE_EVERY_S

        # --- QR detection on the same frame ---
        data, points, _ = detector.detectAndDecode(frame)
        data = (data or "").strip()

        if not data:
            last_code = None
            consec = 0
            continue

        if data == last_code:
            consec += 1
        else:
            last_code = data
            consec = 1

        if consec < CONSEC_REQUIRED:
            continue

        if (now - last_exec_t) < COMMAND_COOLDOWN_S:
            continue

        if data in QR_ACTIONS:
            name, cmd = QR_ACTIONS[data]
            print(f"[QR] EXEC: {name} (QR={data})")
            run_cmd(cmd)
            last_exec_t = now

            # Reset so it doesn't re-trigger instantly while still in view
            last_code = None
            consec = 0


if __name__ == "__main__":
    main()
