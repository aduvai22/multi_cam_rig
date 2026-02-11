#!/usr/bin/env python3
"""
Multi-USB-camera image logger (RPi-friendly, stable by-path selection)

- Records from exactly two cameras (video-index0 nodes), excluding the dedicated QR control camera.
- Uses "grab all, retrieve all" for better inter-camera alignment.
- Detects disconnected/frozen cameras and stops saving for them to avoid wasting storage.
- Onboard ACT LED patterns:
    warmup: fast blink
    logging: slow blink
    error: solid on

Usage:
  python3 multi_cam_logger.py --fps 2 --width 640 --height 480 --outdir /home/camrig2/cam_logs
"""

import os
import sys
import time
import argparse
from datetime import datetime
import hashlib
import atexit

import cv2


# ============================================================
# Stable camera device paths (by-path)
# ============================================================
# Dedicated QR control camera (NOT recorded here)
QR_CONTROL_CAM = "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0"

# Two recording cameras (record these):
#  - H264 camera on port 1.2 (use video-index0)
#  - second icSpring on port 1.3 (use video-index0)
RECORD_CAMS_DEFAULT = [
    "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-video-index0",
    "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0",
]


# ============================================================
# LED helper (ACT)
# ============================================================
LED_NAME = "ACT"
LED_TRIGGER = f"/sys/class/leds/{LED_NAME}/trigger"
LED_BRIGHTNESS = f"/sys/class/leds/{LED_NAME}/brightness"


class PiStatusLED:
    """
    Controls Raspberry Pi ACT LED via /sys/class/leds/ACT/.

    Patterns:
      - warmup: fast blink (~5 Hz)
      - logging: slow blink (~0.25 Hz, i.e. 4s period)
      - error: solid on
    """
    def __init__(self):
        self.available = (os.path.exists(LED_TRIGGER) and os.path.exists(LED_BRIGHTNESS))
        self.manual = False
        self.pattern = "off"
        self.last_toggle = 0.0
        self.state = 0

    def enable_manual(self):
        if not self.available:
            return
        try:
            with open(LED_TRIGGER, "w") as f:
                f.write("none")
            self.manual = True
        except Exception:
            self.available = False

    def _write(self, v: int):
        if not (self.available and self.manual):
            return
        try:
            with open(LED_BRIGHTNESS, "w") as f:
                f.write("1" if v else "0")
        except Exception:
            self.available = False

    def set_pattern(self, name: str):
        self.pattern = name
        if name == "error":
            self._write(1)
        elif name == "off":
            self._write(0)

    def tick(self, now: float):
        if not (self.available and self.manual):
            return

        if self.pattern == "warmup":
            period = 0.2   # fast blink
        elif self.pattern == "logging":
            period = 4.0   # slow blink
        else:
            return

        if (now - self.last_toggle) >= (period / 2.0):
            self.state ^= 1
            self._write(self.state)
            self.last_toggle = now

    def cleanup(self):
        self._write(0)


# ============================================================
# Helpers
# ============================================================
def frame_fingerprint(frame) -> str:
    """
    Small, fast fingerprint to detect frozen/stale frames.
    """
    small = cv2.resize(frame, (64, 48), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    return hashlib.md5(gray.tobytes()).hexdigest()


def open_camera_device(dev: str, width: int, height: int, prefer_mjpeg: bool) -> cv2.VideoCapture | None:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None

    if prefer_mjpeg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    return cap


def make_session_dirs(base_outdir: str, cam_names: list[str]) -> tuple[str, dict[str, str]]:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(base_outdir, f"session_{ts}")
    os.makedirs(session_dir, exist_ok=True)

    cam_dirs: dict[str, str] = {}
    for cam_name in cam_names:
        d = os.path.join(session_dir, cam_name)
        os.makedirs(d, exist_ok=True)
        cam_dirs[cam_name] = d

    return session_dir, cam_dirs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--outdir", type=str, default="/home/camrig2/cam_logs")
    parser.add_argument("--no-mjpeg", action="store_true")
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--duration", type=float, default=0.0)

    # Camera selection
    parser.add_argument(
        "--qr-cam",
        type=str,
        default=QR_CONTROL_CAM,
        help="Dedicated QR control camera device path (excluded from recording)."
    )
    parser.add_argument(
        "--record-cams",
        type=str,
        default=",".join(RECORD_CAMS_DEFAULT),
        help="Comma-separated list of V4L device paths to record (typically 2)."
    )

    # Frozen/disconnect detection
    parser.add_argument("--stale-threshold", type=int, default=8,
                        help="Consecutive identical fingerprints before camera marked inactive.")
    parser.add_argument("--reactivate", action="store_true",
                        help="Occasionally try to reactivate inactive cameras (best effort).")
    parser.add_argument("--reactivate-every", type=int, default=30,
                        help="Try to reactivate inactive cams every N capture sets (only if --reactivate).")

    args = parser.parse_args()

    led = PiStatusLED()
    led.enable_manual()
    atexit.register(led.cleanup)

    prefer_mjpeg = not args.no_mjpeg
    target_period = 1.0 / max(args.fps, 0.1)

    os.makedirs(args.outdir, exist_ok=True)

    # Parse record cams and exclude QR cam if user accidentally included it
    qr_real = os.path.realpath(args.qr_cam)
    record_devs_in = [d.strip() for d in args.record_cams.split(",") if d.strip()]
    record_devs = []
    for d in record_devs_in:
        if os.path.realpath(d) == qr_real:
            continue
        record_devs.append(d)

    if not record_devs:
        led.set_pattern("error")
        print("[ERROR] No recording cameras specified after excluding QR camera.")
        sys.exit(1)

    print("[INFO] QR control camera (excluded):")
    print(f"  - {args.qr_cam} -> {qr_real}")

    print(f"[INFO] Recording camera device(s) requested ({len(record_devs)}):")
    for d in record_devs:
        print(f"  - {d} -> {os.path.realpath(d)}")

    # Open recording cameras
    cams: list[tuple[str, cv2.VideoCapture]] = []
    for cam_i, dev in enumerate(record_devs):
        cap = open_camera_device(dev, args.width, args.height, prefer_mjpeg)
        if cap is None:
            print(f"[WARN] Could not open recording camera: {dev}")
            continue
        cams.append((dev, cap))

    if not cams:
        led.set_pattern("error")
        print("[ERROR] None of the recording cameras could be opened.")
        sys.exit(1)

    # Give friendly stable names cam00, cam01, ...
    cam_names = [f"cam{idx:02d}" for idx in range(len(cams))]
    dev_to_name = {dev: cam_names[i] for i, (dev, _) in enumerate(cams)}

    print(f"[INFO] Opened {len(cams)} recording camera(s):")
    for dev, cap in cams:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"  - {dev_to_name[dev]}: {dev} ({w}x{h})")

    session_dir, cam_dirs = make_session_dirs(args.outdir, cam_names)
    print(f"[INFO] Saving to: {session_dir}")
    print("[INFO] Press Ctrl+C to stop.")

    # Warmup
    led.set_pattern("warmup")
    for _ in range(max(args.warmup, 0)):
        led.tick(time.monotonic())
        for _, cap in cams:
            cap.grab()
        for _, cap in cams:
            cap.retrieve()

    jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpeg_quality)]

    # Per-camera state
    state = {}
    cam_devs = [dev for dev, _ in cams]
    for dev, cap in cams:
        state[dev] = {
            "cap": cap,
            "active": True,
            "last_fp": None,
            "same_fp_count": 0,
            "dropped_sets": 0,
        }

    start_wall = time.time()
    start_mono = time.monotonic()
    set_count = 0
    next_tick = start_mono

    def deactivate(dev, reason):
        st = state[dev]
        if st["active"]:
            st["active"] = False
            st["dropped_sets"] = 0
            print(f"[WARN] {dev_to_name[dev]} marked INACTIVE ({reason}). Will stop saving frames for it.")

    def try_reactivate(dev):
        st = state[dev]
        cap = st["cap"]
        ok, frame = cap.read()
        if ok and frame is not None:
            st["active"] = True
            st["last_fp"] = None
            st["same_fp_count"] = 0
            st["dropped_sets"] = 0
            print(f"[INFO] {dev_to_name[dev]} reactivated.")
        else:
            st["dropped_sets"] += 1

    try:
        led.set_pattern("logging")

        while True:
            now_mono = time.monotonic()
            led.tick(now_mono)

            if args.duration > 0 and (now_mono - start_mono) >= args.duration:
                break

            if now_mono < next_tick:
                time.sleep(min(0.01, next_tick - now_mono))
                continue

            t_wall = time.time()
            t_ns = time.time_ns()

            active_devs = [dev for dev in cam_devs if state[dev]["active"]]
            inactive_devs = [dev for dev in cam_devs if not state[dev]["active"]]

            if not active_devs:
                led.set_pattern("error")
                print("[ERROR] No active cameras left. Stopping.")
                break

            # Grab from all active cameras first
            for dev in active_devs:
                state[dev]["cap"].grab()

            saved_this_set = 0
            for dev in active_devs:
                cap = state[dev]["cap"]
                ok, frame = cap.retrieve()
                if not ok or frame is None:
                    deactivate(dev, "retrieve failed (possibly unplugged)")
                    continue

                # Stale detection
                fp = frame_fingerprint(frame)
                last_fp = state[dev]["last_fp"]
                if last_fp is not None and fp == last_fp:
                    state[dev]["same_fp_count"] += 1
                else:
                    state[dev]["same_fp_count"] = 0
                    state[dev]["last_fp"] = fp

                if state[dev]["same_fp_count"] >= args.stale_threshold:
                    deactivate(dev, f"stale frames x{state[dev]['same_fp_count']}")
                    continue

                cam_name = dev_to_name[dev]
                fname = (
                    f"frame_{set_count:06d}_"
                    f"{t_wall:.6f}_"
                    f"{cam_name}_"
                    f"{t_ns}.jpg"
                )
                outpath = os.path.join(cam_dirs[cam_name], fname)

                if not cv2.imwrite(outpath, frame, jpeg_params):
                    print(f"[WARN] {cam_name}: failed to write {outpath}")
                    continue

                saved_this_set += 1

            if args.reactivate and inactive_devs and (set_count % max(1, args.reactivate_every) == 0):
                for dev in inactive_devs:
                    try_reactivate(dev)

            set_count += 1
            next_tick += target_period

            if set_count % int(max(args.fps, 1)) == 0:
                elapsed = time.time() - start_wall
                active_now = [dev_to_name[d] for d in cam_devs if state[d]["active"]]
                inactive_now = [dev_to_name[d] for d in cam_devs if not state[d]["active"]]
                print(
                    f"[INFO] t={elapsed:.1f}s sets={set_count} "
                    f"saved_frames_last_set={saved_this_set} "
                    f"active={active_now} inactive={inactive_now}"
                )

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
        led.set_pattern("off")
    finally:
        for dev in cam_devs:
            state[dev]["cap"].release()

        elapsed = time.time() - start_wall
        print(f"[INFO] Done. Duration: {elapsed:.1f}s, frame sets saved: {set_count}")
        print(f"[INFO] Output: {session_dir}")


if __name__ == "__main__":
    main()
