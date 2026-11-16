"""Find your cameras and check they really run at 120 fps.

    python tools/list_cameras.py            # try OpenCV indices 0-9
    python tools/list_cameras.py --max 6 --seconds 3

For each camera that opens it prints the mode it actually got, measures the
real frame rate, and saves camera_index_<N>.jpg so you can tell which physical
camera is which (cover one lens with your hand and run it again). Put the
indices you want, in order, into "camera_devices" in api/mocap_config.json.

On Linux it also lists the /dev/v4l/by-path names, which don't change when you
replug or reboot; prefer those in camera_devices.
"""
import argparse
import glob
import os
import platform
import sys
import time

import cv2 as cv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera_backend import resolve_api, _fourcc_str  # noqa: E402
from mocap_config import CONFIG  # noqa: E402


def probe(device, api, cfg, seconds, out_dir):
    cap = cv.VideoCapture(device, api)
    if not cap.isOpened():
        return None
    if cfg["capture_fourcc"]:
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*cfg["capture_fourcc"]))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg["capture_width"])
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg["capture_height"])
    cap.set(cv.CAP_PROP_FPS, cfg["capture_fps"])
    ok, frame = cap.read()
    if not ok:
        cap.release()
        return None
    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        ok, f = cap.read()
        if ok:
            n += 1
            frame = f
    measured = n / (time.perf_counter() - t0)
    info = {
        "device": device,
        "mode": f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))} "
                f"@ {cap.get(cv.CAP_PROP_FPS):.0f} fps reported, {_fourcc_str(cap.get(cv.CAP_PROP_FOURCC))!r}",
        "measured_fps": round(measured, 1),
    }
    cap.release()
    name = os.path.join(out_dir, f"camera_index_{device if isinstance(device, int) else os.path.basename(device)}.jpg")
    cv.putText(frame, f"device {device}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    cv.imwrite(name, frame)
    info["snapshot"] = name
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max", type=int, default=10, help="highest OpenCV index to try (exclusive)")
    ap.add_argument("--seconds", type=float, default=2.0, help="how long to measure each camera")
    ap.add_argument("--api", default=CONFIG["capture_api"], help="auto | dshow | msmf | v4l2 | avfoundation | any")
    ap.add_argument("--out", default=".", help="folder for the snapshots")
    args = ap.parse_args()

    api = resolve_api(args.api)
    found = []
    for i in range(args.max):
        info = probe(i, api, CONFIG, args.seconds, args.out)
        if info:
            found.append(info)
            print(f"index {i}: {info['mode']}, measured {info['measured_fps']} fps -> {info['snapshot']}")
            if info["measured_fps"] < 0.8 * CONFIG["capture_fps"]:
                print(f"         slower than {CONFIG['capture_fps']} fps: this may be a laptop webcam, "
                      "or the camera is sharing a USB controller with another camera")
    if not found:
        print("no cameras opened. Unplug/replug them, close other programs using them, or try --api any")
        return

    if platform.system() == "Linux":
        by_path = sorted(glob.glob("/dev/v4l/by-path/*-video-index0"))
        if by_path:
            print("\nstable Linux names (use these in camera_devices):")
            for p in by_path:
                print(f"  {p} -> {os.path.realpath(p)}")

    print("\nsuggested mocap_config.json entry (drop any laptop webcam, keep the order you want):")
    print('  "camera_devices": [' + ", ".join(str(f["device"]) for f in found) + "]")


if __name__ == "__main__":
    main()
