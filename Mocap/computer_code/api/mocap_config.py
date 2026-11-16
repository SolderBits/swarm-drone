"""Settings for the mocap backend.

Defaults live here; anything in mocap_config.json (same folder) overrides them.
Environment variables override both for the two things you change most:
    MOCAP_SERIAL_PORT   e.g. COM5 or /dev/ttyUSB0, or "none" to run without the ground ESP32
    MOCAP_CAMERA_BACKEND  opencv | ps3eye | synthetic
"""
import json
import os

DEFAULTS = {
    # ---------------- cameras
    # "opencv"    : any UVC webcam, e.g. OV9281 global-shutter USB cameras (default)
    # "ps3eye"    : the original PS3 Eye path through pseyepy
    # "synthetic" : fake cameras that render two drones flying circles (no hardware needed)
    "camera_backend": "opencv",
    # One entry per camera, in the same order as camera-params.json.
    # An int is an OpenCV index; a string is a device path (Linux: /dev/v4l/by-path/...).
    "camera_devices": [0, 1, 2, 3],
    # What to ask the camera for. OV9281 USB boards give 120 fps at 640x480 in MJPG.
    "capture_width": 640,
    "capture_height": 480,
    "capture_fps": 120,
    "capture_fourcc": "MJPG",
    # "auto" picks DSHOW on Windows, V4L2 on Linux, AVFoundation on macOS.
    "capture_api": "auto",
    # Frames are scaled to this before processing. The original PS3 Eye setup used
    # 320x240, but the OV9281 boards have a much wider lens, so the small marker
    # triangle needs 640x480 to stay separate blobs. camera-params.json must be
    # calibrated at the same size (tools/calibrate_intrinsics.py).
    "process_width": 640,
    "process_height": 480,
    # The web UI's camera view is scaled to this height (the original UI is 320 px).
    "display_height": 320,
    # Max distance (px) of a marker from an epipolar line to count as a match.
    # null = 5 px at 320 wide (the original), scaled with process_width.
    "epipolar_threshold_px": None,
    # Exposure/gain the cameras start with, in the web UI's 0-100 units.
    # Exposure is a percentage of the frame time (100 = 8.3 ms at 120 fps).
    "initial_exposure": 100,
    "initial_gain": 0,
    # Brightness threshold for a marker blob, 0-1 of full scale (original: 0.2).
    "dot_threshold": 0.2,

    # ---------------- ground controller (ESP32 on USB)
    # null / "none" runs without it: positions are computed and shown, nothing is sent.
    "serial_port": None,
    "serial_baud": 1000000,

    # ---------------- drones
    "num_drones": 2,
    # Marker triangle per drone, metres. "front" = front LED to each side LED,
    # "side" = side LED to side LED. The swarm frame (v1.1) is 0.110 / 0.150.
    # If every drone has the same triangle, identity comes from tracking
    # (see drone_locator.py); give drones different "front" lengths
    # (at least 2 x marker_tolerance apart) to identify them by shape instead.
    "drone_markers": [
        {"front": 0.110, "side": 0.150},
        {"front": 0.110, "side": 0.150},
    ],
    # How far a measured marker distance may be from the expected one (m).
    "marker_tolerance": 0.02,
    # A tracked drone that jumps more than this between frames is treated as lost (m).
    "tracking_gate": 0.5,
    # Distance between the two markers of the wand used for "determine scale" (m).
    "scale_wand_distance": 0.15,
}


def _load():
    cfg = dict(DEFAULTS)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mocap_config.json")
    if os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        unknown = [k for k in user if k not in DEFAULTS and not k.startswith("_")]
        if unknown:
            print(f"[config] ignoring unknown keys in mocap_config.json: {unknown}")
        cfg.update({k: v for k, v in user.items() if k in DEFAULTS})

    port = os.environ.get("MOCAP_SERIAL_PORT")
    if port is not None:
        cfg["serial_port"] = port
    if isinstance(cfg["serial_port"], str) and cfg["serial_port"].strip().lower() in ("", "none", "null"):
        cfg["serial_port"] = None

    backend = os.environ.get("MOCAP_CAMERA_BACKEND")
    if backend:
        cfg["camera_backend"] = backend

    if len(cfg["drone_markers"]) < cfg["num_drones"]:
        last = cfg["drone_markers"][-1]
        cfg["drone_markers"] = list(cfg["drone_markers"]) + [last] * (cfg["num_drones"] - len(cfg["drone_markers"]))
    return cfg


CONFIG = _load()
