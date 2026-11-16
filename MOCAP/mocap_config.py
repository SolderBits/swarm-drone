import json
import os

DEFAULTS = {

    "camera_backend": "opencv",

    "camera_devices": [0, 1, 2, 3],

    "capture_width": 640,
    "capture_height": 480,
    "capture_fps": 120,
    "capture_fourcc": "MJPG",

    "capture_api": "auto",

    "process_width": 640,
    "process_height": 480,

    "display_height": 320,

    "epipolar_threshold_px": None,

    "initial_exposure": 100,
    "initial_gain": 0,

    "dot_threshold": 0.2,

    "serial_port": None,
    "serial_baud": 1000000,

    "num_drones": 2,

    "drone_markers": [
        {"front": 0.110, "side": 0.150},
        {"front": 0.110, "side": 0.150},
    ],

    "marker_tolerance": 0.02,

    "tracking_gate": 0.5,

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
