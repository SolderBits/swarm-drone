"""Camera capture for Low-Cost-Mocap without PS3 Eyes.

`open_cameras()` returns an object that behaves like pseyepy's `Camera`
as far as helpers.py is concerned:

    cams.read()          -> (frames, timestamps)   frames: list of HxWx3 uint8
    cams.exposure = [..] / cams.gain = [..]        one value per camera (web UI 0-100 units)
    len(cams.exposure)   -> number of cameras

Backends
--------
opencv     Any UVC webcam through cv2.VideoCapture, one capture thread per camera.
           Written for OV9281 global-shutter USB boards (Waveshare / Arducam):
           asks for 640x480 MJPG at 120 fps and scales to 320x240, which is what
           the rest of the pipeline and the web UI expect.
synthetic  No hardware: renders the IR markers of two drones flying circles,
           seen by a built-in 4-camera rig. Use it to check the UI / frame rate.
ps3eye     Handled in helpers.py (original pseyepy path).

The OV9281 boards have no hardware frame-sync line, so frames from different
cameras can be up to one frame period apart (8 ms at 120 fps). read() waits
until every camera has delivered a new frame, then returns the newest set, and
stats() reports how far apart the frames in a set were.
"""
import math
import platform
import threading
import time

import cv2 as cv
import numpy as np


# --------------------------------------------------------------------------- helpers
def _default_api():
    system = platform.system()
    if system == "Windows":
        return cv.CAP_DSHOW
    if system == "Linux":
        return cv.CAP_V4L2
    if system == "Darwin":
        return cv.CAP_AVFOUNDATION
    return cv.CAP_ANY


_API_NAMES = {
    "auto": None,
    "any": cv.CAP_ANY,
    "dshow": cv.CAP_DSHOW,
    "msmf": cv.CAP_MSMF,
    "v4l2": cv.CAP_V4L2,
    "avfoundation": cv.CAP_AVFOUNDATION,
}


def resolve_api(name):
    name = (name or "auto").lower()
    if name not in _API_NAMES:
        raise ValueError(f"capture_api must be one of {list(_API_NAMES)}")
    api = _API_NAMES[name]
    return _default_api() if api is None else api


def exposure_to_device(pct, fps, api):
    """Web-UI exposure (0-100 % of the frame time) -> value for CAP_PROP_EXPOSURE.

    V4L2 (Linux) takes exposure_time_absolute in 100 us steps.
    DirectShow / Media Foundation (Windows) take log2(seconds), integers -13..-1.
    """
    pct = min(max(float(pct), 1.0), 100.0)
    seconds = pct / 100.0 / float(fps)
    if api == cv.CAP_V4L2:
        return max(1, int(round(seconds * 1e4)))
    if api in (cv.CAP_DSHOW, cv.CAP_MSMF):
        return int(min(-1, max(-13, round(math.log2(seconds)))))
    return None


def _fourcc_str(value):
    v = int(value)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def _to_bgr(frame):
    if frame.ndim == 2:
        return cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv.cvtColor(frame, cv.COLOR_BGRA2BGR)
    return frame


# --------------------------------------------------------------------------- OpenCV
class _CaptureThread(threading.Thread):
    def __init__(self, index, device, cfg, api):
        super().__init__(daemon=True, name=f"cam{index}")
        self.index = index
        self.device = device
        self.cfg = cfg
        self.api = api
        self.out_size = (int(cfg["process_width"]), int(cfg["process_height"]))
        self.cond = threading.Condition()
        self.frame = None
        self.seq = 0
        self.timestamp = 0.0
        self.last_returned = 0
        self.running = True
        self.cap = None
        self._frame_times = []
        self._settings_lock = threading.Lock()
        self._pending = {}
        self.info = "not opened"
        self._open()

    def _open(self):
        cfg = self.cfg
        cap = cv.VideoCapture(self.device, self.api)
        if not cap.isOpened():
            self.cap = None
            self.info = f"camera {self.index} ({self.device!r}) failed to open"
            print(f"[cameras] {self.info}")
            return False
        # FOURCC before size: DirectShow only offers 120 fps in MJPG.
        if cfg["capture_fourcc"]:
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*cfg["capture_fourcc"]))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, int(cfg["capture_width"]))
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(cfg["capture_height"]))
        cap.set(cv.CAP_PROP_FPS, float(cfg["capture_fps"]))
        cap.set(cv.CAP_PROP_BUFFERSIZE, 1)  # newest frame, not a queue of old ones
        self.cap = cap
        w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv.CAP_PROP_FPS)
        fourcc = _fourcc_str(cap.get(cv.CAP_PROP_FOURCC))
        self.info = f"camera {self.index} ({self.device!r}): {w}x{h} @ {fps:.0f} fps, {fourcc!r}, backend {cap.getBackendName()}"
        print(f"[cameras] {self.info}")
        if fps and fps < 0.9 * float(cfg["capture_fps"]):
            print(f"[cameras]   camera {self.index} reports {fps:.0f} fps, not {cfg['capture_fps']}: "
                  "check the format (MJPG) and USB port; it will still work, just slower")
        return True

    # settings are applied from the capture thread so they never race cap.read()
    def request(self, exposure=None, gain=None):
        with self._settings_lock:
            if exposure is not None:
                self._pending["exposure"] = exposure
            if gain is not None:
                self._pending["gain"] = gain

    def _apply_pending(self):
        with self._settings_lock:
            pending, self._pending = self._pending, {}
        if not pending or self.cap is None:
            return
        cap = self.cap
        if "exposure" in pending:
            value = exposure_to_device(pending["exposure"], self.cfg["capture_fps"], self.api)
            if value is None:
                print(f"[cameras] camera {self.index}: this OpenCV backend can't set exposure; set it with the camera's own tool")
            else:
                manual = 1 if self.api == cv.CAP_V4L2 else 0.25
                cap.set(cv.CAP_PROP_AUTO_EXPOSURE, manual)
                ok = cap.set(cv.CAP_PROP_EXPOSURE, value)
                print(f"[cameras] camera {self.index}: exposure {pending['exposure']}% -> {value} "
                      f"({'ok' if ok else 'not accepted'}, reads back {cap.get(cv.CAP_PROP_EXPOSURE)})")
        if "gain" in pending:
            ok = cap.set(cv.CAP_PROP_GAIN, float(pending["gain"]))
            print(f"[cameras] camera {self.index}: gain {pending['gain']} "
                  f"({'ok' if ok else 'not accepted'}, reads back {cap.get(cv.CAP_PROP_GAIN)})")

    def run(self):
        failures = 0
        while self.running:
            self._apply_pending()
            if self.cap is None:
                time.sleep(1.0)
                self._open()
                continue
            ok, frame = self.cap.read()
            now = time.perf_counter()
            if not ok or frame is None:
                failures += 1
                if failures == 50:
                    print(f"[cameras] camera {self.index} stopped delivering frames, reopening")
                    self.cap.release()
                    self.cap = None
                    failures = 0
                time.sleep(0.002)
                continue
            failures = 0
            frame = _to_bgr(frame)
            if (frame.shape[1], frame.shape[0]) != self.out_size:
                frame = cv.resize(frame, self.out_size, interpolation=cv.INTER_AREA)
            with self.cond:
                self.frame = frame
                self.seq += 1
                self.timestamp = now
                self._frame_times.append(now)
                if len(self._frame_times) > 240:
                    del self._frame_times[:120]
                self.cond.notify_all()
        if self.cap is not None:
            self.cap.release()

    def fps(self):
        with self.cond:
            t = self._frame_times[-60:]
        if len(t) < 2:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0])


class OpenCVCameraArray:
    def __init__(self, cfg):
        self.cfg = cfg
        self.api = resolve_api(cfg["capture_api"])
        devices = list(cfg["camera_devices"])
        self.threads = [_CaptureThread(i, d, cfg, self.api) for i, d in enumerate(devices)]
        for t in self.threads:
            t.start()
        self._exposure = [cfg["initial_exposure"]] * len(devices)
        self._gain = [cfg["initial_gain"]] * len(devices)
        for t in self.threads:
            t.request(exposure=cfg["initial_exposure"], gain=cfg["initial_gain"])
        self._period = 1.0 / float(cfg["capture_fps"])
        self._blank = np.zeros((cfg["process_height"], cfg["process_width"], 3), dtype=np.uint8)
        self._spreads = []
        self._warned_missing = set()

    # pseyepy-style attributes ------------------------------------------------
    @property
    def exposure(self):
        return list(self._exposure)

    @exposure.setter
    def exposure(self, values):
        values = self._per_camera(values)
        for t, v in zip(self.threads, values):
            t.request(exposure=v)
        self._exposure = values

    @property
    def gain(self):
        return list(self._gain)

    @gain.setter
    def gain(self, values):
        values = self._per_camera(values)
        for t, v in zip(self.threads, values):
            t.request(gain=v)
        self._gain = values

    def _per_camera(self, values):
        if np.isscalar(values):
            return [values] * len(self.threads)
        values = list(values)
        if len(values) != len(self.threads):
            raise ValueError(f"expected {len(self.threads)} values, got {len(values)}")
        return values

    @property
    def num_cameras(self):
        return len(self.threads)

    # -------------------------------------------------------------------------
    def read(self, timeout=None):
        """Wait (up to ~2 frame periods) for a new frame from every camera, then
        return the newest frame of each. A camera with no frame yet gives black."""
        timeout = 2.5 * self._period if timeout is None else timeout
        deadline = time.perf_counter() + timeout
        frames, stamps = [], []
        for t in self.threads:
            with t.cond:
                remaining = deadline - time.perf_counter()
                if remaining > 0:
                    t.cond.wait_for(lambda: t.seq > t.last_returned, timeout=remaining)
                frame = t.frame
                t.last_returned = t.seq
                stamps.append(t.timestamp)
            if frame is None:
                if t.index not in self._warned_missing:
                    print(f"[cameras] no frames yet from camera {t.index}")
                    self._warned_missing.add(t.index)
                frame = self._blank.copy()
            frames.append(frame)
        live = [s for s in stamps if s > 0]
        if len(live) > 1:
            self._spreads.append(max(live) - min(live))
            if len(self._spreads) > 240:
                del self._spreads[:120]
        return frames, stamps

    def stats(self):
        spread = np.percentile(self._spreads[-120:], 95) * 1000 if self._spreads else float("nan")
        return {"fps": [round(t.fps(), 1) for t in self.threads], "sync_spread_ms_p95": round(float(spread), 2)}

    def end(self):
        for t in self.threads:
            t.running = False
        for t in self.threads:
            t.join(timeout=1.0)

    close = end


# --------------------------------------------------------------------------- synthetic
def look_at(position, target, up=(0, 0, 1)):
    """World->camera rotation for an OpenCV camera (x right, y down, z forward)."""
    position = np.asarray(position, float)
    z = np.asarray(target, float) - position
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    t = -R @ position
    return R, t


def swarm_marker_points(center, yaw, front=0.110, side=0.150):
    """World positions of the three LEDs of one drone. yaw = 0 points the front LED along +x."""
    half = side / 2.0
    apex = math.sqrt(front ** 2 - half ** 2)
    body = np.array([[apex, 0.0, 0.0], [0.0, half, 0.0], [0.0, -half, 0.0]])  # x forward, y left
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return (Rz @ body.T).T + np.asarray(center, float)


class SyntheticRig:
    """Cameras on a ring (default 4 on a 2 m radius, 2 m up), looking at a point 0.6 m above the origin."""

    def __init__(self, width=640, height=480, focal=None, num_cameras=4, radius=2.0, height_m=2.0):
        self.width, self.height = width, height
        self.square = max(width, height)
        if focal is None:
            focal = 0.44 * width  # roughly the wide OV9281 USB-board lens (~100 deg across)
        self.K = np.array([[focal, 0, self.square / 2], [0, focal, self.square / 2], [0, 0, 1]], float)
        self.poses = []
        for i in range(num_cameras):
            a = 2 * math.pi * i / num_cameras + math.pi / 4
            R, t = look_at((radius * math.cos(a), radius * math.sin(a), height_m), (0, 0, 0.6))
            self.poses.append((R, t))

    def project(self, cam, points):
        """Pixel coordinates in the raw (un-squared) frame."""
        R, t = self.poses[cam]
        pc = (R @ np.asarray(points, float).T).T + t
        uv = (self.K @ pc.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        uv[:, 1] -= (self.square - self.height) / 2  # undo make_square's vertical padding
        uv[:, 0] -= (self.square - self.width) / 2
        return uv, pc[:, 2]


def default_scene(t):
    """Two swarm drones flying slow circles 0.6 m apart, yawing a little."""
    pts = []
    for d, phase in enumerate((0.0, math.pi)):
        a = 0.4 * t + phase
        center = (0.3 * math.cos(a), 0.3 * math.sin(a), 0.6 + 0.1 * math.sin(0.7 * t + d))
        pts.extend(swarm_marker_points(center, yaw=0.2 * math.sin(0.5 * t + d)))
    return np.array(pts)


class SyntheticCameraArray:
    def __init__(self, cfg, scene=default_scene, rig=None):
        self.cfg = cfg
        w, h = int(cfg["process_width"]), int(cfg["process_height"])
        n = len(cfg["camera_devices"])
        self.rig = rig or SyntheticRig(w, h, num_cameras=n)
        self.scene = scene
        self.fps = float(cfg["capture_fps"])
        self.exposure = [cfg["initial_exposure"]] * n
        self.gain = [cfg["initial_gain"]] * n
        self.t0 = time.perf_counter()
        self.sim_time = None  # set to a number to freeze time (tests)
        self._last = 0.0
        yy, xx = np.mgrid[-3:4, -3:4]
        self._blob = np.exp(-(xx ** 2 + yy ** 2) / (2 * 1.5 ** 2))

    @property
    def num_cameras(self):
        return len(self.rig.poses)

    def render(self, t):
        pts = self.scene(t)
        frames = []
        w, h = self.rig.width, self.rig.height
        for cam in range(self.num_cameras):
            img = np.zeros((h, w), np.float32)
            uv, depth = self.rig.project(cam, pts)
            for (u, v), z in zip(uv, depth):
                if z <= 0:
                    continue
                cx, cy = int(round(u)), int(round(v))
                if 3 <= cx < w - 3 and 3 <= cy < h - 3:
                    img[cy - 3:cy + 4, cx - 3:cx + 4] = np.maximum(img[cy - 3:cy + 4, cx - 3:cx + 4], self._blob)
            img = np.clip(img * 255.0 * min(1.0, self.exposure[cam] / 50.0), 0, 255).astype(np.uint8)
            frames.append(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
        return frames

    def read(self, timeout=None):
        if self.sim_time is None:
            wait = self._last + 1.0 / self.fps - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            self._last = time.perf_counter()
            t = self._last - self.t0
        else:
            t = self.sim_time
        return self.render(t), [t] * self.num_cameras

    def stats(self):
        return {"fps": [self.fps] * self.num_cameras, "sync_spread_ms_p95": 0.0}

    def end(self):
        pass

    close = end


# --------------------------------------------------------------------------- factory
def open_cameras(cfg):
    backend = cfg["camera_backend"].lower()
    if backend == "opencv":
        return OpenCVCameraArray(cfg)
    if backend == "synthetic":
        return SyntheticCameraArray(cfg)
    raise ValueError(f"unknown camera_backend {cfg['camera_backend']!r} (opencv | synthetic | ps3eye)")
