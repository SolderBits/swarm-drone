"""Tests for the OV9281 / swarm-frame changes. No cameras or ESP32 needed.

    cd computer_code/api
    python tests/test_pipeline.py          (or: python -m pytest tests)

Covers: the numpy SFM replacement, the drone locator (shape, heading, identity),
the OpenCV capture backend with fake cameras, and the whole pipeline on
synthetic camera images: camera-pose calibration through index.py's socket
handler, then live triangulation -> drone positions -> serial messages.
"""
import json
import math
import os
import sys
import threading
import time

os.environ["MOCAP_CAMERA_BACKEND"] = "synthetic"
os.environ["MOCAP_SERIAL_PORT"] = "none"
API = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, API)

import cv2 as cv  # noqa: E402
import numpy as np  # noqa: E402

import camera_backend  # noqa: E402
from camera_backend import SyntheticRig, look_at, swarm_marker_points, exposure_to_device  # noqa: E402
from drone_locator import DroneLocator  # noqa: E402
from mocap_config import CONFIG  # noqa: E402
from sfm_compat import _NumpySfm  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT_DIR, exist_ok=True)


# --------------------------------------------------------------------------- SFM
def test_numpy_sfm_matches_geometry():
    K = np.array([[280, 0, 320], [0, 280, 320], [0, 0, 1.0]])
    R1, t1 = look_at((2, 0, 2), (0, 0, 0.6))
    R2, t2 = look_at((0.3, 2, 1.8), (0, 0, 0.6))
    P1, P2 = K @ np.c_[R1, t1], K @ np.c_[R2, t2]
    F = _NumpySfm.fundamentalFromProjections(P1, P2)
    X = np.random.default_rng(0).uniform(-0.5, 0.5, (30, 3)) + [0, 0, 0.6]

    def proj(P):
        x = (P @ np.c_[X, np.ones(len(X))].T).T
        return x / x[:, 2:]

    x1, x2 = proj(P1), proj(P2)
    lines = cv.computeCorrespondEpilines(x1[:, :2].astype(np.float32).reshape(-1, 1, 2), 1, F).reshape(-1, 3)
    dist = np.abs(np.sum(lines * x2, axis=1)) / np.linalg.norm(lines[:, :2], axis=1)
    assert dist.max() < 1e-3, dist.max()
    E = _NumpySfm.essentialFromFundamental(F, K, K)
    Rs, ts = _NumpySfm.motionFromEssential(E)
    R_rel = R2 @ R1.T
    t_rel = t2 - R_rel @ t1
    t_rel /= np.linalg.norm(t_rel)
    assert all(t.shape == (3, 1) for t in ts)
    assert min(np.linalg.norm(R - R_rel) + np.linalg.norm(t.ravel() - t_rel) for R, t in zip(Rs, ts)) < 1e-9


# --------------------------------------------------------------------------- locator
def _scene(drones, rng=None, noise=0.0, extra=0):
    pts = []
    for center, yaw, front, side in drones:
        pts.extend(swarm_marker_points(center, yaw, front, side))
    pts = np.array(pts)
    if rng is not None and noise:
        pts = pts + rng.normal(0, noise, pts.shape)
    if extra:
        pts = np.vstack([pts, rng.uniform(-1, 1, (extra, 3)) + [0, 0, 1]])
    order = (rng.permutation(len(pts)) if rng is not None else np.arange(len(pts)))
    return pts[order]


def test_locator_position_heading_and_first_numbering():
    rng = np.random.default_rng(1)
    loc = DroneLocator(2, [{"front": 0.065, "side": 0.1}] * 2, tolerance=0.02, gate=0.5)
    truth = [((0.4, 0.1, 0.5), 0.3), ((-0.3, -0.2, 0.7), -0.5)]
    pts = _scene([(c, y, 0.065, 0.1) for c, y in truth], rng, noise=0.003, extra=3)
    objs = loc.locate(pts, np.zeros(len(pts)), now=0.0)
    assert len(objs) == 2
    # drone 0 = smaller x at first sight
    by = {o["droneIndex"]: o for o in objs}
    np.testing.assert_allclose(by[0]["pos"], truth[1][0], atol=0.01)
    np.testing.assert_allclose(by[1]["pos"], truth[0][0], atol=0.01)
    # heading sent = minus the counter-clockwise yaw (original convention)
    assert abs(by[0]["heading"] - 0.5) < 0.1 and abs(by[1]["heading"] + 0.3) < 0.1


def test_locator_full_heading_range():
    loc = DroneLocator(1, [{"front": 0.065, "side": 0.1}], tolerance=0.02)
    for yaw in np.linspace(-3.0, 3.0, 13):
        o = loc.locate(_scene([((0, 0, 1), yaw, 0.065, 0.1)]), np.zeros(3), now=0.0)[0]
        assert abs((o["heading"] + yaw + math.pi) % (2 * math.pi) - math.pi) < 1e-6


def test_locator_keeps_identity_when_drones_cross():
    loc = DroneLocator(2, [{"front": 0.065, "side": 0.1}] * 2, tolerance=0.02, gate=0.3)
    rng = np.random.default_rng(2)
    swaps = 0
    for k in range(200):
        t = k / 60.0
        a = (-0.3 + 0.6 * t / 3.3, 0.15, 0.6)     # drone A moves +x
        b = (0.3 - 0.6 * t / 3.3, -0.15, 0.6)     # drone B moves -x: they pass each other
        objs = loc.locate(_scene([(a, 0, 0.065, 0.1), (b, 0, 0.065, 0.1)], rng, noise=0.002), np.zeros(6), now=t)
        by = {o["droneIndex"]: o["pos"] for o in objs}
        if np.linalg.norm(by[0] - a) > 0.05:
            swaps += 1
    assert swaps == 0, swaps


def test_locator_by_shape():
    loc = DroneLocator(2, [{"front": 0.065, "side": 0.1}, {"front": 0.085, "side": 0.1}], tolerance=0.008)
    assert loc.by_shape
    pts = _scene([((-0.5, 0, 1), 0, 0.085, 0.1), ((0.5, 0, 1), 0, 0.065, 0.1)], np.random.default_rng(3), noise=0.002)
    by = {o["droneIndex"]: o["pos"] for o in loc.locate(pts, np.zeros(6), now=0)}
    np.testing.assert_allclose(by[1], (-0.5, 0, 1), atol=0.01)
    np.testing.assert_allclose(by[0], (0.5, 0, 1), atol=0.01)


# --------------------------------------------------------------------------- OpenCV backend
class FakeCapture:
    """Stands in for cv.VideoCapture: 120 fps, records every set()."""
    instances = []

    def __init__(self, device, api):
        self.device, self.api = device, api
        self.props = {cv.CAP_PROP_FRAME_WIDTH: 640, cv.CAP_PROP_FRAME_HEIGHT: 480, cv.CAP_PROP_FPS: 120,
                      cv.CAP_PROP_FOURCC: cv.VideoWriter_fourcc(*"MJPG")}
        self.sets = []
        self.next_t = time.perf_counter()
        FakeCapture.instances.append(self)

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.sets.append((prop, value))
        self.props[prop] = value
        return True

    def get(self, prop):
        return self.props.get(prop, 0)

    def getBackendName(self):
        return "FAKE"

    def read(self):
        wait = self.next_t - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        self.next_t = max(self.next_t + 1 / 120, time.perf_counter() - 0.01)
        img = np.full((480, 640, 3), 10 * self.device, np.uint8)
        return True, img

    def release(self):
        pass


def test_opencv_backend_with_fake_cameras():
    real = camera_backend.cv.VideoCapture
    camera_backend.cv.VideoCapture = FakeCapture
    FakeCapture.instances = []
    try:
        cfg = dict(CONFIG, camera_backend="opencv", camera_devices=[0, 1, 2, 3], capture_api="v4l2",
                   process_width=640, process_height=480, initial_exposure=50, initial_gain=7)
        cams = camera_backend.open_cameras(cfg)
        assert len(cams.exposure) == 4
        t0, n = time.perf_counter(), 0
        while time.perf_counter() - t0 < 1.0:
            frames, stamps = cams.read()
            n += 1
        assert all(f.shape == (480, 640, 3) for f in frames)
        assert [int(f[0, 0, 0]) for f in frames] == [0, 10, 20, 30]  # order kept
        assert n > 90, n                                            # ~120 sets per second
        st = cams.stats()
        assert all(f > 100 for f in st["fps"]), st
        cams.exposure = [100] * 4
        cams.gain = 20
        time.sleep(0.1)
        sets = FakeCapture.instances[0].sets
        assert (cv.CAP_PROP_AUTO_EXPOSURE, 1) in sets              # V4L2 manual
        assert (cv.CAP_PROP_EXPOSURE, 42) in sets and (cv.CAP_PROP_EXPOSURE, 83) in sets
        assert (cv.CAP_PROP_GAIN, 7.0) in sets and (cv.CAP_PROP_GAIN, 20.0) in sets
        cams.end()
    finally:
        camera_backend.cv.VideoCapture = real
    assert exposure_to_device(100, 120, cv.CAP_DSHOW) == -7


# --------------------------------------------------------------------------- whole pipeline
class FakeSocketIO:
    def __init__(self):
        self.events = []

    def emit(self, name, data=None, **kw):
        json.dumps(data)  # must be serialisable, like the real one
        self.events.append((name, data))


class FakeSerial:
    def __init__(self):
        self.data = bytearray()

    def write(self, b):
        self.data += b
        return len(b)


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t


def _to_world_matrix():
    # helpers converts p -> swap_yz(M @ diag(-1,-1,1,1) @ [p,1]); choose M so the result is p again
    Q = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    D = np.diag([-1.0, -1.0, 1.0, 1.0])
    return (Q @ D).tolist()


def _pipeline():
    import helpers
    cams = helpers.Cameras.instance()
    rig = cams.cameras.rig
    cams.camera_params = [{"intrinsic_matrix": rig.K.tolist(), "distortion_coef": [0, 0, 0, 0, 0], "rotation": 0}
                          for _ in range(cams.num_cameras)]
    cams._undistort_maps = {}
    return helpers, cams, rig


def test_camera_pose_calibration_through_index():
    """The UI's 'Calculate Camera Pose' step with numpy SFM, on a synthetic wand."""
    import index
    helpers, cams, rig = _pipeline()
    sock = FakeSocketIO()
    index.socketio = sock
    helpers_sock = sock  # bundle_adjustment emits through the socketio it is given
    rng = np.random.default_rng(5)
    samples = []
    for _ in range(60):
        p = rng.uniform([-0.6, -0.6, 0.2], [0.6, 0.6, 1.2])
        samples.append([rig.project(c, p[None])[0][0] + [0, (rig.square - rig.height) / 2] for c in range(cams.num_cameras)])
    samples = np.array(samples) + rng.normal(0, 0.3, (60, cams.num_cameras, 2))
    index.calculate_camera_pose({"cameraPoints": samples.tolist()})
    poses = [e[1]["camera_poses"] for e in sock.events if e[0] == "camera-pose"][-1]
    poses = [{"R": np.array(p["R"]), "t": np.array(p["t"])} for p in poses]
    obj = helpers.triangulate_points(samples.tolist(), poses)
    err = helpers.calculate_reprojection_errors(samples.tolist(), obj, poses)
    rms = float(np.sqrt(np.mean(err)))
    print(f"    camera-pose calibration: reprojection RMS {rms:.2f} px over {len(samples)} wand samples")
    assert rms < 1.5, rms
    del helpers_sock


def test_live_tracking_to_serial(save_stream=True):
    import KalmanFilter
    import drone_locator
    helpers, cams, rig = _pipeline()
    clock = FakeClock()
    KalmanFilter.time = clock
    drone_locator.time = clock
    sock, ser = FakeSocketIO(), FakeSerial()
    cams.set_socketio(sock)
    cams.set_ser(ser)
    cams.set_serialLock(threading.Lock())
    cams.set_num_objects(2)
    cams.drone_armed = [True, True]
    poses = [{"R": R.tolist(), "t": t.reshape(3, 1).tolist()} for R, t in rig.poses]
    cams.start_trangulating_points(poses)
    cams.to_world_coords_matrix = _to_world_matrix()
    cams.start_locating_objects()

    def truth(t):
        # same as camera_backend.default_scene; scene drone 1 starts at -x so it becomes drone 0
        out = []
        for d, phase in enumerate((0.0, math.pi)):
            a = 0.4 * t + phase
            out.append(((0.3 * math.cos(a), 0.3 * math.sin(a), 0.6 + 0.1 * math.sin(0.7 * t + d)),
                        0.2 * math.sin(0.5 * t + d)))
        return {0: out[1], 1: out[0]}

    pos_err, head_err, frames_with_both = [], [], 0
    n = 600
    for k in range(n):
        t = k / 60.0
        clock.t = 1000.0 + t
        cams.cameras.sim_time = t
        before = len(sock.events)
        cams._camera_read()
        evs = [e for e in sock.events[before:] if e[0] == "object-points"]
        if not evs or k < 30:
            continue
        filt = evs[-1][1]["filtered_objects"]
        if len(filt) == 2:
            frames_with_both += 1
        tr = truth(t)
        for f in filt:
            c, yaw = tr[f["droneIndex"]]
            pos_err.append(np.linalg.norm(np.array(f["pos"]) - c))
            head_err.append(abs(f["heading"] + yaw))
    pos_err, head_err = np.array(pos_err), np.array(head_err)
    print(f"    live tracking: both drones found in {frames_with_both}/{n - 30} frames, "
          f"position error median {np.median(pos_err)*1000:.1f} mm / p95 {np.percentile(pos_err, 95)*1000:.1f} mm, "
          f"heading error p95 {np.degrees(np.percentile(head_err, 95)):.1f} deg")
    # a frame where a marker merges with another in too many cameras just skips
    # that drone; the ground controller only disarms after 500 ms without data
    assert frames_with_both > 0.90 * (n - 30)
    assert np.percentile(pos_err, 95) < 0.03
    assert np.percentile(head_err, 95) < math.radians(10)

    # serial stream: "<idx>{json}" back to back, the format ground_controller.ino parses
    stream = bytes(ser.data).decode()
    msgs, i = [], 0
    while i < len(stream):
        idx = int(stream[i])
        depth, j = 0, i + 1
        while True:
            depth += {"{": 1, "}": -1}.get(stream[j], 0)
            j += 1
            if depth == 0:
                break
        msgs.append((idx, json.loads(stream[i + 1:j])))
        i = j
    assert {m[0] for m in msgs} == {0, 1}
    assert all(len(m[1]["pos"]) == 4 and len(m[1]["vel"]) == 3 for m in msgs)
    if save_stream:
        with open(os.path.join(OUT_DIR, "serial_stream.txt"), "w") as f:
            f.write(stream)
        last = {m[0]: m[1] for m in msgs}
        with open(os.path.join(OUT_DIR, "serial_last.json"), "w") as f:
            json.dump(last, f)
    print(f"    serial: {len(msgs)} messages, {len(ser.data)} bytes")


def test_marker_separation_vs_distance():
    """One camera, 2 m up, drone at 0.6 m, yaw 0-90 deg: does the v1.1 triangle
    (110/150 mm) stay 3 separate blobs at 640x480 with the wide (~100 deg) lens?"""
    import helpers
    cams = helpers.Cameras.instance()
    results = {}
    for radius in (1.0, 2.0, 3.0, 4.0):
        slant = round(math.hypot(radius, 1.4), 1)
        worst = 3
        for yaw in np.radians([0, 30, 60, 90]):
            rig = SyntheticRig(640, 480, num_cameras=1, radius=radius, height_m=2.0)
            syn = camera_backend.SyntheticCameraArray(
                dict(CONFIG, camera_devices=[0], process_width=640, process_height=480),
                scene=lambda t, yaw=yaw: swarm_marker_points((0, 0, 0.6), yaw), rig=rig)
            img = helpers.make_square(syn.render(0.0)[0])
            img = cv.GaussianBlur(img, (9, 9), 0)
            img = cv.filter2D(img, -1, helpers.SHARPEN_KERNEL)
            _, pts = cams._find_dot(img)
            worst = min(worst, len([p for p in pts if p[0] is not None]))
        results[slant] = worst
    print(f"    blobs per drone (worst yaw) vs camera distance in m: {results}")
    with open(os.path.join(OUT_DIR, "blob_separation.json"), "w") as f:
        json.dump(results, f)
    assert results[min(results)] == 3 and results[sorted(results)[1]] == 3


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"ok    {fn.__name__}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"FAIL  {fn.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
