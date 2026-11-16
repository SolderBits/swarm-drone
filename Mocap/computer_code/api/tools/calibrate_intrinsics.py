"""Measure one camera's lens (focal length, centre, distortion) with a chessboard
and write it into api/camera-params.json.

Do this for every camera BEFORE you fit the IR-pass filter (the chessboard has
to be visible), and again if you change a lens or the processing size.

    python tools/calibrate_intrinsics.py --camera 0
    python tools/calibrate_intrinsics.py --camera 2 --cols 9 --rows 6

--camera is the position in "camera_devices" (0 = first camera), which is also
the entry it writes in camera-params.json.

Print a chessboard (e.g. 10 x 7 squares = 9 x 6 inner corners), tape it flat to
something stiff, and hold it in front of the camera:
  * space  save this view (only when the corners are drawn)
  * c      calibrate and save to camera-params.json (needs --min-views views)
  * q      quit without saving
Fill the whole picture over the views: corners, edges, tilted, near and far.

The frames go through the same steps as the mocap backend (scale to 320x240,
rotate by the camera's "rotation", pad to a square), so the numbers match what
helpers.py uses.
"""
import argparse
import json
import os
import shutil
import sys
import time

import cv2 as cv
import numpy as np

API_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, API_DIR)
from camera_backend import OpenCVCameraArray  # noqa: E402
from mocap_config import CONFIG  # noqa: E402
from helpers import make_square  # noqa: E402


def calibrate(views, board, size):
    obj = np.zeros((board[0] * board[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2)
    rms, K, D, _, _ = cv.calibrateCamera([obj] * len(views), views, (size, size), None, None)
    return rms, K, D.ravel()[:5]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, required=True, help="position in camera_devices")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--min-views", type=int, default=15)
    ap.add_argument("--exposure", type=float, default=60, help="0-100, as in the web UI")
    ap.add_argument("--gain", type=float, default=30)
    ap.add_argument("--params", default=os.path.join(API_DIR, "camera-params.json"))
    args = ap.parse_args()

    with open(args.params) as f:
        params = json.load(f)
    if args.camera >= len(params):
        sys.exit(f"camera-params.json has {len(params)} entries; add one for camera {args.camera}")
    if args.camera >= len(CONFIG["camera_devices"]):
        sys.exit(f"camera_devices has {len(CONFIG['camera_devices'])} entries; camera {args.camera} doesn't exist")
    rotation = params[args.camera].get("rotation", 0)

    cfg = dict(CONFIG)
    cfg["camera_devices"] = [CONFIG["camera_devices"][args.camera]]
    cfg["initial_exposure"], cfg["initial_gain"] = args.exposure, args.gain
    cams = OpenCVCameraArray(cfg)

    board = (args.cols, args.rows)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    views, size = [], None
    window = f"camera {args.camera} - space: save view, c: calibrate, q: quit"
    try:
        while True:
            frames, _ = cams.read()
            img = make_square(np.ascontiguousarray(np.rot90(frames[0], k=rotation)))
            size = img.shape[0]
            grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            found, corners = cv.findChessboardCorners(grey, board, cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_FAST_CHECK)
            show = cv.resize(img, (size * 2, size * 2), interpolation=cv.INTER_NEAREST)
            if found:
                cv.drawChessboardCorners(show, board, corners * 2, found)
            cv.putText(show, f"views: {len(views)}/{args.min_views}", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.imshow(window, show)
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):
                print("quit, nothing saved")
                return
            if key == ord(" ") and found:
                corners = cv.cornerSubPix(grey, corners, (5, 5), (-1, -1), criteria)
                views.append(corners)
                print(f"saved view {len(views)}")
            if key == ord("c"):
                if len(views) < args.min_views:
                    print(f"need {args.min_views} views, have {len(views)}")
                    continue
                break
    finally:
        cams.end()
        cv.destroyAllWindows()

    rms, K, D = calibrate(views, board, size)
    print(f"RMS reprojection error: {rms:.3f} px  (good: under 0.5)")
    print("intrinsic_matrix:\n", np.round(K, 2))
    print("distortion_coef:", np.round(D, 4))
    if rms > 1.0:
        print("that's high: retake with the board flat, well lit and covering the whole frame")

    backup = args.params + time.strftime(".%Y%m%d-%H%M%S.bak")
    shutil.copy(args.params, backup)
    params[args.camera]["intrinsic_matrix"] = K.tolist()
    params[args.camera]["distortion_coef"] = D.tolist()
    params[args.camera]["rotation"] = rotation
    with open(args.params, "w") as f:
        json.dump(params, f, indent=4)
    print(f"wrote camera {args.camera} into {args.params} (old file kept as {backup})")


if __name__ == "__main__":
    main()
