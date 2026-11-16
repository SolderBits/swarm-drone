import itertools
import math
import time

import numpy as np

TRACK_TIMEOUT_S = 1.0

def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class DroneLocator:
    def __init__(self, num_drones, drone_markers, tolerance=0.02, gate=0.5):
        self.num_drones = int(num_drones)
        self.markers = [dict(m) for m in drone_markers[: self.num_drones]]
        self.tol = float(tolerance)
        self.gate = float(gate)
        for m in self.markers:
            if not (m["side"] / 2 < m["front"]):
                raise ValueError(f"marker triangle {m} is impossible: front must be longer than side/2")

        self.shapes = []
        for d, m in enumerate(self.markers):
            for shape in self.shapes:
                if abs(shape[0] - m["front"]) < 1e-9 and abs(shape[1] - m["side"]) < 1e-9:
                    shape[2].append(d)
                    break
            else:
                self.shapes.append((m["front"], m["side"], [d]))
        for a, b in itertools.combinations(self.shapes, 2):
            if abs(a[0] - b[0]) < 2 * self.tol and abs(a[1] - b[1]) < 2 * self.tol:
                print(f"[locator] warning: triangles {a[:2]} and {b[:2]} are closer than 2 x marker_tolerance; "
                      "they may be confused")
        self.by_shape = all(len(s[2]) == 1 for s in self.shapes)
        self.reset()

    def reset(self):
        self.tracks = [None] * self.num_drones

    def find_triangles(self, points):

        n = len(points)
        if n < 3:
            return []
        P = np.asarray(points, dtype=np.float64)
        D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
        candidates = []
        for s_idx, (front, side, _) in enumerate(self.shapes):
            for i in range(n):
                near = np.where(np.abs(D[i] - front) < self.tol)[0]
                near = near[near != i]
                for j, k in itertools.combinations(near, 2):
                    if abs(D[j, k] - side) >= self.tol:
                        continue
                    score = abs(D[i, j] - front) + abs(D[i, k] - front) + abs(D[j, k] - side)
                    candidates.append((score, i, int(j), int(k), s_idx))
        candidates.sort()
        used, found = set(), []
        for score, i, j, k, s_idx in candidates:
            if used.isdisjoint((i, j, k)):
                used.update((i, j, k))
                found.append((i, j, k, s_idx, score))
        return found

    def locate(self, object_points, errors, now=None):
        now = time.time() if now is None else now
        P = np.asarray(object_points, dtype=np.float64)
        errors = np.asarray(errors, dtype=np.float64)
        detections = []
        for i, j, k, s_idx, score in self.find_triangles(P):
            pos = (P[j] + P[k]) / 2.0
            front = P[i] - pos
            yaw_ccw = math.atan2(front[1], front[0])
            detections.append({
                "pos": pos,
                "heading": np.float64(_wrap(-yaw_ccw)),
                "error": float(np.mean(errors[[i, j, k]])) if len(errors) == len(P) else float("nan"),
                "shape": s_idx,
                "fit": score,
            })

        if self.by_shape:
            objects = []
            for det in detections:
                d = self.shapes[det["shape"]][2][0]
                objects.append(self._emit(det, d, now))
            return objects
        return self._assign_by_tracking(detections, now)

    def _emit(self, det, drone_index, now):
        self.tracks[drone_index] = {"pos": det["pos"], "t": now}
        return {"pos": det["pos"], "heading": det["heading"], "error": det["error"], "droneIndex": drone_index}

    def _assign_by_tracking(self, detections, now):
        objects = []
        free_dets = list(range(len(detections)))
        fresh = [d for d in range(self.num_drones)
                 if self.tracks[d] is not None and now - self.tracks[d]["t"] <= TRACK_TIMEOUT_S]

        pairs = []
        for d in fresh:
            for di in free_dets:
                if self.shapes[detections[di]["shape"]][2].count(d) == 0:
                    continue
                dist = float(np.linalg.norm(detections[di]["pos"] - self.tracks[d]["pos"]))
                if dist <= self.gate:
                    pairs.append((dist, d, di))
        pairs.sort()
        taken_d, taken_det = set(), set()
        for dist, d, di in pairs:
            if d in taken_d or di in taken_det:
                continue
            taken_d.add(d)
            taken_det.add(di)
            objects.append(self._emit(detections[di], d, now))
        free_dets = [di for di in free_dets if di not in taken_det]
        if not free_dets:
            return objects

        lost = [d for d in range(self.num_drones) if d not in taken_d and d not in fresh and self.tracks[d] is not None]
        pairs = sorted((float(np.linalg.norm(detections[di]["pos"] - self.tracks[d]["pos"])), d, di)
                       for d in lost for di in free_dets
                       if d in self.shapes[detections[di]["shape"]][2])
        for dist, d, di in pairs:
            if d in taken_d or di in taken_det:
                continue
            taken_d.add(d)
            taken_det.add(di)
            objects.append(self._emit(detections[di], d, now))
        free_dets = [di for di in free_dets if di not in taken_det]

        never = [d for d in range(self.num_drones) if self.tracks[d] is None and d not in taken_d]
        for di in sorted(free_dets, key=lambda di: detections[di]["pos"][0]):
            choices = [d for d in never if d in self.shapes[detections[di]["shape"]][2]]
            if not choices:
                continue
            d = choices[0]
            never.remove(d)
            objects.append(self._emit(detections[di], d, now))
        objects.sort(key=lambda o: o["droneIndex"])
        return objects
