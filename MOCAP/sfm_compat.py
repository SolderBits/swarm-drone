import cv2 as cv
import numpy as np

def _fundamental_from_projections(P1, P2):
    P1 = np.asarray(P1, dtype=np.float64)
    P2 = np.asarray(P2, dtype=np.float64)
    X = [P1[[1, 2]], P1[[2, 0]], P1[[0, 1]]]
    Y = [P2[[1, 2]], P2[[2, 0]], P2[[0, 1]]]
    F = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            F[i, j] = np.linalg.det(np.vstack([X[j], Y[i]]))
    return F

def _essential_from_fundamental(F, K1, K2):
    return np.asarray(K2, np.float64).T @ np.asarray(F, np.float64) @ np.asarray(K1, np.float64)

def _motion_from_essential(E):
    U, _, Vt = np.linalg.svd(np.asarray(E, np.float64))
    if np.linalg.det(U) < 0:
        U[:, 2] *= -1
    if np.linalg.det(Vt) < 0:
        Vt[2, :] *= -1
    W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    R1 = U @ W @ Vt
    R2 = U @ W.T @ Vt
    t = U[:, 2:3].copy()
    return (R1, R1, R2, R2), (t, -t, t, -t)

class _NumpySfm:
    fundamentalFromProjections = staticmethod(_fundamental_from_projections)
    essentialFromFundamental = staticmethod(_essential_from_fundamental)
    motionFromEssential = staticmethod(_motion_from_essential)

sfm = cv.sfm if hasattr(cv, "sfm") else _NumpySfm
USING_OPENCV_SFM = hasattr(cv, "sfm")
