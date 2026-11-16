"""Shaded renders of the drone assembly from the tessellated meshes."""
import pickle
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

mesh = pickle.load(open('/tmp/drone_mesh.pkl', 'rb'))
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/img'


def render(fname, elev, azim, only=None, hide=(), title=None, size=(9, 8), zoom=1.0):
    fig = plt.figure(figsize=size, dpi=150)
    ax = fig.add_subplot(111, projection='3d')
    polys, cols = [], []
    el, az = np.radians(elev), np.radians(azim)
    view = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    light = view * 0.6 + np.array([0.3, 0.2, 0.75])
    light /= np.linalg.norm(light)
    allv = []
    for name, verts, tris, color in mesh:
        if (only and name not in only) or name in hide:
            continue
        V = np.array(verts)
        T = np.array(tris)
        if len(T) == 0:
            continue
        tri = V[T]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nn = np.linalg.norm(n, axis=1, keepdims=True)
        nn[nn == 0] = 1
        n = n / nn
        lam = np.abs(n @ light)
        base = np.array(color[:3])
        shade = 0.35 + 0.65 * lam
        c = np.clip(base[None, :] * shade[:, None], 0, 1)
        polys.extend(tri)
        cols.extend(c)
        allv.append(V)
    pc = Poly3DCollection(polys, facecolors=cols, edgecolors='none', linewidths=0)
    ax.add_collection3d(pc)
    A = np.vstack(allv)
    lo, hi = A.min(0), A.max(0)
    ctr = (lo + hi) / 2
    r = (hi - lo).max() / 2 / zoom
    ax.set_xlim(ctr[0] - r, ctr[0] + r)
    ax.set_ylim(ctr[1] - r, ctr[1] + r)
    ax.set_zlim(ctr[2] - r, ctr[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    if title:
        ax.set_title(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(f'{out}/{fname}', bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('wrote', fname, len(polys), 'triangles')


render('drone_iso.png', 28, 60, title='Swarm drone - full assembly (front toward camera)', zoom=1.55)
render('drone_top.png', 89.9, -90, title='Top view (front = up)', zoom=1.25)
render('drone_side.png', 0.5, -90, title='Side view (from rear)', zoom=1.4, size=(10, 6))
for i, m in enumerate(mesh):
    if m[0] == 'frame':
        mesh[i] = (m[0], m[1], m[2], (0.62, 0.66, 0.72, 1))
render('frame_iso.png', 35, -120, only={'frame'}, title='3D-printed frame (print flat, plate down, no supports)', zoom=1.6)
