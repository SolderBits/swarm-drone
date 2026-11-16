"""Clean assembly drawing: every part outlined with reference + value, plus a value table."""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle, Circle  # noqa: E402
import pcbnew  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from design import PARTS  # noqa: E402
from layout import OUTLINE, CHAMFER  # noqa: E402

PCB = sys.argv[1]
OUTPNG = sys.argv[2]
X0, Y0 = 100.0, 100.0
b = pcbnew.LoadBoard(PCB)


def bx(v):
    return pcbnew.ToMM(v.x) - X0


def by(v):
    return Y0 - pcbnew.ToMM(v.y)


COL = {'R': '#f2c94c', 'C': '#7fb3ff', 'D': '#ff9f7f', 'Q': '#9be29b', 'J': '#d9d9d9', 'U': '#cfc3ff', 'H': '#ffffff'}
fig, ax = plt.subplots(figsize=(14, 13.5), dpi=150)
xl, xr, yb, yt = OUTLINE
c = CHAMFER
pts = [(xl + c, yb), (xr - c, yb), (xr, yb + c), (xr, yt - c), (xr - c, yt), (xl + c, yt), (xl, yt - c), (xl, yb + c)]
ax.add_patch(Polygon(pts, closed=True, fc='#1f6f3f', ec='black', lw=2, zorder=0))

for fp in b.GetFootprints():
    ref = fp.GetReference()
    fp.BuildCourtyardCaches()
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    kind = ref.rstrip('0123456789')
    col = COL.get(kind, '#ffffff')
    if ref in ('U1', 'U2'):
        bb = cy.BBox()
        x1, x2 = sorted((pcbnew.ToMM(bb.GetLeft()) - X0, pcbnew.ToMM(bb.GetRight()) - X0))
        y1, y2 = sorted((Y0 - pcbnew.ToMM(bb.GetTop()), Y0 - pcbnew.ToMM(bb.GetBottom())))
        ax.add_patch(Rectangle((x1 + 0.4, y1 + 0.4), x2 - x1 - 0.8, y2 - y1 - 0.8, fc=col, ec='#333', lw=1.5, alpha=0.35, zorder=1))
    elif cy.OutlineCount():
        o = cy.Outline(0)
        poly = [(pcbnew.ToMM(o.CPoint(i).x) - X0, Y0 - pcbnew.ToMM(o.CPoint(i).y)) for i in range(o.PointCount())]
        ax.add_patch(Polygon(poly, closed=True, fc=col, ec='#222', lw=1, alpha=0.9, zorder=2))
    for p in fp.Pads():
        pos = p.GetPosition()
        sz = p.GetSize()
        w, h = pcbnew.ToMM(sz.x), pcbnew.ToMM(sz.y)
        ang = p.GetOrientationDegrees()
        if abs(ang % 180 - 90) < 1:
            w, h = h, w
        is_tht = p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH
        ax.add_patch(Rectangle((bx(pos) - w / 2, by(pos) - h / 2), w, h, fc='#c9a227' if is_tht else '#e0e0e0',
                               ec='#5a4500', lw=0.6, zorder=3))
        if p.GetNumber() == '1' and kind in 'JDRQC' and ref not in ('H1', 'H2'):
            ax.add_patch(Circle((bx(pos), by(pos)), 0.25, fc='red', ec='none', zorder=4))
    val = PARTS.get(ref, ('', fp.GetValue()))[1]
    pos = fp.GetPosition()
    lx, ly = bx(pos), by(pos)
    if kind in ('R', 'D', 'C', 'J') and ref not in ('U1', 'U2'):
        # label at courtyard centre
        if cy.OutlineCount():
            bb = cy.BBox()
            lx = (pcbnew.ToMM(bb.GetLeft()) + pcbnew.ToMM(bb.GetRight())) / 2 - X0
            ly = Y0 - (pcbnew.ToMM(bb.GetTop()) + pcbnew.ToMM(bb.GetBottom())) / 2
    if ref in ('U1', 'U2'):
        ax.text(lx + (0 if ref == 'U1' else 8.9), ly + (0 if ref == 'U1' else -8), f'{ref}\n{val}', ha='center', va='center',
                fontsize=12, weight='bold', zorder=6)
    elif ref.startswith('H'):
        ax.add_patch(Circle((lx, ly), 1.1, fc='white', ec='black', zorder=5))
        ax.text(lx, ly - 1.9, ref, ha='center', va='top', fontsize=7, color='white', zorder=6)
    else:
        short = val.replace(' (PH2.0)', '').replace('BATT 1S', 'BATT').replace('100uF 6.3V', '100µF')
        ax.text(lx, ly, f'{ref}\n{short}', ha='center', va='center', fontsize=6.6, weight='bold', zorder=6,
                linespacing=0.95)

ax.annotate('FRONT', xy=(0, 26.5), ha='center', fontsize=14, weight='bold')
ax.annotate('', xy=(0, 26.2), xytext=(0, 24.2), arrowprops=dict(arrowstyle='->', lw=2))
ax.text(-11.5, -22.2, 'USB-C', ha='center', fontsize=9)
ax.set_xlim(xl - 3, xr + 3)
ax.set_ylim(yb - 3.5, yt + 4.5)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title('Assembly guide - top side (red dot = pin 1 / square pad; gold = through-hole, grey = SMD)', fontsize=12)
fig.tight_layout()
fig.savefig(OUTPNG, facecolor='white')
print('wrote', OUTPNG)
