"""Build the swarm drone flight-controller PCB with KiCad's Python API.

Board coordinates in this file are millimetres with +X right and +Y toward the
FRONT of the drone (up on screen); they are converted to KiCad's Y-down frame.
"""
import os
import sys
import uuid

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from design import PARTS, NETS, MOTORS  # noqa: E402
from layout import PLACE, ROUTES, VIAS, OUTLINE, CHAMFER, HOLES_KEEP  # noqa: E402

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else '../hw')
PCB = f'{OUT}/swarm_fc.kicad_pcb'
X0, Y0 = 100.0, 100.0
FP_ROOT = '/usr/share/kicad/footprints'
NS = uuid.UUID('6b1c3c5e-1f7e-4a8e-9d55-2f0c6a9f0d11')


def sym_uuid(ref):
    return str(uuid.uuid5(NS, 'sym-' + ref))


def V(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(X0 + x), pcbnew.FromMM(Y0 - y))


def mm(v):
    return (round(pcbnew.ToMM(v.x) - X0, 4), round(Y0 - pcbnew.ToMM(v.y), 4))


board = pcbnew.BOARD()
ds = board.GetDesignSettings()
ds.SetBoardThickness(pcbnew.FromMM(0.8))
ds.m_TrackMinWidth = pcbnew.FromMM(0.2)
ds.m_MinClearance = pcbnew.FromMM(0.2)
ds.m_ViasMinSize = pcbnew.FromMM(0.6)
ds.m_MinThroughDrill = pcbnew.FromMM(0.3)
ds.m_CopperEdgeClearance = pcbnew.FromMM(0.3)
ds.m_HoleToHoleMin = pcbnew.FromMM(0.25)
nc = ds.m_NetSettings.m_DefaultNetClass
nc.SetClearance(pcbnew.FromMM(0.2))
nc.SetTrackWidth(pcbnew.FromMM(0.25))
nc.SetViaDiameter(pcbnew.FromMM(0.6))
nc.SetViaDrill(pcbnew.FromMM(0.3))

# ---------------------------------------------------------------- nets
nets = {}
for name in NETS:
    n = pcbnew.NETINFO_ITEM(board, name)
    board.Add(n)
    nets[name] = n
pin_net = {p: net for net, pins in NETS.items() for p in pins}

# ---------------------------------------------------------------- outline
xl, xr, yb, yt = OUTLINE
c = CHAMFER
pts = [(xl + c, yb), (xr - c, yb), (xr, yb + c), (xr, yt - c), (xr - c, yt), (xl + c, yt), (xl, yt - c), (xl, yb + c)]
for i in range(len(pts)):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(V(*pts[i]))
    s.SetEnd(V(*pts[(i + 1) % len(pts)]))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(pcbnew.FromMM(0.1))
    board.Add(s)

# ---------------------------------------------------------------- footprints
fps = {}
for ref, (sym, value, fpid, desc) in PARTS.items():
    lib, name = fpid.split(':')
    path = f'{OUT}/swarm.pretty' if lib == 'swarm' else f'{FP_ROOT}/{lib}.pretty'
    fp = pcbnew.FootprintLoad(path, name)
    assert fp, fpid
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetReference(ref)
    fp.SetValue(value)
    x, y, rot = PLACE[ref]
    fp.SetPosition(V(x, y))
    fp.SetOrientationDegrees(rot)
    fp.SetPath(pcbnew.KIID_PATH('/' + sym_uuid(ref)))
    fp.Reference().SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.8), pcbnew.FromMM(0.8)))
    fp.Reference().SetTextThickness(pcbnew.FromMM(0.12))
    board.Add(fp)
    fps[ref] = fp
    for pad in fp.Pads():
        key = (ref, pad.GetNumber())
        if key in pin_net:
            pad.SetNet(nets[pin_net[key]])


def pad_xy(spec):
    ref, num = spec.split('.')
    for p in fps[ref].Pads():
        if p.GetNumber() == num:
            return mm(p.GetPosition())
    raise KeyError(spec)


LAYER = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu}
import math  # noqa: E402

# points where another route starts/ends: keep those corners (they are T-junctions)
_ends = set()
for _n, _l, _w, _p in ROUTES:
    for q in (_p[0], _p[-1]):
        if not isinstance(q, str):
            _ends.add((round(q[0], 3), round(q[1], 3)))


def chamfer(xy, is_pad, width):
    """Replace every bend sharper than 45 deg with a 45-deg chamfer (no right angles)."""
    out = [xy[0]]
    for i in range(1, len(xy) - 1):
        a, b, d = out[-1], xy[i], xy[i + 1]
        if is_pad[i] or (round(b[0], 3), round(b[1], 3)) in _ends:
            out.append(b)
            continue
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (d[0] - b[0], d[1] - b[1])
        l1, l2 = math.hypot(*v1), math.hypot(*v2)
        if l1 < 1e-6 or l2 < 1e-6:
            out.append(b)
            continue
        u1 = (v1[0] / l1, v1[1] / l1)
        u2 = (v2[0] / l2, v2[1] / l2)
        turn = math.degrees(math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))))
        if turn < 50:                      # already a gentle (<=45 deg) bend
            out.append(b)
            continue
        want = 1.6 if width >= 0.8 else 1.0
        k = min(want, 0.5 * l1 if i > 1 or not is_pad[0] else 0.5 * l1, 0.5 * l2)
        if turn > 100:                     # acute bend: two 45-deg steps would be needed; just round lightly
            k = min(k, 0.6)
        out.append((b[0] - u1[0] * k, b[1] - u1[1] * k))
        out.append((b[0] + u2[0] * k, b[1] + u2[1] * k))
    out.append(xy[-1])
    return out


for net, layer, width, path in ROUTES:
    xy = [pad_xy(p) if isinstance(p, str) else p for p in path]
    is_pad = [isinstance(p, str) for p in path]
    xy = chamfer(xy, is_pad, width)
    for a, b in zip(xy, xy[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(V(*a))
        t.SetEnd(V(*b))
        t.SetWidth(pcbnew.FromMM(width))
        t.SetLayer(LAYER[layer])
        t.SetNet(nets[net])
        board.Add(t)
for net, x, y in VIAS:
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(V(x, y))
    v.SetDrill(pcbnew.FromMM(0.3))
    v.SetWidth(pcbnew.FromMM(0.6))
    v.SetNet(nets[net])
    board.Add(v)

# ---------------------------------------------------------------- GND pours (both layers)
inset = 0.0
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(nets['GND'])
    z.SetLocalClearance(pcbnew.FromMM(0.3))
    z.SetMinThickness(pcbnew.FromMM(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(pcbnew.FromMM(0.3))
    z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    ol = z.Outline()
    ol.NewOutline()
    for p in pts:
        ol.Append(V(*p))
    board.Add(z)


# ---------------------------------------------------------------- silkscreen notes
def silk(textv, x, y, size=1.0, layer=pcbnew.F_SilkS, rot=0, bold=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(textv)
    t.SetPosition(V(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetTextThickness(pcbnew.FromMM(0.15 if size >= 1 else 0.12))
    if rot:
        t.SetTextAngleDegrees(rot)
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    board.Add(t)


from layout import SILK  # noqa: E402
for args in SILK:
    silk(*args)


# ---------------------------------------------------------------- reference placement
def bbox_mm(item):
    b = item.GetBoundingBox()
    x1, y1 = pcbnew.ToMM(b.GetLeft()), pcbnew.ToMM(b.GetTop())
    x2, y2 = pcbnew.ToMM(b.GetRight()), pcbnew.ToMM(b.GetBottom())
    return (x1, y1, x2, y2)


def grow(bb, d):
    return (bb[0] - d, bb[1] - d, bb[2] + d, bb[3] + d)


def hit(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def inside_board(bb, m=0.4):
    for (x, y) in [(bb[0], bb[1]), (bb[2], bb[1]), (bb[0], bb[3]), (bb[2], bb[3])]:
        bx, by = x - X0, Y0 - y
        if not (xl + m <= bx <= xr - m and yb + m <= by <= yt - m):
            return False
        # chamfers
        if (bx - xl) + (by - yb) < c + m * 1.42 or (xr - bx) + (by - yb) < c + m * 1.42:
            return False
        if (bx - xl) + (yt - by) < c + m * 1.42 or (xr - bx) + (yt - by) < c + m * 1.42:
            return False
    return True


obst = []
for fp in fps.values():
    for p in fp.Pads():
        obst.append(grow(bbox_mm(p), 0.12))
    for g in fp.GraphicalItems():
        if g.GetLayer() == pcbnew.F_SilkS and g.GetClass() != 'MTEXT' and not isinstance(g, pcbnew.FP_TEXT):
            obst.append(grow(bbox_mm(g), 0.1))
        elif isinstance(g, pcbnew.FP_TEXT) and g.GetLayer() == pcbnew.F_SilkS and g.IsVisible():
            obst.append(grow(bbox_mm(g), 0.1))
for d in board.GetDrawings():
    if d.GetLayer() == pcbnew.F_SilkS:
        obst.append(grow(bbox_mm(d), 0.1))
for t in board.GetTracks():
    if isinstance(t, pcbnew.PCB_VIA):
        obst.append(grow(bbox_mm(t), 0.1))

moved_to_fab = []
order = sorted(fps, key=lambda r: -fps[r].GetArea() if hasattr(fps[r], 'GetArea') else 0)
for ref in order:
    fp = fps[ref]
    if ref.startswith('H'):
        fp.Reference().SetVisible(False)
        continue
    t = fp.Reference()
    t.SetKeepUpright(True)
    fp.BuildCourtyardCaches()
    cb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox(False, False)
    cx1, cy1, cx2, cy2 = (pcbnew.ToMM(cb.GetLeft()), pcbnew.ToMM(cb.GetTop()), pcbnew.ToMM(cb.GetRight()), pcbnew.ToMM(cb.GetBottom()))
    mx, my = (cx1 + cx2) / 2, (cy1 + cy2) / 2
    cands = []
    for dy in (0.0, 0.6, -0.6):
        cands += [((mx, cy1 - 0.55 + dy * 0), 0), ((mx, cy2 + 0.55), 0)]
    cands += [((cx1 - 0.6, my), 90), ((cx2 + 0.6, my), 90), ((mx, my), 0), ((mx, my), 90)]
    for ox in (-1.5, 1.5, -3, 3):
        cands += [((mx + ox, cy1 - 0.55), 0), ((mx + ox, cy2 + 0.55), 0)]
    for oy in (-1.5, 1.5):
        cands += [((cx1 - 0.6, my + oy), 90), ((cx2 + 0.6, my + oy), 90)]
    # pads-relative ring: just outside each pad, both orientations
    for p in fp.Pads():
        pb = bbox_mm(p)
        pcx, pcy = (pb[0] + pb[2]) / 2, (pb[1] + pb[3]) / 2
        for dx, dy, a in ((0, -1.45, 0), (0, 1.45, 0), (-1.6, 0, 90), (1.6, 0, 90), (-1.9, 0, 0), (1.9, 0, 0)):
            cands.append(((pcx + dx, pcy + dy), a))
    placed = False
    for (px, py), ang in cands:
        t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(px), pcbnew.FromMM(py)))
        t.SetTextAngleDegrees(ang + fp.GetOrientationDegrees() * 0)
        # FP_TEXT angle is relative to footprint; compensate so text is 0/90 on board
        t.SetTextAngleDegrees((ang - fp.GetOrientationDegrees()) % 360)
        bb = grow(bbox_mm(t), 0.05)
        if inside_board(bb) and not any(hit(bb, o) for o in obst):
            obst.append(bb)
            placed = True
            break
    if not placed:
        t.SetLayer(pcbnew.F_Fab)
        moved_to_fab.append(ref)
print('refs moved to F.Fab:', moved_to_fab)

pcbnew.SaveBoard(PCB, board)
board = pcbnew.LoadBoard(PCB)          # reload so the board has a project (zone filler needs it)
board.BuildConnectivity()
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(PCB, board)
# KiCad 7's Python API doesn't expose the stackup; write a 0.8 mm 2-layer stackup into the file
STACKUP = '''  (setup
    (stackup
      (layer "F.SilkS" (type "Top Silk Screen"))
      (layer "F.Paste" (type "Top Solder Paste"))
      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
      (layer "F.Cu" (type "copper") (thickness 0.035))
      (layer "dielectric 1" (type "core") (thickness 0.71) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
      (layer "B.Cu" (type "copper") (thickness 0.035))
      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
      (layer "B.Paste" (type "Bottom Solder Paste"))
      (layer "B.SilkS" (type "Bottom Silk Screen"))
      (copper_finish "HAL lead-free")
      (dielectric_constraints no)
    )
'''
txt = open(PCB).read()
if '(stackup' not in txt:
    txt = txt.replace('  (setup\n', STACKUP, 1)
    open(PCB, 'w').write(txt)
print('saved', PCB)

if '--drc' in sys.argv:
    rpt = f'{OUT}/drc.rpt'
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(rpt).read()
    import re
    print(re.findall(r'\*\* Found \d+ [^*]+\*\*', txt))
