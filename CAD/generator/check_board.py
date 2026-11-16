"""Post-build checks: PCB nets == design.py, and no right-angle bends off-pad."""
import collections, math, sys
import pcbnew
sys.path.insert(0, '.')
from design import NETS
b = pcbnew.LoadBoard(sys.argv[1])
pcbn = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetNetname():
            pcbn.setdefault(p.GetNetname(), set()).add((fp.GetReference(), p.GetNumber()))
print('pcb nets vs design:', 'MATCH' if all(pcbn.get(k) == set(v) for k, v in NETS.items()) and len(pcbn) == len(NETS) else 'MISMATCH')
def mm(v): return (round(pcbnew.ToMM(v.x), 3), round(pcbnew.ToMM(v.y), 3))
segs = [t for t in b.GetTracks() if t.GetClass() == 'PCB_TRACK']
def on_pad(pt):
    v = pcbnew.VECTOR2I(pcbnew.FromMM(pt[0]), pcbnew.FromMM(pt[1]))
    return any(p.HitTest(v) for fp in b.GetFootprints() for p in fp.Pads())
legs = collections.defaultdict(list)
for t in segs:
    a, c = mm(t.GetStart()), mm(t.GetEnd())
    legs[(a, t.GetLayer())].append((c[0] - a[0], c[1] - a[1]))
    legs[(c, t.GetLayer())].append((a[0] - c[0], a[1] - c[1]))
for t in segs:
    for e in (mm(t.GetStart()), mm(t.GetEnd())):
        for u in segs:
            if u is t or u.GetLayer() != t.GetLayer():
                continue
            a, c = mm(u.GetStart()), mm(u.GetEnd())
            if e in (a, c):
                continue
            d = (c[0] - a[0], c[1] - a[1]); L = math.hypot(*d)
            tt = ((e[0] - a[0]) * d[0] + (e[1] - a[1]) * d[1]) / L / L
            if 0 < tt < 1 and math.hypot(a[0] + tt * d[0] - e[0], a[1] + tt * d[1] - e[1]) < 0.01:
                legs[(e, t.GetLayer())] += [d, (-d[0], -d[1])]
bad = set()
for (pt, layer), vs in legs.items():
    if len(vs) < 2 or on_pad(pt):
        continue
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            u, v = vs[i], vs[j]; nu, nv = math.hypot(*u), math.hypot(*v)
            if nu < 1e-6 or nv < 1e-6:
                continue
            ang = math.degrees(math.acos(max(-1, min(1, (u[0] * v[0] + u[1] * v[1]) / nu / nv))))
            if 60 < ang < 125:
                bad.add((round(pt[0] - 100, 2), round(100 - pt[1], 2), round(ang)))
print('right-angle bends off-pad:', sorted(bad) if bad else 'none')
