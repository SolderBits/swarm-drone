"""Frame + full drone assembly in CadQuery.

Frame coordinates match the PCB: origin at board centre, +X right, +Y front, +Z up.
The frame plate's top face is z = 0 (print it flat on its bottom face, z = -1.6).
"""
import json
import math
import os
import sys

import cadquery as cq

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else '../cad')
HW = os.path.abspath(os.path.join(os.path.dirname(__file__), '../hw'))
os.makedirs(OUT, exist_ok=True)

# ------------------------------------------------------------ parameters
PLATE_T = 1.2
MOTOR = 32.0                 # motor centres at (+-MOTOR, +-MOTOR): 64 mm spacing, 90.5 mm wheelbase
MOTOR_D = 7.0                # 720 coreless motor (7 x 20 mm)
MOTOR_L = 20.0
TUBE_ID = 7.05
TUBE_OD = 9.2
TUBE_TOP = 7.0
PROP_D = 55.0                # 9 mm tip gap between neighbouring props
STANDOFF_H = 3.0             # PCB sits 3 mm above the plate (THT leads underneath)
PCB_T = 0.8
PCB_Z = STANDOFF_H           # bottom face of PCB
# IR markers: isosceles triangle, apex in front (mocap: front = 110 mm, side = 150 mm).
# v1.1: enlarged from 65/100 mm so the three LEDs stay separate blobs on wide-angle
# OV9281 USB cameras (see the mocap package's tests).
MARK_SIDE = 75.0
MARK_FRONT = math.sqrt(110.0 ** 2 - MARK_SIDE ** 2)   # 80.47
MARKERS = [(0.0, MARK_FRONT), (-MARK_SIDE, 0.0), (MARK_SIDE, 0.0)]
# standoffs / supports under the PCB (board coords)
SCREWS = [(11.0, 21.3), (-11.0, 21.3)]                # PCB holes H1, H2 (M2), front corners
PILLARS = [(-11.5, -17.0), (8.0, -17.0), (-15.5, 0.0), (15.5, 0.0)]  # plain supports (foam tape on the rear two)
BATT = (20.0, 30.0, 7.0)     # 1S 400 mAh 30C pack, YY702030 (x, y, z)

# ------------------------------------------------------------ frame
plate = (cq.Workplane('XY').workplane(offset=-PLATE_T)
         .center(0, 2.5).rect(40.0, 45.0).extrude(PLATE_T)
         .edges('|Z').fillet(4.0))
# lightening + battery strap slots
plate = (plate.faces('>Z').workplane()
         .pushPoints([(0, -12.5), (0, 16.5)]).slot2D(22, 2.4, 0).cutThruAll())   # 20 mm strap
plate = (plate.faces('>Z').workplane()
         .pushPoints([(-9.0, 2.0), (9.0, 2.0)]).rect(9.0, 22.0).cutThruAll())
plate = (plate.faces('>Z').workplane().center(0, 2.0).rect(6.0, 20.0).cutThruAll())

frame = plate

# arms: plate corner -> motor tube
for sx in (-1, 1):
    for sy in (-1, 1):
        start = cq.Vector(sx * 14.0, sy * 14.0 + 2.5, 0)
        end = cq.Vector(sx * MOTOR, sy * MOTOR, 0)
        L = (end - start).Length
        ang = math.degrees(math.atan2(end.y - start.y, end.x - start.x))
        arm = (cq.Workplane('XY').box(L, 3.5, 3.0, centered=(False, True, False))
               .translate((0, 0, -PLATE_T))
               .rotate((0, 0, 0), (0, 0, 1), ang)
               .translate((start.x, start.y, 0)))
        frame = frame.union(arm)

# motor tubes with bottom lip and clamp slit
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = sx * MOTOR, sy * MOTOR
        tube = (cq.Workplane('XY').workplane(offset=-PLATE_T).center(cx, cy)
                .circle(TUBE_OD / 2).extrude(TUBE_TOP + PLATE_T))
        bore = (cq.Workplane('XY').workplane(offset=-PLATE_T + 0.8).center(cx, cy)
                .circle(TUBE_ID / 2).extrude(TUBE_TOP + PLATE_T))
        lip_hole = (cq.Workplane('XY').workplane(offset=-PLATE_T - 0.1).center(cx, cy)
                    .circle(1.9).extrude(1.2))
        ang = math.degrees(math.atan2(cy, cx))
        slit = (cq.Workplane('XY').box(TUBE_OD, 0.8, TUBE_TOP - 1.0, centered=(False, True, False))
                .translate((0, 0, 1.0))
                .rotate((0, 0, 0), (0, 0, 1), ang)
                .translate((cx, cy, 0)))
        frame = frame.union(tube).cut(bore).cut(lip_hole).cut(slit)

# marker booms + LED holders
def boom(x1, y1, x2, y2, w=2.6, h=2.2):
    L = math.hypot(x2 - x1, y2 - y1)
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return (cq.Workplane('XY').box(L, w, h, centered=(False, True, False))
            .translate((0, 0, -PLATE_T))
            .rotate((0, 0, 0), (0, 0, 1), ang)
            .translate((x1, y1, 0)))


frame = frame.union(boom(0, 20.0, 0, MARK_FRONT, w=3.0, h=2.4))
frame = frame.union(boom(-17.0, 0, -MARK_SIDE, 0, w=3.0, h=2.4)).union(boom(17.0, 0, MARK_SIDE, 0, w=3.0, h=2.4))
for mx, my in MARKERS:
    holder = (cq.Workplane('XY').workplane(offset=-PLATE_T).center(mx, my)
              .circle(2.6).extrude(PLATE_T + 4.0))
    hole = (cq.Workplane('XY').workplane(offset=-PLATE_T + 0.8).center(mx, my)
            .circle(1.55).extrude(10))
    leg_hole = (cq.Workplane('XY').workplane(offset=-PLATE_T - 0.1).center(mx, my)
                .rect(2.8, 0.9).extrude(1.0))
    frame = frame.union(holder).cut(hole).cut(leg_hole)

# PCB standoffs (M2 self-tapping pilot 1.7 mm) and plain support pillars
for x, y in SCREWS:
    s = cq.Workplane('XY').center(x, y).circle(2.4).extrude(STANDOFF_H)
    frame = frame.union(s)
    frame = frame.cut(cq.Workplane('XY').workplane(offset=-PLATE_T - 0.1).center(x, y).circle(0.85).extrude(10))
for x, y in PILLARS:
    frame = frame.union(cq.Workplane('XY').center(x, y).circle(1.6).extrude(STANDOFF_H))

frame = frame.clean()
cq.exporters.export(frame, f'{OUT}/frame.step')
cq.exporters.export(frame, f'{OUT}/frame.stl', tolerance=0.02, angularTolerance=0.1)
frame_vol = frame.val().Volume()

# ------------------------------------------------------------ other parts (simplified models)
parts = []   # (name, shape, color rgba, density g/cm3 or fixed mass g)


def add(name, wp, color, mass=None, density=None):
    parts.append((name, wp, color, mass, density))


add('frame', frame, (0.20, 0.22, 0.25, 1), density=1.24)

pcb = cq.importers.importStep(f'{HW}/swarm_fc_board.step')
bb = pcb.val().BoundingBox()
pcb = pcb.translate((-(bb.xmin + bb.xmax) / 2 + (-21 + 21) / 2, -(bb.ymin + bb.ymax) / 2 + (23.5 - 20) / 2, PCB_Z - bb.zmin))
add('pcb', pcb, (0.05, 0.35, 0.15, 1), density=1.85)
TOP = PCB_Z + PCB_T

# XIAO ESP32-S3 on headers (USB toward rear)
xz = TOP + 2.5
xiao = cq.Workplane('XY').box(17.8, 21.0, 1.0, centered=(True, True, False)).translate((-11.5, -9.5, xz))
xiao = xiao.union(cq.Workplane('XY').box(12.5, 14.0, 2.2, centered=(True, True, False)).translate((-11.5, -8.0, xz + 1.0)))
usb = cq.Workplane('XY').box(9.0, 7.4, 3.2, centered=(True, True, False)).translate((-11.5, -18.0 - 1.2, xz + 1.0))
add('xiao', xiao, (0.12, 0.12, 0.12, 1), mass=3.0)
add('xiao_usb', usb, (0.75, 0.75, 0.78, 1), mass=0.0)
hdr_x = cq.Workplane('XY')
for sx in (-7.62, 7.62):
    hdr_x = hdr_x.union(cq.Workplane('XY').box(2.5, 17.8, 2.5, centered=(True, True, False)).translate((-11.5 + sx, -9.5, TOP)))
add('headers', hdr_x, (0.08, 0.08, 0.08, 1), mass=1.0)

# GY-521 lying over the rear-right
gz = TOP + 2.5
gy = cq.Workplane('XY').box(21.2, 16.4, 1.6, centered=(False, False, False)).translate((-1.5, -20.03, gz))
gy = gy.union(cq.Workplane('XY').box(4, 4, 0.9, centered=(True, True, False)).translate((9.1, -12.0, gz + 1.6)))
add('gy521', gy, (0.10, 0.25, 0.65, 1), mass=1.8)
add('gy_header', cq.Workplane('XY').box(20.3, 2.5, 2.5, centered=(True, True, False)).translate((9.1, -4.9, TOP)), (0.08, 0.08, 0.08, 1), mass=0.5)

# small parts from the PCB placement
sys.path.insert(0, os.path.dirname(__file__))
from layout import PLACE  # noqa: E402
smalls = cq.Workplane('XY')
for ref, (x, y, r) in PLACE.items():
    a = math.radians(r)
    def at(dx, dy):
        return (x + dx * math.cos(a) - dy * math.sin(a), y + dx * math.sin(a) + dy * math.cos(a))
    if ref.startswith('R'):
        px, py = at(0, 0)
        smalls = smalls.union(cq.Workplane('XY').center(px, py).circle(0.8).extrude(3.6).translate((0, 0, TOP + 0.3)))
    elif ref.startswith('D'):
        px, py = at(3.81, 0)
        smalls = smalls.union(cq.Workplane('XY').center(px, py).circle(1.35).extrude(5.2).translate((0, 0, TOP + 0.3)))
    elif ref.startswith('Q'):
        smalls = smalls.union(cq.Workplane('XY').box(2.9, 1.3, 1.0, centered=(True, True, False)).rotate((0, 0, 0), (0, 0, 1), r + 90).translate((x, y, TOP)))
    elif ref == 'C1':
        px, py = at(1.0, 0)
        add('c1', cq.Workplane('XY').center(px, py).circle(2.5).extrude(7.0).translate((0, 0, TOP)), (0.15, 0.2, 0.55, 1), mass=0.5)
    elif ref == 'J1':
        add('j1', cq.Workplane('XY').box(4.5, 6.0, 6.0, centered=(True, True, False)).translate((0, 15.8, TOP)), (0.95, 0.95, 0.92, 1), mass=0.3)
add('smd_tht_parts', smalls, (0.55, 0.45, 0.30, 1), mass=1.5)

# motors + props
motors = cq.Workplane('XY')
props = cq.Workplane('XY')
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = sx * MOTOR, sy * MOTOR
        z0 = -PLATE_T + 0.8
        motors = motors.union(cq.Workplane('XY').center(cx, cy).circle(MOTOR_D / 2).extrude(MOTOR_L).translate((0, 0, z0)))
        motors = motors.union(cq.Workplane('XY').center(cx, cy).circle(0.4).extrude(4).translate((0, 0, z0 + MOTOR_L)))
        hub = cq.Workplane('XY').center(cx, cy).circle(2.2).extrude(4.0).translate((0, 0, z0 + MOTOR_L + 0.8))
        blades = cq.Workplane('XY')
        for k in range(2):           # two-blade 55 mm props
            bl = (cq.Workplane('XY').ellipse(PROP_D / 4, 2.6).extrude(0.6)
                  .translate((PROP_D / 4, 0, 0)).rotate((0, 0, 0), (0, 0, 1), 45 + 180 * k + (0 if sx * sy > 0 else 45))
                  .translate((cx, cy, z0 + MOTOR_L + 2.5)))
            blades = blades.union(bl)
        props = props.union(hub).union(blades)
add('motors', motors, (0.70, 0.70, 0.72, 1), mass=4 * 4.3)
add('props', props, (0.85, 0.15, 0.15, 1), mass=4 * 0.5)

# IR markers
leds = cq.Workplane('XY')
for mx, my in MARKERS:
    leds = leds.union(cq.Workplane('XY').center(mx, my).circle(1.5).extrude(4.6).translate((0, 0, -PLATE_T + 0.8)))
    leds = leds.union(cq.Workplane('XY').sphere(1.5).translate((mx, my, -PLATE_T + 5.4)))
add('ir_leds', leds, (0.55, 0.15, 0.75, 1), mass=3 * 0.2)

# battery under the plate
bat = cq.Workplane('XY').box(*BATT, centered=(True, True, False)).translate((0, 2.0, -PLATE_T - BATT[2]))
add('battery', bat, (0.85, 0.75, 0.15, 1), mass=10.0)

# ------------------------------------------------------------ assembly export
assy = cq.Assembly(name='swarm_drone')
for name, wp, color, mass, dens in parts:
    assy.add(wp, name=name, color=cq.Color(*color))
assy.save(f'{OUT}/swarm_drone_assembly.step')

# ------------------------------------------------------------ mass estimate
masses = {}
for name, wp, color, mass, dens in parts:
    if mass is not None:
        masses[name] = mass
    else:
        masses[name] = round(wp.val().Volume() / 1000.0 * dens, 2)
masses['wires_solder_screws'] = 2.0   # incl. MX2.0 battery pigtail
total = round(sum(masses.values()), 1)
thrust = 4 * 25.0   # 720 motor + 55 mm prop on 1S, rough per-motor figure
info = {'frame_volume_mm3': round(frame_vol, 1), 'masses_g': masses, 'total_g': total,
        'est_thrust_g_720_55mm_1S': thrust, 'thrust_to_weight': round(thrust / total, 2),
        'motor_spacing_mm': 2 * MOTOR, 'wheelbase_mm': round(2 * MOTOR * math.sqrt(2), 1),
        'markers_mm': MARKERS, 'mocap_front_m': 0.110, 'mocap_side_m': 0.150}
json.dump(info, open(f'{OUT}/mass_estimate.json', 'w'), indent=2)
print(json.dumps(info, indent=2))

# tessellated meshes for rendering
import pickle  # noqa: E402
mesh = []
for name, wp, color, mass, dens in parts:
    for solid in wp.solids().vals() if hasattr(wp, 'solids') else [wp.val()]:
        verts, tris = solid.tessellate(0.1, 0.3)
        mesh.append((name, [(v.x, v.y, v.z) for v in verts], tris, color))
pickle.dump(mesh, open('/tmp/drone_mesh.pkl', 'wb'))
print('mesh parts:', len(mesh))
