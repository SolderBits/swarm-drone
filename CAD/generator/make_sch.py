"""Generate the KiCad 7 schematic (swarm_fc.kicad_sch) from design.py."""
import math
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
from design import PARTS, NETS, MOTORS, NO_CONNECT  # noqa: E402
from sexpr import parse, dumps, Sym, find, findall  # noqa: E402

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else '../hw')
SYMDIR = '/usr/share/kicad/symbols'
NS = uuid.UUID('6b1c3c5e-1f7e-4a8e-9d55-2f0c6a9f0d11')
ROOT = str(uuid.uuid5(NS, 'root-sheet'))
_n = [0]


def uid():
    _n[0] += 1
    return str(uuid.uuid5(NS, f'sch-item-{_n[0]}'))


def sym_uuid(ref):
    return str(uuid.uuid5(NS, 'sym-' + ref))


# ------------------------------------------------------------ library symbols
_libcache = {}


def load_lib(lib):
    if lib not in _libcache:
        path = f'{OUT}/swarm.kicad_sym' if lib == 'swarm' else f'{SYMDIR}/{lib}.kicad_sym'
        _libcache[lib] = parse(open(path).read())
    return _libcache[lib]


def get_symbol(lib_id):
    lib, name = lib_id.split(':')
    tree = load_lib(lib)
    syms = {s[1]: s for s in findall(tree, 'symbol')}
    s = syms[name]
    ext = find(s, 'extends')
    if ext:
        parent = syms[ext[1]]
        props = {p[1]: p for p in findall(s, 'property')}
        new = [Sym('symbol'), lib_id]
        for c in parent[2:]:
            if isinstance(c, list) and c[0] == 'property' and c[1] in props:
                new.append(props.pop(c[1]))
            elif isinstance(c, list) and c[0] == 'symbol':
                sub = list(c)
                sub[1] = sub[1].replace(ext[1], name, 1)
                new.append(sub)
            else:
                new.append(c)
        for p in props.values():
            new.append(p)
        s = new
    else:
        s = list(s)
        s[1] = lib_id
    return s


def pins_of(symdef):
    out = {}
    for sub in findall(symdef, 'symbol'):
        for p in findall(sub, 'pin'):
            at = find(p, 'at')
            num = find(p, 'number')[1]
            out[num] = (float(at[1]), float(at[2]), float(at[3]))
    return out


LIBSYMS = {}
for ref, (lib_id, *_r) in PARTS.items():
    if lib_id not in LIBSYMS:
        LIBSYMS[lib_id] = get_symbol(lib_id)

# ------------------------------------------------------------ schematic items
items = []
placed = {}   # ref -> (x, y, rot)


def rot(px, py, r):
    a = math.radians(r)
    return (px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a))


def pin_pos(ref, num):
    x, y, r = placed[ref]
    px, py, pa = PIN[PARTS[ref][0]][num]
    rx, ry = rot(px, py, r)
    out_ang = (pa + 180 + r) % 360
    return (round(x + rx, 3), round(y - ry, 3), out_ang)


PIN = {k: pins_of(v) for k, v in LIBSYMS.items()}


def place(ref, x, y, r=0, ref_off=None, val_off=None):
    placed[ref] = (x, y, r)
    lib_id, value, fpid, desc = PARTS[ref]
    rx, ry = ref_off or (2.54, -2.54)
    vx, vy = val_off or (2.54, 2.54)
    node = [Sym('symbol'), [Sym('lib_id'), lib_id], [Sym('at'), x, y, r], [Sym('unit'), 1],
            [Sym('in_bom'), Sym('yes')], [Sym('on_board'), Sym('yes')], [Sym('dnp'), Sym('no')],
            [Sym('uuid'), sym_uuid(ref)]]

    def prop(name, val, px, py, hide=False, just=None):
        eff = [Sym('effects'), [Sym('font'), [Sym('size'), 1.27, 1.27]]]
        if just:
            eff.append([Sym('justify'), Sym(just)])
        if hide:
            eff.append(Sym('hide'))
        return [Sym('property'), name, val, [Sym('at'), px, py, 0], eff]
    node.append(prop('Reference', ref, x + rx, y + ry, just='left'))
    node.append(prop('Value', value, x + vx, y + vy, just='left'))
    node.append(prop('Footprint', fpid, x, y, hide=True))
    node.append(prop('Datasheet', '~', x, y, hide=True))
    node.append(prop('Description', desc, x, y, hide=True))
    for num in PIN[lib_id]:
        node.append([Sym('pin'), num, [Sym('uuid'), uid()]])
    node.append([Sym('instances'), [Sym('project'), 'swarm_fc',
                 [Sym('path'), '/' + ROOT, [Sym('reference'), ref], [Sym('unit'), 1]]]])
    items.append(node)


def wire(*pts):
    for a, b in zip(pts, pts[1:]):
        items.append([Sym('wire'), [Sym('pts'), [Sym('xy'), a[0], a[1]], [Sym('xy'), b[0], b[1]]],
                      [Sym('stroke'), [Sym('width'), 0], [Sym('type'), Sym('default')]], [Sym('uuid'), uid()]])


def junction(x, y):
    items.append([Sym('junction'), [Sym('at'), x, y], [Sym('diameter'), 0], [Sym('color'), 0, 0, 0, 0], [Sym('uuid'), uid()]])


def label(name, x, y, ang=0):
    just = {0: 'left', 180: 'right', 90: 'left', 270: 'right'}[ang]
    items.append([Sym('label'), name, [Sym('at'), x, y, ang], [Sym('fields_autoplaced')],
                  [Sym('effects'), [Sym('font'), [Sym('size'), 1.27, 1.27]], [Sym('justify'), Sym(just), Sym('bottom')]],
                  [Sym('uuid'), uid()]])


def text(t, x, y, size=1.5):
    items.append([Sym('text'), t, [Sym('at'), x, y, 0],
                  [Sym('effects'), [Sym('font'), [Sym('size'), size, size]], [Sym('justify'), Sym('left'), Sym('top')]],
                  [Sym('uuid'), uid()]])


def nc(ref, num):
    x, y, a = pin_pos(ref, num)
    items.append([Sym('no_connect'), [Sym('at'), x, y], [Sym('uuid'), uid()]])


DIR = {0: (1, 0), 180: (-1, 0), 90: (0, -1), 270: (0, 1)}


def pin_label(ref, num, net, stub=2.54):
    x, y, a = pin_pos(ref, num)
    dx, dy = DIR[int(a)]
    ex, ey = round(x + dx * stub, 3), round(y + dy * stub, 3)
    wire((x, y), (ex, ey))
    label(net, ex, ey, int(a))


def P(ref, num):
    x, y, _ = pin_pos(ref, num)
    return (x, y)


net_of = {p: n for n, ps in NETS.items() for p in ps}

# ------------------------------------------------------------ layout (A3, mm, y down)
G = 2.54
text('SWARM DRONE FLIGHT CONTROLLER  v1', 20, 12, 3)
text('XIAO ESP32-S3 + GY-521 (MPU-6050), 4x brushed coreless motors, 3x IR mocap markers, 1S LiPo.\n'
     'Power is on whenever the battery is plugged in (no switch: motor current is several amps).', 20, 18, 1.5)

# MCU
place('U1', 101.6, 76.2, 0, ref_off=(-15.24, -12.7), val_off=(-2.54, -12.7))
for num in [str(i) for i in range(1, 15)]:
    if (('U1', num)) in net_of:
        pin_label('U1', num, net_of[('U1', num)], 5.08)
    else:
        nc('U1', num)
text('ESP-FC pin map (GPIO):\n M1 RR = 5   M2 FR = 6\n M3 RL = 7   M4 FL = 8\n SDA = 2   SCL = 1\n VBAT ADC = 4 (x2 divider)', 80, 96, 1.5)

# IMU
place('U2', 172.72, 73.66, 0, ref_off=(-7.62, -12.7), val_off=(-7.62, 15.24))
for num in [str(i) for i in range(1, 9)]:
    if ('U2', num) in net_of:
        pin_label('U2', num, net_of[('U2', num)], 5.08)
    else:
        nc('U2', num)
text('GY-521 has its own 3.3V LDO and\nI2C pull-ups. AD0 low -> address 0x68.', 160, 92, 1.3)

# battery, bulk cap, XIAO battery wires
text('POWER', 20, 40, 2)
place('J1', 30.48, 55.88, 0, ref_off=(-5.08, -5.08), val_off=(-12.7, 5.08))
pin_label('J1', '1', 'VBAT', 5.08)
pin_label('J1', '2', 'GND', 5.08)
place('C1', 50.8, 58.42, 0)
pin_label('C1', '1', 'VBAT')
pin_label('C1', '2', 'GND')
place('J9', 30.48, 78.74, 0, ref_off=(-5.08, -5.08), val_off=(-12.7, 5.08))
pin_label('J9', '1', 'VBAT', 5.08)
pin_label('J9', '2', 'GND', 5.08)
text('J9: short wires to the BAT+ / BAT- pads\non the back of the XIAO (its charger\ncharges the LiPo from USB-C).', 20, 86, 1.3)

# battery divider
text('BATTERY SENSE', 20, 104, 2)
place('R9', 35.56, 116.84)
place('R10', 35.56, 132.08)
place('C2', 48.26, 132.08)
pin_label('R9', '1', 'VBAT')
mid = (35.56, 124.46)
wire(P('R9', '2'), mid, P('R10', '1'))
wire(mid, (48.26, 124.46), P('C2', '1'))
junction(*mid)
label('VSENSE', 40.64, 124.46, 0)
pin_label('R10', '2', 'GND')
pin_label('C2', '2', 'GND')

# IR markers
text('IR MOCAP MARKERS (850nm, always on)', 230, 40, 2)
for i, (r, j, net) in enumerate([('R11', 'J6', 'IR_F'), ('R12', 'J7', 'IR_L'), ('R13', 'J8', 'IR_R')]):
    y = 55.88 + i * 15.24
    place(r, 248.92, y, 90, ref_off=(-2.54, -3.81), val_off=(-2.54, 3.81))
    place(j, 274.32, y, 0, ref_off=(-2.54, -5.08), val_off=(5.08, 0))
    pin_label(r, '1', 'VBAT')
    wire(P(r, '2'), P(j, '1'))
    label(net, P(r, '2')[0] + 2.54, P(r, '2')[1], 0)
    pin_label(j, '2', 'GND', 5.08)

# motor drivers
text('MOTOR DRIVERS (Betaflight Quad-X order)', 20, 158, 2)
for k, (name, q, d, rg, rpd, j, pin, lbl) in enumerate(MOTORS):
    bx = 50.8 + k * 88.9
    by = 205.74
    place(q, bx, by, 0, ref_off=(5.08, -1.27), val_off=(5.08, 1.27))
    place(rg, bx - 12.7, by, 90, ref_off=(-2.54, -3.81), val_off=(-2.54, 3.81))
    place(rpd, bx - 7.62, by + 7.62, 0, ref_off=(2.54, -1.27), val_off=(2.54, 1.27))
    place(d, bx + 12.7, by - 12.7, 270, ref_off=(2.54, -1.27), val_off=(2.54, 1.27))
    place(j, bx + 25.4, by - 16.51, 0, ref_off=(-2.54, -5.08), val_off=(-2.54, 7.62))
    gnode = (bx - 7.62, by)
    wire(P(rg, '2'), gnode, P(q, '1'))
    wire(gnode, P(rpd, '1'))
    junction(*gnode)
    label(f'{name}_GATE', gnode[0], gnode[1] - 0, 90)
    pin_label(rg, '1', f'{name}_PWM')
    pin_label(rpd, '2', 'GND')
    pin_label(q, '2', 'GND')
    # drain -> diode anode, motor pad 2 -> diode anode
    dA = P(d, '2')
    wire(P(q, '3'), (P(q, '3')[0], dA[1]), dA)
    jm = P(j, '2')
    wire(jm, (jm[0] - 5.08, jm[1]), (jm[0] - 5.08, dA[1]), dA)
    junction(*dA)
    label(f'{name}_NEG', P(q, '3')[0], dA[1], 0)
    # diode cathode -> motor pad 1 (VBAT)
    dK = P(d, '1')
    wire(dK, P(j, '1'))
    wire(dK, (dK[0], dK[1] - 2.54))
    label('VBAT', dK[0], dK[1] - 2.54, 90)
    junction(*dK)
    text(f'{name.replace("_", " ")}\nXIAO {lbl}', bx - 22.86, by - 22.86, 1.5)

place('H1', 360.68, 30.48)
place('H2', 375.92, 30.48)

# ------------------------------------------------------------ write
lib_symbols = [Sym('lib_symbols')] + list(LIBSYMS.values())
sch = [Sym('kicad_sch'), [Sym('version'), 20230121], [Sym('generator'), Sym('eeschema')],
       [Sym('uuid'), ROOT], [Sym('paper'), 'A3'],
       [Sym('title_block'), [Sym('title'), 'Swarm drone flight controller'], [Sym('rev'), '1'],
        [Sym('comment'), 1, 'XIAO ESP32-S3 + GY-521, mocap IR markers']],
       lib_symbols] + items + [[Sym('sheet_instances'), [Sym('path'), '/', [Sym('page'), '1']]]]
open(f'{OUT}/swarm_fc.kicad_sch', 'w').write(dumps(sch) + '\n')
print('schematic written,', len(items), 'items')
