"""Write the project's custom footprints (swarm.pretty) and symbols (swarm.kicad_sym)."""
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
from design import XIAO_PINS, GY521_PINS  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else '../hw'
os.makedirs(f'{OUT}/swarm.pretty', exist_ok=True)


def tid():
    return str(uuid.uuid4())


def font(size=1.0, th=0.15):
    return f'(effects (font (size {size} {size}) (thickness {th})))'


def line(x1, y1, x2, y2, layer, w=0.12):
    return f'  (fp_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width {w}) (type solid)) (layer "{layer}") (tstamp {tid()}))\n'


def rect(x1, y1, x2, y2, layer, w=0.12):
    return f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width {w}) (type solid)) (fill none) (layer "{layer}") (tstamp {tid()}))\n'


def text(kind, t, x, y, layer, size=1.0, th=0.15, rot=0):
    return f'  (fp_text {kind} "{t}" (at {x} {y} {rot}) (layer "{layer}") {font(size, th)} (tstamp {tid()}))\n'


def pad(num, x, y, shape='circle', size=1.7, drill=1.0):
    return (f'  (pad "{num}" thru_hole {shape} (at {x} {y}) (size {size} {size}) (drill {drill}) '
            f'(layers "*.Cu" "*.Mask") (tstamp {tid()}))\n')


# ---------------------------------------------------------------- XIAO ESP32S3
# Top view, USB-C at the top (-y). Pins 1-7 down the left, 8-14 up the right
# (same numbering as Seeed's pinout: 1=D0 ... 7=D6, 8=D7 ... 11=D10, 12=3V3, 13=GND, 14=5V).
fp = '(footprint "XIAO_ESP32S3_THT" (version 20221018) (generator pcbnew)\n  (layer "F.Cu")\n'
fp += '  (descr "Seeed Studio XIAO ESP32S3 mounted on 2.54mm pin headers (21 x 17.8 mm), USB-C toward -Y")\n'
fp += '  (tags "XIAO ESP32S3 Seeed")\n  (attr through_hole)\n'
fp += text('reference', 'REF**', 0, -1.5, 'F.SilkS')
fp += text('value', 'XIAO_ESP32S3', 0, 1.5, 'F.Fab')
fp += rect(-8.7, -10.3, 8.7, 10.3, 'F.SilkS')
fp += rect(-8.9, -10.5, 8.9, 10.5, 'F.Fab', 0.1)
fp += rect(-4.6, -11.8, 4.6, -7.6, 'F.Fab', 0.1)          # USB-C receptacle
fp += text('user', 'USB-C', 0, -9.2, 'F.SilkS', 0.8, 0.12)
fp += rect(-9.15, -12.05, 9.15, 10.75, 'F.CrtYd', 0.05)
for i in range(7):
    y = round(-7.62 + i * 2.54, 3)
    lnum, rnum = str(1 + i), str(14 - i)
    fp += pad(lnum, -7.62, y, 'rect' if lnum == '1' else 'circle')
    fp += pad(rnum, 7.62, y)
    fp += text('user', XIAO_PINS[lnum].split('/')[0], -5.2, y, 'F.SilkS', 0.8, 0.12)
    fp += text('user', XIAO_PINS[rnum].split('/')[0], 5.0, y, 'F.SilkS', 0.8, 0.12)
fp += ')\n'
open(f'{OUT}/swarm.pretty/XIAO_ESP32S3_THT.kicad_mod', 'w').write(fp)

# ---------------------------------------------------------------- GY-521
# Header along the module's front edge (-Y); module body extends toward +Y.
# Pad numbers follow the module pinout VCC GND SCL SDA XDA XCL AD0 INT.
# With the module lying chip-up over the board, VCC ends up on the right.
fp = '(footprint "GY-521_THT" (version 20221018) (generator pcbnew)\n  (layer "F.Cu")\n'
fp += '  (descr "GY-521 MPU-6050 breakout (21.2 x 16.4 mm) on a 1x8 2.54mm header, lying flat over the board")\n'
fp += '  (tags "GY-521 MPU6050 IMU")\n  (attr through_hole)\n'
fp += text('reference', 'REF**', 8.89, 8.0, 'F.SilkS')
fp += text('value', 'GY-521', 8.89, 10.0, 'F.Fab')
fp += rect(-1.71, -1.27, 19.49, 15.13, 'F.SilkS')
fp += rect(-1.71, -1.27, 19.49, 15.13, 'F.Fab', 0.1)
fp += rect(-1.96, -1.52, 19.74, 15.38, 'F.CrtYd', 0.05)
fp += text('user', 'IMU - match pin labels', 8.89, 5.5, 'F.SilkS', 0.8, 0.12)
for num, name in GY521_PINS.items():
    x = round((8 - int(num)) * 2.54, 3)
    fp += pad(num, x, 0, 'rect' if num == '1' else 'circle')
    fp += text('user', name, x, 2.4, 'F.SilkS', 0.8, 0.12, 90)
fp += ')\n'
open(f'{OUT}/swarm.pretty/GY-521_THT.kicad_mod', 'w').write(fp)


# ---------------------------------------------------------------- symbols
def sym_pin(num, name, x, y, ang, typ='passive', length=2.54):
    return (f'      (pin {typ} line (at {x} {y} {ang}) (length {length})\n'
            f'        (name "{name}" (effects (font (size 1.27 1.27))))\n'
            f'        (number "{num}" (effects (font (size 1.27 1.27))))\n      )\n')


def symbol(name, ref, value, fpname, desc, body, pins):
    s = f'  (symbol "{name}" (in_bom yes) (on_board yes)\n'
    s += f'    (property "Reference" "{ref}" (at 0 1.27 0) (effects (font (size 1.27 1.27))))\n'
    s += f'    (property "Value" "{value}" (at 0 -1.27 0) (effects (font (size 1.27 1.27))))\n'
    s += f'    (property "Footprint" "swarm:{fpname}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))\n'
    s += '    (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))\n'
    s += f'    (property "ki_description" "{desc}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))\n'
    s += f'    (symbol "{name}_0_1"\n{body}    )\n'
    s += f'    (symbol "{name}_1_1"\n{pins}    )\n  )\n'
    return s


lib = '(kicad_symbol_lib (version 20220914) (generator kicad_symbol_editor)\n'
body = '      (rectangle (start -15.24 10.16) (end 15.24 -10.16) (stroke (width 0.254) (type default)) (fill (type background)))\n'
pins = ''
for i in range(7):
    y = 7.62 - i * 2.54
    l, r = str(1 + i), str(14 - i)
    pins += sym_pin(l, XIAO_PINS[l], -17.78, y, 0, 'bidirectional')
    rtype = {'12': 'power_out', '13': 'power_in', '14': 'power_in'}.get(r, 'bidirectional')
    pins += sym_pin(r, XIAO_PINS[r], 17.78, y, 180, rtype)
lib += symbol('XIAO_ESP32S3', 'U', 'XIAO_ESP32S3', 'XIAO_ESP32S3_THT', 'Seeed Studio XIAO ESP32S3 module', body, pins)

body = '      (rectangle (start -7.62 10.16) (end 7.62 -12.7) (stroke (width 0.254) (type default)) (fill (type background)))\n'
pins = ''
types = {'1': 'power_in', '2': 'power_in', '3': 'input', '4': 'bidirectional', '5': 'bidirectional',
         '6': 'output', '7': 'input', '8': 'output'}
for num, name in GY521_PINS.items():
    pins += sym_pin(num, name, -10.16, 7.62 - (int(num) - 1) * 2.54, 0, types[num])
lib += symbol('GY-521', 'U', 'GY-521', 'GY-521_THT', 'GY-521 MPU-6050 accelerometer/gyro breakout', body, pins)
lib += ')\n'
open(f'{OUT}/swarm.kicad_sym', 'w').write(lib)

open(f'{OUT}/fp-lib-table', 'w').write(
    '(fp_lib_table\n  (version 7)\n  (lib (name "swarm")(type "KiCad")(uri "${KIPRJMOD}/swarm.pretty")(options "")(descr "Swarm drone parts"))\n)\n')
open(f'{OUT}/sym-lib-table', 'w').write(
    '(sym_lib_table\n  (version 7)\n  (lib (name "swarm")(type "KiCad")(uri "${KIPRJMOD}/swarm.kicad_sym")(options "")(descr "Swarm drone parts"))\n)\n')
print('libs written to', OUT)
