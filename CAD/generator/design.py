"""Single source of truth: parts, footprints and connections for the swarm drone FC."""

R_THT = 'Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical'
WIRE2 = 'Connector_Wire:SolderWire-0.127sqmm_1x02_P3.7mm_D0.48mm_OD1mm'

# ref: (lib_symbol, value, footprint, description)
PARTS = {
    'U1': ('swarm:XIAO_ESP32S3', 'XIAO ESP32S3', 'swarm:XIAO_ESP32S3_THT', 'Seeed XIAO ESP32S3 on pin headers'),
    'U2': ('swarm:GY-521', 'GY-521 (MPU-6050)', 'swarm:GY-521_THT', 'MPU-6050 IMU breakout on 1x8 header'),
    'J1': ('Connector_Generic:Conn_01x02', 'BATT 1S (PH2.0)', 'Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical', '1S LiPo connector'),
    'C1': ('Device:C_Polarized', '100uF 6.3V', 'Capacitor_THT:CP_Radial_D5.0mm_P2.00mm', 'Bulk cap, 5x7mm'),
    'C2': ('Device:C', '100nF', 'Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm', 'Battery sense filter'),
    'R9': ('Device:R', '100k', R_THT, 'Battery divider top'),
    'R10': ('Device:R', '100k', R_THT, 'Battery divider bottom'),
    'R11': ('Device:R', '68', R_THT, 'IR LED current (front)'),
    'R12': ('Device:R', '68', R_THT, 'IR LED current (left)'),
    'R13': ('Device:R', '68', R_THT, 'IR LED current (right)'),
    'J6': ('Connector_Generic:Conn_01x02', 'IR_FRONT', WIRE2, 'IR marker LED wires (1=anode)'),
    'J7': ('Connector_Generic:Conn_01x02', 'IR_LEFT', WIRE2, 'IR marker LED wires (1=anode)'),
    'J8': ('Connector_Generic:Conn_01x02', 'IR_RIGHT', WIRE2, 'IR marker LED wires (1=anode)'),
    'J9': ('Connector_Generic:Conn_01x02', 'XIAO_BAT', WIRE2, 'Wires to XIAO BAT+/BAT- back pads'),
    'H1': ('Mechanical:MountingHole', 'M2', 'MountingHole:MountingHole_2.2mm_M2', 'Mounting hole'),
    'H2': ('Mechanical:MountingHole', 'M2', 'MountingHole:MountingHole_2.2mm_M2', 'Mounting hole'),
}

# Betaflight Quad-X motor order: 1 RR, 2 FR, 3 RL, 4 FL
MOTORS = [
    # name, Q,    D,    Rgate, Rpd,  pads, XIAO pin, label
    ('M1_RR', 'Q1', 'D1', 'R1', 'R5', 'J2', 5, 'D4/GPIO5'),
    ('M2_FR', 'Q2', 'D2', 'R2', 'R6', 'J3', 6, 'D5/GPIO6'),
    ('M3_RL', 'Q3', 'D3', 'R3', 'R7', 'J4', 9, 'D8/GPIO7'),
    ('M4_FL', 'Q4', 'D4', 'R4', 'R8', 'J5', 10, 'D9/GPIO8'),
]
for name, q, d, rg, rpd, j, pin, lbl in MOTORS:
    PARTS[q] = ('Transistor_FET:AO3400A', 'AO3400A', 'Package_TO_SOT_SMD:SOT-23', f'{name} low-side switch')
    PARTS[d] = ('Diode:1N5819', '1N5819', 'Diode_THT:D_DO-41_SOD81_P3.81mm_Vertical_KathodeUp', f'{name} flyback')
    PARTS[rg] = ('Device:R', '100', R_THT, f'{name} gate resistor')
    PARTS[rpd] = ('Device:R', '10k', R_THT, f'{name} gate pull-down')
    PARTS[j] = ('Connector_Generic:Conn_01x02', name, WIRE2, f'{name} motor wires (1=red/+)')

NETS = {
    'VBAT': [('J1', '1'), ('C1', '1'), ('R9', '1'), ('R11', '1'), ('R12', '1'), ('R13', '1'), ('J9', '1')],
    'GND': [('J1', '2'), ('C1', '2'), ('C2', '2'), ('R10', '2'), ('U1', '13'), ('U2', '2'), ('U2', '7'),
            ('J6', '2'), ('J7', '2'), ('J8', '2'), ('J9', '2')],
    '3V3': [('U1', '12'), ('U2', '1')],
    'SDA': [('U1', '2'), ('U2', '4')],
    'SCL': [('U1', '1'), ('U2', '3')],
    'VSENSE': [('U1', '4'), ('R9', '2'), ('R10', '1'), ('C2', '1')],
    'IR_F': [('R11', '2'), ('J6', '1')],
    'IR_L': [('R12', '2'), ('J7', '1')],
    'IR_R': [('R13', '2'), ('J8', '1')],
}
for name, q, d, rg, rpd, j, pin, lbl in MOTORS:
    NETS['VBAT'] += [(j, '1'), (d, '1')]
    NETS['GND'] += [(q, '2'), (rpd, '2')]
    NETS[f'{name}_PWM'] = [('U1', str(pin)), (rg, '1')]
    NETS[f'{name}_GATE'] = [(rg, '2'), (rpd, '1'), (q, '1')]
    NETS[f'{name}_NEG'] = [(q, '3'), (d, '2'), (j, '2')]

NO_CONNECT = [('U1', '3'), ('U1', '7'), ('U1', '8'), ('U1', '11'), ('U1', '14'),
              ('U2', '5'), ('U2', '6'), ('U2', '8')]

POWER_NETS = {'VBAT', 'GND'} | {f'{m[0]}_NEG' for m in MOTORS}

XIAO_PINS = {  # pin: (name, side)
    '1': 'D0/GPIO1', '2': 'D1/GPIO2', '3': 'D2/GPIO3', '4': 'D3/GPIO4', '5': 'D4/GPIO5',
    '6': 'D5/GPIO6', '7': 'D6/GPIO43', '8': 'D7/GPIO44', '9': 'D8/GPIO7', '10': 'D9/GPIO8',
    '11': 'D10/GPIO9', '12': '3V3', '13': 'GND', '14': '5V',
}
GY521_PINS = {'1': 'VCC', '2': 'GND', '3': 'SCL', '4': 'SDA', '5': 'XDA', '6': 'XCL', '7': 'AD0', '8': 'INT'}


def check():
    seen = {}
    for net, pins in NETS.items():
        for p in pins:
            assert p[0] in PARTS, p
            assert p not in seen, (p, net, seen.get(p))
            seen[p] = net
    for p in NO_CONNECT:
        assert p not in seen, p
    return seen


if __name__ == '__main__':
    s = check()
    print(len(PARTS), 'parts,', len(NETS), 'nets,', len(s), 'connected pins')
