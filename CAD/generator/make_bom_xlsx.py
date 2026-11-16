"""India BOM workbook (Robu / QuartzComponents / MakerBazar / Robocraze), stock checked 8 Nov 2026."""
import csv
import sys

from openpyxl import Workbook
from openpyxl.comments import Comment
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

OUT = sys.argv[1]
CSV = sys.argv[2]
CHECKED = '8 Nov 2026'

Q, M, R, RC = 'QuartzComponents', 'MakerBazar', 'Robu', 'Robocraze'

# (section, ref, part needed, qty needed, store, listing title, url, pcs per pack, packs to buy, price per pack, notes)
ROWS = [
    ('PCB', 'U1', 'Seeed XIAO ESP32-S3 (plain, not Sense/Plus)', 1, R,
     'Seeed Studio XIAO ESP32S3 – 2.4GHz WiFi BLE 5.0',
     'https://robu.in/product/seeed-studio-xiao-esp32s3-2-4ghz-wifi-ble-5-0/', 1, 1, 1139,
     'Only store of the four with it in stock (Robocraze and MakerBazar: sold out). Seeed normally packs the u.FL antenna in the box; plug it in.'),
    ('PCB', 'U2', 'GY-521 MPU-6050 IMU module', 1, Q,
     'MPU6050 3-Axis Gyroscope/Accelerometer Sensor Module',
     'https://quartzcomponents.com/products/mpu6050-gyroscope-accelerometer-sensor', 1, 1, 127,
     'Comes with a header strip. ESP-FC also accepts an MPU6500 if a clone turns up with that chip.'),
    ('PCB', 'Q1–Q4', 'AO3400 N-MOSFET, SOT-23', 4, Q,
     'AO3400 N-Channel MOSFET SOT-23 SMD Package',
     'https://quartzcomponents.com/products/ao3400-mosfet-30v-5-8a-a09t-sot-23-c-n-channel-smd-package', 1, 5, 6,
     '1 spare: SOT-23 parts are easy to lose.'),
    ('PCB', 'D1–D4', '1N5819 Schottky, DO-41', 4, Q,
     '1N5819 Schottky Barrier Diode',
     'https://quartzcomponents.com/products/1n5819-shottky-barrier-diode', 1, 5, 3, '1 spare.'),
    ('PCB', 'R1–R4', '100 Ω, 1/4 W', 4, Q,
     '100 ohm, 1/4 Watt Resistor 5% (Pack of 10)',
     'https://quartzcomponents.com/products/100-ohm-1-4-watt-resistor', 10, 1, 7, 'Gate resistors.'),
    ('PCB', 'R5–R8', '10 kΩ, 1/4 W', 4, Q,
     '10K ohm, 1/4 Watt Resistor 5% (Pack of 10)',
     'https://quartzcomponents.com/products/10k-ohm-1-4-watt-resistor', 10, 1, 7, 'Gate pull-downs.'),
    ('PCB', 'R9, R10', '100 kΩ, 1/4 W', 2, Q,
     '100K ohm, 1/4 Watt Resistor 5% (Pack of 10)',
     'https://quartzcomponents.com/products/100k-ohm-1-4-watt-resistor', 10, 1, 6, 'Battery-voltage divider.'),
    ('PCB', 'R11–R13', '68 Ω, 1/4 W', 3, Q,
     '68 ohm, 1/4 Watt Resistor 5% (Pack of 10)',
     'https://quartzcomponents.com/products/68-ohm-1-4-watt-resistor-with-5-tolerance-pack-of-10', 10, 1, 7,
     'IR LED current, about 40 mA each.'),
    ('PCB', 'C1', '100 µF electrolytic, ≥6.3 V', 1, Q,
     '100uF 25V Radial Electrolytic Capacitor',
     'https://quartzcomponents.com/products/100uf-25v-radial-electrolytic-capacitor', 1, 2, 3,
     'A 25 V part is wider than the 5 mm outline; there is room since J1 is now just two wire holes. Bend the leads to the 2 mm holes, + to the red dot.'),
    ('PCB', 'C2', '100 nF ceramic (104), 2.5 mm pitch', 1, M,
     'Ceramic Disk Capacitors – 104 | 0.1uF | 100nF, Pack of 25',
     'https://makerbazar.in/products/ceramic-capacitor', 25, 1, 19,
     'Pick the "104" variant. Quartz\'s 0.1 µF disc pack was sold out.'),
    ('PCB', 'J1', 'Battery lead to match the MX2.0 battery plug', 1, M,
     'Mx2.0 Connector with Wire 2.0mm Battery Charging Cable – Male+Female, 2-Pins',
     'https://makerbazar.in/products/mx-pair-connector-with-wire-2-0mm-battery-charging-cable', 1, 2, 39,
     'Replaces the PH2.0 header: the in-stock 30C battery has an MX2.0 plug. Solder the half that mates with the battery into J1\'s two holes (red to +). Second pair is for the charger.'),
    ('PCB', 'U1, U2', 'Male pin headers, 2.54 mm', 22, Q,
     '40 Pin Straight Male Header Pins – Berg Strips',
     'https://quartzcomponents.com/products/40-pin-break-away-headers-straight-male-headers', 40, 2, 7,
     'XIAO needs 2 × 7 pins, GY-521 needs 1 × 8.'),
    ('Drone', 'M1–M4', '7 × 20 mm coreless motors, 2 CW + 2 CCW, with props', 4, M,
     '4-Pcs Coreless Micro Motors Kit with 4-Propellers – 720 (2CW+2CCW)',
     'https://makerbazar.in/products/magnetic-micro-coreless-motor-for-drones-quadcopters-rc-1', 4, 1, 179,
     'No 615 motor is in stock at any of the four stores, so the frame now takes 7 mm motors. Listing says wire colours may all match: spin-test each motor before soldering (see Notes).'),
    ('Drone', '—', 'Spare 55 mm props, 2 CW + 2 CCW, ~1 mm shaft', 4, M,
     '4pcs 0.9mm 55mm Two-Blade Mini Propeller CW & CCW – Red+Black [Premium]',
     'https://makerbazar.in/products/4pcs-55mm-mini-drone-propeller-set-cw-ccw-coreless-motor', 4, 1, 39,
     'Props break in the first crashes. The cheaper "Basic" variants are sold out.'),
    ('Drone', '—', '1S LiPo, 30C, ~400 mAh, ≤ 20 × 30 × 7 mm', 1, Q,
     '3.7V 30C 400mAh Lithium-Polymer Rechargeable Battery (YY702030) for Drones',
     'https://quartzcomponents.com/products/3-7v-30c-400mah-lithium-polymer-lipo-battery-yy702030', 1, 1, 237,
     '7 × 20 × 30 mm, MX2.0 plug. Generic 300 mAh "IoT" packs are rated about 1C (0.3 A) and can\'t run four motors. Buy a second one if you want to fly while one charges.'),
    ('Drone', 'J6–J8', '3 mm IR LED (850 nm preferred)', 3, Q,
     '3mm IR Transmitter LED',
     'https://quartzcomponents.com/products/3mm-ir-transmitter-led', 1, 4, 5,
     'No 3 mm 850 nm LED is stocked at any of the four stores. This one doesn\'t state its wavelength (most such LEDs are 940 nm). PS3 Eye cameras still see 940 nm, just dimmer. 1 spare.'),
    ('Drone', '—', '1S LiPo charger', 1, Q,
     'TP4056 Battery Charging Protection Module (Type C)',
     'https://quartzcomponents.com/products/tp4056-battery-charging-protection-module-type-c', 1, 1, 18,
     'Needed: the XIAO\'s 100 mA charger can\'t keep up with the ~110 mA the IR LEDs draw. Stock it charges at 1 A, too much for 400 mAh; swap its program resistor (see next row).'),
    ('Drone', '—', '3.3 kΩ resistor for the charger', 1, Q,
     '3.3K ohm, 1/4 Watt Resistor 5% (Pack of 10)',
     'https://quartzcomponents.com/products/3-3k-ohm-1-4-watt-resistor', 10, 1, 7,
     'Replaces the module\'s 1.2 kΩ PROG resistor (usually marked 122): charge current drops to about 360 mA.'),
    ('Drone', '—', 'M2 × 5–6 mm screws', 2, Q,
     'M2 x 6mm Pan Head Phillips Bolts (Pack of 4)',
     'https://quartzcomponents.com/products/m2-x-6mm-pan-head-phillips-screw-pack-of-4', 4, 1, 8,
     'They cut their own thread in the frame\'s 1.7 mm pilot holes.'),
    ('Drone', '—', 'Double-sided foam tape', 1, Q,
     'White Sponge Double-Sided Tape',
     'https://quartzcomponents.com/products/white-sponge-double-sided-acrylic-foam-adhesive-tapes', 1, 1, 23,
     'Rear PCB pillars and under the GY-521.'),
    ('Drone', '—', 'Battery strap, ≤ 21 mm wide', 1, R,
     '20cm Lipo Battery Strap Belt Reusable Cable Tie Wrap',
     'https://robu.in/product/20cm-lipo-battery-strap-belt-reusable-cable-tie-wrap/', 1, 1, 58,
     '20 mm wide. The frame\'s strap slots are now 22 mm long to suit it.'),
    ('Drone', '—', '30 AWG silicone wire, red', 1, Q,
     '30AWG Silicone Wire Red (1 meter)',
     'https://quartzcomponents.com/products/30awg-silicone-wire-red-1-meter-high-quality-ultra-flexible-wires', 1, 1, 16,
     'LED and XIAO battery wires.'),
    ('Drone', '—', '30 AWG silicone wire, black', 1, Q,
     '30AWG Silicone Wire Black (1 meter)',
     'https://quartzcomponents.com/products/30awg-silicone-wire-black-1-meter-high-quality-ultra-flexible-wire', 1, 1, 16, ''),
]

NOT_FROM_STORES = [
    ('PCB', '42 × 43.5 mm, 2 layers, 0.8 mm thick', 'JLCPCB / PCBWay / an Indian fab', 'Upload pcb/swarm_fc_gerbers.zip. 2 oz copper adds margin for the motor current (optional).'),
    ('Frame', 'cad/frame.stl (v1.1), PLA or PETG, about 6 g, 155 × 120 mm', '3D print', 'Updated for 7 × 20 mm motors, 55 mm props, the 20 mm strap and the larger 110/150 mm marker triangle.'),
]

ALTS = [
    # store, listing, url, price, stock, note
    (Q, '716 Magnetic Micro Coreless Drone Motor with 55MM Propeller', 'https://quartzcomponents.com/products/716-magnetic-micro-coreless-motor-with-55mm-propeller', 50, 'In stock', 'Fits the 7 mm tubes. Sold singly, CW/CCW not stated; 0.8 mm shaft. Weaker than the 720.'),
    (M, '(Low Cost) Magnetic Micro Coreless Motor – 716 CW / 716 CCW', 'https://makerbazar.in/products/magnetic-micro-coreless-motor-for-drones-quadcopters-rc', 59, 'In stock', 'Per motor; also 720 CW/CCW at ₹59 each. Needs 0.8 mm-hole props for the 716.'),
    (RC, 'MPU6050 Module 6-Axis Accelerometer and Gyroscope', 'https://robocraze.com/products/mpu-6050-triple-axis-accelerometer-gyroscope-module', 154, 'In stock (420 left)', 'Listing says the board behaves as an MPU6500.'),
    (M, 'MPU-6050 3-Axis Accelerometer and Gyro Sensor', 'https://makerbazar.in/products/gyroscope-sensor-mpu6050', 169, 'In stock', ''),
    (R, 'AO3400A-HUASHUO SOT-23 MOSFET', 'https://robu.in/product/1-month-warranty-414/', 6, 'In stock', 'Per piece.'),
    (R, '1N5819-MDD DO-41 Schottky', 'https://robu.in/product/1n5819-mdd-40v-600mv1a-1a-do-41-schottky-diodes-rohs/', 6, 'In stock', 'Per piece.'),
    (M, 'Schottky DIP Diodes – 1N5819, Pack of 10', 'https://makerbazar.in/products/1n5819-schottky-diode-1a-40v-do-41', 19, 'In stock', ''),
    (R, '100 uF 10V Through Hole Electrolytic Capacitor', 'https://robu.in/product/100-uf-10v-through-hole-electrolytic-capacitor-pack-of-20/', 1.36, 'In stock', 'Smaller than the 25 V part; check the pack quantity in the cart.'),
    (RC, '0.1 uF 50V Ceramic Capacitor (Pack of 10)', 'https://robocraze.com/products/0-1-uf-50v-ceramic-capacitor', 17, 'In stock', ''),
    (Q, '3.7V 30C 550mAh Lithium-Polymer Battery (YY802030)', 'https://quartzcomponents.com/products/3-7v-30c-550mah-lithium-polymer-rechargeable-battery-yy702030', 235, 'In stock', '8 mm thick and a few grams heavier; longer flights.'),
    (Q, '30cm LiPo Battery Strap Belt with Reusable Cable Tie', 'https://quartzcomponents.com/products/20cm-lipo-battery-strap-belt-with-reusable-cable-tie', 49, 'In stock', '25 mm wide: trim to 20 mm to fit the slots.'),
    (M, 'Oddy Double Sided Foam Tape 5 m – 12 mm', 'https://makerbazar.in/products/oddy-double-side-mounting-foam-tape', 29, 'In stock', ''),
    (M, '2.54mm 40-Pins Male Berg Strip – 11mm, Pack of 5', 'https://makerbazar.in/products/berg-strip-40-pins', 25, 'In stock', ''),
]

OOS = [
    (R, '615 Magnetic Micro Coreless Motor – 2xCW & 2xCCW', 'https://robu.in/product/615-magnetic-micro-coreless-motor-for-micro-quadcopters-2xcw-2xccw/', 259),
    (R, '65mm Blade Propeller + 615 CW & CCW Coreless Motor', 'https://robu.in/product/65mm-blade-propeller-prop-615-cw-ccw-coreless-brushed-motor-indoor-racing-drone-quad-copter/', 408.87),
    (Q, '615 Magnetic Micro Coreless Drone Motor', 'https://quartzcomponents.com/products/615-magnetic-micro-coreless-drone-motor', 29),
    (RC, 'Seeed Studio XIAO ESP32-S3 Development Board', 'https://robocraze.com/products/seeed-studio-xiao-esp32-s3-development-board-supports-wi-fi-bluetooth-5-0', 1067),
    (M, 'Seeed Studio XIAO Series – ESP32S3 BLE variant', 'https://makerbazar.in/products/seeed-studio-xiao-esp32s3-2-4ghz-wifi-ble-5-0', 749),
    (R, 'MPU-6050 3-Axis Accelerometer and Gyro Sensor', 'https://robu.in/product/mpu-6050-gyro-sensor-2-accelerometer/', 159),
    (R, 'PH-A 2 mm 2-pin wafer male connector, through-hole', 'https://robu.in/product/ph-a-2mm-2-pin-wafer-male-connector-through-hole-straight/', 0.33),
    (Q, '100000 pF (0.1uF) 50V Ceramic Capacitor (Pack of 5)', 'https://quartzcomponents.com/products/10000-pf-0-1uf-ceramic-capacitor', 10),
    (R, '55mm Blade Propeller for 6x15/7x20/8.5x20 mm Coreless Motor', 'https://robu.in/product/55mm-blade-propeller-prop-6x15mm-7x20mm-8-5x20mm-coreless-motor-diy-micro-quadcopter/', 77),
    (R, 'WLY702025 3.7V 300mAh 1S LiPo (also only ~1C)', 'https://robu.in/product/wly702025-300mah-3-7v-single-cell-rechargeable-lipo-battery/', 125),
    (RC, '3.7v 716 Magnetic Micro Coreless CW Motor', 'https://robocraze.com/products/cw-motor-for-rc-drone-quadcopter', 332),
    (M, 'KP Drone LiPo – 300mAh 702025 variant', 'https://makerbazar.in/products/kp-drone-lipo-batteries-3-7v-rechargeable-battery-for-mini-rc-aircraft-quadcopters', 255),
]

NOTES = [
    'Prices are what each product page showed on ' + CHECKED + ', including GST, without shipping. Stock moves fast on these sites: re-check before paying.',
    'Blue cells (pack size, packs to buy, price) are inputs. Change them and the line and store totals update.',
    'Three stores cover everything: QuartzComponents (most parts), MakerBazar (motors, props, battery leads, 100 nF) and Robu (XIAO, strap). Robocraze had nothing needed that the others lacked; its in-stock alternatives are on the Alternatives tab.',
    'What changed from the original BOM, all because the original parts were out of stock: 615 motors and 31 mm props → 720 motors and 55 mm props (frame redone: 7 mm tubes, 64 mm motor spacing); 250–300 mAh PH2.0 pack → 400 mAh 30C MX2.0 pack (J1 becomes a soldered pigtail); 850 nm LEDs → unspecified 3 mm IR LEDs; 10 mm velcro → 20 mm strap.',
    'Motor direction: front-left and rear-right spin clockwise (seen from above); front-right and rear-left counter-clockwise. Before soldering, touch each motor to the battery for a moment, note its spin, and mark it. If you end up one short, swapping a motor\'s two leads reverses it at a small cost in thrust.',
    'Ground-controller firmware: set ROTOR_RADIUS to 0.0275 for 55 mm props.',
    'Estimate for the new build: about 48 g all-up and about 100 g of thrust (roughly 2:1). Weigh the real parts.',
]


def main():
    wb = Workbook()
    base = Font(name='Arial', size=10)
    bold = Font(name='Arial', size=10, bold=True)
    head = Font(name='Arial', size=10, bold=True, color='FFFFFF')
    blue = Font(name='Arial', size=10, color='0000FF')
    link = Font(name='Arial', size=10, color='0563C1', underline='single')
    title_f = Font(name='Arial', size=14, bold=True)
    hfill = PatternFill('solid', fgColor='1F3864')
    secfill = PatternFill('solid', fgColor='D9E1F2')
    totfill = PatternFill('solid', fgColor='F2F2F2')
    thin = Side(style='thin', color='BFBFBF')
    box = Border(left=thin, right=thin, top=thin, bottom=thin)
    wrap = Alignment(wrap_text=True, vertical='top')
    rupee = '"₹"#,##0;("₹"#,##0);"-"'
    rupee2 = '"₹"#,##0.00'

    # ---------------- BOM sheet
    ws = wb.active
    ws.title = 'BOM'
    ws['A1'] = 'Swarm drone v1 – parts from Indian stores'
    ws['A1'].font = title_f
    ws['A2'] = f'Stock and prices checked on the product pages on {CHECKED}. Prices include GST; shipping not included.'
    ws['A2'].font = base
    hdr = ['#', 'Section', 'Ref', 'Part needed', 'Qty needed', 'Store', 'Listing (click to open)',
           'Pcs per pack', 'Packs to buy', 'Price per pack (₹)', 'Line total (₹)', f'Stock on {CHECKED}', 'Notes']
    r0 = 4
    for c, h in enumerate(hdr, 1):
        cell = ws.cell(row=r0, column=c, value=h)
        cell.font = head
        cell.fill = hfill
        cell.alignment = Alignment(wrap_text=True, vertical='center')
        cell.border = box
    r = r0 + 1
    first = r
    for i, (sec, ref, part, qty, store, title, url, pack, buy, price, note) in enumerate(ROWS, 1):
        vals = [i, sec, ref, part, qty, store, title, pack, buy, price, f'=I{r}*J{r}', 'In stock', note]
        for c, v in enumerate(vals, 1):
            cell = ws.cell(row=r, column=c, value=v)
            cell.font = base
            cell.alignment = wrap
            cell.border = box
        ws.cell(row=r, column=7).hyperlink = url
        ws.cell(row=r, column=7).font = link
        for c in (8, 9, 10):
            ws.cell(row=r, column=c).font = blue
        ws.cell(row=r, column=10).number_format = rupee2 if price != int(price) else rupee
        ws.cell(row=r, column=11).number_format = rupee
        ws.cell(row=r, column=12).font = Font(name='Arial', size=10, color='006100')
        r += 1
    last = r - 1
    # sanity column: packs * pcs >= qty
    ws.cell(row=r, column=10, value='Total (₹)').font = bold
    t = ws.cell(row=r, column=11, value=f'=SUM(K{first}:K{last})')
    t.font = bold
    t.number_format = rupee
    for c in range(1, 14):
        ws.cell(row=r, column=c).fill = totfill
        ws.cell(row=r, column=c).border = box
    total_row = r

    r += 2
    ws.cell(row=r, column=1, value='Not bought from these stores').font = bold
    r += 1
    for sec, what, where, note in NOT_FROM_STORES:
        ws.cell(row=r, column=2, value=sec).font = base
        ws.cell(row=r, column=4, value=what).font = base
        ws.cell(row=r, column=6, value=where).font = base
        ws.cell(row=r, column=13, value=note).font = base
        for c in (2, 4, 6, 13):
            ws.cell(row=r, column=c).alignment = wrap
        r += 1

    widths = [4, 8, 9, 30, 8, 16, 42, 8, 8, 11, 11, 13, 60]
    for c, w in enumerate(widths, 1):
        ws.column_dimensions[get_column_letter(c)].width = w
    ws.freeze_panes = ws.cell(row=r0 + 1, column=5)
    ws.cell(row=r0, column=9).comment = Comment('Recommended count, including the few spares noted. Edit freely.', 'BOM')

    # ---------------- By store
    ss = wb.create_sheet('By store')
    ss['A1'] = 'Order totals per store'
    ss['A1'].font = title_f
    ss['A2'] = 'Goods only, incl. GST. Each store adds its own shipping below its free-delivery threshold.'
    ss['A2'].font = base
    for c, h in enumerate(['Store', 'Lines', 'Subtotal (₹)'], 1):
        cell = ss.cell(row=4, column=c, value=h)
        cell.font = head
        cell.fill = hfill
        cell.border = box
    stores = [Q, M, R, RC]
    for k, s in enumerate(stores):
        rr = 5 + k
        ss.cell(row=rr, column=1, value=s).font = base
        ss.cell(row=rr, column=2, value=f"=COUNTIF(BOM!$F${first}:$F${last},A{rr})").font = base
        c3 = ss.cell(row=rr, column=3, value=f"=SUMIF(BOM!$F${first}:$F${last},A{rr},BOM!$K${first}:$K${last})")
        c3.font = base
        c3.number_format = rupee
        for c in (1, 2, 3):
            ss.cell(row=rr, column=c).border = box
    tr = 5 + len(stores)
    ss.cell(row=tr, column=1, value='Total').font = bold
    ss.cell(row=tr, column=2, value=f'=SUM(B5:B{tr - 1})').font = bold
    c = ss.cell(row=tr, column=3, value=f'=SUM(C5:C{tr - 1})')
    c.font = bold
    c.number_format = rupee
    ss.cell(row=tr + 1, column=1, value='Check: matches BOM total').font = base
    ss.cell(row=tr + 1, column=3, value=f'=IF(ABS(C{tr}-BOM!K{total_row})<0.01,"OK","MISMATCH")').font = base
    for col, w in zip('ABC', (24, 8, 14)):
        ss.column_dimensions[col].width = w

    # ---------------- Alternatives
    al = wb.create_sheet('Alternatives')
    al['A1'] = 'Other in-stock options (checked ' + CHECKED + ')'
    al['A1'].font = title_f
    for c, h in enumerate(['Store', 'Listing', 'Price (₹)', 'Stock', 'Note'], 1):
        cell = al.cell(row=3, column=c, value=h)
        cell.font = head
        cell.fill = hfill
        cell.border = box
    rr = 4
    for store, title, url, price, stock, note in ALTS:
        for c, v in enumerate([store, title, price, stock, note], 1):
            cell = al.cell(row=rr, column=c, value=v)
            cell.font = base
            cell.alignment = wrap
            cell.border = box
        al.cell(row=rr, column=2).hyperlink = url
        al.cell(row=rr, column=2).font = link
        al.cell(row=rr, column=3).number_format = rupee2 if price != int(price) else rupee
        rr += 1
    rr += 1
    al.cell(row=rr, column=1, value='Checked and out of stock').font = bold
    rr += 1
    for c, h in enumerate(['Store', 'Listing', 'Price (₹)', 'Stock'], 1):
        cell = al.cell(row=rr, column=c, value=h)
        cell.font = head
        cell.fill = hfill
        cell.border = box
    rr += 1
    for store, title, url, price in OOS:
        for c, v in enumerate([store, title, price, 'Out of stock'], 1):
            cell = al.cell(row=rr, column=c, value=v)
            cell.font = base
            cell.alignment = wrap
            cell.border = box
        al.cell(row=rr, column=2).hyperlink = url
        al.cell(row=rr, column=2).font = link
        al.cell(row=rr, column=3).number_format = rupee2 if price != int(price) else rupee
        al.cell(row=rr, column=4).font = Font(name='Arial', size=10, color='9C0006')
        rr += 1
    for col, w in zip('ABCDE', (18, 58, 10, 18, 60)):
        al.column_dimensions[col].width = w

    # ---------------- Notes
    nt = wb.create_sheet('Notes')
    nt['A1'] = 'Notes'
    nt['A1'].font = title_f
    for k, n in enumerate(NOTES):
        cell = nt.cell(row=3 + k, column=1, value=f'{k + 1}. {n}')
        cell.font = base
        cell.alignment = wrap
    nt.column_dimensions['A'].width = 120

    wb.save(OUT)

    # plain CSV copy of the main table (values, not formulas)
    with open(CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Section', 'Ref', 'Part needed', 'Qty needed', 'Store', 'Listing', 'URL',
                    'Pcs per pack', 'Packs to buy', 'Price per pack (INR)', 'Line total (INR)', 'Notes'])
        for sec, ref, part, qty, store, title, url, pack, buy, price, note in ROWS:
            w.writerow([sec, ref, part, qty, store, title, url, pack, buy, price, round(buy * price, 2), note])
        for sec, what, where, note in NOT_FROM_STORES:
            w.writerow([sec, '', what, 1, where, '', '', '', '', '', '', note])

    # self-check: every line buys enough
    for sec, ref, part, qty, store, title, url, pack, buy, price, note in ROWS:
        assert pack * buy >= qty, (ref, part)
    print('rows', len(ROWS), 'goods total', round(sum(x[8] * x[9] for x in ROWS), 2))


if __name__ == '__main__':
    main()
