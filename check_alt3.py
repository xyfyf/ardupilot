import math
from pymavlink import mavutil
log = mavutil.mavlink_connection('00000089.bin')
data = []
while True:
    m = log.recv_match(type=['CTUN', 'XKF1'], blocking=False)
    if m is None: break
    t = getattr(m, 'TimeUS', 0)/1e6
    if 100 < t < 220:
        if m.get_type() == 'CTUN': 
            data.append({'T': t, 'type': 'CTUN', 'Alt': m.Alt, 'DAlt': m.DAlt, 'BAlt': getattr(m, 'BAlt', 0)})
        if m.get_type() == 'XKF1':
            spd = math.sqrt(m.VN**2 + m.VE**2)
            data.append({'T': t, 'type': 'SPD', 'spd': spd})

print("T(s) | Spd  | DAlt | EKF | Baro | Diff (Baro-EKF)")
for sec in range(120, 220):
    c = [x for x in data if x['type'] == 'CTUN' and sec < x['T'] < sec+1]
    s = [x for x in data if x['type'] == 'SPD' and sec < x['T'] < sec+1]
    if c and s:
        dalt = c[0]['DAlt']
        alt = c[0]['Alt']
        balt = c[0]['BAlt']
        spd = s[0]['spd']
        diff = balt - alt
        print(f"T={sec:3d} | {spd:4.1f} | {dalt:4.1f} | {alt:4.1f}| {balt:4.1f}| {diff:4.1f}")
