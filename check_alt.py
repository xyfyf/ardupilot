
import math
from pymavlink import mavutil
log = mavutil.mavlink_connection('00000089.bin')
data = []
while True:
    m = log.recv_match(type=['CTUN', 'XKF1'], blocking=False)
    if m is None: break
    t = getattr(m, 'TimeUS', 0)/1e6
    if 130 < t < 220:
        if m.get_type() == 'CTUN': 
            data.append({'T': t, 'type': 'CTUN', 'Alt': m.Alt, 'DAlt': m.DAlt, 'BAlt': getattr(m, 'BAlt', 0)})
        if m.get_type() == 'XKF1':
            spd = math.sqrt(m.VN**2 + m.VE**2)
            data.append({'T': t, 'type': 'SPD', 'spd': spd})

for sec in range(130, 220):
    c = [x for x in data if x['type'] == 'CTUN' and sec < x['T'] < sec+1]
    s = [x for x in data if x['type'] == 'SPD' and sec < x['T'] < sec+1]
    if c and s:
        dalt = c[0]['DAlt']
        alt = c[0]['Alt']
        balt = c[0]['BAlt']
        spd = s[0]['spd']
        diff = alt - dalt  # How much it thinks it is above target
        print(f'T={sec:3d}s | Spd={spd:4.1f} m/s | DAlt(Target)={dalt:4.1f}m | EKF(Alt)={alt:4.1f}m | Baro={balt:4.1f}m | Err(Alt-DAlt)={diff:5.2f}m')

