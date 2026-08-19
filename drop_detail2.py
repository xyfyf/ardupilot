from pymavlink import mavutil
log = mavutil.mavlink_connection('00000089.bin')
alts = []
while True:
    m = log.recv_match(type=['BARO', 'CTUN'], blocking=False)
    if m is None: break
    t = getattr(m, 'TimeUS', 0)/1e6
    if 146.5 < t < 150.0:
        if m.get_type() == 'BARO':
            alts.append({'t': t, 'type': 'BARO', 'val': m.Alt})
        elif m.get_type() == 'CTUN':
            alts.append({'t': t, 'type': 'CTUN', 'val': m.Alt, 'dalt': m.DAlt})

baros = [x for x in alts if x['type'] == 'BARO']
ctuns = [x for x in alts if x['type'] == 'CTUN']

max_baro = max(b["val"] for b in baros)
min_baro = min(b["val"] for b in baros)
print(f"Barometer max reading (physical peak): {max_baro:.2f} m")
print(f"Barometer min reading (physical valley): {min_baro:.2f} m")
print(f"Physical Drop (Baro diff): {max_baro - min_baro:.2f} m")

max_ekf = max(c["val"] for c in ctuns)
min_ekf = min(c["val"] for c in ctuns)
print(f"EKF max reading: {max_ekf:.2f} m")
print(f"EKF min reading: {min_ekf:.2f} m")

max_err = max(abs(c["val"] - c["dalt"]) for c in ctuns)
print(f"Max Control Error (EKF - Target): {max_err:.2f} m")
