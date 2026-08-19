from pymavlink import mavutil
log = mavutil.mavlink_connection('00000089.bin')
alts = []
while True:
    m = log.recv_match(type=['BARO', 'CTUN'], blocking=False)
    if m is None: break
    t = getattr(m, 'TimeUS', 0)/1e6
    if 150.0 < t < 195.0:
        if m.get_type() == 'BARO':
            alts.append({'t': t, 'type': 'BARO', 'val': m.Alt})
        elif m.get_type() == 'CTUN':
            alts.append({'t': t, 'type': 'CTUN', 'val': m.Alt, 'dalt': m.DAlt})

baros = [x for x in alts if x['type'] == 'BARO']
ctuns = [x for x in alts if x['type'] == 'CTUN']

print(f"Rectangle (150-195s):")
print(f"Barometer max reading: {max(b['val'] for b in baros):.2f} m")
print(f"Barometer min reading: {min(b['val'] for b in baros):.2f} m")
print(f"Physical Drop (Baro diff): {max(b['val'] for b in baros) - min(b['val'] for b in baros):.2f} m")
print(f"EKF max reading: {max(c['val'] for c in ctuns):.2f} m")
print(f"EKF min reading: {min(c['val'] for c in ctuns):.2f} m")
max_err = max(abs(c["val"] - c["dalt"]) for c in ctuns)
print(f"Max Control Error (EKF - Target): {max_err:.2f} m")
