from pymavlink import mavutil
log = mavutil.mavlink_connection('00000089.bin')
alts = []
while True:
    m = log.recv_match(type=['BARO', 'CTUN'], blocking=False)
    if m is None: break
    t = getattr(m, 'TimeUS', 0)/1e6
    if 150.0 < t < 220.0:
        if m.get_type() == 'BARO':
            alts.append({'t': t, 'type': 'BARO', 'val': m.Alt})
        elif m.get_type() == 'CTUN':
            alts.append({'t': t, 'type': 'CTUN', 'val': m.Alt, 'dalt': m.DAlt})

baros = [x for x in alts if x['type'] == 'BARO']
ctuns = [x for x in alts if x['type'] == 'CTUN']

print(f"Baro Range 150-220s: {min(b['val'] for b in baros):.2f} to {max(b['val'] for b in baros):.2f}")
print(f"EKF Range 150-220s: {min(c['val'] for c in ctuns):.2f} to {max(c['val'] for c in ctuns):.2f}")
