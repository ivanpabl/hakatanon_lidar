import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
import json

ts = get_typestore(Stores.ROS2_HUMBLE)
ROOT = Path('/home/pablo/Documents/hakaton/Датасет/archive/for_hackathon')

def load_full(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = buf[:, 0:4].view(np.float32).ravel()
    y = buf[:, 4:8].view(np.float32).ravel()
    z = buf[:, 8:12].view(np.float32).ravel()
    ring = buf[:, 16:18].view(np.uint16).ravel()
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[valid], y[valid], z[valid], ring[valid]

bag = ROOT / 'doubleT_platform'
with AnyReader([bag], default_typestore=ts) as reader:
    conn = reader.connections[0]
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[conn])):
        if i == 10:
            msg = reader.deserialize(rawdata, connection.msgtype)
            x, y, z, ring = load_full(msg)
            break

rng = np.sqrt(x**2 + y**2 + z**2)
real_hit = rng > 0.5  
el = np.degrees(np.arcsin(np.clip(z / np.maximum(rng, 1e-6), -1, 1)))

n_rings = int(ring.max()) + 1
table = {}
active = 0
for r in range(n_rings):
    sel = (ring == r) & real_hit
    n = int(sel.sum())
    if n >= 20:
        table[r] = float(np.median(el[sel]))
        active += 1

print(f'total rings={n_rings}, active (>=20 real hits) rings={active}')
els = np.array(sorted(table.values()))
print('elevation range of active rings:', els.min(), 'to', els.max(), 'deg')
print('median spacing between sorted active elevations:', np.median(np.diff(els)), 'deg')
print('min spacing:', np.diff(els).min(), 'max spacing:', np.diff(els).max())

out_path = Path(__file__).parent / 'ring_table.json'
out_path.write_text(json.dumps(table))
print('saved ring table to', out_path)


for r in sorted(table.keys()):
    print(r, round(table[r], 3))
