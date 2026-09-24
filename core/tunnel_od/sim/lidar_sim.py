"""Синтетический тоннель: трассировка лучей 128-канального лидара (реальная таблица
углов каналов ring_table.json) в цилиндр + пол + рельсы + шар-препятствие."""
import json
import numpy as np
from pathlib import Path


_RING_TABLE_PATH = Path(__file__).parent / 'ring_table.json'
_ring_table = json.loads(_RING_TABLE_PATH.read_text())
REAL_ELEVATIONS_DEG = np.array([_ring_table[str(r)] for r in sorted(map(int, _ring_table.keys()))])

N_CHANNELS = len(REAL_ELEVATIONS_DEG)  
N_AZIMUTH = 2400 
TUNNEL_R = 2.8
TUNNEL_CX, TUNNEL_CZ = 0.5, 0.3 
Z_FLOOR = -1.9
RAIL_HALF_GAUGE = 0.76
RAIL_WIDTH = 0.07
RAIL_HEIGHT = 0.15
MAX_RANGE = 320.0


def build_rays():
    az = np.linspace(0, 2 * np.pi, N_AZIMUTH, endpoint=False)
    el = np.radians(REAL_ELEVATIONS_DEG) 
    AZ, EL = np.meshgrid(az, el)  
    ring = np.repeat(np.arange(N_CHANNELS), N_AZIMUTH)
    AZ, EL = AZ.ravel(), EL.ravel()
    dx = np.cos(EL) * np.sin(AZ)
    dy = np.cos(EL) * np.cos(AZ)
    dz = np.sin(EL)
    return dx, dy, dz, ring


def intersect_cylinder(dx, dy, dz):
    # cylinder axis along Y at (x=TUNNEL_CX, z=TUNNEL_CZ), radius TUNNEL_R
    ox, oz = -TUNNEL_CX, -TUNNEL_CZ
    a = dx ** 2 + dz ** 2
    b = 2 * (ox * dx + oz * dz)
    c = ox ** 2 + oz ** 2 - TUNNEL_R ** 2
    disc = b ** 2 - 4 * a * c
    t = np.full_like(dx, np.inf)
    valid = (disc >= 0) & (a > 1e-9)
    sq = np.sqrt(np.maximum(disc, 0))
    t1 = (-b + sq) / (2 * a)
    t2 = (-b - sq) / (2 * a)
    tpos = np.where(t1 > 1e-3, t1, np.inf)
    tpos = np.minimum(tpos, np.where(t2 > 1e-3, t2, np.inf))
    t = np.where(valid, tpos, np.inf)
    return t


def intersect_plane_z(dx, dy, dz, z_plane):
    t = np.full_like(dz, np.inf)
    down = dz < -1e-6
    t[down] = (z_plane - 0) / dz[down]
    t[t < 1e-3] = np.inf
    return t


def intersect_rail_strips(dx, dy, dz):
    # approximate rails as raised horizontal planes only within their lateral band; use plane at z=Z_FLOOR+RAIL_HEIGHT
    t = intersect_plane_z(dx, dy, dz, Z_FLOOR + RAIL_HEIGHT)
    x_hit = dx * t
    in_band = (np.abs(np.abs(x_hit) - RAIL_HALF_GAUGE) < RAIL_WIDTH / 2)
    t_out = np.where(in_band, t, np.inf)
    return t_out


def intersect_sphere(dx, dy, dz, center, radius):
    cx, cy, cz = center
    ox, oy, oz = -cx, -cy, -cz
    b = 2 * (ox * dx + oy * dy + oz * dz)
    c = ox ** 2 + oy ** 2 + oz ** 2 - radius ** 2
    disc = b ** 2 - 4 * c
    t = np.full_like(dx, np.inf)
    valid = disc >= 0
    sq = np.sqrt(np.maximum(disc, 0))
    t1 = (-b - sq) / 2
    t1 = np.where(t1 > 1e-3, t1, np.inf)
    t = np.where(valid, t1, np.inf)
    return t


def simulate_frame(obstacle_forward=None, obstacle_lateral=0.0, obstacle_height=None, obstacle_radius=0.25):
    dx, dy, dz, ring = build_rays()

    t_wall = intersect_cylinder(dx, dy, dz)
    t_floor = intersect_plane_z(dx, dy, dz, Z_FLOOR)
    t_rail = intersect_rail_strips(dx, dy, dz)

    t = np.minimum(np.minimum(t_wall, t_floor), t_rail)
    surface = np.where(t == t_rail, 2, np.where(t == t_floor, 1, 0))  # 0=wall,1=floor,2=rail

    if obstacle_forward is not None:
        oz = obstacle_height if obstacle_height is not None else (Z_FLOOR + obstacle_radius)
        t_obs = intersect_sphere(dx, dy, dz, (obstacle_lateral, obstacle_forward, oz), obstacle_radius)
        hit_obs = t_obs < t
        t = np.minimum(t, t_obs)
        surface = np.where(hit_obs, 3, surface)  # 3=obstacle

    valid = np.isfinite(t) & (t < MAX_RANGE)
    t = t[valid]
    dxv, dyv, dzv, ringv, surfv = dx[valid], dy[valid], dz[valid], ring[valid], surface[valid]

    X = dxv * t
    Y = dyv * t
    Z = dzv * t

    intensity = np.select(
        [surfv == 0, surfv == 1, surfv == 2, surfv == 3],
        [15.0, 10.0, 60.0, 40.0],
    ).astype(np.float32)
    intensity += np.random.normal(0, 2.0, size=intensity.shape).astype(np.float32)

    # convert to sensor-frame convention used by real bags: forward = -y
    x_out = X.astype(np.float32)
    y_out = (-Y).astype(np.float32)
    z_out = Z.astype(np.float32)

    n_obstacle_hits = int((surfv == 3).sum()) if obstacle_forward is not None else 0
    return x_out, y_out, z_out, intensity, ringv.astype(np.uint16), n_obstacle_hits


if __name__ == '__main__':
    print(f'sensor model: {N_CHANNELS} channels (real, non-uniform), {N_AZIMUTH} azimuth steps, '
          f'{N_CHANNELS * N_AZIMUTH} pts/scan, elevation {REAL_ELEVATIONS_DEG.min():.1f}..{REAL_ELEVATIONS_DEG.max():.1f} deg')
    print()
    for radius in [0.35, 0.15]:
        print(f'--- obstacle radius {radius} m (diameter {2*radius:.2f} m) ---')
        for d in [None, 10, 20, 30, 40, 50, 75, 100, 150, 200, 250, 300]:
            x, y, z, i, r, n_hits = simulate_frame(obstacle_forward=d, obstacle_radius=radius)
            if d is None:
                print(f'  empty tunnel: {len(x)} pts, false hits on empty-tunnel baseline: n/a')
                continue
            # angular gap analysis: elevation of obstacle center vs nearest real channel
            obs_el_deg = np.degrees(np.arctan2(Z_FLOOR + radius, d))
            nearest_gap = np.min(np.abs(REAL_ELEVATIONS_DEG - obs_el_deg))
            print(f'  {d:>4}m: {n_hits:4d} rays hit obstacle | nearest channel gap={nearest_gap:.3f} deg '
                  f'(~{np.radians(nearest_gap)*d:.2f}m miss margin at this range)')
