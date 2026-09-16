import numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from lidar_sim import build_rays, intersect_plane_z, intersect_sphere, Z_FLOOR, MAX_RANGE
from rail_pathfit import find_floor_bumps, cluster_lines, pick_rail_pair, fit_path

OUT = Path(__file__).parent / 'out'
OUT.mkdir(exist_ok=True)


CURVE_RATE = 0.012

TUNNEL_R = 2.8
TUNNEL_CZ = 0.3
RAIL_HALF_GAUGE = 0.76
RAIL_WIDTH = 0.07
RAIL_HEIGHT = 0.15

OBSTACLE_FORWARD = 100.0
OBSTACLE_RADIUS = 0.35


def true_centerline(fwd):
    """Ground-truth path centerline: exact, by construction of the synthetic tunnel."""
    return CURVE_RATE * fwd


def intersect_curved_cylinder(dx, dy, dz):
    dx_eff = dx - CURVE_RATE * dy
    oz = -TUNNEL_CZ
    a = dx_eff ** 2 + dz ** 2
    b = 2 * (oz * dz)
    c = oz ** 2 - TUNNEL_R ** 2
    disc = b ** 2 - 4 * a * c
    t = np.full_like(dx, np.inf)
    valid = (disc >= 0) & (a > 1e-9)
    sq = np.sqrt(np.maximum(disc, 0))
    t1 = (-b + sq) / (2 * a)
    t2 = (-b - sq) / (2 * a)
    tpos = np.where(t1 > 1e-3, t1, np.inf)
    tpos = np.minimum(tpos, np.where(t2 > 1e-3, t2, np.inf))
    return np.where(valid, tpos, np.inf)


def intersect_curved_rails(dx, dy, dz):
    t = intersect_plane_z(dx, dy, dz, Z_FLOOR + RAIL_HEIGHT)
    x_hit = dx * t
    y_hit = dy * t
    rail_center = true_centerline(y_hit)
    in_band = np.abs(np.abs(x_hit - rail_center) - RAIL_HALF_GAUGE) < RAIL_WIDTH / 2
    return np.where(in_band, t, np.inf)


def simulate_curved_frame(obstacle_forward=None, obstacle_radius=0.25):
    dx, dy, dz, ring = build_rays()

    t_wall = intersect_curved_cylinder(dx, dy, dz)
    t_floor = intersect_plane_z(dx, dy, dz, Z_FLOOR)
    t_rail = intersect_curved_rails(dx, dy, dz)
    t = np.minimum(np.minimum(t_wall, t_floor), t_rail)

    if obstacle_forward is not None:
        # obstacle sits ON the curved track ahead, not at a fixed absolute lateral offset
        obs_lat = true_centerline(obstacle_forward)
        oz = Z_FLOOR + obstacle_radius
        t_obs = intersect_sphere(dx, dy, dz, (obs_lat, obstacle_forward, oz), obstacle_radius)
        t = np.minimum(t, t_obs)

    valid = np.isfinite(t) & (t < MAX_RANGE)
    t = t[valid]
    dxv, dyv, dzv = dx[valid], dy[valid], dz[valid]
    x = (dxv * t).astype(np.float32)
    y = (-dyv * t).astype(np.float32)  # sensor-frame convention: forward = -y
    z = (dzv * t).astype(np.float32)
    return x, y, z


def corridor_check(x, y, z, center_of_fwd, near_cutoff=2.0, half_width=1.0,
                    z_min=None, z_max=1.0):
    if z_min is None:
        z_min = Z_FLOOR + 0.3
    fwd = -y
    center = center_of_fwd(fwd)
    mask = (fwd > near_cutoff) & (np.abs(x - center) < half_width) & (z > z_min) & (z < z_max)
    d = fwd[mask]
    return (d.min() if d.size else np.nan), int(mask.sum())


def main():
    print(f'Синтетический изогнутый тоннель: CURVE_RATE={CURVE_RATE} м/м '
          f'(снос {CURVE_RATE*100:.1f}м за 100м вперёд)')
    print(f'Препятствие: на дистанции {OBSTACLE_FORWARD}м, точно НА кривой колее '
          f'(боковое смещение={true_centerline(OBSTACLE_FORWARD):.2f}м)\n')

    x_empty, y_empty, z_empty = simulate_curved_frame(obstacle_forward=None)
    pair, fit_fwd, fitted_cl = fit_path(x_empty, y_empty, z_empty)

    if pair is None:
        print('rail_pathfit НЕ нашёл пару рельсов на синтетическом кадре -- дальше нечего сравнивать.')
        return

    gauge = pair[3]
    print(f'rail_pathfit нашёл рельсы: колея={gauge:.2f}м (эталон 1.52м), '
          f'покрытие {fit_fwd[0]:.0f}-{fit_fwd[-1]:.0f}м')


    true_cl = true_centerline(fit_fwd)
    err = fitted_cl - true_cl
    print(f'Ошибка фита кривизны против истинной кривой: '
          f'RMS={np.sqrt(np.mean(err**2)):.3f}м, max={np.max(np.abs(err)):.3f}м\n')

    x_obs, y_obs, z_obs = simulate_curved_frame(obstacle_forward=OBSTACLE_FORWARD,
                                                 obstacle_radius=OBSTACLE_RADIUS)

    straight_center = lambda f: np.zeros_like(f)
    curved_center = lambda f: np.interp(f, fit_fwd, fitted_cl)

    print('--- Пустой (кривой) тоннель: проверка ложных тревог ---')
    d_s, n_s = corridor_check(x_empty, y_empty, z_empty, straight_center)
    d_c, n_c = corridor_check(x_empty, y_empty, z_empty, curved_center)
    print(f'  прямой коридор:      {"ложная тревога! " + f"{d_s:.1f}м" if not np.isnan(d_s) else "чисто"} ({n_s} точек)')
    print(f'  коридор по кривой:   {"ложная тревога! " + f"{d_c:.1f}м" if not np.isnan(d_c) else "чисто"} ({n_c} точек)')

    print(f'\n--- Кадр с препятствием на {OBSTACLE_FORWARD}м (лежит на изогнутой колее) ---')
    d_s, n_s = corridor_check(x_obs, y_obs, z_obs, straight_center)
    d_c, n_c = corridor_check(x_obs, y_obs, z_obs, curved_center)
    print(f'  прямой коридор:      {f"обнаружено на {d_s:.1f}м" if not np.isnan(d_s) else "ПРОПУЩЕНО"} ({n_s} точек)')
    print(f'  коридор по кривой:   {f"обнаружено на {d_c:.1f}м" if not np.isnan(d_c) else "ПРОПУЩЕНО"} ({n_c} точек)')

    # --- visualization ---
    fig, ax = plt.subplots(figsize=(9, 10))
    m = (-y_empty > 0) & (-y_empty < 130) & (np.abs(x_empty) < 5)
    ax.scatter(x_empty[m], -y_empty[m], s=0.2, c='lightgray', alpha=0.5, label='тоннель (пустой кадр)')

    fwd_line = np.linspace(0, 130, 200)
    ax.plot(true_centerline(fwd_line), fwd_line, 'g-', lw=2, label='истинная колея (ground truth)')
    ax.plot(fitted_cl, fit_fwd, 'k--', lw=2, label='фит rail_pathfit')
    ax.fill_betweenx(fwd_line, -1.0, 1.0, color='red', alpha=0.15, label='прямой коридор (±1м от x=0)')
    ax.fill_betweenx(fit_fwd, fitted_cl - 1.0, fitted_cl + 1.0, color='blue', alpha=0.15, label='коридор по кривой (±1м от фита)')
    ax.scatter([true_centerline(OBSTACLE_FORWARD)], [OBSTACLE_FORWARD], s=200, c='red', marker='X',
               zorder=5, label='препятствие на колее')

    ax.set_xlabel('x, м (вбок)')
    ax.set_ylabel('вперёд, м')
    ax.set_xlim(-5, 5)
    ax.invert_xaxis()
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('rail_pathfit + lidar_sim: обнаружение на изогнутой колее')
    fig.tight_layout()
    fig.savefig(OUT / 'rail_sim_pipeline.png', dpi=130)
    print(f'\nsaved {OUT / "rail_sim_pipeline.png"}')


if __name__ == '__main__':
    main()
