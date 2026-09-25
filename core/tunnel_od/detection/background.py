"""Остаток до фона в развёртке вдоль оси пути.

Точки переводятся в координаты пути: s -- вдоль оси, theta -- угол вокруг оси,
rho -- расстояние от оси (ось на AXIS_HEIGHT над головкой рельса). Пустой тоннель
в этих координатах почти не меняется вдоль s: для каждого угла ближайшая
поверхность (рельс, лоток, стена, свод, кабели) лежит на одном и том же rho.
Фон ячейки (s, theta) -- медиана по соседним метрам вдоль пути ближайшего rho
в ячейках того же угла. Препятствие занимает несколько метров вдоль пути и в
медиану не попадает, поэтому его точки оказываются ближе фона.
"""
import numpy as np

AXIS_HEIGHT = 1.8
S_BIN = 1.0
THETA_BIN = np.radians(3.0)
N_THETA = int(round(2 * np.pi / THETA_BIN))
WINDOWS = (10, 25)
MIN_CELLS = 5
RESIDUAL_M = 0.3
RESIDUAL_PER_M = 0.005
RESIDUAL_FROM_M = 40.0
ZONE = ((0.05, 3.7, 1.35),)
FAR_HALF_WIDTH = 1.0


def residual_threshold(fwd, base=RESIDUAL_M, per_m=RESIDUAL_PER_M, from_m=RESIDUAL_FROM_M):
    return base + per_m * np.maximum(fwd - from_m, 0.0)


def unwrap(lat, z_rel, axis_height=AXIS_HEIGHT):
    """(theta, rho) точек относительно оси пути на высоте axis_height над головкой рельса."""
    dz = z_rel - axis_height
    return np.arctan2(dz, lat), np.hypot(lat, dz)


def background_residual(fwd, lat, z_rel, query, s_max):
    """Остаток rho_фона - rho для точек query (маска); nan -- фон в этой ячейке не оценить."""
    theta, rho = unwrap(lat, z_rel)
    si = np.floor(fwd / S_BIN).astype(np.int64)
    ti = np.floor((theta + np.pi) / THETA_BIN).astype(np.int64) % N_THETA
    n_s = int(s_max / S_BIN) + 1
    ok = (si >= 0) & (si < n_s)
    grid = np.full(n_s * N_THETA, np.inf)
    np.minimum.at(grid, si[ok] * N_THETA + ti[ok], rho[ok])
    grid = grid.reshape(n_s, N_THETA)
    grid[np.isinf(grid)] = np.nan

    q = np.where(query & ok)[0]
    cells, inv = np.unique(si[q] * N_THETA + ti[q], return_inverse=True)
    bg = np.full(len(cells), np.nan)
    for k, c in enumerate(cells):
        a, t = divmod(int(c), N_THETA)
        for w in WINDOWS:
            col = grid[max(0, a - w):a + w + 1, t]
            col = col[np.isfinite(col)]
            if len(col) >= MIN_CELLS:
                bg[k] = np.median(col)
                break
    res = np.full(len(fwd), np.nan)
    res[q] = bg[inv] - rho[q]
    return res
