"""Точность продления оси по стенам на синтетической дуге (стены +-2м, редкие точки дальше 60м)."""
import sys
import numpy as np
from tunnel_od.geometry.path import extend_path_by_walls


def scene(R, sparse_from=60.0, step=6.0):
    center = lambda s: s ** 2 / (2 * R)
    s = np.r_[np.arange(2, sparse_from, 0.2), np.arange(sparse_from, 180, step)]
    xs, ys, zs = [], [], []
    for side in (1, -1):
        for h in np.linspace(0.6, 2.4, 6):
            xs.append(center(s) + side * 2.0); ys.append(-s); zs.append(np.full_like(s, h))
    return center, *map(np.concatenate, (xs, ys, zs))


if __name__ == '__main__':
    for R in (300.0, 400.0, 800.0, 1e9):
        center, x, y, z = scene(R)
        ff = np.arange(2.0, 40.0, 1.0)
        f, c = extend_path_by_walls(x, y, z, ff, center(ff), lambda f: np.zeros_like(f))
        e = c - center(f)
        at = {d: np.round(e[np.argmin(np.abs(f - d))], 2) for d in (50, 80, 100, 130, 160) if f[-1] >= d}
        print(f'R={R:>6.0f}  конец {f[-1]:.0f}м  ошибка по дальности {at}')
