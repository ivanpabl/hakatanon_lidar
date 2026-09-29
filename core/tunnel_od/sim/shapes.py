# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Простые фигуры для вставки в кадр: ящик, вертикальный цилиндр, шар, свисающий кабель.

Координаты (lat, fwd, z): lat = x (вбок), fwd = -y (вперёд), z -- вверх.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Box:
    fwd: float
    lat: float
    z0: float
    length: float
    width: float
    height: float
    yaw_deg: float = 0.0

    def bound(self):
        c = np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        return c, 0.5 * np.sqrt(self.length ** 2 + self.width ** 2 + self.height ** 2)

    def intersect(self, d):
        a = np.radians(self.yaw_deg)
        ca, sa = np.cos(a), np.sin(a)
        o = -np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        o = np.array([ca * o[0] + sa * o[1], -sa * o[0] + ca * o[1], o[2]])
        dl = ca * d[:, 0] + sa * d[:, 1]
        df = -sa * d[:, 0] + ca * d[:, 1]
        half = np.array([self.width, self.length, self.height]) / 2
        tmin = np.full(len(d), -np.inf)
        tmax = np.full(len(d), np.inf)
        for k, dk in enumerate((dl, df, d[:, 2])):
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (-half[k] - o[k]) / dk
                t2 = (half[k] - o[k]) / dk
            par = np.abs(dk) < 1e-12
            inside = np.abs(o[k]) <= half[k]
            t1 = np.where(par, np.where(inside, -np.inf, np.inf), t1)
            t2 = np.where(par, np.where(inside, np.inf, -np.inf), t2)
            tmin = np.maximum(tmin, np.minimum(t1, t2))
            tmax = np.minimum(tmax, np.maximum(t1, t2))
        return np.where((tmax >= tmin) & (tmin > 1e-3), tmin, np.inf)


@dataclass
class Cylinder:
    """Вертикальный цилиндр (столб, бочка, человек -- r~0.25, h~1.7)."""
    fwd: float
    lat: float
    z0: float
    radius: float
    height: float

    def bound(self):
        c = np.array([self.lat, self.fwd, self.z0 + self.height / 2])
        return c, np.sqrt(self.radius ** 2 + (self.height / 2) ** 2)

    def intersect(self, d):
        ol, of = -self.lat, -self.fwd
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = 2 * (ol * d[:, 0] + of * d[:, 1])
        c = ol ** 2 + of ** 2 - self.radius ** 2
        disc = b ** 2 - 4 * a * c
        with np.errstate(divide='ignore', invalid='ignore'):
            t_side = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
        zs = d[:, 2] * t_side
        side_ok = (disc >= 0) & (a > 1e-12) & (t_side > 1e-3) & (zs >= self.z0) & (zs <= self.z0 + self.height)
        t = np.where(side_ok, t_side, np.inf)
        with np.errstate(divide='ignore', invalid='ignore'):
            t_cap = (self.z0 + self.height) / d[:, 2]
        pl, pf = d[:, 0] * t_cap, d[:, 1] * t_cap
        cap_ok = (d[:, 2] < 0) & (t_cap > 1e-3) & ((pl - self.lat) ** 2 + (pf - self.fwd) ** 2 <= self.radius ** 2)
        return np.minimum(t, np.where(cap_ok, t_cap, np.inf))


@dataclass
class Sphere:
    fwd: float
    lat: float
    z0: float
    radius: float

    def bound(self):
        return np.array([self.lat, self.fwd, self.z0 + self.radius]), self.radius

    def intersect(self, d):
        o = -self.bound()[0]
        b = 2 * (d @ o)
        c = o @ o - self.radius ** 2
        disc = b ** 2 - 4 * c
        t = (-b - np.sqrt(np.maximum(disc, 0))) / 2
        return np.where((disc >= 0) & (t > 1e-3), t, np.inf)


def _capsule(d, a, b, r):
    """Лучи из начала координат (d -- единичные направления, N x 3) и отрезок a-b толщиной r
    с полусферами на концах: дальность до первого пересечения, inf -- мимо."""
    ba, oa = b - a, -a
    baba, baoa, oaoa = ba @ ba, ba @ oa, oa @ oa
    bard, rdoa = d @ ba, d @ oa
    k2 = baba - bard ** 2
    k1 = baba * rdoa - baoa * bard
    k0 = baba * oaoa - baoa ** 2 - r ** 2 * baba
    h = k1 ** 2 - k2 * k0
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (-k1 - np.sqrt(np.maximum(h, 0))) / k2
    y = baoa + t * bard
    t = np.where((h >= 0) & (k2 > 1e-12) & (y > 0) & (y < baba) & (t > 1e-3), t, np.inf)
    for c in (a, b):
        oc = -c
        bb = d @ oc
        disc = bb ** 2 - (oc @ oc - r ** 2)
        tc = -bb - np.sqrt(np.maximum(disc, 0))
        t = np.minimum(t, np.where((disc >= 0) & (tc > 1e-3), tc, np.inf))
    return t


@dataclass
class Cable:
    """Оборванный кабель, свисающий со свода в габарит: полилиния толщиной diameter.
    Нижний конец -- (lat, fwd, z0 + bottom), z0 -- головка рельса; вверх уходит на length,
    отклоняясь от вертикали на tilt_deg вбок (к +lat); sag -- провис: середина кабеля
    отходит от прямой вбок (к -lat) на sag, м (0 -- прямой)."""
    fwd: float
    lat: float
    z0: float
    diameter: float
    length: float
    tilt_deg: float = 0.0
    bottom: float = 2.0
    sag: float = 0.0
    n_seg: int = 8

    def points(self):
        a = np.radians(self.tilt_deg)
        s = np.linspace(0.0, 1.0, self.n_seg + 1)
        u = np.array([np.sin(a), 0.0, np.cos(a)])
        side = np.array([-np.cos(a), 0.0, np.sin(a)])
        base = np.array([self.lat, self.fwd, self.z0 + self.bottom])
        return base + np.outer(s * self.length, u) + np.outer(4 * self.sag * s * (1 - s), side)

    def bound(self):
        p = self.points()
        c = p.mean(axis=0)
        return c, float(np.linalg.norm(p - c, axis=1).max()) + self.diameter / 2

    def intersect(self, d):
        p = self.points()
        t = np.full(len(d), np.inf)
        for a, b in zip(p[:-1], p[1:]):
            t = np.minimum(t, _capsule(d, a, b, self.diameter / 2))
        return t


def make_shape(kind, dims, fwd, lat, z0, yaw_deg=0.0):
    """kind: box (length,width,height) | cylinder (radius,height) | sphere (radius) |
    cable (diameter,length[,tilt_deg[,bottom[,sag]]])."""
    if kind == 'box':
        return Box(fwd, lat, z0, *dims, yaw_deg=yaw_deg)
    if kind == 'cylinder':
        return Cylinder(fwd, lat, z0, *dims)
    if kind == 'sphere':
        return Sphere(fwd, lat, z0, *dims)
    if kind == 'cable':
        return Cable(fwd, lat, z0, *dims)
    raise ValueError(kind)
