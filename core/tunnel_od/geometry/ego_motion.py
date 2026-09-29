# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Продольное движение поезда между кадрами по самой сцене, без одометрии.

Кадр переводится в развёртку вдоль оси пути: ячейки (s, theta) на участке
S_RANGE, значение -- расстояние от оси до ближайшей поверхности (как в
detection/background.py). Сдвиг вдоль пути, при котором развёртка текущего кадра
лучше всего совпадает с развёрткой кадра LAG кадров назад, -- пройденный путь.
Сравнение не с соседним кадром, а через LAG: на малой скорости сдвиг за один кадр
меньше ячейки, и совпадение тянет к нулю (сетка лучей датчика движется вместе
с поездом). Итоговая скорость -- медиана последних оценок.
"""
from collections import deque

import numpy as np

from ..detection.background import unwrap

S_RANGE = (5.0, 60.0)
S_BIN = 0.1
THETA_BIN = np.radians(3.0)
N_THETA = int(round(2 * np.pi / THETA_BIN))
LAG = 5
MAX_SPEED = 30.0
SEARCH_HALF = 1.5
CLIP_M = 0.3
MIN_CELLS = 200
SMOOTH = 5
DEFAULT_DT = 0.1
MAX_GAP_S = 1.0


class EgoMotion:
    def __init__(self, lag=LAG, max_speed=MAX_SPEED):
        self.lag = lag
        self.max_speed = max_speed
        self._max_shift = max_speed * DEFAULT_DT * lag * 1.5
        self._n = int((S_RANGE[1] - S_RANGE[0]) / S_BIN)
        self._n_img = self._n + int(np.ceil(self._max_shift / S_BIN)) + 1
        self._hist = deque(maxlen=lag)
        self._raw = deque(maxlen=SMOOTH)
        self.speed = None
        self._last_stamp = None

    def _image(self, fwd, lat, z_rel):
        theta, rho = unwrap(lat, z_rel)
        si = np.floor((fwd - S_RANGE[0]) / S_BIN).astype(np.int64)
        ok = (si >= 0) & (si < self._n_img)
        ti = np.floor((theta[ok] + np.pi) / THETA_BIN).astype(np.int64) % N_THETA
        g = np.full(self._n_img * N_THETA, np.inf)
        np.minimum.at(g, si[ok] * N_THETA + ti, rho[ok])
        g = g.reshape(self._n_img, N_THETA)
        g[np.isinf(g)] = np.nan
        return g

    def _match(self, prev, cur, shifts):
        if len(shifts) == 0:
            return None, False
        a = cur[:self._n]
        scores = np.full(len(shifts), np.inf)
        for j, k in enumerate(shifts):
            b = prev[k:k + self._n] if k >= 0 else np.vstack([np.full((-k, N_THETA), np.nan), prev[:self._n + k]])
            d = np.abs(a - b)
            ok = np.isfinite(d)
            if ok.sum() >= MIN_CELLS:
                scores[j] = np.mean(np.minimum(d[ok], CLIP_M))
        j = int(np.argmin(scores))
        if not np.isfinite(scores[j]):
            return None, False
        sub = 0.0
        if 0 < j < len(scores) - 1 and np.isfinite(scores[j - 1:j + 2]).all():
            y0, y1, y2 = scores[j - 1:j + 2]
            den = y0 - 2 * y1 + y2
            sub = 0.5 * (y0 - y2) / den if den > 0 else 0.0
        return (shifts[j] + sub) * S_BIN, j in (0, len(scores) - 1)

    def update(self, fwd, lat, z_rel, stamp=None):
        """Новый кадр в координатах пути. Возвращает (скорость м/с или None, смещение за кадр м или None)."""
        dt_frame = DEFAULT_DT if stamp is None or self._last_stamp is None else stamp - self._last_stamp
        self._last_stamp = stamp
        if not 0.0 < dt_frame <= MAX_GAP_S:
            # время назад / повтор кадра / разрыв: сдвиг не сопоставить со временем -- история заново
            self._hist.clear()
            self._raw.clear()
            self.speed = None
            dt_frame = DEFAULT_DT
        img = self._image(fwd, lat, z_rel)
        t = stamp
        if len(self._hist) == self.lag:
            prev, t_prev = self._hist[0]
            dt = DEFAULT_DT * self.lag if t is None or t_prev is None else t - t_prev
            if self.speed is None:
                lo, hi = -int(1.0 / S_BIN), int(self._max_shift / S_BIN)
            else:
                c = int(round(self.speed * dt / S_BIN))
                w = int(SEARCH_HALF / S_BIN)
                lo, hi = max(-int(1.0 / S_BIN), c - w), min(int(self._max_shift / S_BIN), c + w)
                if lo > hi:
                    lo, hi = -int(1.0 / S_BIN), int(self._max_shift / S_BIN)
            shift, at_edge = self._match(prev, img, np.arange(lo, hi + 1))
            if at_edge and self.speed is not None:
                shift, _ = self._match(prev, img, np.arange(-int(1.0 / S_BIN), int(self._max_shift / S_BIN) + 1))
            if shift is not None:
                self._raw.append(max(shift, 0.0) / dt)
                self.speed = float(np.median(self._raw))
        self._hist.append((img, t))
        return self.speed, (None if self.speed is None else self.speed * dt_frame)
