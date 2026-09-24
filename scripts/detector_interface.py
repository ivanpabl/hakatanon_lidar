"""
Стабильный контракт между "алгоритмом обнаружения" и "остальной командой"
(ROS2-нода, Docker, демо). Ничего в этом файле не знает про rclpy/ROS2/Docker --
только numpy. Кто угодно может вызвать ObstacleDetector из живой ноды, из
offline-скрипта на bag-файлах, или из тестов -- интерфейс не меняется.

Как это использовать в ROS2-ноде (псевдокод, для человека, который её пишет):

    from detector_interface import ObstacleDetector, parse_pointcloud2

    det = ObstacleDetector()
    frame_i = 0

    def on_pointcloud(msg):  # msg: sensor_msgs.msg.PointCloud2
        global frame_i
        x, y, z = parse_pointcloud2(msg.data, msg.point_step)
        result = det.detect(x, y, z, refit_path=(frame_i % 10 == 0))
        # refit_path=True раз в 10 кадров -- геометрия пути пересчитывается
        # не каждый кадр (~90мс), проверка препятствия -- каждый кадр (<2мс)
        publish(result)
        frame_i += 1
"""
import numpy as np

from rail_pathfit import fit_path, extend_path_by_walls

# --- сырой парсинг PointCloud2 -----------------------------------------
# Layout в записях: x,y,z,intensity (float32), ring (uint16), timestamp (float64)
# -> point_step=26 байт. Если передать msg.fields, смещения x/y/z берутся из них.
# Работает как с сообщением из rosbags (offline), так и с настоящим
# sensor_msgs/msg/PointCloud2 из rclpy (msg.data, msg.point_step[, msg.fields]).
#
# Что в облаке лишнее (проверено на всех 6 записях):
# - 38-62% точек -- нули (0,0,0): луч без отражения;
# - облако идёт столбцами по 128 каналов, и пары столбцов (2k, 2k+1) -- это
#   dual return одного залпа: в 97-98% пар точки совпадают.
# Без очистки всё дальше обрабатывает в 3-5 раз больше точек, чем есть на самом деле.

COLUMN_HEIGHT = 128  # каналов в одном столбце (залпе) облака
DUAL_RETURN_DUP_M = 0.01  # второе отражение ближе 1см к первому -- дубль


def parse_pointcloud2(data: bytes, point_step: int, fields=None, dedupe_dual_return=True):
    buf = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    offs = {'x': 0, 'y': 4, 'z': 8}
    if fields is not None:
        offs.update({f.name: f.offset for f in fields if f.name in offs})
    x, y, z = (buf[:, o:o + 4].view(np.float32).ravel() for o in (offs['x'], offs['y'], offs['z']))
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ((x != 0) | (y != 0) | (z != 0))

    n = len(x)
    if dedupe_dual_return and n % (2 * COLUMN_HEIGHT) == 0:
        # второй столбец пары выкидываем там, где он повторяет первый
        cols = lambda a: a.reshape(-1, 2, COLUMN_HEIGHT)
        xa, ya, za = cols(x), cols(y), cols(z)
        dup = ((np.abs(xa[:, 1] - xa[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(ya[:, 1] - ya[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(za[:, 1] - za[:, 0]) < DUAL_RETURN_DUP_M))
        valid.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] &= ~dup
    return x[valid], y[valid], z[valid]


def estimate_floor_z(x, y, z, near=2.0, far=15.0, half_width=3.0):
    """Грубая оценка высоты пола одной константой -- запасной вариант, когда
    профиль полотна по дальности (estimate_bed_profile) построить не удалось."""
    fwd = -y
    m = (fwd > near) & (fwd < far) & (np.abs(x) < half_width)
    if m.sum() < 50:
        return float(np.percentile(z, 10))
    return float(np.percentile(z[m], 10))


# --- профиль полотна по дальности ---------------------------------------
# Полотно вдали НЕ на той же высоте, что вблизи: уклон пути + наклон лидара.
# В doubleT_obstacle (pitch 0.8°) дно лотка на 80м на 1.3м ниже, чем на 5м --
# при плоском "полу" объект на 56м оказывается под коридором, а подъём
# полотна вдали даёт ложные тревоги. Поэтому высота дна лотка между рельсами
# оценивается в каждом кадре как функция дальности, а головка рельса = дно +
# смещение (0.27-0.56м в зависимости от типа пути, оценивается по найденным рельсам).

BED_BIN_M = 4.0        # шаг профиля по дальности
BED_CORE_HALF = 0.5    # полуширина полосы между рельсами (рельсы на ±0.76м)
BED_QUANTILE = 10      # перцентиль z в бине: дно лотка, а не шпалы/мусор
BED_MIN_PTS = 5
MAX_GRADE = 0.06       # уклон пути + наклон лидара, м/м (метро: уклон до 4%)
BED_TOL = 0.15         # насколько бин может отклониться от продолжения профиля


def _robust_line_pred(f, z, at):
    """Продолжение профиля в точку at: прямая Тейла-Сена (медиана попарных
    наклонов) -- один принятый бин-выброс (предмет на пути) её не сдвигает."""
    f, z = np.asarray(f), np.asarray(z)
    if len(f) < 2:
        return z[-1]
    i, j = np.triu_indices(len(f), 1)
    slope = np.clip(np.median((z[j] - z[i]) / (f[j] - f[i])), -MAX_GRADE, MAX_GRADE)
    return np.median(z - slope * f) + slope * at


def estimate_bed_profile(x, y, z, center_fn, near=2.0, far=250.0):
    """Высота дна лотка между рельсами по дальности: (fwd_bins, z_bed) или None.

    Бины принимаются последовательно от ближних к дальним, только если
    продолжают уже принятый профиль с правдоподобным уклоном. Это защищает от
    того, чтобы крупное препятствие (стоящий состав), закрывающее полотно,
    было принято за "поднявшийся пол" и тем самым само себя спрятало."""
    fwd = -y
    sel = (fwd > near) & (fwd < far)
    fwd, x, z = fwd[sel], x[sel], z[sel]
    sel = np.abs(x - center_fn(fwd)) < BED_CORE_HALF
    fwd, z = fwd[sel], z[sel]
    if len(fwd) < BED_MIN_PTS:
        return None
    n_bins = int(np.ceil((far - near) / BED_BIN_M))
    b = ((fwd - near) / BED_BIN_M).astype(np.int64)
    order = np.lexsort((z, b))
    b_s, z_s = b[order], z[order]
    cnt = np.bincount(b_s, minlength=n_bins)
    start = np.concatenate([[0], np.cumsum(cnt)[:-1]])
    have = np.where(cnt >= BED_MIN_PTS)[0]
    vals = z_s[start[have] + (cnt[have] - 1) * BED_QUANTILE // 100]
    centers = near + (have + 0.5) * BED_BIN_M

    acc_f, acc_z = [], []
    for f, v in zip(centers, vals):
        if not acc_f:
            if f < 15.0:  # профиль начинается только с ближней зоны, где полотно видно надёжно
                acc_f.append(f); acc_z.append(v)
            continue
        pred = _robust_line_pred(acc_f[-6:], acc_z[-6:], f)
        if abs(v - pred) < BED_TOL + 0.01 * (f - acc_f[-1]):
            acc_f.append(f); acc_z.append(v)
    if not acc_f:
        return None
    return np.array(acc_f), np.array(acc_z)


def bed_at(fwd, profile, tail_m=20.0):
    """Высота дна лотка на дальностях fwd: интерполяция внутри профиля,
    за его концом -- линейное продолжение по последним tail_m метрам."""
    pf, pz = profile
    out = np.interp(fwd, pf, pz)
    beyond = fwd > pf[-1]
    if beyond.any():
        t = pf >= pf[-1] - tail_m
        slope = np.clip(np.polyfit(pf[t], pz[t], 1)[0], -MAX_GRADE, MAX_GRADE) if t.sum() >= 2 else 0.0
        out[beyond] = pz[-1] + slope * (fwd[beyond] - pf[-1])
    return out


# --- кластеры и подтверждение по кадрам ---------------------------------

def cluster_points(fwd, lat, z_rel, fwd_gap=1.0, lat_gap=0.6):
    """Точки в зоне -> объекты: разрыв >fwd_gap по дальности или >lat_gap вбок
    разделяет объекты."""
    if len(fwd) == 0:
        return []
    o = np.argsort(fwd)
    fwd, lat, z_rel = fwd[o], lat[o], z_rel[o]
    objects = []
    for g in np.split(np.arange(len(fwd)), np.where(np.diff(fwd) > fwd_gap)[0] + 1):
        go = g[np.argsort(lat[g])]
        for h in np.split(go, np.where(np.diff(lat[go]) > lat_gap)[0] + 1):
            objects.append({
                'distance_m': float(fwd[h].min()),
                'lateral_m': float(np.median(lat[h])),
                'height_m': float(z_rel[h].max()),  # верх объекта над головкой рельса
                'n_points': int(len(h)),
            })
    return objects


class ObstacleDetector:
    """Состояние между кадрами: путь (рельсы), смещение головки рельса над
    полотном, треки объектов для подтверждения.

    Тревога поднимается не по одной точке, а когда объект виден в
    confirm_hits из последних confirm_window кадров: одиночная точка шума
    (пыль, капля, отражение) между кадрами не повторяется, настоящий объект --
    повторяется. Цена -- задержка на confirm_hits кадров (~0.3с при 10Гц)."""

    def __init__(self, near_cutoff=2.0, max_range=250.0, half_width=1.0,
                 clearance=0.15, height=2.0, confirm_hits=3, confirm_window=5,
                 max_path_age=30, default_rail_offset=0.5, path_margin=10.0):
        self.near_cutoff = near_cutoff
        self.max_range = max_range
        self.half_width = half_width          # полуширина коридора от оси пути
        self.clearance = clearance            # зона начинается на столько выше головки рельса
        self.height = height                  # и заканчивается на столько выше неё
        self.confirm_hits = confirm_hits
        self.confirm_window = confirm_window
        self.max_path_age = max_path_age      # кадров; старше -- путь сбрасывается
        self.path_margin = path_margin        # тревога -- только до конца известной оси + столько
        self.rail_offset = default_rail_offset  # головка рельса над дном лотка, м
        self._fit_fwd = None
        self._fitted_cl = None
        self._gauge = None
        self._path_frame = None
        self._bed = None
        self._rail_prof = None
        self._path_range = None
        self._offset_measured = False
        self._frame = 0
        self._tracks = []

    # --- путь ---
    def update_path(self, x, y, z) -> bool:
        """Пересчёт пути по рельсам (~15-40мс). True -- путь найден и обновлён;
        False -- держим предыдущий фит, пока он не старше max_path_age кадров."""
        pair, fit_fwd, cl = fit_path(x, y, z)
        if pair is None:
            return False
        self._fit_fwd, self._fitted_cl, self._gauge = fit_fwd, cl, pair[3]
        self._path_frame = self._frame

        # высота головки рельса там, где рельсы видны: медиана обоих рельсов по 1м-бинам
        _, left, right, _ = pair
        rf = np.concatenate([left['fwd'], right['fwd']])
        rz = np.concatenate([left['z'], right['z']])
        bins, inv = np.unique(rf, return_inverse=True)
        tor = np.array([np.median(rz[inv == k]) for k in range(len(bins))])
        if len(tor) >= 3:  # сгладить одиночные выбросы профиля
            tor = np.median(np.stack([np.r_[tor[:1], tor[:-1]], tor, np.r_[tor[1:], tor[-1:]]]), axis=0)
        self._rail_prof = (bins, tor)

        # смещение головки над дном лотка (0.45-0.6м) -- по участку 5-25м: там рельсы
        # надёжны, а дальше 30м найденные пики всё чаще не рельсы
        bed = estimate_bed_profile(x, y, z, self._center_of_fwd)
        if bed is not None:
            mid = (bins > 5) & (bins < 25) if ((bins > 5) & (bins < 25)).sum() >= 5 else np.ones_like(bins, bool)
            offset = float(np.median(tor[mid] - bed_at(bins[mid], bed)))
            if 0.1 < offset < 0.8:
                if self._offset_measured:
                    self.rail_offset += 0.3 * (offset - self.rail_offset)
                else:  # первое измерение -- сразу, без сглаживания от значения по умолчанию
                    self.rail_offset, self._offset_measured = offset, True
            # ось дальше рельсов -- по стенам тоннеля
            self._fit_fwd, self._fitted_cl = extend_path_by_walls(
                x, y, z, fit_fwd, cl, lambda f: self._tor_at(f, bed))
        self._path_range = float(self._fit_fwd[-1])
        return True

    def _tor_at(self, fwd, bed):
        """Высота головки рельса по дальности: по самим рельсам, где они
        найдены, дальше -- дно лотка + смещение."""
        tor = bed_at(fwd, bed) + self.rail_offset
        if self._rail_prof is not None:
            rf, rz = self._rail_prof
            inside = fwd <= rf[-1]
            tor[inside] = np.interp(fwd[inside], rf, rz)
        return tor

    def _path_valid(self):
        if self._fitted_cl is not None and self._frame - self._path_frame > self.max_path_age:
            self._fit_fwd = self._fitted_cl = self._gauge = self._rail_prof = None  # устарел -- лучше прямая, чем чужая кривая
            self._path_range = None
        return self._fitted_cl is not None

    def _center_of_fwd(self, fwd):
        if self._fitted_cl is None:
            return np.zeros_like(fwd)  # нет фита пути -- прямая линия как fallback
        return np.interp(fwd, self._fit_fwd, self._fitted_cl)

    # --- подтверждение ---
    def _update_tracks(self, objects):
        for tr in self._tracks:
            tr['hist'].append(False)
        for ob in objects:
            best = None
            for tr in self._tracks:
                df = abs(ob['distance_m'] - tr['distance_m'])
                dl = abs(ob['lateral_m'] - tr['lateral_m'])
                # за кадр поезд проходит до ~2м (70км/ч), плюс разброс дальней точки объекта
                if df < 2.5 + 0.05 * ob['distance_m'] and dl < 0.8 and not tr['hist'][-1]:
                    if best is None or df < best[0]:
                        best = (df, tr)
            if best is None:
                tr = {'hist': [True]}
                self._tracks.append(tr)
            else:
                tr = best[1]
                tr['hist'][-1] = True
            tr['distance_m'], tr['lateral_m'] = ob['distance_m'], ob['lateral_m']
            ob['_track'] = tr
        for tr in self._tracks:
            del tr['hist'][:-self.confirm_window]
        self._tracks = [tr for tr in self._tracks if any(tr['hist'])]
        for ob in objects:
            ob['confirmed'] = sum(ob.pop('_track')['hist']) >= self.confirm_hits

    # --- проверка кадра ---
    def check_frame(self, x, y, z, stamp=None) -> dict:
        """Быстрый путь, каждый кадр: использует последний известный путь."""
        self._frame += 1
        path_ok = self._path_valid()

        bed = estimate_bed_profile(x, y, z, self._center_of_fwd)
        if bed is not None:
            self._bed = bed
        elif self._bed is None:
            self._bed = (np.array([0.0]), np.array([estimate_floor_z(x, y, z) - self.rail_offset]))

        fwd = -y
        m = (fwd > self.near_cutoff) & (fwd < self.max_range)
        fwd, x, z = fwd[m], x[m], z[m]
        lat = x - self._center_of_fwd(fwd)
        m = np.abs(lat) < self.half_width
        fwd, lat, z = fwd[m], lat[m], z[m]
        z_rel = z - self._tor_at(fwd, self._bed)  # высота над головкой рельса
        m = (z_rel > self.clearance) & (z_rel < self.height)

        objects = cluster_points(fwd[m], lat[m], z_rel[m])
        self._update_tracks(objects)
        # дальше известной оси коридор -- догадка (прямая), и в кривой он лежит на
        # стене: такие объекты отдаём, но тревогу по ним не поднимаем
        path_range = self._path_range if path_ok else None
        limit = (path_range if path_range is not None else 0.0) + self.path_margin
        for o in objects:
            o['beyond_path'] = o['distance_m'] > limit
        confirmed = [o for o in objects if o['confirmed'] and not o['beyond_path']]
        return {
            'obstacle': bool(confirmed),
            'distance_m': min(o['distance_m'] for o in confirmed) if confirmed else None,
            'n_points': int(m.sum()),
            'path_available': path_ok,
            'gauge_m': self._gauge,
            'path_age_frames': None if not path_ok else self._frame - self._path_frame,
            'path_range_m': path_range,  # до какой дальности ось пути известна
            'objects': objects,
            'stamp': stamp,
        }

    def detect(self, x, y, z, refit_path=False, stamp=None) -> dict:
        """Единая точка входа. refit_path=True -- пересчитать путь в этом же вызове."""
        if refit_path:
            self.update_path(x, y, z)
        return self.check_frame(x, y, z, stamp=stamp)


if __name__ == '__main__':
    # самопроверка контракта на синтетических данных -- без ROS2, без bag-файлов
    from lidar_sim import simulate_frame

    det = ObstacleDetector()
    x, y, z, *_ = simulate_frame(obstacle_forward=None)
    det.update_path(x, y, z)
    print('путь после update_path:', 'найден' if det._fitted_cl is not None else 'не найден')

    x, y, z, *_ = simulate_frame(obstacle_forward=50.0, obstacle_radius=0.35)
    for _ in range(det.confirm_hits):
        result = det.detect(x, y, z, refit_path=False)
    print('результат на кадре с препятствием на 50м:', result)
