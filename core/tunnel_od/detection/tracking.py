"""Подтверждение объектов по последовательности кадров.

Tracker -- объект виден в confirm_hits из последних confirm_window кадров:
одиночная точка шума (пыль, капля, отражение) между кадрами не повторяется,
настоящий объект -- повторяется. Цена -- задержка на confirm_hits кадров (~0.3с при 10Гц).

EvidenceTracker -- накопление свидетельства с учётом движения поезда. Неподвижный
объект за кадр приближается ровно на пройденный поездом путь, поэтому трек
прогнозируется на это смещение, и ворота сопоставления узкие. Каждый кадр трек
получает вклад n_points / ожидаемый минимум на этой дальности (min_points_at),
не больше EVIDENCE_CAP, и накопленное затухает с коэффициентом decay. Так объект
вдали из 1-2 точек (меньше порога одного кадра) подтверждается за несколько кадров,
а крупный объект вблизи -- так же быстро, как по "3 из 5".

ЭГО-тест: трек хранит пары (пробег поезда, дистанция) последних EGO_OBS сопоставлений. Неподвижный
в мире объект приближается на пройденный путь -- наклон d(дистанция)/d(пробег) = -1; артефакт,
который едет вместе с поездом, держит дистанцию -- наклон ~0. Наклон -- медиана по парам
(Тейл--Сен), решение -- только при пробеге >= ego_min_travel.
"""
import numpy as np

EVIDENCE_CAP = 1.5
EGO_OBS = 20


def theil_sen_slope(obs, min_dt=0.5, min_pairs=3):
    """Медиана наклонов (d_j - d_i) / (t_j - t_i) по парам с |t_j - t_i| > min_dt; None -- пар мало."""
    t = np.array([p[0] for p in obs], float)
    d = np.array([p[1] for p in obs], float)
    i, j = np.triu_indices(len(t), 1)
    dt = t[j] - t[i]
    ok = np.abs(dt) > min_dt
    if ok.sum() < min_pairs:
        return None
    return float(np.median((d[j] - d[i])[ok] / dt[ok]))


class Tracker:
    def __init__(self, confirm_hits=3, confirm_window=5):
        self.confirm_hits = confirm_hits
        self.confirm_window = confirm_window
        self.tracks = []

    def update(self, objects, displacement=None, expected=None, travel=None):
        """Сопоставляет объекты кадра с треками и ставит каждому ob['confirmed']. travel не используется."""
        objects = [o for o in objects if not o.get('too_small')]
        for tr in self.tracks:
            tr['hist'].append(False)
        for ob in objects:
            best = None
            for tr in self.tracks:
                df = abs(ob['distance_m'] - tr['distance_m'])
                dl = abs(ob['lateral_m'] - tr['lateral_m'])
                if df < 2.5 + 0.05 * ob['distance_m'] and dl < 0.8 and not tr['hist'][-1]:
                    if best is None or df < best[0]:
                        best = (df, tr)
            if best is None:
                tr = {'hist': [True]}
                self.tracks.append(tr)
            else:
                tr = best[1]
                tr['hist'][-1] = True
            tr['distance_m'], tr['lateral_m'] = ob['distance_m'], ob['lateral_m']
            ob['_track'] = tr
        for tr in self.tracks:
            del tr['hist'][:-self.confirm_window]
        self.tracks = [tr for tr in self.tracks if any(tr['hist'])]
        for ob in objects:
            hist = ob.pop('_track')['hist']
            ob['hits'] = sum(hist)
            ob['confirmed'] = ob['hits'] >= self.confirm_hits

    def set_alarm(self, track_ids):
        pass

    def set_levels(self, levels):
        pass

    def held(self, min_distance=None):
        return []


class EvidenceTracker:
    """hold -- сколько кадров подряд без объекта трек ещё держит тревогу (held()): вдали объект
    из 2-3 точек иногда пропадает на кадр-два, и тревога мигает. Держится только трек, который
    поднял тревогу в последнем кадре с объектом (set_alarm) и был в тревоге не меньше hold_min
    кадров: подтверждённый трек за концом оси тревоги не поднимал, и его прогноз не должен поднять
    её, когда ось в следующем кадре длиннее; ложная вспышка на 1-3 кадра не должна удлиняться."""

    def __init__(self, threshold=2.2, decay=0.8, min_hits=3, gate_fwd=1.0, gate_rel=0.02,
                 gate_lat=0.6, max_miss=10, hold=0, hold_min=5, ego_check=False, ego_min_travel=4.0,
                 ego_max_slope=-0.35, persist_hits=0, persist_slope=(-1.25, -0.75), persist_max_d=None, persist_max_lat=None,
                 persist_min_top=0.0, far_min_hits=None, persist_min_top_lat=None):
        self.persist_max_lat = persist_max_lat
        self.persist_min_top = persist_min_top  # persist не подтверждает объект с верхом ниже (0 -- выкл)
        # ... но только для объектов с |lateral_m| >= persist_min_top_lat (у головки рельса); None -- для всех
        self.persist_min_top_lat = persist_min_top_lat
        self.persist_hits = persist_hits
        self.persist_slope = persist_slope
        self.persist_max_d = persist_max_d
        # far_min_hits -- (дальность, м; кадров): дальше этой дальности подтверждение по evidence требует
        # не меньше стольких кадров с объектом (накопление вдали, см. ObstacleDetector far_acc_*). None -- выкл.
        self.far_min_hits = far_min_hits
        self.ego_check = ego_check
        self.ego_min_travel = ego_min_travel
        self.ego_max_slope = ego_max_slope
        self.hold = hold
        self.hold_min = hold_min
        self.threshold = threshold
        self.decay = decay
        self.min_hits = min_hits
        self.gate_fwd = gate_fwd
        self.gate_rel = gate_rel
        self.gate_lat = gate_lat
        self.max_miss = max_miss
        self.tracks = []
        self._next_id = 0

    def update(self, objects, displacement=None, expected=None, travel=None):
        """objects -- все объекты кадра (и too_small); displacement -- путь поезда за кадр, м
        (None -- неизвестен: широкие ворота без прогноза); expected(d) -- минимум точек на
        дальности d; travel -- накопленный пробег поезда (None -- неизвестен или смещение кадра
        неизвестно: ЭГО-тест не копит пары). Ставит ob['confirmed'], ob['evidence'],
        ob['ego_slope'], ob['ego_carried']."""
        ds = displacement or 0.0
        slack = 0.0 if displacement is not None else 2.5
        for tr in self.tracks:
            tr['distance_m'] -= ds
            tr['score'] *= self.decay
            tr['miss'] += 1
        order = sorted(range(len(objects)), key=lambda k: -objects[k]['n_points'])
        taken = set()
        for k in order:
            ob = objects[k]
            d = ob['distance_m']
            gate = self.gate_fwd + self.gate_rel * d + slack + (0.03 * d if displacement is None else 0.0)
            best = None
            for j, tr in enumerate(self.tracks):
                if j in taken:
                    continue
                df = abs(d - tr['distance_m'])
                if df < gate and abs(ob['lateral_m'] - tr['lateral_m']) < self.gate_lat:
                    if best is None or df < best[0]:
                        best = (df, j)
            if best is None:
                tr = {'id': self._next_id, 'score': 0.0, 'hits': 0, 'miss': 0, 'lateral_m': ob['lateral_m']}
                self._next_id += 1
                self.tracks.append(tr)
                taken.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[best[1]]
                taken.add(best[1])
            need = expected(d) if expected is not None else 1.0
            tr['score'] += min(ob['n_points'] / max(need, 1e-6), EVIDENCE_CAP)
            tr['hits'] += 1
            tr['miss'] = 0
            tr['distance_m'] = d
            tr['lateral_m'] = 0.5 * (tr['lateral_m'] + ob['lateral_m'])
            for k in ('height_m', 'low_m'):
                if k in ob:
                    tr[k] = ob[k]
            if travel is not None:
                tr.setdefault('obs', []).append((travel, d))
                del tr['obs'][:-EGO_OBS]
            obs = tr.get('obs', [])
            slope = None
            if len(obs) >= 4 and max(p[0] for p in obs) - min(p[0] for p in obs) >= self.ego_min_travel:
                slope = theil_sen_slope(obs)
            ob['ego_slope'] = None if slope is None else round(slope, 3)
            ob['ego_carried'] = bool(self.ego_check and slope is not None and slope > self.ego_max_slope)
            tr['alarm'] = False
            ob['track_id'] = tr['id']
            ob['evidence'] = round(tr['score'], 2)
            ob['hits'] = tr['hits']
            ob['persistent'] = bool(self.persist_hits and tr['hits'] >= self.persist_hits and slope is not None
                                    and self.persist_slope[0] <= slope <= self.persist_slope[1]
                                    and (self.persist_max_d is None or d <= self.persist_max_d)
                                    and (not self.persist_min_top or ob.get('height_m', np.inf) >= self.persist_min_top
                                         or (self.persist_min_top_lat is not None
                                             and abs(ob['lateral_m']) < self.persist_min_top_lat))
                                    and (self.persist_max_lat is None
                                         or max(abs(ob.get('lat_min_m', ob['lateral_m'])),
                                                abs(ob.get('lat_max_m', ob['lateral_m']))) <= self.persist_max_lat))
            hits_need = self.min_hits
            if self.far_min_hits is not None and d > self.far_min_hits[0]:
                hits_need = max(hits_need, self.far_min_hits[1])
            ob['confirmed'] = (tr['score'] >= self.threshold and tr['hits'] >= hits_need) or ob['persistent']
        self.tracks = [tr for tr in self.tracks
                       if tr['miss'] <= self.max_miss and tr['distance_m'] > -5.0 and tr['score'] > 0.05]

    def set_alarm(self, track_ids):
        """Треки, объекты которых в этом кадре подняли тревогу."""
        for tr in self.tracks:
            if tr['miss'] == 0 and tr['id'] in track_ids:
                tr['alarm'] = True
                tr['alarm_frames'] = tr.get('alarm_frames', 0) + 1

    def set_levels(self, levels):
        """{track_id: (level, reason)} объектов этого кадра -- их наследует удержанный объект (held)."""
        for tr in self.tracks:
            if tr['id'] in levels:
                tr['level'], tr['reason'] = levels[tr['id']]

    def held(self, min_distance=None):
        """Треки, поднявшие тревогу в последнем кадре с объектом; пропущено 1..hold кадров.
        distance_m -- прогноз с учётом пути поезда; ближе min_distance (поезд объект уже проехал
        или он в мёртвой зоне у лидара) трек не держится."""
        return [tr for tr in self.tracks if tr.get('alarm') and tr['alarm_frames'] >= self.hold_min
                and 1 <= tr['miss'] <= self.hold
                and (min_distance is None or tr['distance_m'] >= min_distance)]
