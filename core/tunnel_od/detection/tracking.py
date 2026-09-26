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
"""
EVIDENCE_CAP = 1.5


class Tracker:
    def __init__(self, confirm_hits=3, confirm_window=5):
        self.confirm_hits = confirm_hits
        self.confirm_window = confirm_window
        self.tracks = []

    def update(self, objects, displacement=None, expected=None):
        """Сопоставляет объекты кадра с треками и ставит каждому ob['confirmed']."""
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
            ob['confirmed'] = sum(ob.pop('_track')['hist']) >= self.confirm_hits


class EvidenceTracker:
    def __init__(self, threshold=2.2, decay=0.8, min_hits=3, gate_fwd=1.0, gate_rel=0.02,
                 gate_lat=0.6, max_miss=10):
        self.threshold = threshold
        self.decay = decay
        self.min_hits = min_hits
        self.gate_fwd = gate_fwd
        self.gate_rel = gate_rel
        self.gate_lat = gate_lat
        self.max_miss = max_miss
        self.tracks = []

    def update(self, objects, displacement=None, expected=None):
        """objects -- все объекты кадра (и too_small); displacement -- путь поезда за кадр, м
        (None -- неизвестен: широкие ворота без прогноза); expected(d) -- минимум точек на
        дальности d. Ставит ob['confirmed'] и ob['evidence']."""
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
                tr = {'score': 0.0, 'hits': 0, 'miss': 0, 'lateral_m': ob['lateral_m']}
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
            ob['evidence'] = round(tr['score'], 2)
            ob['confirmed'] = tr['score'] >= self.threshold and tr['hits'] >= self.min_hits
        self.tracks = [tr for tr in self.tracks
                       if tr['miss'] <= self.max_miss and tr['distance_m'] > -5.0 and tr['score'] > 0.05]
