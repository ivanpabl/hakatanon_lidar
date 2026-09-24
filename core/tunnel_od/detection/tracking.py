"""Подтверждение объектов по последовательности кадров.

Тревога поднимается не по одной точке, а когда объект виден в confirm_hits из
последних confirm_window кадров: одиночная точка шума (пыль, капля, отражение)
между кадрами не повторяется, настоящий объект -- повторяется. Цена -- задержка
на confirm_hits кадров (~0.3с при 10Гц).
"""


class Tracker:
    def __init__(self, confirm_hits=3, confirm_window=5):
        self.confirm_hits = confirm_hits
        self.confirm_window = confirm_window
        self.tracks = []

    def update(self, objects):
        """Сопоставляет объекты кадра с треками и ставит каждому ob['confirmed']."""
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
