"""Фоновый пересчёт оси пути в отдельном процессе (refit_mode: async).

Пересчёт пути (ObstacleDetector.update_path) стоит ~80 мс -- вместе с разбором и
проверкой кадра это больше 100 мс. Поэтому проверка кадра идёт в узле каждый кадр, а
путь считается параллельно в дочернем процессе по самому свежему кадру (как только
процесс освободился) и подставляется в детектор узла перед следующим кадром.

Узел не знает, какие поля детектора относятся к пути: процесс держит свой экземпляр
ObstacleDetector с теми же kwargs, вызывает update_path и возвращает все атрибуты,
которые этот вызов переприсвоил. Так обёртка переживает изменения ядра, пока
update_path не зависит от состояния, которое меняет только check_frame (тогда --
refit_mode: sync).
"""
import multiprocessing as mp
import time


def _child(conn, kwargs):
    import os
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    from tunnel_od import ObstacleDetector
    det = ObstacleDetector(**kwargs)
    while True:
        job = conn.recv()
        if job is None:
            return
        frame, x, y, z = job
        t0 = time.monotonic()
        try:
            if hasattr(det, '_frame'):
                det._frame = frame          # возраст пути считается в кадрах узла
            before = dict(vars(det))
            ok = bool(det.update_path(x, y, z))
            changed = {k: v for k, v in vars(det).items()
                       if k != '_frame' and (k not in before or before[k] is not v)}
            conn.send((frame, ok, changed, (time.monotonic() - t0) * 1e3, None))
        except Exception as e:  # ошибка пересчёта не должна останавливать процесс
            conn.send((frame, False, {}, (time.monotonic() - t0) * 1e3, f'{type(e).__name__}: {e}'))


class AsyncPathFitter:
    """submit() отдаёт кадр, если процесс свободен; apply() переносит готовый путь в детектор."""

    def __init__(self, kwargs):
        ctx = mp.get_context('spawn')      # не fork: в процессе узла уже работают потоки DDS
        self._conn, child = ctx.Pipe()
        self._proc = ctx.Process(target=_child, args=(child, kwargs), daemon=True, name='tunnel_od_path')
        self._proc.start()
        self._busy = False
        self.fits = 0
        self.failures = 0
        self.last_ms = None
        self.last_error = None

    def submit(self, frame, x, y, z):
        if self._busy:
            return False
        self._conn.send((frame, x, y, z))
        self._busy = True
        return True

    def apply(self, det, wait=False):
        """Если путь готов -- переносит его в det. True -- перенесён новый путь."""
        if not self._busy or not self._conn.poll(None if wait else 0):
            return False
        frame, ok, changed, ms, err = self._conn.recv()
        self._busy = False
        self.last_ms, self.last_error = ms, err
        if not ok:
            self.failures += 1
            return False
        for k, v in changed.items():
            setattr(det, k, v)
        self.fits += 1
        return True

    def close(self):
        try:
            if self._busy:
                self._conn.poll(1.0)
            self._conn.send(None)
        except Exception:
            pass
        self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            self._proc.terminate()
