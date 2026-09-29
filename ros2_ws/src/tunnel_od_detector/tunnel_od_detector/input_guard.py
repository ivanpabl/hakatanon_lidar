# © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
"""Защита по входу узла детектора, без ROS 2.

- Watchdog: облака от лидара не приходят дольше watchdog_timeout_s -- узел публикует снимок
  с `node_state: fault`, `reason: no_lidar_data` (статус контракта -- `unknown`), пока данные
  не возобновятся. До первого кадра действует запас watchdog_grace_s (плеер ещё на паузе).
- Прогрев: первые warmup_frames кадров (ось и треки ещё строятся) -- `node_state: warmup`,
  `reason: warmup`, статус `unknown` вместо `clear`/`caution` (`stop` остаётся). Кончается по
  числу кадров и тогда, когда ось так и не построилась (пустая запись, кривая) -- дальше
  решение детектора с `reason: no_path`. После провала входа прогрев начинается заново.
- Догон всплеска ros2 bag play: из очереди берётся самый свежий кадр, старые отбрасываются
  и считаются в dropped (catch_up).
- Аргументы `ros2 bag play` для play.launch.py (`--read-ahead-queue-size`).

Статус контракта v1 (`status`: stop | caution | clear | unknown) не расширяется: потребители
и evaluation читают его как раньше; fault и warmup -- в `node_state` и `reason`.
Логика вынесена сюда, чтобы проверять её pytest-ом без rclpy (tests/test_input_guard.py).
"""

STATE_OK = 'ok'
STATE_WARMUP = 'warmup'
STATE_FAULT = 'fault'
REASON_NO_DATA = 'no_lidar_data'
REASON_WARMUP = 'warmup'
REASON_NO_PATH = 'no_path'
REASON_SHORT_SIGHT = 'short_sight'

DEFAULT_READ_AHEAD = 20
CATCH_UP_AUTO_MAX_PENDING = 10


class Watchdog:
    """Решение «нет данных от лидара» по времени последнего кадра (монотонные секунды).

    check(now) -> (fault, age_s, changed): fault -- данных нет дольше timeout_s
    (до первого кадра -- дольше grace_s от t_start); changed -- состояние сменилось
    с прошлого вызова (начало или конец провала). episodes -- сколько раз вход пропадал."""

    def __init__(self, timeout_s=0.5, grace_s=2.0, t_start=0.0):
        self.timeout_s = max(0.01, float(timeout_s))
        self.grace_s = max(0.0, float(grace_s))
        self.t_start = float(t_start)
        self.t_last = None
        self.fault = False
        self.episodes = 0
        self.snapshots = 0

    def on_frame(self, t):
        """Кадр пришёл в момент t. Возвращает True, если этим кадром провал закончился."""
        self.t_last = float(t)
        recovered, self.fault = self.fault, False
        return recovered

    def age(self, now):
        return float(now) - (self.t_last if self.t_last is not None else self.t_start)

    def check(self, now):
        age = self.age(now)
        limit = self.timeout_s if self.t_last is not None else max(self.grace_s, self.timeout_s)
        fault = age > limit
        changed = fault != self.fault
        if changed and fault:
            self.episodes += 1
        self.fault = fault
        return fault, age, changed

    def period_s(self):
        """Период таймера проверки: вдвое чаще порога, не реже 20 Гц."""
        return max(0.05, self.timeout_s / 2.0)


def fault_result(age_s, timeout_s, topic=''):
    """Снимок для /tunnel_od/result, пока данных нет: статус контракта unknown, тревоги нет,
    ничего не проверено (clear_to_m 0). Это не кадр: frame/stamp -- null, в <запись>_result.jsonl
    снимки не пишутся (в итоге -- fault_episodes / fault_snapshots)."""
    return {
        'status': 'unknown',
        'node_state': STATE_FAULT,
        'reason': REASON_NO_DATA,
        'obstacle': False,
        'distance_m': None,
        'caution_distance_m': None,
        'sight_m': 0.0,
        'clear_to_m': 0.0,
        'objects': [],
        'path_available': False,
        'fault': True,
        'since_last_cloud_s': float(age_s),
        'watchdog_timeout_s': float(timeout_s),
        'frame': None,
        'stamp': None,
        'topic': topic,
        'repeated': False,
    }


class Warmup:
    """Прогрев: первые min_frames обработанных кадров (min_frames <= 0 -- прогрева нет).
    Заканчивается по числу кадров независимо от того, построилась ли ось: если нет, дальше
    честный unknown детектора с reason no_path, а не вечный warmup. path_seen -- построилась ли
    ось за прогрев (в лог)."""

    def __init__(self, min_frames=3):
        self.min_frames = int(min_frames)
        self.reset()

    def reset(self):
        self.frames = 0
        self.path_seen = False
        self.restarts = getattr(self, 'restarts', -1) + 1

    @property
    def active(self):
        return self.frames < self.min_frames

    def update(self, path_available):
        """Учесть обработанный кадр. Возвращает True, если кадр ещё прогревочный (решение по
        нему -- прогрев)."""
        if not self.active:
            return False
        self.frames += 1
        self.path_seen = self.path_seen or bool(path_available)
        return True


def annotate(out, warming):
    """Поля node_state/reason в результате кадра. Во время прогрева clear/caution -> unknown,
    clear_to_m -> 0 (габарит ещё не проверен); stop не гасится. Для unknown от детектора --
    причина: no_path (оси нет) или short_sight (путь виден ближе min_sight_m)."""
    status = out.get('status')
    if warming and status != 'stop':
        out['status'] = 'unknown'
        out['clear_to_m'] = 0.0
        out['node_state'] = STATE_WARMUP
        out['reason'] = REASON_WARMUP
        return out
    out['node_state'] = STATE_WARMUP if warming else STATE_OK
    if status == 'unknown':
        out['reason'] = REASON_NO_PATH if not out.get('path_available') else REASON_SHORT_SIGHT
    else:
        out['reason'] = None
    return out


def take_latest(pending):
    """Догон: из очереди -- самый свежий элемент, старые отбрасываются.
    -> (элемент | None, сколько отброшено). Очередь очищается."""
    if not pending:
        return None, 0
    item = pending[-1]
    n = len(pending) - 1
    pending.clear()
    return item, n


def catch_up_enabled(value, max_pending):
    """Параметр catch_up: true | false | auto (auto -- включён, пока очередь короткая;
    детерминированный прогон с max_pending 100000 обрабатывает все кадры)."""
    v = str(value).strip().lower()
    if v in ('1', 'true', 'yes', 'on'):
        return True
    if v in ('0', 'false', 'no', 'off'):
        return False
    if v != 'auto':
        raise ValueError(f'catch_up: true | false | auto, получено {value!r}')
    return int(max_pending) <= CATCH_UP_AUTO_MAX_PENDING


def watchdog_params(start_delay_s, rate=1.0, timeout_s=0.5, margin_s=2.0):
    """Параметры watchdog узла для прогона записи: до первого кадра плеер стоит на паузе
    start_delay_s -- запас grace = start_delay + margin, иначе штатный прогон начинался бы с
    ложного fault; порог между кадрами -- под скорость проигрывания (rate 0.25 -> кадры раз в
    0.4 с при 10 Гц, порог 0.5/rate = 2 с)."""
    rate = float(rate)
    if rate <= 0:
        raise ValueError(f'rate: > 0, получено {rate}')
    return {'watchdog_grace_s': float(start_delay_s) + float(margin_s),
            'watchdog_timeout_s': max(float(timeout_s), float(timeout_s) / rate)}


def play_args(bag, rate=1.0, read_ahead=DEFAULT_READ_AHEAD, loop=False, start_paused=True, qos_file=None):
    """Команда ros2 bag play. read_ahead -- --read-ahead-queue-size (по умолчанию плеер Humble
    читает вперёд 1000 сообщений: вся запись в памяти и ~20 с тишины на старте)."""
    read_ahead = int(read_ahead)
    if read_ahead < 1:
        raise ValueError(f'read_ahead: целое >= 1, получено {read_ahead}')
    cmd = ['ros2', 'bag', 'play', str(bag), '--rate', str(float(rate)),
           '--read-ahead-queue-size', str(read_ahead), '--disable-keyboard-controls']
    if start_paused:
        cmd.append('--start-paused')
    if loop:
        cmd.append('--loop')
    if qos_file:
        cmd += ['--qos-profile-overrides-path', str(qos_file)]
    return cmd
