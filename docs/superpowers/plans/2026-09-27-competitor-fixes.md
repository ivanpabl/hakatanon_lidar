# Доводка под данные организаторов и сдача — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Поверх текущего конвейера добавить слой решения (`status` / `level` / `clear_to_m`), инструменты оценки на бэге организаторов `cloud_with_fake_obj`, подобрать зону и ЭГО-тест по этому бэгу и сдать решение (образ без интернета, README, видео, презентация) до 29.09 20:00.

**Architecture:** Геометрию пути (`rails.py`, `path.py`, `bed.py`) не трогаем. Решение по кадру выносится в чистый модуль `core/tunnel_od/detection/decision.py`, который `check_frame` вызывает вместо сегодняшнего `confirmed`. `stop` на этапе 1 в точности совпадает с сегодняшним `obstacle`. Инструменты (`alarms.py` → CSV + objects.jsonl → `eval_fake_obj.py`, `selflabel.py`, `compare_row.py`) дают одну строку таблицы на вариант детектора. По этим строкам принимаются решения этапов 2–3 (раздел 8 спеки).

**Tech Stack:** Python 3.14 (venv), numpy, rosbags, ruamel.yaml (приходит с rosbags), pytest; ROS 2 Humble в Docker; zsh-скрипты в `research/`.

**Spec:** `docs/superpowers/specs/2026-09-27-competitor-fixes-design.md`. Исполнитель читает её вместе с планом: номера разделов (2.1, 3.2, 4.4, 8.1 …) в плане отсылают туда.

Все пути в плане — от корня репозитория `hakatanon_lidar/` (это отдельный git-репозиторий внутри `LCT/`). Команды запускаются из него.

## Global Constraints

- Заморозка алгоритма — **28.09 21:00**. Загрузка на платформу — **29.09 до 20:00**.
- Этап 2 — до 28.09 14:00, этап 3 — до 28.09 21:00. Если время кончается, первым выкидывается этап 3 (задачи 14–15), затем подбор `h_top`/`far_top` в задаче 12. Этап 4 (задачи 17–19) не сокращается.
- Геометрию пути (`rails.py`, `path.py`, `bed.py`) не переписывать.
- Каждое изменение алгоритма сначала проверяется на бэге организаторов, потом на наших данных.
- Изменение этапов 2–3 включается по умолчанию только при выполнении всех жёстких критериев 8.1 относительно базы этапа 1 (тег `base`).
- `obstacle = (status == 'stop')`. Узел, `alarms.py`, дашборд и демо, которые читают `obstacle`, продолжают работать.
- Точная граница решения: предмет в жёлобе ниже 0,15 м над головкой рельса препятствием не считается (#435). Низ зоны 0,15 м не менять.
- `far_half_width` (0,7 м) и `far_from` (20 м) не менять.
- `ego_check = False` по умолчанию, пока не выполнено условие 4.4.
- В тестовой среде нет интернета (#452): образ собирается и запускается без сети.
- Цифры в README, видео и презентации берутся только из `runs/compare_table.md` и отчётов инструментов, а в презентацию — только из README.
- Ветка работы — `grader-fixes` (от `gui-metrics` после коммита WIP). Коммит в конце каждой задачи. В коммит, закрывающий этап, входит строка `compare.sh` (строка из `runs/compare_table.md`).
- Каждое сообщение коммита заканчивается строками:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd
  ```
- Тесты ядра: `venv/bin/python -m pytest -q tests` (сейчас 40 passed, ~2,5 мин). Тесты узла без ROS: `PYTHONPATH=ros2_ws/src/tunnel_od_detector venv/bin/python -m pytest -q ros2_ws/src/tunnel_od_detector/test/test_util.py`.
- Стиль кода: комментарии и docstring по-русски, как в соседнем коде; без новых зависимостей (YAML читать через `ruamel.yaml`, PyYAML в venv нет).

## Review Focus

1. **Бэг организаторов читается не до конца** (у TunnelGuard ошибка zstd на 438-м кадре из 1510). Ожидается, что `alarms.py`, `input_report.py` и `eval_fake_obj.py` не падают, а сообщают, сколько кадров прочитано и какая ошибка. Тест `test_safe_messages_stops_on_read_error` (задача 3).
2. **Пустое или целиком нулевое 16-байтное облако** (кадр без отражений, обрезанный кадр). Ожидается, что `parse_pointcloud2` и `dedupe_rounded` вернут пустые массивы без исключения. Тест `test_parse_empty_and_all_zero_unordered_cloud` (задача 2).
3. **Удержанный трек, прогноз которого ушёл за поезд** (`distance_m ≤ 0` у `held`-объекта). Ожидается, что `clear_to_m` не станет отрицательным, а `status` останется `stop`. Тест `test_decide_clear_to_never_negative` (задача 4).
4. **Объект эталона, который детектор ни разу не увидел.** Ожидается, что `eval_fake_obj` запишет для него уровень `none` и пустую дистанцию, а не упадёт. Тест `test_evaluate_unseen_object_is_none` (задача 7).
5. **Узел с C++-приёмом, когда meta кадра не пришла** (`n_in` неизвестен). Ожидается, что запасное удаление дублей не применяется и кадр обрабатывается. Тест `test_unordered_raw_unknown_count` (задача 10).

---

## Структура файлов

| Файл | Что делает | Задачи |
|---|---|---|
| `core/tunnel_od/detection/decision.py` (новый) | `object_level`, `axis_curvature`, `sight_distance`, `decide`: уровни объектов и решение по кадру, без состояния | 4 |
| `core/tunnel_od/pointcloud.py` | `dedupe_rounded`, запасное удаление дублей в `parse_pointcloud2`, `FrameRepeat` | 2 |
| `core/tunnel_od/detection/tracking.py` | `hits` в объекте, `set_levels`; ЭГО-тест (`travel`, `obs`, Тейл–Сен) | 5, 14 |
| `core/tunnel_od/detection/detector.py` | вызов `decide`, новые поля результата, `rail_z_m`, `travel_m`; параметры ЭГО-теста | 5, 14 |
| `core/tunnel_od/geometry/ego_motion.py` | промежуток за пределами окна поиска → `displacement = None` (если нужно, 5.1.6) | 13 |
| `tools/bags.py` | `EMPTY_BAGS`, `safe_messages`, запасное удаление дублей в `cloud_parser('cpp')` | 3, 6 |
| `tools/input_report.py` | Python-сводка облака: `point_step`, поля, упорядоченность, дубли, промежутки, повторы | 3 |
| `tools/alarms.py` | колонки `status`, `caution`, `sight_m`, `clear_to_m`, `travel_m`, `displacement_m`, `repeated`; `alarms_<tag>_objects.jsonl`; сводка СТОП/ВНИМ/unknown | 6 |
| `tools/eval_approach.py`, `research/show_frame.py` | тревога = `level == 'stop'` | 6 |
| `tools/eval_fake_obj.py` (новый) | `draft` — черновик эталона; `eval` — метрики по 10 объектам | 7 |
| `tools/selflabel.py` (новый) | саморазметка эпизодов СТОП проездом | 15 |
| `research/compare.sh`, `research/compare_row.py` (новый) | полный прогон варианта и строка в `runs/compare_table.md` | 9 |
| `ros2_ws/src/tunnel_od_detector/tunnel_od_detector/{node,markers,util}.py` | повтор кадра, запасное удаление дублей, цвета и текст по `level`/`status` | 10 |
| `tools/dashboard.py`, `tools/demo.py`, `gui/demo_template.html` | цвет объекта по `level`, строка решения | 10 |
| `config/detector.yaml` | параметры ЭГО-теста; выбранная зона | 12, 14 |
| `run.sh` | `NETWORK=none` для проверки без сети | 17 |
| `tests/test_core.py`, `tests/test_tools.py` (новый), `ros2_ws/.../test/test_util.py` | тесты раздела 9 и Review Focus | 2–7, 10, 13–15 |
| `data/cloud_with_fake_obj/objects.yaml` + копия `reference/cloud_with_fake_obj/objects.yaml` | эталон 10 объектов | 8 |
| `README.md`, `docs/PLAN.md`, `docs/COMPETITOR_TUNNELGUARD.md` | сдача | 18 |

---

## Этап 0 (27.09)

### Task 0: Коммит WIP и ветка `grader-fixes`

**Files:**
- Commit: все изменённые файлы `gui-metrics` (`README.md`, `config/detector.yaml`, `core/.../detector.py`, `tracking.py`, `path.py`, `gui/demo_template.html`, `tests/test_core.py`, `tools/dashboard.py`, `tools/demo.py`, `tools/run_all.py`), `docs/COMPETITOR_TUNNELGUARD.md`, `docs/superpowers/`

- [ ] **Step 1: pytest на текущем WIP**

Run: `venv/bin/python -m pytest -q tests`
Expected: `40 passed`

- [ ] **Step 2: Коммит WIP в `gui-metrics`**

```bash
git add README.md config/detector.yaml core/tunnel_od/detection/detector.py core/tunnel_od/detection/tracking.py \
  core/tunnel_od/geometry/path.py gui/demo_template.html tests/test_core.py tools/dashboard.py tools/demo.py \
  tools/run_all.py docs/COMPETITOR_TUNNELGUARD.md docs/superpowers
git commit -F - <<'EOF'
alarm_hold, path_hold, demo and dashboard updates; TunnelGuard review; spec for grader fixes

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd
EOF
git switch -c grader-fixes
```

- [ ] **Step 3: Эталонный прогон до изменений (для проверки «`stop` ≡ `obstacle`» в задаче 6)**

Run (≈5 мин):
```bash
cd tools && ../venv/bin/python alarms.py --tag pre --bags doubleT_obstacle doubleT_platform roundT_doubleT \
  roundT_pressureGate_roundT roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch --workers 2 \
  > ../runs/pre_alarms.log 2>&1; cd ..
tail -10 runs/pre_alarms.log
```
Expected: таблица, строка `doubleT_obstacle` ≈ 183 кадра с тревогой, файл `runs/alarms_pre.csv`.

### Task 1: Вопросы постановщикам и доступ к коду (ручные шаги, делаются первыми)

Ответы идут часами, поэтому вопросы отправляются в самом начале. Код не меняется.

- [ ] **Step 1: Перепроверить факты 1.3 в чате задачи №5** (#401, 435, 437, 440, 442, 452, 458, 460, 461). Цитаты с номерами сообщений записать в `docs/PLAN.md` в раздел «Факты постановщиков (проверено 27.09)». Список 10 объектов из #437/#460 (номер, описание, «в габарите»/«вне габарита») скопировать дословно: он нужен в задаче 8.

- [ ] **Step 2: Отправить в чат задачи вопросы**
  1. «Где собирается образ на проверке: `docker build` на стенде без интернета или заранее? Можно ли приложить готовый образ (`docker save`, ~N ГБ)?» (6.1)
  2. «Архив `cloud_with_fake_obj.zst`: md5 `5c0cefe7ef10fae249b5ae653cbd165d` — это целый файл? У части команд он читается до 438-го кадра.» (5.1.2)
  3. «Итоговые данные будут 26-байтными (как записи) или 16-байтными (как синтетика)?» (#442, #458)

- [ ] **Step 3: Доступ к коду (6.2).** Проверить, видят ли эксперты `ivanpabl/hakatanon_lidar`: `gh repo view ivanpabl/hakatanon_lidar --json visibility`. Если `PRIVATE`, согласовать с владельцем публичный репозиторий к сдаче или зеркало на SourceCraft. Ссылку прикрепить на платформе в карточке задачи сегодня (Q&A 4.12). Результат записать в `docs/PLAN.md`.

### Task 2: Запасное удаление дублей и повтор кадра в ядре

**Files:**
- Modify: `core/tunnel_od/pointcloud.py`
- Test: `tests/test_core.py`

**Interfaces:**
- Produces:
  - `dedupe_rounded(x, y, z, cell=DEDUPE_CELL_M) -> (x, y, z)` — одна точка на ячейку округления 1 см, порядок сохраняется.
  - `parse_pointcloud2(...)` — если число точек не кратно `2·COLUMN_HEIGHT` и `dedupe_dual_return=True`, вызывает `dedupe_rounded`.
  - `class FrameRepeat: check(data) -> bool` — `True`, если байты облака побитово совпадают с предыдущим вызовом.

- [ ] **Step 1: Написать падающие тесты** (в конец `tests/test_core.py`; импорт вверху файла расширить до `from tunnel_od.pointcloud import COLUMN_HEIGHT, FrameRepeat, dedupe_rounded`)

```python
def _cloud16(x, y, z):
    """16-байтное облако синтетики организаторов: x, y, z, intensity (float32), без ring и времени."""
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
    a = np.zeros(len(x), dt)
    a['x'], a['y'], a['z'] = x, y, z
    return a.tobytes(), dt.itemsize, [_Field(n, dt.fields[n][1]) for n in dt.names]


def test_parse_unordered_16_byte_cloud_drops_duplicates():
    rng = np.random.default_rng(0)
    base = rng.uniform(-20, 20, (200, 3)).astype(np.float32)
    pts = np.vstack([base, base[:100]])                      # 300 точек: не кратно 256, 100 дублей
    data, step, fields = _cloud16(pts[:, 0], pts[:, 1], pts[:, 2])
    assert step == 16
    px, py, pz = parse_pointcloud2(data, step, fields)
    assert len(px) == 200
    assert np.array_equal(px, base[:, 0])                    # порядок первых вхождений сохранён


def test_parse_ordered_cloud_keeps_close_points_outside_dual_pair():
    """Упорядоченное облако: дубли снимаются только внутри пары столбцов, запасной способ не включается."""
    n_cols = 4
    x = np.arange(n_cols * COLUMN_HEIGHT, dtype=np.float32) * 0.01 + 1.0      # 512 точек: пары (0,1), (2,3)
    x.reshape(n_cols, COLUMN_HEIGHT)[2] = x.reshape(n_cols, COLUMN_HEIGHT)[0]   # столбец 2 = столбец 0 (другая пара)
    y, z = np.full_like(x, -5.0), np.zeros_like(x)
    data, step, fields = _cloud_bytes(x, y, z)
    px, _, _ = parse_pointcloud2(data, step, fields)
    assert len(px) == n_cols * COLUMN_HEIGHT


def test_parse_empty_and_all_zero_unordered_cloud():
    data, step, fields = _cloud16(np.zeros(0), np.zeros(0), np.zeros(0))
    assert all(len(a) == 0 for a in parse_pointcloud2(data, step, fields))
    data, step, fields = _cloud16(np.zeros(300), np.zeros(300), np.zeros(300))
    assert all(len(a) == 0 for a in parse_pointcloud2(data, step, fields))
    assert all(len(a) == 0 for a in dedupe_rounded(np.zeros(0, np.float32), np.zeros(0, np.float32), np.zeros(0, np.float32)))


def test_frame_repeat_detects_bitwise_same_cloud():
    rep = FrameRepeat()
    a = np.arange(1000, dtype=np.uint8).tobytes()
    b = bytes(reversed(a))
    assert not rep.check(a)
    assert rep.check(a)
    assert not rep.check(b)
    assert rep.check(np.frombuffer(b, np.uint8))             # numpy-массив (rosbags) и bytes -- одно и то же
```

- [ ] **Step 2: Убедиться, что тесты падают**

Run: `venv/bin/python -m pytest -q tests/test_core.py -k "unordered or ordered_cloud_keeps or empty_and_all_zero or frame_repeat"`
Expected: FAIL (`ImportError: cannot import name 'FrameRepeat'`)

- [ ] **Step 3: Реализация в `core/tunnel_od/pointcloud.py`**

В docstring модуля добавить абзац:
```
Синтетика организаторов (cloud_with_fake_obj) -- 16-байтные точки x, y, z, intensity; облако может
быть не из столбцов по 128. Тогда дубли снимаются запасным способом: одна точка на ячейку 1 см.
```
После `AZ_STEP_DEG = 0.1`:
```python
DEDUPE_CELL_M = 0.01
```
Хвост `parse_pointcloud2` (после строки `n = len(x)`) заменить на:
```python
    n = len(x)
    if dedupe_dual_return and n % (2 * COLUMN_HEIGHT) == 0:
        cols = lambda a: a.reshape(-1, 2, COLUMN_HEIGHT)
        xa, ya, za = cols(x), cols(y), cols(z)
        dup = ((np.abs(xa[:, 1] - xa[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(ya[:, 1] - ya[:, 0]) < DUAL_RETURN_DUP_M)
               & (np.abs(za[:, 1] - za[:, 0]) < DUAL_RETURN_DUP_M))
        valid.reshape(-1, 2, COLUMN_HEIGHT)[:, 1] &= ~dup
        return x[valid], y[valid], z[valid]
    x, y, z = x[valid], y[valid], z[valid]
    if dedupe_dual_return:
        return dedupe_rounded(x, y, z)
    return x, y, z


def dedupe_rounded(x, y, z, cell=DEDUPE_CELL_M):
    """Дубли облака не из столбцов: точки, совпавшие после округления до cell, -- одна точка.
    Порядок оставшихся точек -- как во входе (первое вхождение)."""
    if len(x) == 0:
        return x, y, z
    q = lambda a: (np.round(np.asarray(a, np.float64) / cell).astype(np.int64) + 32768) & 0xFFFF
    key = (q(x) << 32) | (q(y) << 16) | q(z)          # +-327 м по каждой оси в 16 битах
    _, first = np.unique(key, return_index=True)
    keep = np.sort(first)
    return x[keep], y[keep], z[keep]


class FrameRepeat:
    """Облако, побитово совпадающее с предыдущим: в бэге организаторов кадры повторяются
    («объекты замирают»), и повтор нельзя подавать в детектор как новый кадр -- трекер и
    оценка скорости посчитают его кадром с нулевым движением."""

    def __init__(self):
        self._prev = None

    def check(self, data) -> bool:
        cur = np.frombuffer(data, np.uint8)
        same = self._prev is not None and len(cur) == len(self._prev) and np.array_equal(cur, self._prev)
        if not same:
            self._prev = cur.copy()
        return same
```

- [ ] **Step 4: Тесты проходят**

Run: `venv/bin/python -m pytest -q tests/test_core.py -k "parse or frame_repeat"`
Expected: PASS (5 тестов)

- [ ] **Step 5: Коммит**

```bash
git add core/tunnel_od/pointcloud.py tests/test_core.py
git commit -m "Fallback dedupe for unordered 16-byte clouds, bitwise frame repeat check" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 3: Бэг организаторов: распаковка, чтение до конца, сводка облака

**Files:**
- Modify: `tools/bags.py`, `tools/input_report.py`
- Create: `tests/test_tools.py`

**Interfaces:**
- Consumes: `FrameRepeat`, `dedupe_rounded`, `parse_pointcloud2(..., dedupe_dual_return=False)` (задача 2).
- Produces:
  - `bags.safe_messages(it, errors) -> generator` — отдаёт элементы `it`; ошибка чтения останавливает поток, текст дописывается в список `errors`.
  - `bags.EMPTY_BAGS` — 5 записей без препятствий.
  - `input_report.cloud_stats(name, max_frames=None) -> dict` с ключами `frames_read`, `read_errors`, `point_step`, `fields`, `ordered_share`, `dup_share_median`, `gap_s`, `gap_max_s`, `gaps_over_0_15s`, `bitwise_repeats`.

- [ ] **Step 1: Проверить и распаковать архив (5.1.1)**

```bash
md5 -q data/fake_obj/cloud_with_fake_obj.zst      # ждём 5c0cefe7ef10fae249b5ae653cbd165d
zstd -t data/fake_obj/cloud_with_fake_obj.zst     # целостность потока zstd
zstd -dc data/fake_obj/cloud_with_fake_obj.zst | tar -tf - | head
zstd -dc data/fake_obj/cloud_with_fake_obj.zst | tar -xf - -C data/
ls data/cloud_with_fake_obj                       # metadata.yaml + *.db3 (или *.mcap)
```
Файл сейчас 1,75 ГБ, а спека говорит 1,2 ГБ. Если md5 не совпадает или `zstd -t` падает, записать это в `docs/PLAN.md`, распаковать что читается (`zstd -dc … | tar -xf - -C data/` выдаст часть файлов) и отметить в сообщении задачи 1 (вопрос 2). Если после распаковки каталог называется иначе, переименовать его в `data/cloud_with_fake_obj` (так ждёт `tools/bags.py:bag_path`).

- [ ] **Step 2: Написать падающий тест на `safe_messages`** (новый файл `tests/test_tools.py`)

```python
"""Тесты чистых функций офлайн-инструментов (tools/), без записей."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))

from bags import EMPTY_BAGS, safe_messages          # noqa: E402


def test_safe_messages_stops_on_read_error():
    def broken():
        yield 1
        yield 2
        raise OSError('zstd checksum mismatch')

    errors = []
    assert list(safe_messages(broken(), errors)) == [1, 2]
    assert errors == ['OSError: zstd checksum mismatch']


def test_empty_bags_are_five_recordings_without_obstacle():
    assert len(EMPTY_BAGS) == 5 and 'doubleT_obstacle' not in EMPTY_BAGS and 'new_data' not in EMPTY_BAGS
```

Run: `venv/bin/python -m pytest -q tests/test_tools.py`
Expected: FAIL (`ImportError: cannot import name 'EMPTY_BAGS'`)

- [ ] **Step 3: `tools/bags.py`**

После `BAGS = [...]`:
```python
EMPTY_BAGS = [b for b in BAGS if b not in ('doubleT_obstacle', 'new_data')]   # 5 записей без препятствий
FAKE_OBJ = 'cloud_with_fake_obj'           # бэг организаторов с 10 синтетическими объектами (#437)
```
После `open_cloud_bag`:
```python
def safe_messages(it, errors):
    """Сообщения из reader.messages(): ошибка чтения (битый архив, обрыв zstd) останавливает
    поток, текст ошибки -- в errors. Прочитанное до ошибки остаётся в работе."""
    try:
        yield from it
    except Exception as e:
        errors.append(f'{type(e).__name__}: {e}')
```
В `cloud_parser('cpp')` внутри `parse` (так же, как узел в задаче 10):
```python
    def parse(data, point_step, fields):
        x, y, z = native.parse_fast(data, point_step, fields, crop=CROP)
        if (len(data) // point_step) % (2 * COLUMN_HEIGHT):
            x, y, z = dedupe_rounded(x, y, z)          # C++ снимает дубли только в облаке из столбцов
        buf = np.empty((len(x), 3), np.float32)
        buf[:, 0], buf[:, 1], buf[:, 2] = x, y, z
        return buf[:, 0], buf[:, 1], buf[:, 2]
```
и в начале ветки `cpp` импорт `from tunnel_od.pointcloud import COLUMN_HEIGHT, dedupe_rounded`.

`tools/dashboard.py:44`: `EMPTY = [...]` заменить на импорт `EMPTY_BAGS as EMPTY` из `bags`.

- [ ] **Step 4: Тест проходит**

Run: `venv/bin/python -m pytest -q tests/test_tools.py`
Expected: PASS

- [ ] **Step 5: `tools/input_report.py` — Python-сводка облака и устойчивое чтение**

В `monitor_bag` цикл `for c, t, raw in reader.messages():` заменить на:
```python
        errors = []
        for c, t, raw in safe_messages(reader.messages(), errors):
```
а после цикла — `summary = mon.summary(); summary['read_errors'] = errors; return summary`. Импорт: `from bags import BAGS, RUNS, bag_path, open_cloud_bag, safe_messages`.

Новая функция (перед `main`):
```python
def cloud_stats(name, max_frames=None):
    """Облако глазами ядра: point_step и поля, доля кадров из столбцов по 128 (кратно 256),
    доля дублей (ячейка 1 см), промежутки между кадрами по времени записи, побитовые повторы."""
    import numpy as np
    from tunnel_od.pointcloud import COLUMN_HEIGHT, FrameRepeat, dedupe_rounded, parse_pointcloud2
    steps, fields, ordered, dups, stamps, repeats, frames, errors = set(), None, 0, [], [], 0, 0, []
    rep = FrameRepeat()
    with open_cloud_bag(name) as (reader, conn):
        for c, t, raw in safe_messages(reader.messages(connections=[conn]), errors):
            if max_frames and frames >= max_frames:
                break
            m = reader.deserialize(raw, c.msgtype)
            frames += 1
            stamps.append(t * 1e-9)
            steps.add(int(m.point_step))
            fields = fields or [(f.name, int(f.offset), int(f.datatype)) for f in m.fields]
            ordered += (len(m.data) // m.point_step) % (2 * COLUMN_HEIGHT) == 0
            repeats += rep.check(m.data)
            if frames % 10 == 1:
                x, y, z = parse_pointcloud2(m.data, m.point_step, m.fields, dedupe_dual_return=False)
                if len(x):
                    dups.append(1.0 - len(dedupe_rounded(x, y, z)[0]) / len(x))
    gaps = np.diff(stamps)
    return {'frames_read': frames, 'read_errors': errors, 'point_step': sorted(steps), 'fields': fields,
            'ordered_share': ordered / max(frames, 1),
            'dup_share_median': float(np.median(dups)) if dups else None,
            'gap_s': {str(p): round(float(np.percentile(gaps, p)), 3) for p in (5, 50, 95, 99)} if len(gaps) else None,
            'gap_max_s': round(float(gaps.max()), 3) if len(gaps) else None,
            'gaps_over_0_15s': int((gaps > 0.15).sum()),
            'bitwise_repeats': repeats}
```
В `main` добавить `ap.add_argument('--no-native', action='store_true', help='только Python-сводка облака')` и цикл:
```python
    for bag in args.bags:
        s = {} if args.no_native else monitor_bag(bag, args.max_frames or None, progress=True)
        s['cloud'] = cloud_stats(bag, args.max_frames or None)
        report[bag] = s
        if s.get('checks'):
            print(f'\n== {bag}: {s["frames"]} кадров, нарушено: {", ".join(s["violations"]) or "ничего"}')
            for cid, ch in s['checks'].items():
                print(f'   {cid:<7}{ch["level"]:<8}{ch["message"]}')
        print(f'   облако: {json.dumps(s["cloud"], ensure_ascii=False)}')
```

- [ ] **Step 6: Прогнать на бэге организаторов и на одной нашей записи**

```bash
venv/bin/python tools/input_report.py --bags cloud_with_fake_obj doubleT_platform --out runs/input_report_fake.json
```
Expected: для `cloud_with_fake_obj` — `frames_read` (ждём 1510), `point_step: [16]`, заполнены `gap_s`, `bitwise_repeats`, `dup_share_median`; для `doubleT_platform` — `point_step: [26]`, `ordered_share: 1.0`. Цифры записать в `docs/PLAN.md` («Бэг организаторов: вход»). Если `read_errors` не пуст — записать, на каком кадре оборвалось.

- [ ] **Step 7: C++-приём на 16-байтном облаке**

```bash
venv/bin/python tools/check_preproc.py --bags cloud_with_fake_obj --frames 30
```
Expected: C++ разбирает кадры (нет ошибок формата). Облака python/cpp совпадают или расходятся только на снятых дублях. Если C++ отвергает 16-байтный формат, это блокер e2e: записать в `docs/PLAN.md` и решить в задаче 17 (`LAUNCH_ARGS=use_cpp_preproc:=false` как значение по умолчанию в `play.sh` / `detector.launch.py`).

- [ ] **Step 8: Коммит**

```bash
git add tools/bags.py tools/input_report.py tools/dashboard.py tests/test_tools.py
git commit -m "Organizers' bag: robust reading, cloud stats in input_report, EMPTY_BAGS" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

---

## Этап 1 (27.09): слой решения и замер базы

### Task 4: `decision.py` — уровни объектов и решение по кадру

**Files:**
- Create: `core/tunnel_od/detection/decision.py`
- Test: `tests/test_core.py`

**Interfaces:**
- Produces:
  - `object_level(obj, *, in_path: bool, pending_score: float) -> tuple[str | None, str | None]`
  - `axis_curvature(fwd, cl, lo=40.0, hi=120.0) -> float` — вторая производная квадратичной аппроксимации `cl(fwd)` на `[lo, hi]`; `0.0`, если точек меньше 5 или отрезок короче 20 м.
  - `sight_distance(path_range, curvature, c=1.5, cap=200.0) -> float`
  - `decide(objects, *, path_available, path_range, curvature, sight_c=1.5, range_cap=200.0, pending_score=1.1) -> dict` с ключами `status`, `distance_m`, `caution_distance_m`, `sight_m`, `clear_to_m`. Пишет `level` и `reason` в каждый объект.
  - Поля объекта, которые читает `object_level`: `held`, `level`/`reason` (у `held`), `edge_line`, `confirmed`, `ego_carried`, `hits`, `evidence`.

- [ ] **Step 1: Падающие тесты** (в `tests/test_core.py`, импорт `from tunnel_od.detection.decision import axis_curvature, decide, object_level, sight_distance`)

```python
def _o(d, **kw):
    o = {'distance_m': d, 'confirmed': False, 'beyond_path': False, 'held': False, 'edge_line': False,
         'hits': 0, 'evidence': 0.0}
    o.update(kw)
    return o


def _decide(objs, path=True, path_range=150.0, curvature=0.0):
    return decide(objs, path_available=path, path_range=path_range if path else None, curvature=curvature,
                  pending_score=1.1)


def test_object_level_table_and_reason_order():
    lv = lambda o, in_path=True: object_level(o, in_path=in_path, pending_score=1.1)
    assert lv(_o(50, confirmed=True)) == ('stop', 'in_gauge')
    assert lv(_o(50, confirmed=True), in_path=False) == ('caution', 'beyond_path')
    assert lv(_o(50, confirmed=True, ego_carried=True)) == ('caution', 'ego_carried')
    assert lv(_o(50, confirmed=True, ego_carried=True), in_path=False) == ('caution', 'beyond_path')
    assert lv(_o(50, hits=2, evidence=1.1)) == ('caution', 'pending')
    assert lv(_o(50, hits=1, evidence=5.0)) == (None, None)
    assert lv(_o(50, hits=3, evidence=1.0)) == (None, None)
    assert lv(_o(50, edge_line=True, hits=5, evidence=5.0)) == (None, None)
    assert lv(_o(50, held=True, level='stop', reason='in_gauge'), in_path=False) == ('stop', 'in_gauge')


def test_decide_status_priority():
    assert _decide([_o(50, confirmed=True), _o(80, confirmed=True, beyond_path=True)])['status'] == 'stop'
    held = _o(50, confirmed=True, held=True, level='stop', reason='in_gauge')
    assert _decide([held], path=False)['status'] == 'stop'                       # stop > unknown
    assert _decide([_o(80, confirmed=True, beyond_path=True)], path=False)['status'] == 'unknown'
    assert _decide([_o(80, confirmed=True, beyond_path=True)], path_range=60.0)['status'] == 'caution'
    assert _decide([_o(80, hits=1)])['status'] == 'clear'


def test_decide_distances():
    r = _decide([_o(70, confirmed=True), _o(40, confirmed=True), _o(30, confirmed=True, beyond_path=True)])
    assert r['distance_m'] == 40 and r['caution_distance_m'] == 30
    r = _decide([_o(80, hits=2, evidence=2.0)])
    assert r['distance_m'] is None and r['caution_distance_m'] == 80


def test_sight_and_clear_to():
    r = _decide([], path=False)
    assert r['sight_m'] == 0.0 and r['clear_to_m'] == 0.0
    assert _decide([], path_range=250.0)['sight_m'] == 200.0
    assert _decide([], path_range=143.0)['sight_m'] == 143.0
    assert _decide([], path_range=250.0, curvature=1 / 300)['sight_m'] == pytest.approx(60.0)   # sqrt(8*300*1.5)
    assert _decide([], path_range=250.0, curvature=1 / 6000)['sight_m'] == 200.0                # |k| < 1/5000
    r = _decide([_o(40, confirmed=True)], path_range=150.0)
    assert r['clear_to_m'] == 40 and r['sight_m'] == 150.0
    assert sight_distance(None, 0.0) == 0.0


def test_decide_clear_to_never_negative():
    held = _o(-2.0, confirmed=True, held=True, level='stop', reason='in_gauge')
    r = _decide([held])
    assert r['status'] == 'stop' and r['clear_to_m'] == 0.0


def test_axis_curvature_of_arc():
    R = 300.0
    f = np.arange(2.0, 150.0, 1.0)
    assert axis_curvature(f, f ** 2 / (2 * R)) == pytest.approx(1 / R, rel=1e-3)
    assert axis_curvature(f, 0.01 * f) == pytest.approx(0.0, abs=1e-9)
    assert axis_curvature(np.arange(2.0, 45.0, 1.0), np.zeros(43)) == 0.0         # до 45 м: на 40-120 мало точек
```

Run: `venv/bin/python -m pytest -q tests/test_core.py -k "object_level or decide or sight or curvature"`
Expected: FAIL (`ModuleNotFoundError: tunnel_od.detection.decision`)

- [ ] **Step 2: Реализация `core/tunnel_od/detection/decision.py`**

```python
"""Решение по кадру поверх подтверждённых объектов: уровень каждого объекта, статус кадра
и докуда габарит проверен свободным. Чистые функции без состояния (состояние -- у трекера).

Уровень объекта (level, reason), первая подходящая строка:
    stop     in_gauge     подтверждён, в пределах оси, не едет с поездом
    caution  beyond_path  подтверждён, за концом известной оси
    caution  ego_carried  подтверждён, ЭГО-тест: едет вместе с поездом
    caution  pending      не подтверждён, трек >= 2 кадров и счёт >= pending_score
    None     None         остальное (мелкие одиночные, продольные конструкции у края)
Удержанный объект (held) наследует уровень трека из последнего кадра, где трек был виден.

Статус кадра: stop > unknown (нет оси -- габарит не проверить) > caution > clear.
"""
import math

import numpy as np

RANGE_CAP_M = 200.0          # паспортная дальность Pandar128 при отражении 10 %
SIGHT_C_M = 1.5              # хорда прямой видимости на кривой: sqrt(8 R c)
MIN_CURVATURE = 1 / 5000     # кривая радиусом больше 5 км -- прямая


def object_level(obj, *, in_path, pending_score):
    if obj.get('held'):
        return obj.get('level', 'stop'), obj.get('reason', 'in_gauge')
    if obj.get('edge_line'):
        return None, None
    if obj.get('confirmed'):
        if in_path and not obj.get('ego_carried'):
            return 'stop', 'in_gauge'
        return 'caution', ('ego_carried' if in_path else 'beyond_path')
    if obj.get('hits', 0) >= 2 and obj.get('evidence', 0.0) >= pending_score:
        return 'caution', 'pending'
    return None, None


def axis_curvature(fwd, cl, lo=40.0, hi=120.0):
    """Кривизна оси (1/R со знаком) по квадратичной аппроксимации cl(fwd) на [lo, hi]."""
    if fwd is None or cl is None:
        return 0.0
    fwd, cl = np.asarray(fwd, float), np.asarray(cl, float)
    m = (fwd >= lo) & (fwd <= hi)
    if m.sum() < 5 or fwd[m].max() - fwd[m].min() < 20.0:
        return 0.0
    return float(2.0 * np.polyfit(fwd[m], cl[m], 2)[0])


def sight_distance(path_range, curvature, c=SIGHT_C_M, cap=RANGE_CAP_M):
    """Докуда путь виден: ось, прямая видимость на кривой, паспортная дальность."""
    if path_range is None:
        return 0.0
    s = min(float(path_range), cap)
    if abs(curvature) >= MIN_CURVATURE:
        s = min(s, math.sqrt(8.0 * c / abs(curvature)))
    return s


def decide(objects, *, path_available, path_range, curvature, sight_c=SIGHT_C_M, range_cap=RANGE_CAP_M,
           pending_score=1.1):
    for o in objects:
        o['level'], o['reason'] = object_level(o, in_path=not o.get('beyond_path', False),
                                               pending_score=pending_score)
    stop = [o['distance_m'] for o in objects if o['level'] == 'stop']
    caution = [o['distance_m'] for o in objects if o['level'] == 'caution']
    if stop:
        status = 'stop'
    elif not path_available:
        status = 'unknown'
    elif caution:
        status = 'caution'
    else:
        status = 'clear'
    sight = sight_distance(path_range, curvature, sight_c, range_cap) if path_available else 0.0
    near = min(stop + caution, default=None)
    clear_to = 0.0 if not path_available else max(0.0, sight if near is None else min(sight, near))
    return {'status': status,
            'distance_m': min(stop) if stop else None,
            'caution_distance_m': min(caution) if caution else None,
            'sight_m': sight,
            'clear_to_m': clear_to}
```

- [ ] **Step 3: Тесты проходят**

Run: `venv/bin/python -m pytest -q tests/test_core.py -k "object_level or decide or sight or curvature"`
Expected: PASS (6 тестов)

- [ ] **Step 4: Коммит**

```bash
git add core/tunnel_od/detection/decision.py tests/test_core.py
git commit -m "decision.py: object levels, frame status with unknown, sight_m and clear_to_m" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 5: `decide` в детекторе, `hits` и уровни в трекере

**Files:**
- Modify: `core/tunnel_od/detection/tracking.py`, `core/tunnel_od/detection/detector.py`
- Test: `tests/test_core.py`

**Interfaces:**
- Consumes: `decide`, `axis_curvature` (задача 4).
- Produces:
  - В каждом объекте трекера: `hits` (int), в `EvidenceTracker` — ещё `track_id`, `evidence`.
  - `EvidenceTracker.set_levels(levels: dict[int, tuple[str|None, str|None]])`, `Tracker.set_levels(levels)` (ничего не делает).
  - Результат `check_frame`/`detect`: к прежним ключам добавлены `status`, `caution_distance_m`, `sight_m`, `clear_to_m`, `travel_m`. `obstacle == (status == 'stop')`, `distance_m` — до ближайшего `stop`.
  - Каждый объект результата: `level`, `reason`, `rail_z_m` (z головки рельса на его дистанции, система лидара). У `held`-объекта есть `track_id`.

- [ ] **Step 1: Падающие тесты**

```python
def test_detector_status_fields_clear_and_stop():
    det = ObstacleDetector()
    res = _run(det, [{}] * 3)
    assert res['status'] == 'clear' and not res['obstacle']
    assert 100 < res['clear_to_m'] <= 200 and res['sight_m'] >= res['clear_to_m']
    assert res['travel_m'] == pytest.approx(0.0, abs=1.0)
    res = _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 4)
    assert res['status'] == 'stop' and res['obstacle'] is True
    assert res['clear_to_m'] == pytest.approx(res['distance_m'])
    stop = [o for o in res['objects'] if o['level'] == 'stop']
    assert stop and all(o['reason'] == 'in_gauge' for o in stop)
    assert all('rail_z_m' in o for o in res['objects'])


def test_detector_unknown_without_path():
    det = ObstacleDetector()
    x, y, z, *_ = simulate_frame()
    res = det.check_frame(x, y, z)                    # путь ещё не считался
    assert res['status'] == 'unknown' and not res['obstacle']
    assert res['sight_m'] == 0.0 and res['clear_to_m'] == 0.0


def test_held_object_inherits_level():
    det = ObstacleDetector()
    _run(det, [{}] * 3)
    _run(det, [{'obstacle_forward': 30.0, 'obstacle_radius': 0.35}] * 8)
    res = _run(det, [{}])
    held = [o for o in res['objects'] if o.get('held')]
    assert held and held[0]['level'] == 'stop' and held[0]['reason'] == 'in_gauge' and 'track_id' in held[0]
```

Run: `venv/bin/python -m pytest -q tests/test_core.py -k "status_fields or unknown_without_path or inherits_level"`
Expected: FAIL (`KeyError: 'status'`)

- [ ] **Step 2: `tracking.py`**

В `Tracker.update` последний цикл:
```python
        for ob in objects:
            hist = ob.pop('_track')['hist']
            ob['hits'] = sum(hist)
            ob['confirmed'] = ob['hits'] >= self.confirm_hits
```
В `Tracker` после `set_alarm`:
```python
    def set_levels(self, levels):
        pass
```
В `EvidenceTracker.update` после `ob['evidence'] = round(tr['score'], 2)`:
```python
            ob['hits'] = tr['hits']
```
В `EvidenceTracker` после `set_alarm`:
```python
    def set_levels(self, levels):
        """{track_id: (level, reason)} объектов этого кадра -- их наследует удержанный объект (held)."""
        for tr in self.tracks:
            if tr['id'] in levels:
                tr['level'], tr['reason'] = levels[tr['id']]
```

- [ ] **Step 3: `detector.py`**

Импорт: `from .decision import axis_curvature, decide`.
В `__init__` рядом с `self.far_axis = far_axis`: `self.evidence_threshold = evidence_threshold`.
В `check_frame` после цикла `for o in objects: o.setdefault('confirmed', False); o['held'] = False` добавить:
```python
        if objects:
            rz = self._tor_at(np.array([o['distance_m'] for o in objects]), self._bed)
            for o, r in zip(objects, rz):
                o['rail_z_m'] = float(r)
```
Словарь удержанного объекта дополнить ключами:
```python
                            'evidence': round(tr['score'], 2), 'track_id': tr['id'],
                            'level': tr.get('level', 'stop'), 'reason': tr.get('reason', 'in_gauge'),
                            'rail_z_m': float(self._tor_at(np.array([d]), self._bed)[0])})
```
Строки от `confirmed = [...]` до конца `return {...}` заменить на:
```python
        curvature = axis_curvature(self._fit_fwd, self._fitted_cl) if path_ok else 0.0
        dec = decide(objects, path_available=path_ok, path_range=path_range, curvature=curvature,
                     pending_score=0.5 * self.evidence_threshold)
        self._tracker.set_alarm({o['track_id'] for o in objects if o['level'] == 'stop' and 'track_id' in o})
        self._tracker.set_levels({o['track_id']: (o['level'], o['reason']) for o in objects
                                  if 'track_id' in o and not o['held']})
        return {
            'status': dec['status'],
            'obstacle': dec['status'] == 'stop',
            'distance_m': dec['distance_m'],
            'caution_distance_m': dec['caution_distance_m'],
            'sight_m': dec['sight_m'],
            'clear_to_m': dec['clear_to_m'],
            'n_points': int(m.sum()),
            'path_available': path_ok,
            'gauge_m': self._track_gauge,
            'path_age_frames': None if not path_ok else self._frame - self._path_frame,
            'path_range_m': path_range,
            'objects': objects,
            'speed_mps': self.speed,
            'displacement_m': self._displacement,
            'travel_m': self._travel,
            'stamp': stamp,
        }
```
В docstring модуля строку «путь -> полотно -> зона -> объекты -> подтверждение -> решение о тревоге» дополнить: «решение -- detection/decision.py (status: stop | unknown | caution | clear)».

- [ ] **Step 4: Все тесты ядра**

Run: `venv/bin/python -m pytest -q tests`
Expected: всё PASS (в том числе прежние 40: `stop` ≡ прежний `confirmed and not beyond_path`)

- [ ] **Step 5: Коммит**

```bash
git add core/tunnel_od/detection/tracking.py core/tunnel_od/detection/detector.py tests/test_core.py
git commit -m "check_frame: status/level/clear_to_m via decide(); held objects inherit level" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 6: `alarms.py` — новые колонки, objects.jsonl, повторы; тревога = `level == 'stop'` в остальных инструментах

**Files:**
- Modify: `tools/alarms.py`, `tools/eval_approach.py`, `research/show_frame.py`

**Interfaces:**
- Consumes: `safe_messages`, `EMPTY_BAGS` (задача 3), `FrameRepeat` (задача 2), поля результата (задача 5).
- Produces:
  - `runs/alarms_<tag>.csv` с колонками: `bag, frame, t_s, alarm, status, caution, distance_m, caution_distance_m, lateral_m, height_m, n_points, n_confirmed, path_range_m, sight_m, clear_to_m, travel_m, displacement_m, repeated` (`''` вместо `None`).
  - `runs/alarms_<tag>_objects.jsonl`: строка на кадр `{"bag", "frame", "repeated", "objects": [...]}`; в объекте — ключи `OBJ_KEYS`. По умолчанию только объекты уровней `stop`/`caution`, с `--all-objects` — все, кроме `edge_line`.
  - `alarms.obj_record(o) -> dict`.

- [ ] **Step 1: `alarms.py`**

Импорты: `from tunnel_od.pointcloud import FrameRepeat`; `from bags import BAGS, EMPTY_BAGS, RUNS as OUT, cloud_parser, open_cloud_bag, safe_messages`.

Константа и функции:
```python
OBJ_KEYS = ('track_id', 'distance_m', 'lateral_m', 'height_m', 'low_m', 'n_points', 'level', 'reason',
            'ego_slope', 'evidence', 'held', 'too_small', 'rail_z_m')


def obj_record(o):
    return {k: (round(o[k], 3) if isinstance(o[k], float) else o[k]) for k in OBJ_KEYS if k in o}


def _fmt(v, nd):
    return '' if v is None else round(v, nd)
```
`_stream` получает `errors` и читает через `safe_messages`:
```python
def _stream(reader, conn, segments, seglen, errors):
    if not segments:
        for i, (c, t, raw) in enumerate(safe_messages(reader.messages(connections=[conn]), errors)):
            yield i, t, c, raw, i == 0
        return
    t0, dur = reader.start_time, reader.duration
    for k in range(segments):
        start = t0 + int(dur * (k + 0.5) / segments)
        for j, (c, t, raw) in enumerate(safe_messages(reader.messages(connections=[conn], start=start), errors)):
            if j == seglen:
                break
            yield k * seglen + j, t, c, raw, j == 0
```
`run_bag` целиком:
```python
def run_bag(job):
    bag, corridor, minpts, method, det_kwargs, segments, seglen, parser, all_objects = job
    warnings.simplefilter('ignore')
    parse_pointcloud2 = cloud_parser(parser)
    make = lambda: ObstacleDetector(zone=CORRIDORS[corridor], method=method, **{**MINPTS[minpts], **det_kwargs})
    rows, objs, errors = [], [], []
    with open_cloud_bag(bag) as (reader, conn):
        t0 = reader.start_time
        for i, t, c, raw, fresh in _stream(reader, conn, segments, seglen, errors):
            if fresh:
                det, repeat, prev = make(), FrameRepeat(), None
            m = reader.deserialize(raw, c.msgtype)
            t_s = round((t - t0) / 1e9, 2)
            if repeat.check(m.data) and prev is not None:
                # побитовый повтор кадра: детектор не вызывается, результат -- прошлый
                rows.append(dict(prev[0], frame=i, t_s=t_s, repeated=1))
                objs.append(dict(prev[1], frame=i, repeated=1))
                continue
            res = det.detect(*parse_pointcloud2(m.data, m.point_step, m.fields), refit_path=True, stamp=t / 1e9)
            stop = [o for o in res['objects'] if o['level'] == 'stop']
            near = min(stop, key=lambda o: o['distance_m']) if stop else None
            row = {'bag': bag, 'frame': i, 't_s': t_s, 'alarm': int(res['obstacle']), 'status': res['status'],
                   'caution': int(res['status'] == 'caution'),
                   'distance_m': _fmt(res['distance_m'], 1), 'caution_distance_m': _fmt(res['caution_distance_m'], 1),
                   'lateral_m': round(near['lateral_m'], 2) if near else '',
                   'height_m': round(near['height_m'], 2) if near else '',
                   'n_points': near['n_points'] if near else '',
                   'n_confirmed': len(stop),
                   'path_range_m': round(res['path_range_m'], 0) if res['path_range_m'] else '',
                   'sight_m': round(res['sight_m'], 1), 'clear_to_m': round(res['clear_to_m'], 1),
                   'travel_m': round(res['travel_m'], 2), 'displacement_m': _fmt(res['displacement_m'], 3),
                   'repeated': 0}
            keep = [o for o in res['objects'] if o['level'] or (all_objects and not o.get('edge_line'))]
            ol = {'bag': bag, 'frame': i, 'repeated': 0, 'objects': [obj_record(o) for o in keep]}
            rows.append(row)
            objs.append(ol)
            prev = (row, ol)
            if i % 2000 == 0 and i:
                print(f'  {bag}: {i} кадров', flush=True)
    if errors:
        print(f'  {bag}: чтение оборвалось после {len(rows)} кадров: {errors[0]}', flush=True)
    return rows, objs, errors
```
В `main`: аргумент `ap.add_argument('--all-objects', action='store_true', help='в objects.jsonl -- все объекты, не только stop/caution (для эталона cloud_with_fake_obj)')`; в кортеж задания добавить `args.all_objects`; разбор результата:
```python
    rows = [r for p in parts for r in p[0]]
    obj_lines = [o for p in parts for o in p[1]]
    if not rows:                       # запись не читается с первого кадра
        print('ни одного кадра:', '; '.join(e for p in parts for e in p[2]))
        return
    ...
    with open(OUT / f'alarms_{args.tag}_objects.jsonl', 'w', encoding='utf-8') as f:
        for o in obj_lines:
            f.write(json.dumps(o, ensure_ascii=False) + '\n')
```
Сводку заменить на (доли кадров СТОП / ВНИМАНИЕ / unknown, строка «ПУСТЫЕ» по `EMPTY_BAGS`):
```python
    head = (f'\n{"запись":<37}{"кадров":>7}{"мин":>6}{"СТОП":>8}{"ВНИМ.":>8}{"unknown":>9}{"повт.":>7}'
            f'{"эпизодов":>10}{"эпизодов/ч":>11}{"дист. медиана":>14}')
    print(head)

    def line(name, part):
        st = np.array([r['status'] for r in part])
        a = st == 'stop'
        dur = len(part) / 10.0 / 3600   # 10 Гц; по t_s нельзя: отрезки
        d = [r['distance_m'] for r in part if r['alarm']]
        print(f'{name:<37}{len(part):>7}{60 * dur:>6.1f}{100 * a.mean():>7.1f}%{100 * (st == "caution").mean():>7.1f}%'
              f'{100 * (st == "unknown").mean():>8.1f}%{sum(r["repeated"] for r in part):>7}'
              f'{len(episodes(a)):>10}{len(episodes(a)) / max(dur, 1e-9):>11.0f}'
              f'{(f"{np.median(d):.0f}м" if d else "-"):>14}')

    by_bag = {b: [r for r in rows if r['bag'] == b] for b in args.bags}
    for bag in args.bags:
        if by_bag[bag]:
            line(bag, by_bag[bag])
    empty = [r for b in args.bags if b in EMPTY_BAGS for r in by_bag[b]]
    if empty:
        line('ПУСТЫЕ', empty)
    line('ВСЕГО', rows)
    for bag, p in zip(args.bags, parts):
        if p[2]:
            print(f'{bag}: ошибка чтения -- {p[2][0]}')
    print(f'\nМетод: {args.method}, коридор: {args.corridor}, порог точек: {args.minpts}. Записано: {path}')
```
Строку про «1-2 точки» убрать. `episodes` не меняется. `rows[0]` в `DictWriter` не менять.

- [ ] **Step 2: Остальные инструменты читают уровень**

`tools/eval_approach.py` (в `run_bag`):
```python
                    conf = [o for o in res['objects'] if o.get('level') == 'stop']
```
`research/show_frame.py`:
```python
objs = [o for o in res['objects'] if o.get('level') in ('stop', 'caution')]
print({k: res[k] for k in ('status', 'obstacle', 'distance_m', 'caution_distance_m', 'sight_m', 'clear_to_m', 'path_range_m')})
```
и в подписи картинки `f"{o['level']} {o['distance_m']:.0f}м/{o['lateral_m']:+.2f}"`.

- [ ] **Step 3: Проверка «`stop` ≡ прежний `obstacle`» на 6 записях**

```bash
cd tools && ../venv/bin/python alarms.py --tag post --bags doubleT_obstacle doubleT_platform roundT_doubleT \
  roundT_pressureGate_roundT roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch --workers 2 \
  > ../runs/post_alarms.log 2>&1; cd ..
venv/bin/python - <<'PY'
import csv
a = {(r['bag'], r['frame']): r['alarm'] for r in csv.DictReader(open('runs/alarms_pre.csv'))}
b = {(r['bag'], r['frame']): (r['alarm'], r['repeated']) for r in csv.DictReader(open('runs/alarms_post.csv'))}
diff = [k for k in a if b[k][1] == '0' and a[k] != b[k][0]]
print('кадров', len(a), 'расхождений alarm', len(diff), diff[:10])
PY
```
Expected: `расхождений alarm 0`. Кадры с `repeated=1` из сравнения исключены: в наших записях повторов быть не должно (колонка «повт.» = 0). Если повторы есть, записать их число в `docs/PLAN.md`.

- [ ] **Step 4: Тесты и коммит**

Run: `venv/bin/python -m pytest -q tests` → всё PASS.
```bash
git add tools/alarms.py tools/eval_approach.py research/show_frame.py
git commit -m "alarms.py: status/caution/sight/clear_to/travel columns, objects.jsonl, frame repeats" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 7: `eval_fake_obj.py` — черновик эталона и оценка по 10 объектам

**Files:**
- Create: `tools/eval_fake_obj.py`
- Test: `tests/test_tools.py`

**Interfaces:**
- Consumes: `runs/alarms_<tag>.csv`, `runs/alarms_<tag>_objects.jsonl` (задача 6).
- Produces:
  - `group_x(points, gap=15.0) -> list[list]` — `points`: список `(x, frame, obj)`, сортирует по `x` и режет, где соседние дальше `gap`.
  - `evaluate(ref, frames, objs, half=5.0) -> (rows, false_frames)`. `ref` — список словарей `{id, desc, expect ('stop'|'not_stop'), x_m, ...}`; `frames` — список словарей `{frame, travel_m, status, sight_m}` по порядку кадров; `objs` — `{frame: [obj]}`. Строка результата: `id, desc, expect, level ('stop'|'caution'|'none'), reason, first_stop_m, first_frame, sight_m, stop_share, ok`.
  - `summary_line(tag, rows, false_frames, n_frames) -> str`.
  - CLI: `eval_fake_obj.py draft --tag T` → `runs/fake_obj_draft_<T>.yaml` и таблица групп; `eval_fake_obj.py eval --tag T [--ref PATH]` → `runs/fake_obj_<T>.csv`, `runs/fake_obj_<T>.txt` (строка сводки).

- [ ] **Step 1: Падающие тесты** (в `tests/test_tools.py`; импорт `from eval_fake_obj import evaluate, group_x, summary_line`)

```python
def _ref():
    return [{'id': 1, 'desc': 'куб', 'expect': 'stop', 'x_m': 100.0},
            {'id': 5, 'desc': 'вне габарита', 'expect': 'not_stop', 'x_m': 200.0},
            {'id': 9, 'desc': 'не виден', 'expect': 'stop', 'x_m': 300.0}]


def test_group_x_splits_on_gap():
    pts = [(100.0, 1, {}), (101.0, 2, {}), (200.0, 3, {}), (102.0, 4, {}), (201.0, 5, {})]
    groups = group_x(pts, gap=15.0)
    assert [len(g) for g in groups] == [3, 2]


def test_evaluate_first_stop_share_and_false_frames():
    frames = [{'frame': k, 'travel_m': 2.0 * k, 'status': 'clear', 'sight_m': 150.0} for k in range(40)]
    objs = {}
    for k in range(5, 20):                                   # объект 1: X = 100, СТОП с кадра 5
        objs[k] = [{'distance_m': 100.0 - 2.0 * k, 'level': 'stop', 'reason': 'in_gauge'}]
        frames[k]['status'] = 'stop'
    for k in range(10, 12):                                  # объект 5: X = 200, ВНИМАНИЕ
        objs.setdefault(k, []).append({'distance_m': 200.0 - 2.0 * k, 'level': 'caution', 'reason': 'beyond_path'})
    objs[30] = [{'distance_m': 20.0, 'level': 'stop', 'reason': 'in_gauge'}]   # X = 80: ни в одно окно
    frames[30]['status'] = 'stop'
    rows, false_frames = evaluate(_ref(), frames, objs)
    r1, r5, r9 = rows
    assert r1['level'] == 'stop' and r1['first_frame'] == 5 and r1['first_stop_m'] == 90.0 and r1['sight_m'] == 150.0
    assert r1['stop_share'] == 1.0 and r1['ok']
    assert r5['level'] == 'caution' and r5['reason'] == 'beyond_path' and r5['ok']
    assert false_frames == [30]
    line = summary_line('t', rows, false_frames, 40)
    assert 'СТОП 1/2' in line and 'ложных СТОП-кадров 1 из 40' in line


def test_evaluate_unseen_object_is_none():
    frames = [{'frame': k, 'travel_m': float(k), 'status': 'clear', 'sight_m': 150.0} for k in range(5)]
    rows, false_frames = evaluate(_ref(), frames, {})
    assert rows[2]['level'] == 'none' and rows[2]['first_stop_m'] is None and not rows[2]['ok']
    assert false_frames == []


def test_evaluate_stop_on_out_of_gauge_object_is_not_ok():
    frames = [{'frame': 0, 'travel_m': 0.0, 'status': 'stop', 'sight_m': 150.0}]
    objs = {0: [{'distance_m': 199.0, 'level': 'stop', 'reason': 'in_gauge'}]}
    rows, _ = evaluate(_ref(), frames, objs)
    assert rows[1]['level'] == 'stop' and not rows[1]['ok']
    assert 'СТОП вне габарита: 5' in summary_line('t', rows, [], 1)
```

Run: `venv/bin/python -m pytest -q tests/test_tools.py`
Expected: FAIL (`ModuleNotFoundError: eval_fake_obj`)

- [ ] **Step 2: Реализация `tools/eval_fake_obj.py`**

```python
"""Оценка на бэге организаторов cloud_with_fake_obj (#437, #460): 10 синтетических объектов
через ~100 м. Объект привязан к миру координатой X = travel_m + distance_m (пробег поезда по
оценке скорости + дистанция в кадре), окно эталона X +- 5 м.

    python tools/alarms.py --tag fo_base0 --bags cloud_with_fake_obj --workers 1 --all-objects
    python tools/eval_fake_obj.py draft --tag fo_base0     # группы по X -> черновик эталона
    python tools/eval_fake_obj.py eval --tag fo_base       # метрики варианта по эталону

Эталон -- data/cloud_with_fake_obj/objects.yaml (копия в git: reference/cloud_with_fake_obj/objects.yaml):
    objects:
      - {id: 1, desc: '...', expect: stop, x_m: 312.4, lateral_m: 0.02, low_m: 1.40, rail_z_m: -1.07}
expect: stop -- объекты 1, 2, 3, 4, 6, 9, 10 (в габарите); not_stop -- 5, 7, 8.

Метрики по объекту: первый СТОП (дистанция, кадр, sight_m в этом кадре), доля кадров СТОП после
первого (пока объект впереди), итоговый уровень (высший за проезд) и причина. Ложный СТОП-кадр --
кадр со status=stop, в котором есть объект stop вне всех окон. Пробег зависит от оценки скорости:
если меняется EgoMotion (задача 13), эталон строится заново.
"""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import DATA, FAKE_OBJ, ROOT, RUNS          # noqa: E402

HALF_WINDOW = 5.0
RANK = {None: 0, 'caution': 1, 'stop': 2}
REF_PATHS = [DATA / FAKE_OBJ / 'objects.yaml', ROOT / 'reference' / FAKE_OBJ / 'objects.yaml']


def group_x(points, gap=15.0):
    pts = sorted(points, key=lambda p: p[0])
    groups = []
    for p in pts:
        if groups and p[0] - groups[-1][-1][0] <= gap:
            groups[-1].append(p)
        else:
            groups.append([p])
    return groups


def evaluate(ref, frames, objs, half=HALF_WINDOW):
    per = {r['id']: {'first_frame': None, 'first_stop_m': None, 'sight_m': None,
                     'after': 0, 'stop_after': 0, 'levels': []} for r in ref}
    false_frames = []
    for fr in frames:
        f, travel = fr['frame'], fr['travel_m']
        seen, outside = {}, False
        for o in objs.get(f, []):
            x = travel + o['distance_m']
            hit = [r for r in ref if abs(x - r['x_m']) <= half]
            if o.get('level') == 'stop' and not hit:
                outside = True
            for r in hit:
                if RANK[o.get('level')] > RANK[seen.get(r['id'], (None,))[0]]:
                    seen[r['id']] = (o.get('level'), o.get('reason'), o)
        if fr['status'] == 'stop' and outside:
            false_frames.append(f)
        for r in ref:
            p = per[r['id']]
            level, reason, o = seen.get(r['id'], (None, None, None))
            if level:
                p['levels'].append((level, reason))
            if level == 'stop' and p['first_frame'] is None:
                p['first_frame'], p['first_stop_m'], p['sight_m'] = f, o['distance_m'], fr['sight_m']
            if p['first_frame'] is not None and travel < r['x_m'] - half:
                p['after'] += 1
                p['stop_after'] += level == 'stop'
    rows = []
    for r in ref:
        p = per[r['id']]
        top = max((lv for lv, _ in p['levels']), key=RANK.get, default=None)
        reasons = Counter(s for lv, s in p['levels'] if lv == top)
        rows.append({'id': r['id'], 'desc': r.get('desc', ''), 'expect': r['expect'], 'level': top or 'none',
                     'reason': reasons.most_common(1)[0][0] if reasons else '',
                     'first_stop_m': p['first_stop_m'], 'first_frame': p['first_frame'], 'sight_m': p['sight_m'],
                     'stop_share': round(p['stop_after'] / p['after'], 2) if p['after'] else None,
                     'ok': (top == 'stop') == (r['expect'] == 'stop')})
    return rows, false_frames


def summary_line(tag, rows, false_frames, n_frames):
    need = [r for r in rows if r['expect'] == 'stop']
    got = sum(r['level'] == 'stop' for r in need)
    bad = [str(r['id']) for r in rows if r['expect'] != 'stop' and r['level'] == 'stop']
    firsts = ' '.join(f"{r['id']}:{r['first_stop_m']:.0f}м" for r in need if r['first_stop_m'] is not None)
    return (f'fake_obj {tag}: в габарите СТОП {got}/{len(need)}; СТОП вне габарита: {", ".join(bad) or "нет"}; '
            f'ложных СТОП-кадров {len(false_frames)} из {n_frames}; первые СТОП {firsts}')


def _num(v):
    return None if v in ('', None) else float(v)


def load_run(tag):
    frames = [{'frame': int(r['frame']), 'travel_m': float(r['travel_m']), 'status': r['status'],
               'sight_m': _num(r['sight_m'])}
              for r in csv.DictReader(open(RUNS / f'alarms_{tag}.csv', encoding='utf-8')) if r['bag'] == FAKE_OBJ]
    objs = {}
    with open(RUNS / f'alarms_{tag}_objects.jsonl', encoding='utf-8') as f:
        for line in f:
            o = json.loads(line)
            if o['bag'] == FAKE_OBJ:
                objs[o['frame']] = o['objects']
    return frames, objs


def load_ref(path=None):
    from ruamel.yaml import YAML
    path = Path(path) if path else next(p for p in REF_PATHS if p.exists())
    return YAML(typ='safe').load(path.read_text(encoding='utf-8'))['objects']


def draft(tag):
    frames, objs = load_run(tag)
    travel = {f['frame']: f['travel_m'] for f in frames}
    pts = [(travel[k] + o['distance_m'], k, o) for k, os in objs.items() for o in os]
    groups = [g for g in group_x(pts) if len({p[1] for p in g}) >= 3]
    print(f'кадров {len(frames)}, пробег {frames[-1]["travel_m"]:.0f} м, групп (>= 3 кадров) {len(groups)}')
    print(f'{"#":>3}{"X, м":>9}{"шаг":>7}{"кадры":>12}{"lat":>7}{"low":>7}{"height":>8}{"rail_z":>8}{"уровень":>9}')
    out, prev = [], None
    for k, g in enumerate(groups, 1):
        med = lambda key: float(np.median([p[2][key] for p in g if key in p[2]])) if any(key in p[2] for p in g) else None
        x = float(np.median([p[0] for p in g]))
        top = max((p[2].get('level') for p in g), key=RANK.get)
        fr = sorted({p[1] for p in g})
        print(f'{k:>3}{x:>9.1f}{"" if prev is None else f"{x - prev:.0f}":>7}{f"{fr[0]}-{fr[-1]}":>12}'
              f'{med("lateral_m"):>7.2f}{med("low_m"):>7.2f}{med("height_m"):>8.2f}{med("rail_z_m"):>8.2f}{str(top):>9}')
        out.append({'id': k, 'desc': '', 'expect': 'stop', 'x_m': round(x, 1), 'lateral_m': round(med('lateral_m'), 2),
                    'low_m': round(med('low_m'), 2), 'rail_z_m': round(med('rail_z_m'), 2),
                    'frames': f'{fr[0]}-{fr[-1]}'})
        prev = x
    from ruamel.yaml import YAML
    path = RUNS / f'fake_obj_draft_{tag}.yaml'
    with open(path, 'w', encoding='utf-8') as f:
        YAML().dump({'objects': out}, f)
    print(f'черновик: {path} -- сверить с #437/#460 и show_frame.py, заполнить desc/expect, сохранить как '
          f'{REF_PATHS[0]} и {REF_PATHS[1]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['draft', 'eval'])
    ap.add_argument('--tag', required=True, help='тег прогона alarms.py по cloud_with_fake_obj')
    ap.add_argument('--ref', default=None, help='эталон objects.yaml (по умолчанию data/, затем reference/)')
    args = ap.parse_args()
    if args.cmd == 'draft':
        return draft(args.tag)
    frames, objs = load_run(args.tag)
    rows, false_frames = evaluate(load_ref(args.ref), frames, objs)
    with open(RUNS / f'fake_obj_{args.tag}.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        first = '' if r['first_stop_m'] is None else f"{r['first_stop_m']:.0f} м (видимость {r['sight_m']:.0f} м)"
        print(f"{r['id']:>3} {r['expect']:<9}{r['level']:<9}{r['reason']:<13}{first}")
    line = summary_line(args.tag, rows, false_frames, len(frames))
    (RUNS / f'fake_obj_{args.tag}.txt').write_text(line + '\n', encoding='utf-8')
    print(line)


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Тесты проходят**

Run: `venv/bin/python -m pytest -q tests/test_tools.py`
Expected: PASS (6 тестов)

- [ ] **Step 4: Коммит**

```bash
git add tools/eval_fake_obj.py tests/test_tools.py
git commit -m "eval_fake_obj.py: reference draft by world X and per-object metrics on organizers' bag" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 8: Прогон детектора на бэге организаторов и эталон `objects.yaml` (этап 0, ручная проверка)

**Files:**
- Create: `data/cloud_with_fake_obj/objects.yaml`, `reference/cloud_with_fake_obj/objects.yaml` (копия в git)
- Modify: `docs/PLAN.md`

- [ ] **Step 1: Прогон со всеми объектами**

```bash
cd tools && ../venv/bin/python alarms.py --tag fo_base0 --bags cloud_with_fake_obj --workers 1 --all-objects \
  > ../runs/fo_base0.log 2>&1; cd ..; tail -8 runs/fo_base0.log
venv/bin/python tools/eval_fake_obj.py draft --tag fo_base0
```
Expected: число кадров (1510 или сколько прочиталось), доля повторов, таблица групп по X с шагом «около 100 м».

- [ ] **Step 2: Сверка пробега (5.1.6).** Шаг между соседними группами — около 100 м. Если шаги систематически меньше (пробег недооценён: сдвиг не помещается в окно 22,5 м) или скачут больше чем на ±20 м, выполнить задачу 13 до построения эталона. Цифры записать в `docs/PLAN.md` («Бэг организаторов: пробег»).

- [ ] **Step 3: Проверка каждой группы глазами.** Для каждой группы — кадр из середины её диапазона `frames`:
```bash
venv/bin/python research/show_frame.py cloud_with_fake_obj <кадр> '{}' runs/img/fo_<id>.png
```
Сопоставить группы по порядку с объектами 1–10 из #437/#460 (задача 1, шаг 1). Объект, которого нет среди групп, найти глазами по ожидаемому X: соседний X ± 100 м, кадры, где `travel_m` на 30–150 м меньше X. Записать его X вручную.

- [ ] **Step 4: Записать эталон.** Скопировать `runs/fake_obj_draft_fo_base0.yaml` в `data/cloud_with_fake_obj/objects.yaml`, заполнить `desc` (дословно из чата), `expect` (`stop` для 1, 2, 3, 4, 6, 9, 10; `not_stop` для 5, 7, 8), поправить `x_m` у найденных вручную, удалить лишние группы. Для каждого объекта `low_m` и `rail_z_m`. Если `low_m < 0.15` или объект ниже полотна, пометить `note: ниже зоны` (3.3). Скопировать в `reference/cloud_with_fake_obj/objects.yaml`.

- [ ] **Step 5: Проверить эталон на том же прогоне**

Run: `venv/bin/python tools/eval_fake_obj.py eval --tag fo_base0`
Expected: строка сводки. Цифры — это база ответа на вопросы 1.3 (граница ±1,15 м, парящие объекты): записать в `docs/PLAN.md`.

- [ ] **Step 6: Коммит**

```bash
git add reference/cloud_with_fake_obj/objects.yaml docs/PLAN.md
git commit -m "Organizers' bag: reference objects.yaml, input and odometry check" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 9: `compare.sh` + `compare_row.py` и замер базы

**Files:**
- Modify: `research/compare.sh`
- Create: `research/compare_row.py`
- Test: `tests/test_tools.py`

**Interfaces:**
- Consumes: CSV `alarms.py`, `runs/approach_<tag>.csv`, `runs/fake_obj_fo_<tag>.{csv,txt}`, `runs/selflabel_nd_<tag>.txt` (задача 15, если есть).
- Produces:
  - `research/compare.sh <tag> '<json>'` — полный прогон; `FAST=1` — только бэг организаторов; `ND_SEGMENTS=8` — `new_data` отрезками вместо всей записи.
  - `compare_row.shares(rows) -> (stop%, caution%, unknown%)`, `compare_row.first_stop_56(rows) -> (frame, distance) | None`, `compare_row.approach_median(rows, shape, start=None) -> float | None`.
  - Строка в `runs/compare_table.md`.

- [ ] **Step 1: Падающие тесты** (`tests/test_tools.py`; в начало добавить `sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'research'))` и `from compare_row import approach_median, first_stop_56, shares`)

```python
def test_compare_row_helpers():
    rows = [{'bag': 'doubleT_obstacle', 'frame': str(k), 'status': s, 'distance_m': d}
            for k, (s, d) in enumerate([('clear', ''), ('stop', '120.0'), ('stop', '56.2'), ('caution', ''), ('unknown', '')])]
    assert shares(rows) == pytest.approx((40.0, 20.0, 20.0))
    assert first_stop_56(rows) == (2, 56.2)
    ap = [{'shape': 'человек стоит', 'start_m': '170', 'detected': '1', 'first_m': '160'},
          {'shape': 'человек стоит', 'start_m': '170', 'detected': '1', 'first_m': '150'},
          {'shape': 'человек стоит', 'start_m': '90', 'detected': '1', 'first_m': '85'},
          {'shape': 'человек стоит', 'start_m': '170', 'detected': '0', 'first_m': ''}]
    assert approach_median(ap, 'человек стоит', 170) == 155.0
    assert approach_median(ap, 'куб 0.4') is None
```

Run: `venv/bin/python -m pytest -q tests/test_tools.py -k compare_row` → FAIL (`ModuleNotFoundError: compare_row`)

- [ ] **Step 2: `research/compare_row.py`**

```python
"""Строка сводной таблицы вариантов по файлам research/compare.sh (раздел 5.4 спеки):
    python research/compare_row.py <tag> '<json kwargs>'  -> печать + строка в runs/compare_table.md
Колонки: объекты организаторов (N из 7, СТОП на 5/7/8, ложные кадры), % кадров СТОП/ВНИМ./unknown
на 5 пустых записях и на new_data, человек со 170 м и куб 0,4 м (медиана первого СТОП),
первый СТОП на объекте 56 м в doubleT_obstacle (кадр, дистанция)."""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
from bags import EMPTY_BAGS, RUNS            # noqa: E402

HEADER = ('| тег | параметры | объекты организаторов | пустые СТОП/ВНИМ./unk, % | new_data СТОП/ВНИМ./unk, % '
          '| человек 170 м | куб 0,4 м | 56 м: кадр/дист. |\n|---|---|---|---|---|---|---|---|\n')


def _read(path):
    return list(csv.DictReader(open(path, encoding='utf-8'))) if path.exists() else []


def shares(rows):
    n = max(len(rows), 1)
    return tuple(100.0 * sum(r['status'] == s for r in rows) / n for s in ('stop', 'caution', 'unknown'))


def first_stop_56(rows, lo=45.0, hi=65.0):
    for r in rows:
        if r['bag'] == 'doubleT_obstacle' and r['status'] == 'stop' and r['distance_m'] and lo <= float(r['distance_m']) <= hi:
            return int(r['frame']), float(r['distance_m'])
    return None


def approach_median(rows, shape, start=None):
    d = [float(r['first_m']) for r in rows
         if r['shape'] == shape and r['detected'] == '1' and (start is None or int(r['start_m']) == start)]
    return float(np.median(d)) if d else None


def row(tag, det):
    alarms = _read(RUNS / f'alarms_{tag}.csv')
    nd = _read(RUNS / f'alarms_nd_{tag}.csv')
    ap = _read(RUNS / f'approach_{tag}.csv')
    fo = RUNS / f'fake_obj_fo_{tag}.txt'
    fo = fo.read_text(encoding='utf-8').strip().split(': ', 1)[-1] if fo.exists() else '-'
    pct = lambda t: '/'.join(f'{v:.1f}' for v in t)
    f56 = first_stop_56(alarms)
    m = lambda v: '-' if v is None else f'{v:.0f} м'
    return (f'| {tag} | `{det}` | {fo} | {pct(shares([r for r in alarms if r["bag"] in EMPTY_BAGS]))} '
            f'| {pct(shares(nd)) if nd else "-"} | {m(approach_median(ap, "человек стоит", 170))} '
            f'| {m(approach_median(ap, "куб 0.4"))} | {"-" if f56 is None else f"{f56[0]}/{f56[1]:.1f}"} |')


def main():
    tag, det = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else '{}')
    line = row(tag, det)
    table = RUNS / 'compare_table.md'
    if not table.exists():
        table.write_text(HEADER, encoding='utf-8')
    with open(table, 'a', encoding='utf-8') as f:
        f.write(line + '\n')
    print(line)


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: `research/compare.sh`**

```zsh
#!/bin/zsh
# Вариант детектора -> строка таблицы (раздел 5.4 спеки): бэг организаторов, 5 пустых записей +
# doubleT_obstacle, вся new_data, подъезд (eval_approach), саморазметка new_data.
#   research/compare.sh <tag> '<json kwargs>'
#   FAST=1        -- только бэг организаторов (для сетки вариантов зоны, ~несколько минут)
#   ND_SEGMENTS=8 -- new_data отрезками по 400 кадров вместо всей записи (быстрее, для прикидки)
cd "$(dirname $0)/../tools"
TAG=$1; DET=${2:-'{}'}; PY=../venv/bin/python
B=(doubleT_obstacle doubleT_platform roundT_doubleT roundT_pressureGate_roundT roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch)
$PY alarms.py --tag fo_$TAG --det "$DET" --bags cloud_with_fake_obj --workers 1 > ../runs/cmp_${TAG}_fo.log 2>&1 &
if [ -z "$FAST" ]; then
  ND=(); [ -n "$ND_SEGMENTS" ] && ND=(--segments $ND_SEGMENTS --seglen 400)
  $PY alarms.py --tag $TAG --det "$DET" --bags $B --workers 2 > ../runs/cmp_${TAG}_alarms.log 2>&1 &
  $PY alarms.py --tag nd_$TAG --det "$DET" --bags new_data $ND --workers 1 > ../runs/cmp_${TAG}_nd.log 2>&1 &
  $PY eval_approach.py --tag $TAG --det "$DET" --workers 2 > ../runs/cmp_${TAG}_approach.log 2>&1 &
fi
wait
$PY eval_fake_obj.py eval --tag fo_$TAG > ../runs/cmp_${TAG}_fo_eval.log 2>&1
[ -z "$FAST" ] && [ -f selflabel.py ] && $PY selflabel.py --tag nd_$TAG > ../runs/cmp_${TAG}_self.log 2>&1
echo "== $TAG $DET"
cat ../runs/cmp_${TAG}_fo_eval.log
[ -z "$FAST" ] && grep -E "^ПУСТЫЕ|^new_data|^doubleT_obstacle" ../runs/cmp_${TAG}_alarms.log ../runs/cmp_${TAG}_nd.log
[ -z "$FAST" ] && sed -n '/испытаний/,$p' ../runs/cmp_${TAG}_approach.log
[ -z "$FAST" ] && [ -f ../runs/cmp_${TAG}_self.log ] && tail -5 ../runs/cmp_${TAG}_self.log
$PY ../research/compare_row.py $TAG "$DET"
```

- [ ] **Step 4: Тесты**

Run: `venv/bin/python -m pytest -q tests/test_tools.py` → PASS

- [ ] **Step 5: Замер базы этапа 1**

Run (≈30–40 мин, 6 процессов): `research/compare.sh base '{}'`
Expected: строка `| base | ... |` в `runs/compare_table.md`: объекты организаторов, % кадров СТОП/ВНИМ./unknown на пустых и на `new_data`, человек 170 м ≈ 159 м, 56 м в `doubleT_obstacle`. Это база для критериев 8.1. Строку скопировать в `docs/PLAN.md` («Замеры»).

- [ ] **Step 6: Коммит этапа 1** (сообщение содержит строку base)

```bash
git add research/compare.sh research/compare_row.py tests/test_tools.py docs/PLAN.md
git commit -F - <<'EOF'
compare.sh: organizers' bag, full new_data, summary row; baseline measurement

<строка base из runs/compare_table.md>

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd
EOF
```

### Task 10: Узел, маркеры, дашборд и демо — `status`/`level`, повтор кадра, дубли

**Files:**
- Modify: `ros2_ws/src/tunnel_od_detector/tunnel_od_detector/util.py`, `node.py`, `markers.py`, `config/detector.yaml`, `tools/dashboard.py`, `tools/demo.py`, `gui/demo_template.html`
- Test: `ros2_ws/src/tunnel_od_detector/test/test_util.py`

**Interfaces:**
- Consumes: `FrameRepeat`, `dedupe_rounded` (задача 2), поля результата (задача 5).
- Produces:
  - `util.unordered_raw(n) -> bool`, `util.raw_point_count(n_msg, meta, canonical) -> int | None`, `util.status_text(result) -> (str, (r, g, b))`, `util.LEVEL_COLOR: dict`.
  - Параметр узла `skip_repeated` (по умолчанию `true`). В JSON результата — `repeated: true|false`.

- [ ] **Step 1: Падающие тесты** (`test_util.py`, импорт расширить до `from tunnel_od_detector.util import (LEVEL_COLOR, Stats, build_detector_kwargs, canonical_xyz, dumps, is_canonical, raw_point_count, required_fwd_range, status_text, to_jsonable, unordered_raw)`)

```python
def test_unordered_raw_unknown_count():
    assert unordered_raw(300) and not unordered_raw(512)
    assert not unordered_raw(None) and not unordered_raw(0)
    assert raw_point_count(300, None, canonical=False) == 300
    assert raw_point_count(999, None, canonical=True) is None                       # meta не пришла
    assert raw_point_count(999, {'parse': {'n_in': 300}}, canonical=True) == 300


def test_status_text():
    assert status_text({'status': 'stop', 'distance_m': 56.3})[0] == 'СТОП 56 м'
    assert status_text({'status': 'caution', 'caution_distance_m': 80.2})[0] == 'ВНИМАНИЕ 80 м'
    assert status_text({'status': 'clear', 'clear_to_m': 143.4})[0] == 'СВОБОДНО до 143 м'
    assert status_text({'status': 'unknown'})[0] == 'ПУТЬ НЕ ОПРЕДЕЛЁН'
    assert set(LEVEL_COLOR) == {'stop', 'caution', None}
```

Run: `PYTHONPATH=ros2_ws/src/tunnel_od_detector venv/bin/python -m pytest -q ros2_ws/src/tunnel_od_detector/test/test_util.py`
Expected: FAIL (ImportError)

- [ ] **Step 2: `util.py`** (в конец блока функций, до раздела статистики)

```python
# ---------------------------------------------------------------- решение и входное облако

COLUMN_PAIR = 256        # 2 x 128 каналов: облако из столбцов, дубли dual return снимает разбор

LEVEL_COLOR = {'stop': (1.0, 0.1, 0.1, 0.9), 'caution': (1.0, 0.85, 0.1, 0.8), None: (0.6, 0.6, 0.6, 0.5)}


def unordered_raw(n) -> bool:
    """Исходное облако не из столбцов по 128 (синтетика организаторов): дубли снимаются запасным
    способом (tunnel_od.pointcloud.dedupe_rounded). n неизвестно (None) -- не трогаем."""
    return bool(n) and n % COLUMN_PAIR != 0


def raw_point_count(n_msg, meta, canonical):
    """Число точек исходного облака: из сообщения или, за C++-приёмом, из его meta (parse.n_in)."""
    if canonical:
        return ((meta or {}).get('parse') or {}).get('n_in')
    return n_msg


def status_text(result):
    """Строка решения для RViz и логов и её цвет."""
    s = result.get('status')
    if s == 'stop':
        return f"СТОП {float(result['distance_m']):.0f} м", (1.0, 0.2, 0.2)
    if s == 'caution':
        return f"ВНИМАНИЕ {float(result['caution_distance_m']):.0f} м", (1.0, 0.85, 0.1)
    if s == 'unknown':
        return 'ПУТЬ НЕ ОПРЕДЕЛЁН', (0.7, 0.7, 0.7)
    return f"СВОБОДНО до {float(result.get('clear_to_m') or 0.0):.0f} м", (0.3, 1.0, 0.3)
```

- [ ] **Step 3: `node.py`**

Импорт: `from tunnel_od.pointcloud import FrameRepeat, dedupe_rounded`; `from .util import Stats, build_detector_kwargs, canonical_xyz, dumps, percentile, raw_point_count, unordered_raw`.
В `__init__` после `self.parse_backend = ...`: `self.skip_repeated = bool(p('skip_repeated', True))`; после `self._parse = self._make_parser()`: `self._repeat = FrameRepeat(); self._last_out = None`.
В `_make_parser` запоминать вид разбора: `self._parse_kind = 'canonical'` / `'native'` / `'python'` перед соответствующим `return`.
Начало `_process` до вызова `detect` заменить на:
```python
    def _process(self, msg, t_recv):
        t0 = time.monotonic()
        stamp = _stamp_sec(msg.header)
        with self._lock:
            meta = self._meta.pop((msg.header.stamp.sec, msg.header.stamp.nanosec), None)
        if self.skip_repeated and self._repeat.check(msg.data) and self._last_out is not None:
            # побитовый повтор облака (синтетика организаторов): детектор не вызывается
            out = dict(self._last_out, frame=self._frame, stamp=stamp, repeated=True,
                       queue_ms=(t0 - t_recv) * 1e3, latency_ms=(time.monotonic() - t_recv) * 1e3)
            text = dumps(out)
            self.pub_result.publish(String(data=text))
            with self._lock:
                self.stats.on_processed(out['latency_ms'], 0.0, out.get('obstacle'), out.get('distance_m'))
            self._frame += 1
            if self._result_fh:
                self._result_fh.write(text + '\n')
            return
        x, y, z = self._parse(msg)
        if self._parse_kind != 'python' and unordered_raw(
                raw_point_count(msg.width * msg.height, meta, self.canonical_input)):
            x, y, z = dedupe_rounded(x, y, z)       # C++ снимает дубли только в облаке из столбцов
        t1 = time.monotonic()
```
Дальше `stamp = _stamp_sec(...)` в старом месте удалить. В блоке `with self._lock: out['dropped_total'] = ...; meta = self._meta.pop(...)` строку с `meta = ...` удалить (meta уже взята). В `out.update({...})` добавить `'repeated': False`. После `text = dumps(out)` — `self._last_out = out`.
Лог тревоги:
```python
        if self.log_alarms and res.get('obstacle'):
            n_stop = sum(1 for o in res.get('objects') or [] if o.get('level') == 'stop')
            pr = res.get('path_range_m')
            self.get_logger().warning(
                f'СТОП кадр {self._frame - 1}: препятствие {float(res["distance_m"]):.1f} м, '
                f'объектов stop {n_stop}, ось до {pr if pr is None else round(pr)} м, '
                f'свободно до {res.get("clear_to_m", 0):.0f} м, задержка {out["latency_ms"]:.0f} мс')
```
В docstring модуля в раздел «Выход» дописать: `status` (stop | caution | unknown | clear), `level`/`reason` объектов, `sight_m`, `clear_to_m`, `caution_distance_m`; повтор кадра — прошлый результат с `repeated: true`.

- [ ] **Step 4: `markers.py`**

Импорт: `from .util import LEVEL_COLOR, status_text`. Блок объектов:
```python
    # объекты: цвет по уровню -- stop красный, caution жёлтый, прочие серые
    rank = {'stop': 0, 'caution': 1}
    shown = sorted(result.get('objects') or [], key=lambda o: (rank.get(o.get('level'), 2), o.get('distance_m', 1e9)))
    for i, o in enumerate(shown[:max_objects]):
        d = float(o.get('distance_m', 0.0))
        lat = float(o.get('lateral_m', 0.0) or 0.0)
        h = max(float(o.get('height_m', 0.3) or 0.3), 0.2)
        base = float(track.rail_top_at(d)[0])
        x = float(track.center_at(d)) + lat
        box = _marker(header, 100 + i, Marker.CUBE, _color(*LEVEL_COLOR.get(o.get('level'), LEVEL_COLOR[None])),
                      (0.6, 0.6, h))
        box.pose.position = _pt(x, -(d + 0.3), base + h / 2)
        arr.markers.append(box)
```
Текст решения:
```python
    text, rgb = status_text(result)
    if path_range:
        text += f' (ось {float(path_range):.0f} м)'
    label = _marker(header, 2, Marker.TEXT_VIEW_FACING, _color(*rgb), (0, 0, 1.0))
```
(старые ветки `if result.get('obstacle')` / `extra` удалить).

- [ ] **Step 5: `config/detector.yaml`** — под `tunnel_od_detector.ros__parameters` после `log_alarms`:
```yaml
    skip_repeated: true      # побитовый повтор облака -> прошлый результат с repeated: true, детектор не вызывается
```

- [ ] **Step 6: Дашборд и демо**

`tools/dashboard.py` (`frame_view`):
```python
    for o in res['objects']:
        if o.get('level') is None and o['too_small']:
            continue
        ...
        alarm = o.get('level') == 'stop'
        ...
            objects.append({..., 'alarm': alarm, 'level': o.get('level'), 'reason': o.get('reason'), ...})
```
В `scene` в словарь кадра добавить `'status': res['status'], 'clear_to_m': res['clear_to_m'], 'caution_distance_m': res['caution_distance_m']`.

`tools/demo.py`: в обоих `frames.append({...})` добавить те же три ключа.

`gui/demo_template.html`:
- CSS после `.status.nopath`: `.status.caution { color: var(--cand); }`
- в `hud` ветку `else` заменить на:
```js
  } else {
    const cand = fr.objects.filter(x => !x.alarm).length;
    const st = fr.status || (fr.path_range_m ? 'clear' : 'unknown');
    const head = st === 'caution' ? `<span class="status caution">Внимание: объект ${num(fr.caution_distance_m, 0)} м</span>`
      : st === 'unknown' ? `<span class="status nopath">Путь не определён</span>`
      : `<span class="status clear">Путь свободен${fr.clear_to_m != null ? ` до ${num(fr.clear_to_m, 0)} м` : ''}</span>`;
    $('#hud').innerHTML = `${head}
      <div class="big dim">—<small>м</small></div>
      <p class="what">${cand ? `${cand} ${plural(cand, 'кандидат', 'кандидата', 'кандидатов')} копят подтверждение` : 'подтверждённых объектов нет'}</p>
      <dl>${common}</dl>`;
  }
```
- в `common` после строки «ось пути» добавить: ``${fr.clear_to_m != null ? `<dt>проверено</dt><dd>свободно до ${num(fr.clear_to_m, 0)} м</dd>` : ''}``

- [ ] **Step 7: Тесты и сборка демо**

```bash
PYTHONPATH=ros2_ws/src/tunnel_od_detector venv/bin/python -m pytest -q ros2_ws/src/tunnel_od_detector/test/test_util.py
venv/bin/python -m pytest -q tests
venv/bin/python tools/demo.py --clip doubleT_obstacle:90:30
```
Expected: всё PASS; демо собирается. Открыть HTML, посмотреть строку решения на кадре без тревоги и с тревогой.

- [ ] **Step 8: Коммит**

```bash
git add ros2_ws/src/tunnel_od_detector config/detector.yaml tools/dashboard.py tools/demo.py gui/demo_template.html
git commit -m "Node: status/level markers and text, repeated frames, fallback dedupe after native/C++ parse" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

---

## Этап 2 (28.09 до 14:00): зона по габариту организаторов

### Task 11: Приёмка варианта — общий порядок (используется в задачах 12–14)

Кода нет. Порядок проверки любого варианта `<tag>` против `base` по строкам `runs/compare_table.md` и `runs/fake_obj_fo_<tag>.csv`:

- [ ] 8.1.1: объекты, дававшие СТОП в `fake_obj_fo_base.csv`, дают СТОП и в `<tag>`; `first_stop_m` каждого ≥ база − 10 м; у 5, 7, 8 `level != stop`.
- [ ] 8.1.2: СТОП % на пустых и на `new_data` ≤ базы (для зоны — ≤ база + 0,5 п.п., 3.2.4).
- [ ] 8.1.3: ВНИМ. % ≤ база + 5 п.п. на тех же наборах.
- [ ] 8.1.4: человек 170 м ≥ 150 м; куб 0,4 м ≥ база − 10 м.
- [ ] 8.1.5: 56 м в `doubleT_obstacle`: кадр первого СТОП ≤ базы.
- [ ] 8.1.6: `venv/bin/python -m pytest -q tests` — PASS (e2e в Docker — в задаче 17, на итоговых параметрах).
- [ ] Результат (принят / отклонён, по какому пункту) записать строкой в `docs/PLAN.md`, таблица «Варианты этапов 2–3».

### Task 12: Сетка зоны и выбор по 3.2

**Files:**
- Modify: `core/tunnel_od/detection/detector.py` (значения по умолчанию), `config/detector.yaml`, `docs/PLAN.md`

Параметры: `half_width` (полуширина прямоугольной зоны), `height` (это `h_top` спеки), `far_top`.

- [ ] **Step 1: Ширина на бэге организаторов (быстро)**

```bash
FAST=1 research/compare.sh w115 '{"half_width": 1.15}'
```
Сравнить `runs/fake_obj_fo_w115.csv` с `fake_obj_fo_base.csv`: получили ли СТОП объекты 4 и 6, не получили ли 5, 7, 8 (3.2.1).

- [ ] **Step 2: Высота при выбранной ширине W** (W = 1.15, если шаг 1 прошёл 3.2.1 и добавил объекты, иначе 1.0). Пять вариантов, быстрый режим:

```bash
W=1.15      # или 1.0 -- по итогу шага 1
for h in 2.0 2.5 3.0; do for ft in 1.5 $h; do
  [ "$h" = 2.0 ] && [ "$ft" = 1.5 ] && continue          # уже измерено: base или w115
  FAST=1 research/compare.sh "z_w${W}_h${h}_f${ft}" "{\"half_width\": $W, \"height\": $h, \"far_top\": $ft}"
done; done
```
Отобрать варианты, которые по 3.2.1–3.2.2 лучше базы по объектам (не СТОП на 5/7/8, больше объектов из 1, 2, 3, 4, 6, 9, 10 со СТОП). Варианты, которые ответ по объектам не меняют, дальше не гонять.

- [ ] **Step 3: Полный прогон отобранных** (по одному, ≈30–40 мин каждый):
```bash
research/compare.sh <tag> '<json>'
```

- [ ] **Step 4: Выбор по 3.2.** Среди прошедших 3.2.1 — больше объектов со СТОП; при равенстве — меньше СТОП % на пустых и `new_data`. Проверить 3.2.4 (рост СТОП ≤ +0,5 п.п. на каждом наборе) и задачу 11. Если ни один вариант не прошёл, остаёмся на `half_width=1.0, height=2.0, far_top=1.5`, а расхождение с габаритом организаторов (±1,15 м, номера объектов) идёт в README (задача 18).

- [ ] **Step 5: Применить выбранное** в значениях по умолчанию `ObstacleDetector.__init__` (`half_width=…, height=…, far_top=…`) и в `config/detector.yaml` (`half_width`, `height`, `far_top` с комментарием «подобрано по cloud_with_fake_obj, см. docs/PLAN.md»). Проверить: `venv/bin/python -m pytest -q tests` → PASS.

- [ ] **Step 6: Объекты ниже полотна (3.3).** По `fake_obj_fo_<выбранный>.csv` и эталону: для пропущенных объектов с `low_m < 0.15` или `note: ниже зоны` записать в `docs/PLAN.md` номер объекта, `low_m` и `rail_z_m` (для README, «Известные ограничения»).

- [ ] **Step 7: Коммит этапа 2**

```bash
git add core/tunnel_od/detection/detector.py config/detector.yaml docs/PLAN.md
git commit -F - <<'EOF'
Zone tuned on organizers' bag: <half_width/height/far_top или "без изменений">

<строка выбранного варианта из runs/compare_table.md>

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd
EOF
```

### Task 13 (при необходимости, 5.1.6): промежуток больше окна поиска → `displacement = None`

Делается, если задача 8, шаг 2 показала, что пробег на бэге организаторов неверен. После неё эталон `objects.yaml` строится заново (задача 8, шаги 1–5), а база перемеряется (`research/compare.sh base2 '{}'`).

**Files:**
- Modify: `core/tunnel_od/geometry/ego_motion.py`
- Test: `tests/test_core.py`

- [ ] **Step 1: Падающий тест** (импорт `from tunnel_od.geometry.ego_motion import EgoMotion`)

```python
def test_ego_motion_gap_beyond_window_gives_no_displacement():
    em = EgoMotion()
    rng = np.random.default_rng(0)
    fwd, lat, z = rng.uniform(5, 60, 5000), rng.uniform(-2, 2, 5000), rng.uniform(0, 3, 5000)
    img = em._image(fwd, lat, z)
    em.speed = 20.0
    for k in range(em.lag):
        em._hist.append((img, 0.1 * k))
    em._last_stamp = 0.1 * (em.lag - 1)
    speed, disp = em.update(fwd, lat, z, stamp=em._last_stamp + 3.0)   # 3 с без кадров: ~68 м > окна 22,5 м
    assert disp is None and speed == 20.0
```
Run: `venv/bin/python -m pytest -q tests/test_core.py -k gap_beyond_window` → FAIL (`disp` — число)

- [ ] **Step 2: Реализация** — в `EgoMotion.update` ветку `else:` при известной скорости заменить на:
```python
            else:
                c = int(round(self.speed * dt / S_BIN))
                w = int(SEARCH_HALF / S_BIN)
                if c - w > int(self._max_shift / S_BIN):
                    # промежуток между кадрами больше окна поиска: сдвиг не измерить -- не угадываем
                    self._hist.append((img, t))
                    return self.speed, None
                lo, hi = max(-int(1.0 / S_BIN), c - w), min(int(self._max_shift / S_BIN), c + w)
```
(прежние строки `if lo > hi: ...` удалить). В docstring модуля: «Если ожидаемый сдвиг больше окна поиска (пропуск кадров), смещение за кадр -- None: трекер расширяет ворота».

- [ ] **Step 3: Тесты** — `venv/bin/python -m pytest -q tests` → PASS

- [ ] **Step 4: Перестроить эталон и базу**, прогнать `research/compare.sh base2 '{}'`, принять по задаче 11 (новая база = `base2`, если 8.1 не хуже `base`). Коммит:
```bash
git add core/tunnel_od/geometry/ego_motion.py tests/test_core.py reference/cloud_with_fake_obj/objects.yaml docs/PLAN.md
git commit -m "EgoMotion: frame gap beyond search window -> displacement None" -m "<строка base2>" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

---

## Этап 3 (28.09 до 21:00): ЭГО-тест и саморазметка. Первым выкидывается при нехватке времени

### Task 14: ЭГО-тест в `EvidenceTracker`

**Files:**
- Modify: `core/tunnel_od/detection/tracking.py`, `core/tunnel_od/detection/detector.py`, `config/detector.yaml`
- Test: `tests/test_core.py`

**Interfaces:**
- Consumes: `object_level` читает `obj['ego_carried']` (задача 4).
- Produces:
  - `EvidenceTracker(..., ego_check=False, ego_min_travel=4.0, ego_max_slope=-0.35)`; `update(objects, displacement=None, expected=None, travel=None)`; `Tracker.update(..., travel=None)` (аргумент игнорируется).
  - `theil_sen_slope(obs, min_dt=0.5, min_pairs=3) -> float | None`.
  - В объекте: `ego_slope` (float или `None`), `ego_carried` (bool; `True` только при `ego_check`).
  - `ObstacleDetector(ego_check=False, ego_min_travel=4.0, ego_max_slope=-0.35)`.

- [ ] **Step 1: Падающие тесты**

```python
def _ego_run(disp, dist, n=8, travel_none=False, **kw):
    tr = EvidenceTracker(ego_check=True, **kw)
    travel, ob = 0.0, None
    for k in range(n):
        ob = {'distance_m': dist(k), 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=disp, expected=lambda d: 5.0, travel=None if travel_none else travel)
        travel += disp
    return ob


def test_ego_constant_distance_is_carried():
    ob = _ego_run(1.2, lambda k: 30.0)
    assert ob['ego_carried'] and ob['ego_slope'] == pytest.approx(0.0, abs=1e-6)


def test_ego_approaching_object_is_not_carried():
    ob = _ego_run(1.2, lambda k: 60.0 - 1.2 * k)
    assert not ob['ego_carried'] and ob['ego_slope'] == pytest.approx(-1.0, abs=1e-6)


def test_ego_no_decision_when_train_stands_or_travel_unknown():
    ob = _ego_run(0.0, lambda k: 30.0)
    assert ob['ego_slope'] is None and not ob['ego_carried']
    ob = _ego_run(1.2, lambda k: 30.0, travel_none=True)
    assert ob['ego_slope'] is None and not ob['ego_carried']


def test_ego_short_travel_no_decision():
    ob = _ego_run(1.2, lambda k: 30.0, n=4)          # пробег 3,6 м < 4 м
    assert ob['ego_slope'] is None and not ob['ego_carried']


def test_ego_check_off_only_measures():
    tr = EvidenceTracker(ego_check=False)
    travel = 0.0
    for k in range(8):
        ob = {'distance_m': 30.0, 'lateral_m': 0.0, 'n_points': 20}
        tr.update([ob], displacement=1.2, expected=lambda d: 5.0, travel=travel)
        travel += 1.2
    assert ob['ego_slope'] == pytest.approx(0.0, abs=1e-6) and not ob['ego_carried']


def test_detector_passes_ego_params():
    det = ObstacleDetector(ego_check=True, ego_min_travel=5.0, ego_max_slope=-0.5)
    assert det._tracker.ego_check and det._tracker.ego_min_travel == 5.0 and det._tracker.ego_max_slope == -0.5
```
Run: `venv/bin/python -m pytest -q tests/test_core.py -k ego` → FAIL (`TypeError: unexpected keyword 'ego_check'`)

- [ ] **Step 2: `tracking.py`**

В docstring модуля абзац:
```
ЭГО-тест: трек хранит пары (пробег поезда, дистанция) последних EGO_OBS сопоставлений. Неподвижный
в мире объект приближается на пройденный путь -- наклон d(дистанция)/d(пробег) = -1; артефакт,
который едет вместе с поездом, держит дистанцию -- наклон ~0. Наклон -- медиана по парам
(Тейл--Сен), решение -- только при пробеге >= ego_min_travel.
```
Код:
```python
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
```
`Tracker.update(self, objects, displacement=None, expected=None, travel=None)`.
`EvidenceTracker.__init__` — добавить параметры `ego_check=False, ego_min_travel=4.0, ego_max_slope=-0.35` и присвоить в атрибуты. `update(self, objects, displacement=None, expected=None, travel=None)`; в docstring: «travel -- накопленный пробег поезда (None -- неизвестен или смещение кадра неизвестно: ЭГО-тест не копит пары)». После `for k in ('height_m', 'low_m'): ...`:
```python
            if travel is not None:
                tr.setdefault('obs', []).append((travel, d))
                del tr['obs'][:-EGO_OBS]
            obs = tr.get('obs', [])
            slope = None
            if len(obs) >= 4 and max(p[0] for p in obs) - min(p[0] for p in obs) >= self.ego_min_travel:
                slope = theil_sen_slope(obs)
            ob['ego_slope'] = None if slope is None else round(slope, 3)
            ob['ego_carried'] = bool(self.ego_check and slope is not None and slope > self.ego_max_slope)
```

- [ ] **Step 3: `detector.py`**

В `__init__` параметры `ego_check=False, ego_min_travel=4.0, ego_max_slope=-0.35` (после `path_hold=0`) и передача в `EvidenceTracker(..., ego_check=ego_check, ego_min_travel=ego_min_travel, ego_max_slope=ego_max_slope)`. Вызов трекера в `check_frame`:
```python
        self._tracker.update([o for o in objects if not o.get('edge_line')], self._displacement,
                             lambda d: min_points_at(d, self.min_points_k, self.min_points_floor),
                             travel=self._travel if self._displacement is not None else None)
```
В словарь удержанного объекта: `'ego_slope': None, 'ego_carried': False`.

`config/detector.yaml` после `path_hold`:
```yaml
      ego_check: false       # ЭГО-тест: подтверждённый объект, держащий дистанцию при движении поезда
      ego_min_travel: 4.0    #    (наклон дистанции по пробегу > ego_max_slope), -- ВНИМАНИЕ, не СТОП.
      ego_max_slope: -0.35   #    Включается только по итогам cloud_with_fake_obj (спека 4.4)
```

- [ ] **Step 4: Тесты** — `venv/bin/python -m pytest -q tests` → PASS

- [ ] **Step 5: Замер и решение по 4.4**

```bash
research/compare.sh ego '{"ego_check": true}'
```
(к выбранной в задаче 12 зоне добавляется только `ego_check`: её значения уже в умолчаниях). `ego_check = true` в `config/detector.yaml` и в умолчаниях включается, только если: на бэге организаторов ни один объект не потерял СТОП, `first_stop_m` каждого не хуже −10 м (сравнивать `fake_obj_fo_ego.csv` с последним принятым вариантом) и выполнена задача 11. Иначе флаг остаётся `false`, а в `docs/PLAN.md` пишутся цифры и причина (какой объект понижен до `ego_carried`, его `ego_slope` из `alarms_fo_ego_objects.jsonl`).

- [ ] **Step 6: Коммит**

```bash
git add core/tunnel_od/detection/tracking.py core/tunnel_od/detection/detector.py config/detector.yaml tests/test_core.py docs/PLAN.md
git commit -m "EGO test (Theil-Sen slope of distance vs travel) in EvidenceTracker; <включён|выключен>" -m "<строка ego>" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 15: `selflabel.py` — саморазметка `new_data` проездом

**Files:**
- Create: `tools/selflabel.py`
- Test: `tests/test_tools.py`

**Interfaces:**
- Consumes: `runs/alarms_<tag>.csv` (`bag, frame, t_s, travel_m, displacement_m`), `runs/alarms_<tag>_objects.jsonl` (`track_id`, `distance_m`, `level`, `reason`).
- Produces:
  - `segments(rows) -> list[int]` — номер сегмента одометрии на строку.
  - `episodes(stops, gap=5) -> list[dict]` — `stops`: список `(row_index, frame, track_id, distance_m, reason)`; эпизод: `{track_id, rows: [...], first_row, first_frame, last_frame, distance_m, reason}`.
  - `label(ep, rows, seg) -> 'false_proven' | 'unresolved'`.
  - CLI: `selflabel.py --tag nd_<T>` → `runs/selflabel_nd_<T>.csv`, `runs/selflabel_nd_<T>.txt`, сводка в stdout.

- [ ] **Step 1: Падающие тесты** (`from selflabel import episodes, label, segments`)

```python
def _rows(n, dt=0.1, disp=1.0, none_at=()):
    rows, travel = [], 0.0
    for k in range(n):
        d = None if k in none_at else disp
        travel += d or 0.0
        rows.append({'bag': 'new_data', 'frame': k, 't_s': k * dt, 'travel_m': travel, 'displacement_m': d})
    return rows


def test_segments_break_on_long_odometry_gap_and_restart():
    assert len(set(segments(_rows(30, none_at=range(10, 15))))) == 1          # 0,5 с без смещения
    seg = segments(_rows(40, none_at=range(10, 26)))                          # 1,6 с без смещения
    assert seg[9] == seg[25] == 0 and seg[26] == 1
    rows = _rows(10) + [dict(r, frame=100 + r['frame']) for r in _rows(10)]   # отрезок alarms.py --segments
    assert segments(rows)[10] == 1


def test_episodes_gap_rule():
    st = lambda frames: [(f, f, 7, 50.0, 'in_gauge') for f in frames]
    assert len(episodes(st([0, 1, 2, 8, 9]))) == 1        # пропущено 5 кадров (3..7) -- эпизод не рвётся
    assert len(episodes(st([0, 1, 2, 9, 10]))) == 2       # пропущено 6
    two = episodes(st([0, 1]) + [(f, f, 8, 30.0, 'in_gauge') for f in (0, 1)])
    assert len(two) == 2                                   # разные треки -- разные эпизоды


def test_label_false_proven_only_in_same_segment():
    rows = _rows(200)                                      # 1 м за кадр
    seg = segments(rows)
    ep = episodes([(10, 10, 1, 50.0, 'in_gauge')])[0]     # X = 11 + 50 = 61; нужно travel >= 61 + 3 + 1 = 65
    assert label(ep, rows, seg) == 'false_proven'
    assert label(ep, rows[:60], seg[:60]) == 'unresolved'
    rows2 = _rows(200, none_at=range(20, 40))              # сегмент рвётся до проезда X
    assert label(episodes([(10, 10, 1, 50.0, 'in_gauge')])[0], rows2, segments(rows2)) == 'unresolved'
```
Run: `venv/bin/python -m pytest -q tests/test_tools.py -k "segments or episodes or label"` → FAIL (`ModuleNotFoundError: selflabel`)

- [ ] **Step 2: Реализация `tools/selflabel.py`**

```python
"""Саморазметка эпизодов СТОП на new_data проездом: если поезд потом проехал место, где стоял
«объект», и не остановился, тревога доказанно ложная. В подбор параметров не входит -- только
утверждение для README: «N из M эпизодов доказанно ложные, остальные не разрешены».

    python tools/alarms.py --tag nd_P --bags new_data --workers 1
    python tools/selflabel.py --tag nd_P

Эпизод -- кадры СТОП одного track_id, разрыв до GAP кадров не рвёт. X = travel_m + distance_m на
первом кадре. Сегмент одометрии рвётся, если displacement_m был None дольше 1 с, пробег убыл или
кадры идут не подряд (отрезки alarms.py --segments). Метка false_proven -- в том же сегменте
позже travel_m >= X + 3 + 0,02 * distance_m; иначе unresolved.
"""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import RUNS          # noqa: E402

GAP = 5
NONE_SEG_S = 1.0
MARGIN_M, MARGIN_REL = 3.0, 0.02
ND_KM = (10.0, 16.0)           # ориентир TunnelGuard -- 13,0 км за проезд


def segments(rows):
    seg, out, none_t0, prev = 0, [], None, None
    for r in rows:
        if prev is not None and (r['bag'] != prev['bag'] or r['frame'] != prev['frame'] + 1
                                 or r['travel_m'] < prev['travel_m'] - 1e-6):
            seg, none_t0 = seg + 1, None
        if r['displacement_m'] is None:
            none_t0 = r['t_s'] if none_t0 is None else none_t0
        else:
            if none_t0 is not None and r['t_s'] - none_t0 > NONE_SEG_S:
                seg += 1
            none_t0 = None
        out.append(seg)
        prev = r
    return out


def episodes(stops, gap=GAP):
    by_track = {}
    for s in sorted(stops, key=lambda s: (s[2], s[1])):
        by_track.setdefault(s[2], []).append(s)
    eps = []
    for tid, ss in by_track.items():
        cur = [ss[0]]
        for s in ss[1:]:
            if s[1] - cur[-1][1] - 1 <= gap:
                cur.append(s)
            else:
                eps.append(cur)
                cur = [s]
        eps.append(cur)
    return [{'track_id': e[0][2], 'first_row': e[0][0], 'first_frame': e[0][1], 'last_frame': e[-1][1],
             'frames': len(e), 'distance_m': e[0][3], 'reason': Counter(s[4] for s in e).most_common(1)[0][0]}
            for e in sorted(eps, key=lambda e: e[0][0])]


def label(ep, rows, seg):
    i = ep['first_row']
    x = rows[i]['travel_m'] + ep['distance_m']
    need = x + MARGIN_M + MARGIN_REL * ep['distance_m']
    for j in range(i + 1, len(rows)):
        if seg[j] != seg[i]:
            break
        if rows[j]['travel_m'] >= need:
            return 'false_proven'
    return 'unresolved'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tag', required=True)
    args = ap.parse_args()
    num = lambda v: None if v == '' else float(v)
    rows = [{'bag': r['bag'], 'frame': int(r['frame']), 't_s': float(r['t_s']), 'travel_m': float(r['travel_m']),
             'displacement_m': num(r['displacement_m'])}
            for r in csv.DictReader(open(RUNS / f'alarms_{args.tag}.csv', encoding='utf-8'))]
    index = {(r['bag'], r['frame']): k for k, r in enumerate(rows)}
    stops = []
    with open(RUNS / f'alarms_{args.tag}_objects.jsonl', encoding='utf-8') as f:
        for line in f:
            o = json.loads(line)
            k = index[(o['bag'], o['frame'])]
            stops += [(k, o['frame'], ob['track_id'], ob['distance_m'], ob.get('reason'))
                      for ob in o['objects'] if ob.get('level') == 'stop' and 'track_id' in ob]
    seg = segments(rows)
    eps = episodes(stops)
    for e in eps:
        e['label'] = label(e, rows, seg)
        e['segment'] = seg[e['first_row']]
        e['x_m'] = round(rows[e['first_row']]['travel_m'] + e['distance_m'], 1)
    with open(RUNS / f'selflabel_{args.tag}.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['track_id', 'first_frame', 'last_frame', 'frames', 'distance_m', 'x_m',
                                          'reason', 'segment', 'label', 'first_row'])
        w.writeheader()
        w.writerows(eps)
    km = sum(max(rows[k]['travel_m'] for k in ks) - min(rows[k]['travel_m'] for k in ks)
             for s in set(seg) for ks in [[k for k, v in enumerate(seg) if v == s]]) / 1000
    reliable = not (len(rows) >= 11000 and not ND_KM[0] <= km <= ND_KM[1])
    c = Counter(e['label'] for e in eps)
    d = np.array([e['distance_m'] for e in eps]) if eps else np.array([])
    bins = [(0, 40), (40, 80), (80, 130), (130, 1e9)]
    lines = [f'{args.tag}: эпизодов СТОП {len(eps)}, доказанно ложных {c["false_proven"]}, не разрешено {c["unresolved"]}'
             + ('' if reliable else f' -- НЕНАДЁЖНО: пробег {km:.1f} км вне {ND_KM[0]:.0f}-{ND_KM[1]:.0f} км'),
             f'пробег по одометрии {km:.1f} км, сегментов {len(set(seg))}',
             'по дистанции: ' + ', '.join(f'{lo:.0f}-{"" if hi > 1e8 else f"{hi:.0f}"} м: {int(((d >= lo) & (d < hi)).sum())}'
                                          for lo, hi in bins),
             'по причине: ' + ', '.join(f'{k}: {v}' for k, v in Counter(e['reason'] for e in eps).items())]
    (RUNS / f'selflabel_{args.tag}.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Тесты** — `venv/bin/python -m pytest -q tests/test_tools.py` → PASS

- [ ] **Step 4: Прогон** на последнем принятом варианте `<T>` (его `alarms_nd_<T>.csv` по всей `new_data` уже есть):
```bash
venv/bin/python tools/selflabel.py --tag nd_<T>
```
Сводку записать в `docs/PLAN.md` (для README).

- [ ] **Step 5: Коммит этапа 3**

```bash
git add tools/selflabel.py tests/test_tools.py docs/PLAN.md
git commit -m "selflabel.py: STOP episodes on new_data proven false by driving through" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 16: Разовые проверки 8.3 и заморозка (28.09 21:00)

- [ ] **Step 1: Детерминизм.** Два прогона итогового варианта по 5 пустым записям:
```bash
cd tools
for r in 1 2; do ../venv/bin/python alarms.py --tag det$r --bags doubleT_platform roundT_doubleT roundT_pressureGate_roundT \
  roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch --workers 2 > /dev/null; done
cd .. && cmp runs/alarms_det1.csv runs/alarms_det2.csv && echo ОДИНАКОВО
```
Если различаются: `diff <(cut -d, -f1-6 runs/alarms_det1.csv) <(cut -d, -f1-6 runs/alarms_det2.csv) | grep -c '^<'` — число затронутых кадров записать в `docs/PLAN.md` (для README).

- [ ] **Step 2: Что стоит на 56 м в `doubleT_obstacle`.** Кадры первого СТОП на объекте (из `compare_row`, колонка «56 м») и +50, +100:
```bash
for f in <кадр> <кадр+50> <кадр+100>; do venv/bin/python research/show_frame.py doubleT_obstacle $f '{}' runs/img/dt56_$f.png; done
```
Вывод (человек A или неподвижный предмет, сколько кадров он в габарите) — в `docs/PLAN.md` для README.

- [ ] **Step 3: Заморозка.** Итоговая строка `runs/compare_table.md` → `docs/PLAN.md`. Тег `git tag freeze-2809` на коммите с итоговыми параметрами. После этого алгоритм не меняется.

---

## Этап 4 (29.09 до 20:00): сдача. Не сокращается

### Task 17: Образ без интернета и e2e

**Files:**
- Modify: `run.sh`

- [ ] **Step 1: `run.sh` — проверка без сети.** В блок переменных:
```bash
#   NETWORK        сеть контейнера для play/shell, например NETWORK=none -- проверка без интернета
```
и после `COMMON=(...)`:
```bash
NET=(); [ -n "${NETWORK:-}" ] && NET=(--network "$NETWORK")
```
В `play` и `shell` вставить `"${NET[@]}"` после `"${COMMON[@]}"`. Если `--sysctl net.core.*` несовместим с `--network none`, при `NETWORK=none` не передавать `NET_SYSCTL`: `[ -n "${NETWORK:-}" ] && NET_SYSCTL=()`.

- [ ] **Step 2: Сборка под x86-64 и сохранение**
```bash
./run.sh build --platform linux/amd64
docker image inspect tunnel-od --format '{{.Architecture}}'      # amd64
docker save tunnel-od | gzip -1 > tunnel_od_image.tar.gz && ls -lh tunnel_od_image.tar.gz
```
Образ собирается на Mac: без `--platform linux/amd64` он будет arm64, и на сервере проверки не запустится.

- [ ] **Step 3: e2e без сети из загруженного образа**
```bash
docker rmi tunnel-od && docker load -i tunnel_od_image.tar.gz
./run.sh test
NETWORK=none ./run.sh play data/Датасет/archive/for_hackathon/doubleT_obstacle
NETWORK=none ./run.sh play data/Датасет/archive/for_hackathon/doubleT_platform
NETWORK=none ./run.sh play data/cloud_with_fake_obj
```
Expected (8.1.6): в `runs/docker/<запись>_stats.json` обработано 100 % принятых кадров на 8-МБ записи (`doubleT_platform`), задержка p95 < 100 мс; на `cloud_with_fake_obj` узел не падает, в `_result.jsonl` есть `status`, `level`, `repeated`. Если C++-приём не принимает 16-байтное облако (задача 3, шаг 7), в `docker/play.sh` и `detector.launch.py` сделать запасной путь по умолчанию и повторить. Цифры → README.

- [ ] **Step 4: Размещение образа.** Если `tunnel_od_image.tar.gz` больше лимита платформы — Яндекс Диск, ссылка в README и на платформе. Коммит `run.sh`:
```bash
git add run.sh && git commit -m "run.sh: NETWORK=none for offline e2e check" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 18: README, `docs/PLAN.md`, `docs/COMPETITOR_TUNNELGUARD.md`

**Files:**
- Modify: `README.md`, `docs/PLAN.md`, `docs/COMPETITOR_TUNNELGUARD.md`

- [ ] **Step 1: README «Запуск»** — `docker load -i tunnel_od_image.tar.gz` как первый способ, `./run.sh build` как второй, ответ постановщиков про сборку (задача 1); `./run.sh play <запись>`; топики `/tunnel_od/result`, `/tunnel_od/markers`; таблица значений `status` (stop / caution / unknown / clear) и что значит `clear_to_m`.
- [ ] **Step 2: README «Обработка кадра», «Шаги алгоритма», параметры** — шаг «решение» (`decision.py`), таблица уровней 2.1, приоритет статусов, `sight_m`; таблица параметров с `ego_*`, итоговыми `half_width/height/far_top`, `skip_repeated`; поля выхода `status`, `level`, `reason`, `sight_m`, `clear_to_m`, `caution_distance_m`, `ego_slope`, `repeated`.
- [ ] **Step 3: README «Качество»** — только из `runs/compare_table.md` и отчётов:
  - таблица «база → итог» (строки `base` и итоговая);
  - таблица 10 объектов организаторов из `runs/fake_obj_fo_<итог>.csv`: номер, описание, ожидаемый уровень, первый СТОП, `sight_m`, уровень, `reason`;
  - фраза «первый СТОП на X м при видимости Y м» по каждому объекту в габарите;
  - доля `unknown` на `new_data` и где она возникает (записи и участки по `alarms_nd_<итог>.csv`: подряд идущие кадры `unknown` с `t_s`);
  - саморазметка `new_data` (задача 15);
  - детерминизм и 56 м (задача 16).
- [ ] **Step 4: README «Известные ограничения»** — предметы ниже 0,15 м над головкой рельса, в том числе 300×300×100 мм плашмя (верх 0,1 м < 0,15 м); объекты генератора ниже полотна с номерами (задача 12, шаг 6); поезд впереди с той же скоростью / человек, уходящий от поезда, → ВНИМАНИЕ (4.5, если ЭГО-тест включён); стекло (видим только отражения); ширина зоны, если она расходится с ±1,15 м организаторов.
- [ ] **Step 5: `docs/PLAN.md`** — статус этапов 0–4, таблица вариантов (принят/отклонён, цифры, причина); `docs/COMPETITOR_TUNNELGUARD.md`, раздел 7 — наши цифры на `cloud_with_fake_obj` рядом с их «9 из 10».
- [ ] **Step 6: Проверка.** Все цифры README есть в `runs/compare_table.md` или отчётах (сверить построчно). Коммит:
```bash
git add README.md docs/PLAN.md docs/COMPETITOR_TUNNELGUARD.md
git commit -m "README: run, decision layer, quality on organizers' bag, limitations" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01D6bsUAz934k5h219yEwubd"
```

### Task 19: Видео, презентация, загрузка (ручные шаги)

- [ ] **Step 1: Видео 2–3 мин с русскими подписями (6.4).** Сценарий: `docker run` → `ros2 bag play` → облако в RViz → объект организаторов краснеет «СТОП X м» → `doubleT_obstacle` → строка решения со `status` и `clear_to_m`. RViz: `./run.sh detector` + плеер, или запись экрана `rviz2` в контейнере с X11. Подписи — цифры из README.
- [ ] **Step 2: Презентация (6.5).** Шаблон ЛЦТ, слайды 7–11 без изменения структуры. Проблема → идея («всё в габарите — препятствие, классы не нужны») → алгоритм → демо → результаты. Главный кадр — объект организаторов, увиденный за X м (`runs/img/fo_<id>.png`). «Что пробовали и что не сработало» — таблица вариантов из `docs/PLAN.md`. Цифры только из README.
- [ ] **Step 3: Загрузка до 29.09 20:00.** Ссылка на код (задача 1, шаг 3), образ или ссылка на него, README, видео, презентация. Push ветки `grader-fixes` и PR в `main`, если код сдаётся из репозитория.
