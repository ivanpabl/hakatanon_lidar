# Архитектура

Из каких частей состоит решение и как между ними идут данные. Сам алгоритм — в [algorithm.md](algorithm.md).

## Компоненты

```
ros2 bag play / лидар
        │  sensor_msgs/PointCloud2 (8–23 МБ, 10 Гц)
        ▼
┌─────────────────────────┐   /tunnel_od/cloud    ┌──────────────────────────────┐
│ tunnel_od_preproc (C++) │ ───────────────────▶  │ tunnel_od_detector (Python)  │
│ формат, оси, dual return│   x, y, z float32     │   ObstacleDetector (ядро)    │
│ обрезка 2–250 м         │                       │   path_worker (процесс)      │
│ проверка потока         │                       └──────────────┬───────────────┘
└───────────┬─────────────┘                                      │
            │ /tunnel_od/input_diagnostics            /tunnel_od/result   (JSON на кадр)
            ▼                                         /tunnel_od/markers  (RViz)
       диагностика                                               ▼
                                                   RViz2 / потребитель результата
```

| Часть | Где | Что делает |
|---|---|---|
| Ядро `tunnel_od` | `core/` | алгоритм: пакет Python + numpy, без ROS. Тот же код работает в узле и в офлайн-оценке |
| Приём облака | `ros2_ws/src/tunnel_od_preproc` | C++-компонент rclcpp: разбирает облако любого поддерживаемого формата ([input_format.md](input_format.md)), удаляет дубли dual return, обрезает по дальности, проверяет поток и публикует каноническое облако |
| Узел детектора | `ros2_ws/src/tunnel_od_detector` | rclpy: вызывает ядро на каждом кадре, публикует результат и маркеры, пишет итог прогона |
| Запуск | `ros2_ws/src/tunnel_od_detector/launch` | `detector.launch.py` — узлы; `play.launch.py` — узлы + проигрывание записи; `demo.launch.py` — то же + RViz в браузере |
| Контейнер | `Dockerfile`, `docker-compose.yml` | стадия `detector` (ROS 2 Humble, ядро, узлы), стадия `viz` (+ RViz2, Xvfb, noVNC) |
| Параметры | `config/` | `detector.yaml` (узлы и алгоритм), `fastdds.xml` (транспорт), `tunnel_od.rviz` (вид RViz) |
| Оценка качества | `evaluation/` | офлайн, без ROS: ложные тревоги, дальность, скорость, отчёт — `python -m evaluation` |

## Запуск

```bash
docker compose build
BAG=/путь/к/записи docker compose up play      # узел + ros2 bag play, итог в output/
BAG=/путь/к/записи docker compose up demo      # то же по кругу + RViz в браузере: http://localhost:6080
docker compose up detector                     # только узлы: для живого лидара или своего плеера
docker compose run --rm test                   # gtest + pytest внутри образа
```

`play` проигрывает запись один раз и завершается. В `output/` остаются:

| Файл | Что внутри |
|---|---|
| `<запись>_result.jsonl` | результат каждого кадра — то же, что уходит в `/tunnel_od/result` |
| `<запись>_stats.json` | итог: принято / обработано / отброшено кадров, задержка p50/p95, кадров с тревогой |
| `<запись>_stats_preproc.json` | приём облака: время разбора, проверка входного потока |
| `<запись>_e2e.json` | задержка «плеер опубликовал облако → узел опубликовал результат» (`latency_probe`) |

Порядок в `play.launch.py`:
1. узлы стартуют; топик облака и `frame_id` читаются из самой записи;
2. через 3 с — `ros2 bag play --start-paused --read-ahead-queue-size 20`;
3. через 6 с — снятие с паузы (`/rosbag2_player/resume`);
4. плеер закончил — через 3 с запуск останавливается, узел пишет итог. Страховка — длительность записи / rate + 60 с.

Пауза и короткая очередь чтения обходят две ловушки `ros2 bag play` в Humble:
- по умолчанию плеер перед стартом читает вперёд 1000 сообщений — для этих записей это вся запись: 3–5 ГБ в памяти, ~20 с тишины, затем пачка кадров, а на кадрах 23 МБ — зависание;
- часы плеера запускаются раньше, чем готова очередь, и первые 1–2 с записи уходят пачкой — узел берёт последний кадр пачки, остальные отбрасывает. Пока плеер на паузе, DDS успевает связать его с узлом (QoS `VOLATILE`: кадры до связи теряются).

Свой плеер нужно запускать так же: `--read-ahead-queue-size 20 --start-paused` (в `play.launch.py` очередь чтения — аргумент `read_ahead:=`), затем `ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume`.

`demo` поднимает в контейнере виртуальный экран (Xvfb), RViz2 с программным OpenGL и noVNC, запись идёт по кругу. Порт 6080 открыт только для этой машины; для показа по сети — `"6080:6080"` в `docker-compose.yml`.

## Узел `tunnel_od_detector`

| Топик | Тип | Что внутри |
|---|---|---|
| вход: `/tunnel_od/cloud` | `sensor_msgs/PointCloud2` | каноническое облако от `tunnel_od_preproc` (x, y, z float32) |
| `/tunnel_od/result` | `std_msgs/String` | JSON на каждый обработанный кадр |
| `/tunnel_od/markers` | `visualization_msgs/MarkerArray` | ось пути, коридор, объекты, текст с решением |
| `/tunnel_od/input_diagnostics` | `diagnostic_msgs/DiagnosticArray` | проверка входного потока (от `tunnel_od_preproc`) |

Топик лидара задаётся параметром `topic`. Если он пустой, берётся первый топик `PointCloud2`, у которого есть издатель (`/lidar_points`, `/sensing/lidar/hesai128/pointcloud`). QoS — `RELIABLE`, как у `ros2 bag play`. Без C++-узла (`use_cpp_preproc:=false`) детектор подписывается на облако лидара сам.

Поля JSON — всё, что вернул `ObstacleDetector.detect()`, плюс служебные поля узла. Новые поля ядра попадают в JSON без правки узла; NaN и inf — `null`.

| Поле | Значение |
|---|---|
| `status` | решение по кадру: `stop` / `caution` / `clear` / `unknown` (путь не найден), см. [algorithm.md](algorithm.md#решение-по-кадру-detectiondecisionpy) |
| `obstacle`, `distance_m` | `status == 'stop'` и дистанция до ближайшего объекта уровня `stop`, м (`null` — нет) |
| `caution_distance_m`, `sight_m`, `clear_to_m` | до ближайшего объекта уровня `caution`; докуда путь виден; докуда габарит проверен свободным, м |
| `objects` | все объекты кадра: `distance_m`, `lateral_m`, `height_m`, `n_points`, `level`, `reason`, `confirmed`, `too_small`, `beyond_path`, `held`, … |
| `path_available`, `path_range_m`, `path_age_frames` | найден ли путь, до какой дальности известна ось, возраст пути в кадрах |
| `speed_mps` | скорость поезда, оценённая по сцене |
| `stamp`, `frame_id`, `topic`, `frame` | `header.stamp` облака (с), система координат, топик, номер кадра |
| `latency_ms`, `parse_ms`, `detect_ms`, `queue_ms` | от приёма до публикации; разбор; детектор; ожидание в очереди |
| `refit_mode`, `refit_path`, `path_fit_ms` | как пересчитывается путь, обновился ли он перед кадром, время пересчёта |
| `repeated` | кадр — побитовый повтор предыдущего облака: результат прошлого кадра, детектор не вызывался (`skip_repeated`) |
| `dropped_total` | сколько кадров отброшено к этому моменту: очередь была полна (`dropped_stale` в итоге) плюс догон (`dropped_catchup`) |
| `node_state`, `reason` | защита по входу: `ok` \| `warmup` (первые `warmup_frames` кадров — `status: unknown`, `clear_to_m: 0`, `stop` не гасится) \| `fault` (нет облаков дольше `watchdog_timeout_s` — снимок `reason: no_lidar_data`, `since_last_cloud_s`, `fault: true`, публикуется по таймеру в топик, пока данные не возобновятся; в `<запись>_result.jsonl` снимки не пишутся, `frame`/`stamp` в них `null`). При `unknown` от детектора `reason` — `no_path` / `short_sight`. `status` остаётся в множестве `stop` / `caution` / `clear` / `unknown` |

```bash
docker compose run --rm play bash                               # оболочка с настроенным ROS
ros2 topic echo /tunnel_od/result --field data                  # поток JSON
jq -c 'select(.obstacle) | {frame, distance_m}' output/doubleT_obstacle_result.jsonl
```

В лог узел пишет `СТОП кадр …` на каждый кадр со СТОП, раз в `stats_period_s` — сводку (кадров в секунду, задержка p50/p95, принято / обработано / отброшено), при остановке — `ИТОГ {...}`.

**Очередь.** Колбэк подписки кладёт кадр в слот, обработка идёт в отдельном потоке. Если детектор не успел, старый кадр заменяется новым и учитывается в `dropped_stale` — задержка не копится.

**Защита по входу** (`input_guard.py`, без ROS, тесты `tests/test_input_guard.py`): watchdog по таймеру — нет облаков дольше `watchdog_timeout_s` (0,5 с) → снимок `node_state: fault` / `reason: no_lidar_data` в `/tunnel_od/result` и красная подпись `NO LIDAR DATA` в RViz, пока данные не возобновятся (в итоге `fault_episodes`, `fault_snapshots`); прогрев — первые `warmup_frames` кадров отдают `status: unknown`, а не `clear`; после провала входа прогрев начинается заново; догон стартового всплеска `ros2 bag play` — обрабатывается самый свежий кадр очереди, старые считаются в `dropped_catchup`.

**Пересчёт пути** стоит 25–30 мс (на Mac — ~80 мс), разбор и проверка кадра вместе — 15–25 мс. По умолчанию (`refit_mode: async`) путь считается в отдельном процессе по самому свежему кадру и подставляется в детектор перед следующим кадром: путь отстаёт на 1–2 кадра и побитно совпадает с синхронным пересчётом. `refit_mode: sync` — пересчёт в том же вызове раз в `refit_every` кадров.

**Конец оси в асинхронном режиме.** Дальность известной оси скачет от кадра к кадру (199 → 153 → 204 → 131 м), а путь из фонового процесса опаздывает на кадр. С осью прошлого кадра дальний кандидат у самого конца оси поднимал тревогу, которую пересчёт на текущем кадре не дал бы (197 и 166 м в начале `doubleT_obstacle`). Поэтому объект в последних 20 % оси поднимает тревогу, только если он внутри оси по трём последним пересчётам (`alarm_range_fits: 3`); пока пересчётов меньше трёх, конец оси считается ненадёжным. Объекты дальше от конца оси это правило не затрагивает. Такие объекты помечены в результате `beyond_recent_path`, ограничение — поле `alarm_range_m`.

## Параметры

Все параметры — в `config/detector.yaml`. Каталог `config/` монтируется в контейнер, после правки пересобирать образ не нужно. Не заданные в файле аргументы алгоритма берутся по умолчанию из ядра.

**Алгоритм** (`tunnel_od_detector.detector`). Всё под `detector:` передаётся в `ObstacleDetector(**kwargs)` как есть: новый аргумент ядра достаточно дописать в YAML, ключ, которого нет у ядра, узел пропускает с предупреждением. Смысл параметров — в [algorithm.md](algorithm.md#основные-параметры-obstacledetector). `zone` — имя константы ядра (`GAUGE_METRO`) или плоский список троек `[низ, верх, полуширина, …]`.

**Узел детектора** (`tunnel_od_detector`):

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `topic` | `""` | топик облака; пусто — первый `PointCloud2` с издателем |
| `refit_mode` | `async` | `async` — путь в отдельном процессе по свежему кадру; `sync` — в том же вызове |
| `refit_every` | 1 | для `sync`: пересчёт пути раз в N кадров |
| `alarm_range_fits` | 3 | для `async`: объект в последних 20 % оси поднимает тревогу, только если он внутри оси по N последним пересчётам; 1 — без этого |
| `qos_reliability`, `qos_depth` | `reliable`, 5 | QoS подписки (`ros2 bag play` публикует `RELIABLE`) |
| `max_pending` | 2 | очередь кадров к детектору; полна — отбрасывается самый старый |
| `catch_up` | `auto` | догон: из очереди берётся самый свежий кадр, старые отбрасываются (`dropped_catchup`); `auto` — включён при `max_pending` ≤ 10 (детерминированный прогон с 100000 обрабатывает все кадры) |
| `watchdog_timeout_s`, `watchdog_grace_s` | 0.5, 2.0 | нет облаков дольше порога — `node_state: fault` в `/tunnel_od/result` и подпись `NO LIDAR DATA` в RViz; до первого кадра порог `watchdog_grace_s`. `play.launch.py` ставит grace = `start_delay` + 2 с (плеер на паузе — не провал) и порог max(0.5, 0.5 / `rate`) |
| `warmup_frames` | 3 | прогрев: первые N обработанных кадров — `node_state: warmup`, `status: unknown` (0 — без прогрева); кончается по числу кадров, даже если ось не построилась (дальше `unknown` / `reason: no_path`) |
| `parse_backend` | `auto` | без C++-узла: `auto` (нативная библиотека, иначе Python) \| `native` \| `python` |
| `publish_markers`, `markers_every`, `marker_max_objects` | `true`, 1, 30 | маркеры RViz: публиковать, раз в N кадров, не больше стольких объектов |
| `log_alarms` | `true` | строка `СТОП кадр …` в лог на каждый кадр со СТОП |
| `skip_repeated` | `true` | побитовый повтор облака (синтетика организаторов) — прошлый результат с `repeated: true`, детектор не вызывается |
| `stats_period_s` | 5 | период сводки в логе |
| `result_file`, `stats_file` | `""` | JSONL с результатом каждого кадра; итог JSON при остановке (их задаёт `play.launch.py`) |
| `detector_json` | `""` | JSON поверх `detector.*` для того, что ROS-параметром не задать (`null`, вложенные списки) |

**Приём облака** (`tunnel_od_preproc`). На детекцию влияют только формат, оси, дубли и обрезка; проверка потока (`/tunnel_od/input_diagnostics`) только сообщает.

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `input_topic` | `""` | топик облака; пусто — первый `PointCloud2` с издателем |
| `input_format` | `auto` | `auto` \| `legacy_hesai` \| `contract_v1` \| `generic` ([input_format.md](input_format.md)) |
| `input_axes` | `auto` | `legacy` (x вбок, вперёд −y) \| `rep103`; `auto` — по формату |
| `dedupe_dual_return` | `true` | удалять второе отражение, совпадающее с первым |
| `dedupe_rounded` | `true` | удалять точки, совпадающие по сетке 1 см (облако не из пар столбцов): раньше это делал узел детектора в Python (21–35 мс на кадр), теперь C++ (~2 мс); в meta облака ставится флаг, и узел свой дедуп пропускает. Результат детектора побитово тот же (A/B на 1261 кадрах) |
| `crop_enabled`, `crop_fwd_min`, `crop_fwd_max` | `true`, 2, 250 м | обрезка по дальности вперёд — только то, что ядро отбрасывает само; launch проверяет это по `detector.*` и выключает обрезку, если она небезопасна |
| `apply_mount_tf`, `base_frame` | `false`, `base_link` | поворот облака по `/tf_static`; меняет вход ядра — включать только после прогона `python -m evaluation alarms` и `approach` |
| `speed_topic`, `speed_type` | `""`, `auto` | скорость поезда из `TwistStamped` / `Odometry` — только запись в результат |
| `description_topic` | `/lidar/description` | описание датчика (M6 в [input_format.md](input_format.md)) |
| `stats_period_s`, `qos_reliability`, `qos_depth` | 5, `reliable`, 5 | сводка и QoS |

Свой bag подаётся каталогом: `BAG=/data/my_bag docker compose up play`. Облако должно быть в системе координат лидара, как в записях (`x` — вбок, вперёд — `−y`, `z` — вверх), либо в REP-103 по контракту ([input_format.md](input_format.md)).

## RViz

`demo` открывает RViz2 в браузере. На машине с установленным ROS 2 тот же конфиг открывается напрямую: `rviz2 -d config/tunnel_od.rviz -f <frame_id>`.

Облако раскрашено по высоте, ось пути — зелёная, коридор — синий. Красные объекты — тревога, жёлтые — не подтверждены, серые — мелкие или за концом оси. Текст решения — латиницей (`OBSTACLE 56.4 m`): шрифт RViz2 не содержит кириллицы. Текст — отдельное пространство имён `tunnel_od_text`, его можно выключить.

## Большие кадры и DDS

Кадр весит 8 МБ, в `doubleT_obstacle` — 23 МБ (полный оборот). Плеер и узлы работают в одном контейнере, FastDDS передаёт кадр через общую память (`config/fastdds.xml`: сегмент 256 МБ, `shm_size: 2g` в compose) — без фрагментации UDP и без настройки `net.core.rmem_max`. Для внешнего плеера или драйвера на той же машине сервис `detector` запускается с `network_mode: host` и `ipc: host`.

Проверка подписчиком без обработки (26.09): на кадрах 8 МБ 10 Гц держат все три варианта DDS; на 23 МБ — FastDDS SHM 201/201, FastDDS по умолчанию 190/201, CycloneDDS 189/201. Поэтому выбран FastDDS SHM, остальные варианты убраны.

Для 23 МБ × 10 Гц запись читается со скоростью ~240 МБ/с. Если в логе плеера «Message queue starved», кадры идут реже 10 Гц — это ограничение диска, а не узла; можно проигрывать медленнее: `RATE=0.5`.
