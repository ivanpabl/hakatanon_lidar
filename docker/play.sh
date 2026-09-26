#!/bin/bash
# Внутри контейнера: узел детектора + ros2 bag play в одном контейнере.
#   play.sh <bag_dir> [rate] [доп. аргументы ros2 bag play]
# Результат: /out/<запись>_result.jsonl (кадр за кадром), /out/<запись>_stats.json (итог).
set -e
# job control: фоновый launch в своей группе процессов и не игнорирует SIGINT
# (в неинтерактивном bash фоновые задачи SIGINT игнорируют)
set -m
BAG=${1:?укажите каталог записи (metadata.yaml + .db3)}; shift
RATE=${1:-1.0}; [ $# -gt 0 ] && shift
NAME=$(basename "$BAG")
OUT=${OUT_DIR:-/out}; mkdir -p "$OUT"

ros2 launch tunnel_od_detector detector.launch.py \
    result_file:="$OUT/${NAME}_result.jsonl" stats_file:="$OUT/${NAME}_stats.json" \
    ${TOPIC:+topic:=$TOPIC} ${REFIT_EVERY:+refit_every:=$REFIT_EVERY} &
LAUNCH=$!
trap 'kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH' EXIT

# ждём, пока узел поднимется (импорт numpy и ядра)
for _ in $(seq 1 60); do
    ros2 node list 2>/dev/null | grep -q tunnel_od_detector && break
    sleep 0.5
done
sleep 1
# --read-ahead-queue-size: плеер Humble перед стартом заполняет очередь чтения (по
#   умолчанию 1000 сообщений = вся запись, 3-5 ГБ в памяти, ~20 с тишины и зависания
#   на 23-МБ кадрах). 20 кадров достаточно.
# --start-paused + resume через PLAY_DELAY с: за это время DDS находит узел (QoS
#   VOLATILE -- кадры до этого теряются), а плеер заполняет очередь. Если стартовать
#   сразу, часы плеера идут раньше, чем очередь готова, и первые 1-2 с записи
#   выдаются пачкой -- узел берёт последний кадр пачки, остальные отбрасывает.
PLAY_ARGS=(--rate "$RATE" --read-ahead-queue-size "${READ_AHEAD:-20}" --disable-keyboard-controls --start-paused)
# страховка от зависания плеера: длительность записи / rate + 60 с
LIMIT=$(python3 -c "import yaml; m=yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']; print(int(m['duration']['nanoseconds']/1e9/float('$RATE')) + 60)" 2>/dev/null || echo 3600)
echo ">>> ros2 bag play $BAG ${PLAY_ARGS[*]} $*"
timeout "$LIMIT" ros2 bag play "$BAG" "${PLAY_ARGS[@]}" "$@" &
PLAYER=$!
sleep "${PLAY_DELAY:-3}"
for _ in $(seq 1 20); do
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > /dev/null 2>&1 && break
    sleep 0.5
done
echo ">>> воспроизведение запущено"
wait $PLAYER || echo ">>> ros2 bag play завершился с кодом $?"
sleep 3            # дообработать последние кадры
# SIGINT -- как Ctrl+C: launch останавливает узел, узел пишет итоговую сводку
kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH || true
trap - EXIT
echo ">>> результат: $OUT/${NAME}_result.jsonl, итог: $OUT/${NAME}_stats.json"
cat "$OUT/${NAME}_stats.json" 2>/dev/null || true
