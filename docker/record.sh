#!/bin/bash
# Внутри контейнера tunnel-od-viz: узел детектора + RViz2 на виртуальном экране + ros2 bag play,
# экран пишется в mp4 со строкой статуса (решение, дистанция, задержка) поверх; меню и
# строка состояния RViz обрезаются (crop), остаётся 3D-вид 1550x790.
#   record.sh <bag_dir> <out.mp4> [секунд записи] [rate]
# Переменные: FRAME -- frame_id облака для RViz (hesai_lidar; doubleT_obstacle -- lidar_livox),
#             TOPIC -- топик облака ("" -- автопоиск), TITLE -- подпись в строке статуса,
#             START -- с какой секунды записи начать (--start-offset), FPS -- кадров видео в секунду.
set -e
set -m
BAG=${1:?каталог записи}; OUT=${2:?файл mp4}; DUR=${3:-}; RATE=${4:-1.0}
NAME=$(basename "$BAG"); OUTDIR=$(dirname "$OUT"); mkdir -p "$OUTDIR"
W=1600; H=900; FPS=${FPS:-15}
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=${LP_NUM_THREADS:-3} QT_X11_NO_MITSHM=1

Xvfb :99 -screen 0 ${W}x${H}x24 -nolisten tcp > /dev/null 2>&1 &
XVFB=$!
sleep 1

ros2 launch tunnel_od_detector detector.launch.py \
    result_file:="$OUTDIR/${NAME}_video_result.jsonl" stats_file:="$OUTDIR/${NAME}_video_stats.json" \
    ${TOPIC:+topic:=$TOPIC} > "$OUTDIR/${NAME}_video_node.log" 2>&1 &
LAUNCH=$!
python3 /opt/tunnel_od/docker/hud.py /tmp/hud &
HUD=$!
rviz2 -d /opt/tunnel_od/docker/record.rviz -f "${FRAME:-hesai_lidar}" > "$OUTDIR/${NAME}_video_rviz.log" 2>&1 &
RVIZ=$!
cleanup() { kill -INT $LAUNCH $HUD $RVIZ 2>/dev/null; wait $LAUNCH 2>/dev/null; kill $XVFB 2>/dev/null; }
trap cleanup EXIT

for _ in $(seq 1 60); do
    ros2 node list 2>/dev/null | grep -q tunnel_od_detector && break
    sleep 0.5
done
sleep 6            # RViz на программной отрисовке поднимается несколько секунд

FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
TXT="fontfile=$FONT:reload=1:box=1:boxcolor=black@0.55:boxborderw=12"
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -video_size ${W}x${H} -framerate "$FPS" -i :99 \
    -vf "crop=1550:790:25:70,drawtext=$TXT:textfile=/tmp/hud_alarm.txt:fontcolor=0xff4040:fontsize=44:x=24:y=24,\
drawtext=$TXT:textfile=/tmp/hud_clear.txt:fontcolor=0x50ff70:fontsize=44:x=24:y=24,\
drawtext=$TXT:textfile=/tmp/hud_info.txt:fontcolor=white:fontsize=24:x=24:y=h-60" \
    -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$OUT" &
FF=$!

PLAY_ARGS=(--rate "$RATE" --read-ahead-queue-size 20 --disable-keyboard-controls --start-paused)
[ -n "${START:-}" ] && PLAY_ARGS+=(--start-offset "$START")
if [ -n "$DUR" ]; then
    timeout -s INT "$(python3 -c "print(int($DUR / $RATE) + 3)")" ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
else
    ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
fi
PLAYER=$!
sleep 2
for _ in $(seq 1 20); do
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > /dev/null 2>&1 && break
    sleep 0.5
done
echo ">>> запись: $NAME ${DUR:+$DUR с}"
wait $PLAYER || true
sleep 2
kill -INT $FF 2>/dev/null; wait $FF || true
echo ">>> видео: $OUT"
