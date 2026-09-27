#!/usr/bin/env bash
# Сценарий инженера: docker build -> docker run -> ros2 bag play -> результат.
#
#   ./run.sh build                          собрать образ (все зависимости ставятся здесь)
#   ./run.sh play /путь/к/записи [rate] [аргументы ros2 bag play...]
#                                           узел + плеер в одном контейнере; запись
#                                           монтируется только на чтение; результат в $OUT_DIR
#   ./run.sh detector [аргументы launch...] только узел (для внешнего плеера / живого лидара),
#                                           например: ./run.sh detector topic:=/lidar_points
#   ./run.sh test                           gtest + pytest узлов внутри образа
#   ./run.sh shell                          bash в контейнере с настроенным окружением
#
# Переменные окружения:
#   IMAGE          имя образа (tunnel-od)
#   OUT_DIR        куда писать результат play (./runs/docker)
#   CONFIG_DIR     каталог с detector.yaml, монтируется поверх встроенного (./config)
#   TUNNEL_OD_DDS  fastdds_shm (по умолчанию) | cyclone | default
#   TOPIC          топик PointCloud2 для play ("" -- автопоиск по типу)
#   READ_AHEAD, PLAY_DELAY  параметры плеера для play (очередь 20 кадров, старт через 3 с)
#   LAUNCH_ARGS    доп. аргументы detector.launch.py для play, например "use_cpp_preproc:=false"
#   PROBE          1 (по умолчанию) -- замер e2e-задержки latency_probe в play, 0 -- без него
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${IMAGE:-tunnel-od}"
OUT_DIR="${OUT_DIR:-$ROOT/runs/docker}"
CONFIG_DIR="${CONFIG_DIR:-$ROOT/config}"
DDS="${TUNNEL_OD_DDS:-fastdds_shm}"

# Большие кадры (8-23 МБ): /dev/shm под сегменты FastDDS SHM (по 256 МБ на участника)
# и большие буферы сокетов на случай UDP/CycloneDDS.
BIG_FRAMES=(--shm-size=2g)
NET_SYSCTL=(--sysctl net.core.rmem_max=134217728 --sysctl net.core.wmem_max=134217728)
COMMON=(--rm -e "TUNNEL_OD_DDS=$DDS" -v "$CONFIG_DIR:/opt/tunnel_od/config:ro")
[ -t 0 ] && [ -t 1 ] && COMMON+=(-it)

usage() { sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

cmd="${1:-}"; [ $# -gt 0 ] && shift
case "$cmd" in
  build)
    docker build -t "$IMAGE" "$@" "$ROOT"
    ;;
  play)
    [ $# -ge 1 ] || usage 1
    BAG="$(cd "$1" && pwd)"; shift
    [ -f "$BAG/metadata.yaml" ] || { echo "нет $BAG/metadata.yaml -- укажите каталог записи ros2 bag" >&2; exit 1; }
    NAME="$(basename "$BAG")"
    mkdir -p "$OUT_DIR"
    docker run "${COMMON[@]}" "${BIG_FRAMES[@]}" "${NET_SYSCTL[@]}" \
        -e "TOPIC=${TOPIC:-}" -e "READ_AHEAD=${READ_AHEAD:-20}" -e "PLAY_DELAY=${PLAY_DELAY:-3}" \
        -e "LAUNCH_ARGS=${LAUNCH_ARGS:-}" -e "PROBE=${PROBE:-1}" \
        -v "$BAG:/bags/$NAME:ro" -v "$OUT_DIR:/out" \
        "$IMAGE" play.sh "/bags/$NAME" "$@"
    ;;
  detector)
    # --network host --ipc host: чтобы плеер/драйвер в другом контейнере на этом хосте
    # видел узел и кадры шли через общую память (/dev/shm хоста).
    mkdir -p "$OUT_DIR"
    docker run "${COMMON[@]}" --network host --ipc host -v "$OUT_DIR:/out" \
        "$IMAGE" ros2 launch tunnel_od_detector detector.launch.py "$@"
    ;;
  test)
    # gtest tunnel_od_preproc + pytest обоих пакетов
    docker run "${COMMON[@]}" "$IMAGE" bash -o pipefail -c \
        '/ws/install/tunnel_od_preproc/lib/tunnel_od_preproc/test_canonical 2>&1 | tail -3 && \
         python3 -m pytest -q -p no:cacheprovider /ws/src/tunnel_od_detector/test /ws/src/tunnel_od_preproc/test "$@"' _ "$@"
    ;;
  shell)
    docker run "${COMMON[@]}" "${BIG_FRAMES[@]}" "${NET_SYSCTL[@]}" -v "$OUT_DIR:/out" "$IMAGE" bash "$@"
    ;;
  -h|--help|help|"") usage 0 ;;
  *) echo "неизвестная команда: $cmd" >&2; usage 1 ;;
esac
