#!/usr/bin/env bash
# e2e-замер образа, в котором нет latency_probe (например, образ до C++-приёма облака):
# образ под замером проигрывает запись своим play.sh, а latency_probe из текущего образа
# слушает тот же DDS из соседнего контейнера (общие сеть и /dev/shm).
#   research/e2e_sidecar.sh <образ> <каталог записи> <каталог результата> [rate]
# CONFIG_DIR -- смонтировать свой detector.yaml поверх встроенного в образ.
set -euo pipefail
IMG=$1; BAG=$(cd "$2" && pwd); mkdir -p "$3"; OUT=$(cd "$3" && pwd); RATE=${4:-1.0}
NAME=$(basename "$BAG")
C=tod_e2e_$$
CFG=(); [ -n "${CONFIG_DIR:-}" ] && CFG=(-v "$(cd "$CONFIG_DIR" && pwd):/opt/tunnel_od/config:ro")
docker run -d --name "$C" --ipc shareable --shm-size=2g \
    --sysctl net.core.rmem_max=134217728 --sysctl net.core.wmem_max=134217728 \
    -e TUNNEL_OD_DDS=fastdds_shm "${CFG[@]}" -v "$BAG:/bags/$NAME:ro" -v "$OUT:/out" \
    "$IMG" play.sh "/bags/$NAME" "$RATE" > /dev/null
docker run -d --name "${C}_probe" --network "container:$C" --ipc "container:$C" \
    -e TUNNEL_OD_DDS=fastdds_shm -v "$OUT:/out" "${PROBE_IMAGE:-tunnel-od}" \
    ros2 run tunnel_od_preproc latency_probe --ros-args -p "out_file:=/out/${NAME}_e2e.json" > /dev/null
docker wait "$C" > /dev/null
docker logs "$C" > "$OUT/${NAME}.log" 2>&1
docker kill -s INT "${C}_probe" > /dev/null; docker wait "${C}_probe" > /dev/null
docker logs "${C}_probe" >> "$OUT/${NAME}.log" 2>&1
docker rm "$C" "${C}_probe" > /dev/null
tail -3 "$OUT/${NAME}.log"
