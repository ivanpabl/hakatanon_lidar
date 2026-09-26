#!/bin/bash
# Окружение ROS 2 + рабочее пространство + выбор DDS (TUNNEL_OD_DDS).
#   fastdds_shm (по умолчанию) -- FastDDS, shared memory с большим сегментом
#   cyclone                    -- CycloneDDS с большими буферами сокетов
#   default                    -- настройки DDS по умолчанию (для сравнения)
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
case "${TUNNEL_OD_DDS:-fastdds_shm}" in
  fastdds_shm)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tunnel_od/docker/fastdds_shm.xml ;;
  cyclone)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///opt/tunnel_od/docker/cyclone_big.xml ;;
  default) ;;
  *) echo "неизвестный TUNNEL_OD_DDS=${TUNNEL_OD_DDS}" >&2; exit 2 ;;
esac
exec "$@"
