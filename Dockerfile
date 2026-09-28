# ROS 2 Humble + ядро tunnel_od + узел tunnel_od_detector.
# Все зависимости ставятся при сборке; при запуске сеть не нужна.
# Собирается через docker compose (см. docker-compose.yml), вручную не требуется.
#   detector -- сдаваемое решение: узел + плеер, минимальный образ
#   tools    -- то же плюс matplotlib/rosbags для офлайн-метрик (профиль metrics)
FROM ros:humble-ros-base-jammy AS detector

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-numpy python3-pytest python3-yaml \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-sensor-msgs ros-humble-visualization-msgs \
        ros-humble-rclcpp-components ros-humble-diagnostic-msgs ros-humble-nav-msgs ros-humble-tf2-msgs \
        ros-humble-ament-cmake-gtest \
        ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ядро (без ROS, зависимость -- numpy)
COPY core /opt/tunnel_od/core
# pip из jammy (22.0) собирает pyproject.toml ядра как "UNKNOWN" -- берём новый pip.
# numpy из jammy (1.21) не умеет view(float32) на срезе буфера (parse_pointcloud2),
# нужен >= 1.23; < 2 -- модули сообщений Humble собраны под ABI numpy 1.x.
RUN python3 -m pip install --upgrade "pip>=23" \
    && python3 -m pip install "numpy>=1.23,<2" \
    && python3 -m pip install /opt/tunnel_od/core && python3 -c "import tunnel_od; import numpy; print('tunnel_od OK, numpy', numpy.__version__)"

# рабочее пространство ROS 2: tunnel_od_preproc (C++) + tunnel_od_detector (Python).
# -march=x86-64-v2, не native: образ собирают на одной машине, а запускают на другой.
# gtest пакета tunnel_od_preproc -- здесь же; бинарник теста остаётся в install для профиля test.
COPY ros2_ws/src /ws/src
RUN source /opt/ros/humble/setup.bash && cd /ws \
    && colcon build --event-handlers console_direct- \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DTUNNEL_OD_MARCH=x86-64-v2 \
    && ./build/tunnel_od_preproc/test_canonical \
    && rm -rf build log

COPY config /opt/tunnel_od/config
COPY docker /opt/tunnel_od/docker
RUN chmod +x /opt/tunnel_od/docker/*.sh

ENV TUNNEL_OD_CONFIG=/opt/tunnel_od/config/detector.yaml \
    TUNNEL_OD_DDS=fastdds_shm \
    PATH=/opt/tunnel_od/docker:$PATH
WORKDIR /ws
ENTRYPOINT ["/opt/tunnel_od/docker/entrypoint.sh"]
CMD ["ros2", "launch", "tunnel_od_detector", "detector.launch.py"]


# ---- образ для офлайн-метрик: те же зависимости плюс чтение bag без ROS и графики. ----
# Сами инструменты не копируются -- профиль metrics монтирует tools/, gui/ и reference/,
# поэтому правка инструмента не требует пересборки образа.
FROM detector AS tools
RUN python3 -m pip install "matplotlib" "rosbags" \
    && python3 -c "import matplotlib, rosbags; print('tools OK')"
