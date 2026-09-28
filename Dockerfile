# ROS 2 Humble + ядро tunnel_od + узлы tunnel_od_preproc (C++) и tunnel_od_detector (Python).
# Две стадии:
#   detector -- всё для обработки записи и живого лидара (docker compose up play / detector);
#   viz      -- + RViz2, виртуальный экран и noVNC для показа в браузере (docker compose up demo).
# Все зависимости ставятся при сборке; при запуске сеть не нужна.
FROM ros:humble-ros-base-jammy AS detector

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-pytest python3-yaml \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-sensor-msgs ros-humble-visualization-msgs \
        ros-humble-rclcpp-components ros-humble-diagnostic-msgs ros-humble-nav-msgs ros-humble-tf2-msgs \
        ros-humble-ament-cmake-gtest ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Ядро (без ROS, зависимость -- numpy). pip из jammy (22.0) собирает pyproject.toml как "UNKNOWN" --
# берём новый pip. numpy >= 1.23 (view(float32) на срезе буфера), < 2 (сообщения Humble собраны под ABI 1.x).
COPY core /opt/tunnel_od/core
RUN python3 -m pip install --upgrade "pip>=23" \
    && python3 -m pip install "numpy>=1.23,<2" /opt/tunnel_od/core \
    && python3 -c "import tunnel_od, numpy; print('tunnel_od OK, numpy', numpy.__version__)"

# Рабочее пространство ROS 2. -march=x86-64-v2, не native: образ собирают на одной машине, а
# запускают на другой. gtest tunnel_od_preproc запускается здесь же, бинарник остаётся для тестов.
COPY ros2_ws/src /ws/src
RUN source /opt/ros/humble/setup.bash && cd /ws \
    && colcon build --event-handlers console_direct- \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DTUNNEL_OD_MARCH=x86-64-v2 \
    && ./build/tunnel_od_preproc/test_canonical \
    && rm -rf build log

# Окружение: стандартный /ros_entrypoint.sh базового образа подключает ROS, дописываем рабочее
# пространство. DDS -- FastDDS с большим сегментом shared memory под кадры 8-23 МБ.
# HOME и ROS_HOME в /tmp: контейнер запускается от имени пользователя хоста (файлы в out -- его).
COPY config /opt/tunnel_od/config
RUN sed -i 's|^exec "\$@"|source /ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh
ENV TUNNEL_OD_CONFIG=/opt/tunnel_od/config/detector.yaml \
    TUNNEL_OD_RVIZ=/opt/tunnel_od/config/tunnel_od.rviz \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tunnel_od/config/fastdds.xml \
    HOME=/tmp \
    ROS_HOME=/tmp/ros
WORKDIR /ws
CMD ["ros2", "launch", "tunnel_od_detector", "detector.launch.py"]


FROM detector AS viz
# RViz2 на виртуальном экране (Xvfb, программный OpenGL) + VNC этого экрана в браузере (noVNC).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 xvfb x11vnc novnc websockify libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
EXPOSE 6080
