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

COPY core /opt/tunnel_od/core
RUN python3 -m pip install --upgrade "pip>=23" \
    && python3 -m pip install "numpy>=1.23,<2" /opt/tunnel_od/core \
    && python3 -c "import tunnel_od, numpy; print('tunnel_od OK, numpy', numpy.__version__)"

COPY ros2_ws/src /ws/src
RUN source /opt/ros/humble/setup.bash && cd /ws \
    && colcon build --event-handlers console_direct- \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DTUNNEL_OD_MARCH=x86-64-v2 \
    && ./build/tunnel_od_preproc/test_canonical \
    && rm -rf build log

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
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 xvfb x11vnc novnc websockify libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
EXPOSE 6080
