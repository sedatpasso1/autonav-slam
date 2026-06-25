FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Sistem bağımlılıkları
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-imu-complementary-filter \
    ros-humble-ros2bag \
    ros-humble-rosbag2-transport \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
RUN pip3 install --no-cache-dir \
    "kiss-icp>=1.0.0" \
    numpy \
    mulran2bag

# Workspace kurulum
WORKDIR /ros2_ws/src
COPY . autonav_slam/

WORKDIR /ros2_ws

# ROS2 ortamını source'layarak colcon build
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && colcon build \
        --packages-select autonav_slam \
        --symlink-install \
    && echo "Build OK"

# Varsayılan komut: SLAM node başlat
CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash \
      && source /ros2_ws/install/setup.bash \
      && ros2 launch autonav_slam slam.launch.py"]
