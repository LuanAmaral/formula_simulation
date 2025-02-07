# Official ROS Galactic base image
FROM osrf/ros:galactic-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    vim \
    tmux \
    locales \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    x11-apps \
    ros-galactic-gazebo-ros \
    ros-galactic-rviz2 \
    ros-galactic-joint-state-publisher \
    ros-galactic-ackermann-msgs \
    ros-galactic-xacro \
    ros-galactic-yaml-cpp-vendor \
    libyaml-cpp-dev \
    python3-pip \
    ros-galactic-ackermann-msgs \
    python3-tk \
    &&\
    rm -rf /var/lib/apt/lists/*

# Create a workspace for your simulation
WORKDIR /ros2_ws

# Install pytorch
RUN pip3 install \
    numpy \
    torch \
    torchvision \
    torchaudio \
    gym \
    numpy-quaternion \
    tensorboard     

# Set .bashrc
COPY bash.sh /ros2_ws/bash.sh

# COPY initial_setup.sh /ros2_ws/initial_setup.sh
RUN cat /ros2_ws/bash.sh >/root/.bashrc

ENV EUFS_MASTER=/root/workspace
ENV DISPLAY=:$DISPLAY
ENV QT_X11_NO_MITSHM=1
ENV CMAKE_PREFIX_PATH=/opt/ros/galactic

# Default command to keep the container running
CMD ["bash"]


