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
    python3-tk \
    libgl1-mesa-glx \
    libxrender1 \
    libxext6 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libfontconfig1 \
    libdbus-1-3 \
    libxcomposite1 \
    libxcursor1 \
    libxi6 \
    libxtst6 \
    libxrandr2 \
    libxss1 \
    libxv1 \
    libxinerama1 \
    libxkbcommon0 \
    libwayland-client0 \
    libwayland-cursor0 \
    libwayland-egl1 \
    libegl1-mesa \
    libegl1 \
    libgbm1 \
    libgl1-mesa-dri \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libopengl0 \
    libx11-xcb1 \
    libxcb-glx0 \
    libxcb-dri3-0 \
    libxcb-dri2-0 \
    libxcb-present0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-shape0 \
    libxcb-render0 \
    libxcb-randr0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    ros-galactic-gazebo-ros \
    ros-galactic-rviz2 \
    ros-galactic-joint-state-publisher \
    ros-galactic-ackermann-msgs \
    ros-galactic-xacro \
    ros-galactic-yaml-cpp-vendor \
    libyaml-cpp-dev &&\
    rm -rf /var/lib/apt/lists/*

# Create a workspace for your simulation
WORKDIR /ros2_ws

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


