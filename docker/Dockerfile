ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS Humble Desktop Full
#       - https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble-desktop-full
################################################################################################

FROM osrf/ros:humble-desktop-full AS base

LABEL org.opencontainers.image.authors="robert940121@gmail.com"

ARG USERNAME=user
ARG USER_UID=1004
ARG USER_GID=$USER_UID

################################################################################################
# - User Setup stage
#   - In this stage, create a non-root user and configure passwordless sudo.
################################################################################################

FROM base AS user-setup

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    apt-get update && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*

################################################################################################
# - Tools Installation stage
#   - In this stage, I will install convenient tools for development.
################################################################################################

FROM user-setup AS tools

## Install necessary packages
RUN apt-get update && \
    apt-get install -y \
    cmake pkg-config \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-numpy \
    apt-utils \
    git \
    nano \
    net-tools \
    ssh \
    usbutils \
    udev \ 
    x11-apps \
    tmux \
    htop \ 
    ros-humble-foxglove-bridge \
    libsdl-image1.2-dev \
    libarmadillo-dev \
    libboost-all-dev \
    libsuitesparse-dev \
    ros-humble-libg2o \
    ros-humble-rmw-cyclonedds-cpp\
    libusb-1.0-0 libusb-1.0-0-dev -y && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir rotop
    
################################################################################################
# - Sensor Dependencies Installation stage
#   - In this stage, install neccesary stuff for sensors.
################################################################################################

FROM tools AS sensor-dep

RUN apt-get update && \
    apt-get install -y \
    ros-humble-imu-tools \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_WS_PATH=/home/$USERNAME/localization-ws

WORKDIR ${ROS_WS_PATH}/src/sensor_dep

RUN cd ${ROS_WS_PATH}/src/sensor_dep && \
    mkdir phidgets_drivers YDLidar-SDK ydlidar_ros2_driver obstacle_detector_2
RUN git clone -b $ROS_DISTRO https://github.com/ros-drivers/phidgets_drivers.git ${ROS_WS_PATH}/src/sensor_dep/phidgets_drivers && \
    git clone https://github.com/wintera1233/YDLidar-SDK.git ${ROS_WS_PATH}/src/sensor_dep/YDLidar-SDK && \
    git clone -b $ROS_DISTRO https://github.com/YDLIDAR/ydlidar_ros2_driver.git ${ROS_WS_PATH}/src/sensor_dep/ydlidar_ros2_driver && \
    git clone -b $ROS_DISTRO-devel https://github.com/yjHuang251/obstacle_detector_2.git ${ROS_WS_PATH}/src/sensor_dep/obstacle_detector_2

WORKDIR ${ROS_WS_PATH}

RUN cd ${ROS_WS_PATH}/src/sensor_dep/YDLidar-SDK/build && \
    cmake .. && \
    make -j4 && \
    sudo make install
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install' \
    && sudo rm -rf /var/lib/apt/lists/*

################################################################################################
# - Final stage
# 
################################################################################################

FROM sensor-dep AS final

COPY .bashrc /home/$USERNAME/.bashrc.conf

RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc
RUN sudo chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/localization-ws

USER ${USERNAME}

CMD ["/bin/bash"]