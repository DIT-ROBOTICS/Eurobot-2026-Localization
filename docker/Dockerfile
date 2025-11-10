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
SHELL ["/bin/bash", "-c"]

################################################################################################
# - User Setup stage
#   - In this stage, create a non-root user and configure passwordless sudo.
################################################################################################

FROM base AS user-setup

## update ROS GPG key
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

## Create a non-root user
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
# - IMU Installation stage
#   - In this stage, install neccesary stuff for phidget IMU spatial.
################################################################################################

FROM tools AS sensor-dep
## Install necessary tools for imu
RUN apt-get update && \
    apt-get install -y \
    ros-humble-imu-tools \
    && rm -rf /var/lib/apt/lists/*
ARG ROS_WS_PATH=/home/$USERNAME/localization-ws
WORKDIR ${ROS_WS_PATH}/src/sensor_dep/phidgets_drivers
RUN git clone -b $ROS_DISTRO https://github.com/ros-drivers/phidgets_drivers.git ${ROS_WS_PATH}/src/sensor_dep/phidgets_drivers
WORKDIR ${ROS_WS_PATH}/src/sensor_dep/YDLidar-SDK
RUN git clone https://github.com/wintera1233/YDLidar-SDK.git ${ROS_WS_PATH}/src/sensor_dep/YDLidar-SDK/
WORKDIR ${ROS_WS_PATH}/src/sensor_dep/ydlidar_ros2_driver
RUN git clone -b $ROS_DISTRO https://github.com/YDLIDAR/ydlidar_ros2_driver.git ${ROS_WS_PATH}/src/sensor_dep/ydlidar_ros2_driver
WORKDIR ${ROS_WS_PATH}/src/sensor_dep/obstavle_detector_2
RUN git clone -b $ROS_DISTRO-devel https://github.com/harmony-eu/obstacle_detector_2.git ${ROS_WS_PATH}/src/sensor_dep/obsstacle_detector_2
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
