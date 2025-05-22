# base image
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu22.04

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive
ARG MAKEFLAGS
ARG PARALLELWORKERS
ARG DISPLAY
ARG WAYLAND_DISPLAY
ARG XDG_RUNTIME_DIR
ARG PULSE_SERVER
ENV MAKEFLAGS $MAKEFLAGS
ENV PARALLELWORKERS $PARALLELWORKERS
ENV DISPLAY $DISPLAY
ENV WAYLAND_DISPLAY $WAYLAND_DISPLAY
ENV XDG_RUNTIME_DIR $XDG_RUNTIME_DIR
ENV PULSE_SERVER $PULSE_SERVER
ENV __NV_PRIME_RENDER_OFFLOAD 1
ENV __GLX_VENDOR_LIBRARY_NAME nvidia
ENV ROS_DISTRO=humble

# setting time zone
RUN apt update && apt install -y --no-install-recommends tzdata
ENV TZ=Asia/Tokyo

RUN apt update -y && apt upgrade -y && apt install -y --no-install-recommends \
    software-properties-common \
    libxkbcommon-x11-0 \
    build-essential \
    libgl1-mesa-dev \
    libeigen3-dev \
    iputils-ping \
    lsb-release \
    libudev-dev \
    python3-pip \
    python3-tk \
    pkg-config \
    mesa-utils \
    net-tools \
    xorg-dev \
    x11-apps \
    libfuse2 \
    libxcb* \
    gnupg \
    byobu \
    cmake \
    curl \
    wget \
    dbus \
    git \
    vim \
    zip \
    && rm -rf /var/lib/apt/lists/*

# adding the ROS2 GPG key with apt and add the repository to your sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN /bin/bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# updating mesa (fix the issue of displaying black world in Rviz2 and Gazebo)
# https://github.com/ros2/rviz/issues/948
RUN add-apt-repository ppa:kisak/kisak-mesa -y && apt update -y

# installing ROS2 packages and colcon build system with mixin
# changing the middleware (CycloneDDS recommended for MoveIt2)
RUN apt install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake \
    ros-humble-warehouse-* \
    ros-humble-gazebo-* \
    ros-humble-ros-gz* \
    ros-humble-moveit* \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-eigenpy \
    ros-humble-ros-testing \
    ros-dev-tools \
    gazebo \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

RUN /bin/bash -c "colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default"

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN	/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN	echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
	echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/root/ros2_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/root/ros2_ws" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# building ros2 packages
WORKDIR /root/ros2_ws
COPY ./denso_robot_drivers_ros2.repos /root/denso_robot_drivers_ros2.repos
RUN vcs import src --skip-existing --input /root/denso_robot_drivers_ros2.repos
COPY ./ros2_ws/src /root/ros2_ws/src
RUN rosdep init && rosdep update && rosdep install --ignore-src --from-paths src -y -r --rosdistro=$ROS_DISTRO

# Generates bash commands
WORKDIR /root/ros2_ws/src/
RUN echo "ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=cobotta sim:=false ip_address:=192.168.0.1 send_format:=0 recv_format:=2" >> ~/.bash_history
RUN echo "ros2 run cobotta_tutorials hello_moveit.py" >> ~/.bash_history