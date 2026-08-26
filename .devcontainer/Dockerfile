FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-exo", "pipefail", "-c"]

# Disable interactive prompts during apt installs
ARG DEBIAN_FRONTEND=noninteractive

# Install Intel GPU drivers
RUN apt update && apt upgrade -y && apt install -y \
    mesa-utils \
    software-properties-common && \
    add-apt-repository ppa:kisak/turtle && \
    apt update && apt upgrade -y

# System dependencies
RUN apt-get update && apt-get install -y \
    nano \
    curl \
    # Build tools
    build-essential \
    cmake \
    clang-format \
    git \
    python3-pip \
    # Eigen
    libeigen3-dev \
    # KDL
    ros-humble-orocos-kdl-vendor \
    ros-humble-kdl-parser \
    liborocos-kdl-dev \
    # MoveIt
    ros-humble-moveit \
    # ros2_control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-controller-interface \
    ros-humble-control-msgs \
    # Gazebo (Ignition Fortress is the one paired with Humble)
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros2-control \
    ros-humble-effort-controllers \
    # Utilities
    ros-humble-plotjuggler-ros \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install numpy matplotlib

# matplotlib-cpp for plotting
RUN mkdir -p /usr/local/include/matplotlibcpp && \
    curl -o /usr/local/include/matplotlibcpp/matplotlibcpp.h \
    https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h

# Create a non-root user matching typical host UID
ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

ARG WORKDIR=/home/$USERNAME/ros2_ws/
USER $USERNAME
WORKDIR ${WORKDIR}

# Write source commands to .bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/student/.bashrc && \
    echo '[ -f /home/student/ros2_ws/install/setup.bash ] && source /home/student/ros2_ws/install/setup.bash' >> /home/student/.bashrc

# Entrypoint: source ROS and workspace if it exists
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]