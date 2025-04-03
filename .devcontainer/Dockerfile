FROM ros:humble-ros-base

ARG USERNAME=nano
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG NODE_VERSION=22.14

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# Install additional ROS packages and necessary tools
RUN apt-get update && apt-get install -y \
  ca-certificates \
  curl \
  joystick \
  kmod \
  libi2c-dev \
  libudev-dev \
  openssh-client \
  udev \
  ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
  ros-$ROS_DISTRO-demo-nodes-cpp \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-joy \
  ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-twist-mux \
  # ros-$ROS_DISTRO-twist-stamper \
  ros-$ROS_DISTRO-rosbridge-server \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-imu-filter-madgwick \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-nav2-util \
  ros-$ROS_DISTRO-nav2-msgs \
  ros-$ROS_DISTRO-nav2-lifecycle-manager \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-bond \
  ros-$ROS_DISTRO-bondcpp \
  ros-$ROS_DISTRO-ament-cmake-clang-format \
  ros-$ROS_DISTRO-image-transport-plugins \
  ros-$ROS_DISTRO-librealsense2* \
  ros-$ROS_DISTRO-realsense2* \
  ros-$ROS_DISTRO-rqt-graph \
  ros-$ROS_DISTRO-rqt-image-view \ 
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-ros-gz \
  ros-$ROS_DISTRO-ign-ros2-control

# Install Docker
RUN apt-get update \
  && install -m 0755 -d /etc/apt/keyrings \
  && curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc \
  && chmod a+r /etc/apt/keyrings/docker.asc
RUN echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  tee /etc/apt/sources.list.d/docker.list > /dev/null
RUN apt-get update && apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Install tools for IMU calibration
RUN pip install mpu9250-jmdev smbus2

# Add user to the necessary groups
RUN adduser $USERNAME dialout && adduser $USERNAME video

USER $USERNAME

# Finish ROS configuration for the user
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc
# RUN echo "export CYCLONEDDS_URI=/home/nanobot/ros_ws/cyclonedds_profile.xml" >> /home/$USERNAME/.bashrc
RUN rosdep update

# Setup node
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.2/install.sh | bash
RUN bash -c "source ~/.nvm/nvm.sh && nvm install ${NODE_VERSION}"

CMD ["/bin/bash"]
