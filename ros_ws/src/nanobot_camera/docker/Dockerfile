FROM nvcr.io/nvidia/l4t-base:r32.7.1

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
ENV SHELL /bin/bash

# Install necessary packages
RUN apt-get update && apt-get install -y \
  curl \
  git \
  gnupg2 \
  libudev-dev \
  lsb-release \
  python3-pip \
  software-properties-common \
  udev

# Install ROS 2 dashing
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y ros-dashing-ros-base python3-colcon-common-extensions python3-rosdep
RUN pip3 install -U argcomplete
RUN rosdep init --include-eol-distros

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
  ros-dashing-rmw-cyclonedds-cpp \
  ros-dashing-xacro \
  ros-dashing-diagnostic-updater \
  ros-dashing-image-transport-plugins

# Install Realsense 2 SDK 
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update && apt-get install -y librealsense2-utils librealsense2-dev 

# Add user to 'video' group
RUN adduser $USERNAME video

USER $USERNAME

# Finish ROS configuration for the user
RUN rosdep update

RUN mkdir -p /home/$USERNAME/ros_ws/src  \
  && cd /home/$USERNAME/ros_ws/src/ \
  && git clone --depth 1 --branch $(git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1) https://github.com/IntelRealSense/realsense-ros.git
RUN bash -c "source /opt/ros/dashing/setup.bash \
  && cd /home/$USERNAME/ros_ws \
  && rosdep install -i --from-path src --rosdistro dashing --skip-keys=librealsense2 -y \
  && colcon build --symlink-install"

COPY entrypoint.sh entrypoint.sh
COPY camera_config.yaml /home/$USERNAME/camera_config.yaml

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["/bin/bash"]
