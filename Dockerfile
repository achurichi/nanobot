FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# 2. Fix ROS 2 GPG keys and Install System Dependencies
RUN apt-get update || true && \
    apt-get install -y curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    git cmake libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev usbutils libcap-dev libspnav-dev libbluetooth-dev libcwiid-dev libexpected-dev \
    openssh-server python3-pip python3-typeguard python3-jinja2 nano build-essential rapidjson-dev nlohmann-json3-dev libwebsocketpp-dev && \
    # SSH Configuration: Port 2222 to avoid conflict with Jetson Host
    mkdir /var/run/sshd && echo 'root:root' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config && \
    rm -rf /var/lib/apt/lists/*

# 3. Build LibRealSense v2.57.7
WORKDIR /opt
RUN git clone --depth=1 --branch v2.57.7 https://github.com/realsenseai/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /opt/librealsense/build

# 5. Install ROS dependencies
WORKDIR /root/ros_ws/src/dependencies
RUN git clone https://github.com/realsenseai/realsense-ros.git -b ros2-development && \
    git clone https://github.com/ros/diagnostics.git -b ros2-humble && \
    # Delete the buggy modules not needed
    rm -rf diagnostics/diagnostic_remote_logging diagnostics/diagnostic_aggregator && \
    git clone https://github.com/ros-perception/image_common.git -b humble && \
    git clone https://github.com/ros-perception/vision_opencv.git -b humble && \
    # git clone https://github.com/ros2/demos.git -b humble && \
    # git clone https://github.com/ros2/teleop_twist_keyboard.git -b humble && \
    git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2 && \
    git clone https://github.com/ros2/teleop_twist_joy.git -b humble && \
    git clone https://github.com/ros-teleop/twist_mux.git -b humble && \
    git clone https://github.com/ros/xacro.git -b ros2 && \
    git clone https://github.com/ros/filters.git -b ros2 && \
    git clone https://github.com/PickNikRobotics/generate_parameter_library.git -b humble && \
    git clone https://github.com/PickNikRobotics/cpp_polyfills.git -b humble && \
    git clone https://github.com/PickNikRobotics/RSL.git && \
    git clone https://github.com/pal-robotics/backward_ros.git -b foxy-devel && \
    git clone https://github.com/ros-controls/realtime_tools.git -b humble && \
    git clone https://github.com/ros-controls/control_toolbox.git -b humble && \
    git clone https://github.com/ros-controls/control_msgs.git -b humble && \
    git clone https://github.com/ros-controls/ros2_control.git -b humble && \
    git clone https://github.com/ros-controls/ros2_controllers.git -b humble && \
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble && \
    git clone https://github.com/facontidavide/rosx_introspection.git -b 2.0.0 && \
    git clone https://github.com/foxglove/foxglove-sdk.git -b sdk/v0.17.1 && \
    git clone https://github.com/ros/resource_retriever.git -b humble

# 7. Final Environment Setup
WORKDIR /root/ros_ws
RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/ros_ws/install/setup.bash ]; then source /root/ros_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
EXPOSE 2222
CMD ["/usr/sbin/sshd", "-D"]