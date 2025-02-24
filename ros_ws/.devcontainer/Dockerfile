FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_a69efc046bca4a173371acbf4029fbd6
ARG USERNAME=nano
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard
ENV SHELL /bin/bash

# Automatically source ROS setup script for the user
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Add user to 'dialout' for serial port access
RUN adduser $USERNAME dialout

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

CMD ["/bin/bash"]