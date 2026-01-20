FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-dwb-core \
    ros-humble-dwb-msgs \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    build-essential \
    cmake \
    git \
    vim \
    usbutils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/projects/mirsws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'export PS1="\[\e[32m\]\u@mirs-sim\[\e[m\]:\[\e[34m\]\w\[\e[m\]\\$ "' >> /root/.bashrc

CMD ["bash"]