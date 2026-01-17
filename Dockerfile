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
    build-essential \
    cmake \
    git \
    vim \
    usbutils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/projects/mirsws

COPY ./src ./src
RUN git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2
RUN git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/projects/mirsws/install/setup.bash" >> /root/.bashrc && \
    echo 'export PS1="\[\e[32m\]\u@mirs-sim\[\e[m\]:\[\e[34m\]\w\[\e[m\]\\$ "' >> /root/.bashrc

COPY entrypoint.sh /root/projects/mirsws/entrypoint.sh
RUN chmod +x /root/projects/mirsws/entrypoint.sh
ENTRYPOINT ["/root/projects/mirsws/entrypoint.sh"]
CMD ["bash"]