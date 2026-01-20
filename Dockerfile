FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get install -y \
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