# Base image
# Needs to be based on ubuntu Jammy so that libcamera library compiles
FROM ghcr.io/sloretz/ros:humble-desktop

WORKDIR /root/exomy_ws/src
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && apt -y upgrade

# Basic tools
RUN apt install git tmux wget curl python3-pip net-tools iputils-ping -y

# Install packages for libcamera
RUN sudo apt install -y --no-install-recommends \
    meson \
    ninja-build \
    pkg-config \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    libevent-dev \
    libdrm-dev \
    libcap-dev \
    python3-opencv \
    python3-colcon-meson \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Get a very specific build of libcamera, which is the last known to be working with docker
RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
RUN git clone https://github.com/christianrauch/camera_ros.git

WORKDIR /root/exomy_ws/
SHELL ["/bin/bash", "-c"] 
RUN apt update; \
    source /opt/ros/humble/setup.bash; \
    rosdep install --from-paths src --ignore-src --skip-keys=libcamera -y; \
    colcon build

# Install additional ros packages
RUN sudo apt install -y \
    ros-humble-joy \
    '~nros-humble-rqt*' \
    '~nros-humble-image-view*'

RUN pip3 install adafruit-pca9685

# Install packages for web application
# RUN curl -sL https://deb.nodesource.com/setup_12.x | sudo bash -
# RUN sudo apt-get update && \
#     sudo apt-get install nodejs -y

# ENV NPM_CONFIG_PREFIX=/home/exomy/.npm-global
# ENV PATH=$PATH:/home/exomy/.npm-global/bin
# RUN npm install http-server -g

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
ADD entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "bash" ]
