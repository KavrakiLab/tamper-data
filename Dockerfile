FROM ros:melodic-ros-base
LABEL author="Tianyang Pan tp36@rice.edu"

# ROS
RUN apt-get update && \
    apt-get install -y g++ \
    cmake \
    pkg-config \
    libode-dev \
    wget \
    gdb
RUN rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-moveit-* \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-srdfdom \
    ros-${ROS_DISTRO}-collada-urdf \
    ros-${ROS_DISTRO}-rviz
RUN rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-fetch-description
RUN apt-get update && \
    apt-get install -y python-catkin-tools \
    python-pip
RUN rm -rf /var/lib/apt/lists/*

# RUN pip install --upgrade pip
# RUN pip install pyyaml

# Make ROS workspace
RUN mkdir -p /home/robot/catkin_ws/src

# Copy over the repos
COPY . /home/robot/catkin_ws/src

# Setup ROS workspace
WORKDIR /home/robot/catkin_ws


RUN catkin init
WORKDIR /home/robot/catkin_ws/src
RUN rm Dockerfile
RUN rm run.bash README.md

WORKDIR /home/robot/catkin_ws

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/robot/catkin_ws/; catkin build'

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN echo 'source /home/robot/catkin_ws/devel/setup.bash' >> /root/.bashrc

