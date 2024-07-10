FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    make \
    gcc \
    g++ \
    cmake \
    python3-catkin-tools \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    libopencv-dev \
    ros-noetic-urdf \
    ros-noetic-urdf-parser-plugin \
    ros-noetic-xacro \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros \
    libgoogle-glog-dev \
    x11-apps

RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace /root/catkin_ws/src"

CMD ["bash"]