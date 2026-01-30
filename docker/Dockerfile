FROM ros:jazzy

RUN apt-get update && apt-get upgrade -y

# Install essential packages
RUN apt-get update && apt-get install -y wget git curl \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ament-cmake-core

RUN mkdir -p /ros_ws/src

RUN git clone https://github.com/dfki-ric-underactuated-lab/robot_control_msgs /ros_ws/src/robot_control_msgs
COPY . /ros_ws/src/cubemars_driver_node

WORKDIR /ros_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
