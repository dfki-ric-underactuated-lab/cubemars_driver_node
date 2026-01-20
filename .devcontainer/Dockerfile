FROM osrf/ros:jazzy-desktop-full

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN apt-get update && apt-get upgrade -y

# Install essential packages
RUN apt-get update && apt-get install -y wget git curl \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ament-cmake-core

# Create overlay workspace
WORKDIR /root/ros_ws
RUN git clone https://github.com/dfki-ric-underactuated-lab/robot_control_msgs src/robot_control_msgs
COPY .. /root/ros_ws/src/cubemars_driver_node
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y;
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
# COPY packages.repos .

# RUN vcs import < packages.repos; \
#     cd ..; \
#     rosdep install --from-paths src --ignore-src -r -y; \
#     . /opt/ros/${ROS_DISTRO}/setup.bash; \
#     colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc