#!/bin/bash
SCRIPT_PATH=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
PARENT_PATH=$(dirname "$SCRIPT_PATH")
# Build
docker build -t cubemars_ros -f ${PARENT_PATH}/docker/Dockerfile ${PARENT_PATH}
docker container rm cubemars_docker
docker run -it \
	--name cubemars_docker \
	--volume ${PARENT_PATH}/:/ros_ws/src/cubemars_driver_node \
	--volume /tmp:/tmp \
	--net=host \
	--pid=host \
	--ipc=host \
	cubemars_ros bash -c ". /ros_ws/install/setup.bash && ros2 run cubemars_hardware_interface cubemars_hardware_node --ros-args --params-file src/cubemars_driver_node/config/specimen_config.yaml"
