#!/bin/bash
SCRIPT_PATH=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
PARENT_PATH=$(dirname "$SCRIPT_PATH")
# Build
if [[ $1 == "rebuild" ]]; then
	docker build -t cubemars_ros -f ${PARENT_PATH}/docker/Dockerfile ${PARENT_PATH}
fi
docker container rm cubemars_docker

mkdir -p ${SCRIPT_PATH}/mtb-data
sudo chown -R 1000:1000 ${SCRIPT_PATH}/mtb-data

bash ${PARENT_PATH}/scripts/open_can.sh

docker run -it \
	--name cubemars_docker \
	--env ROS_LOCALHOST_ONLY=1 \
	--volume ${PARENT_PATH}/:/ros_ws/src/cubemars_driver_node \
	--volume /tmp:/tmp \
	--volume ${SCRIPT_PATH}/mtb-data:/home/testbench/mtb-data \
	--net=host \
	--pid=host \
	--ipc=host \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--device /dev/dri \
	--user 1000:1000 \
	cubemars_ros bash -c ". /ros_ws/install/setup.bash && ros2 run cubemars_hardware_interface cubemars_hardware_node --ros-args --params-file src/cubemars_driver_node/config/specimen_config.yaml"

# Testbench command
# . install/setup.bash && python3 src/cubemars_driver_node/scripts/testbench/main.py