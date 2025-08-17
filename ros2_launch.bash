#!/bin/bash
CONTAINER_NAME="ros2_rox_container"
CONTROLLER_PATH="/dev/input/js0"

if [ ! -e "$CONTROLLER_PATH" ]; then
    echo "Controller not found at $CONTROLLER_PATH"
    exit 1
fi

if [ ! "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
    docker compose up -d
fi

docker exec -it "${CONTAINER_NAME}" bash -c "source install/setup.bash && bash"