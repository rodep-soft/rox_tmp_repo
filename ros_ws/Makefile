
.PHONY: build clean fullbuild launch

BUILD_DIR = build
INSTALL_DIR = install
LOG_DIR = log

build:
	colcon build

source:
	bash
	source $(INSTALL_DIR)/setup.bash

clean:
	rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)

fullbuild: 
	$(make) clean
	$(make) build

launch:
	$(make) source
	ros2 launch launch/launch.py

kill:
	pkill -f ros2
	pkill -f ros






