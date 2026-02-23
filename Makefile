# Define the shell
SHELL := /bin/bash

.PHONY: all clean build

# Default target
all: clean build

# Remove build artifacts
clean:
	@echo "Cleaning workspace..."
	rm -rf build/ install/ log/

# Source ROS 2 and build
build:
	@echo "Sourcing and building..."
	source /opt/ros/$(shell ls /opt/ros | head -n 1)/setup.bash && \
	if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
	colcon build

# Build with symlinks
dev: clean
	source /opt/ros/$(shell ls /opt/ros | head -n 1)/setup.bash && \
	colcon build --symlink-install