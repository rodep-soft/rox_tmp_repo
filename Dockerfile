# Dockerfile
FROM ros:humble-ros-base

# --- Install ROS 2 Packages and System Dependencies ---
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git \
    vim \
    less \
    tree \
    fzf \
    fish \
    python3 \
    python3-pip \
    libboost-system-dev \
    ros-humble-joy \
    libgpiod-dev \
    gpiod && \
    rm -rf /var/lib/apt/lists/* # Clean up apt cache

# --- Install Python Packages ---
RUN pip install smbus2

# --- Configure Shell Defaults and ROS 2 setup ---
# This ensures colcon --symlink-install is default and ROS setup.bash is sourced
RUN mkdir -p /root/.config/colcon && \
    echo 'build:' > /root/.config/colcon/defaults.yaml && \
    echo '  args: ['\''--symlink-install'\'']' >> /root/.config/colcon/defaults.yaml && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.profile

# Set the working directory for subsequent commands and when the container starts
# This should match the working_dir in docker-compose.yml
WORKDIR /root/ros_ws

CMD ["bash"]
