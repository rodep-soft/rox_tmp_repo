# Dockerfile
FROM ros:humble-ros-base

# --- Install ROS 2 Packages and System Dependencies ---
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    gh \
    git \
    vim \
    less \
    tree \
    fzf \
    tmux \
    fish \
    lsof \
    ccache \
    python3 \
    python3-pip \
    python3-gpiozero \
    libboost-system-dev \
    ros-humble-joy \
    ros-humble-demo-nodes-cpp \
    ros-humble-foxglove-bridge \
    libgpiod-dev \
    gpiod && \
    rm -rf /var/lib/apt/lists/* # Clean up apt cache

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN cargo install just

ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G


# --- Install Python Packages ---
RUN pip install smbus2 \
    adafruit-circuitpython-neopixel \
    adafruit-circuitpython-neopixel-spi \
    rpi_ws281x

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

COPY ./ros_ws/src ./src/

CMD ["bash"]


# launch foxglove
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
