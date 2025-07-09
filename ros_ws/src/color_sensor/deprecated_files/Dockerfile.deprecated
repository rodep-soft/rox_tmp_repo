FROM ros:humble-ros-base

# install ros2 build tool
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    git \
    vim \
    wget \
    curl \
    bash-completion \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install smbus2

# create ros2_ws
RUN mkdir -p /ros2_ws/src/tcs34725
# copy package
COPY ./ /ros2_ws/src/tcs34725
WORKDIR /ros2_ws

# source ros2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

