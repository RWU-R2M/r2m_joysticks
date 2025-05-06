ARG ROS_DISTRO=jazzy
ARG PREFIX=

FROM ros:$ROS_DISTRO-ros-base AS pkg-builder

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY ./joy2twist ./src/joy2twist

# Update Ubuntu Software repository and initialise ROS workspace
RUN apt update -y && apt upgrade -y &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Create the final image
FROM ros:$ROS_DISTRO-ros-base

# select bash as default shell
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-joy-linux && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy from the build stage
COPY --from=pkg-builder /ros2_ws /ros2_ws

# Add setup files to entrypoint
RUN echo "source /ros2_ws/install/setup.bash" >> /ros_entrypoint.sh

RUN echo $(cat /ros2_ws/src/joy2twist/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt