# syntax=docker/dockerfile:1

FROM ros:foxy-ros-base-focal

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# default location
WORKDIR /ptracked

# add own source code to image
COPY . .
	
# launch ros package
CMD ["ros2"]