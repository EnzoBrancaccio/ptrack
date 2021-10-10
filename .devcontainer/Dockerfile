# syntax=docker/dockerfile:1

FROM althack/ros2:foxy-dev

# install ros2 packages
RUN apt-get update && apt-get install -y \
  ros-foxy-desktop \
  && rm -rf /var/lib/apt/lists/*

# https://www.allisonthackston.com/articles/vscode-docker-ros2.html