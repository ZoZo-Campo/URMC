FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv python3-pytest python3-colcon-common-extensions ros-humble-cv-bridge && rm -rf /var/lib/apt/lists/*
ENV ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws
