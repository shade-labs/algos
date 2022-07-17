#!/bin/bash

# Make sure the video folder exists
mkdir -p /root

# Install the deps to run the video streamer
apt update && \
    apt install -y \
    python3-natsort \
    curl \
    python3-pip ffmpeg libsm6 libxext6 ros-"${ROS_DISTRO}"-cv-bridge ros-"${ROS_DISTRO}"-vision-opencv ros-"${ROS_DISTRO}"-rclpy && \
    python3 -m pip install opencv-python && \
    curl https://sample-videos.com/video123/mp4/480/big_buck_bunny_480p_10mb.mp4 --output /root/video.mp4

curl https://raw.githubusercontent.com/open-shade/algos/main/tests/ros_tests.py --output ./ros_tests.py
curl https://raw.githubusercontent.com/open-shade/video_simulator/master/camera_simulator/camera_simulator.py --output ./camera_simulator.py

python3 -m pip install requests

echo "Starting the ROS algo"
/home/shade/shade_ws/start.sh &

source /opt/ros/"${ROS_DISTRO}"/setup.sh
source ./install/setup.sh

echo "Starting testing suite"
python3 ./ros_tests.py
result=$?
# sudo python3 ./ros_tests.py

echo "Killing the ROS algo"
kill -9 $!

exit "$result"
