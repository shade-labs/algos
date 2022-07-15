#!/bin/bash

source /opt/ros/"${ROS_DISTRO}"/setup.bash

# Install the deps to run the video streamer
apt update && \
    apt install -y \
    python3-natsort \
    curl \
    python3-pip ffmpeg libsm6 libxext6 ros-"${ROS_DISTRO}"-cv-bridge ros-"${ROS_DISTRO}"-vision-opencv && \
    python3 -m pip install opencv-python && \
    curl https://sample-videos.com/video123/mp4/480/big_buck_bunny_480p_10mb.mp4 --output ~/video.mp4

curl http://fileserver.com/files/ros_tests.py --output ./ros_tests.py
curl https://raw.githubusercontent.com/open-shade/video_simulator/master/camera_simulator/camera_simulator.py --output ./camera_simulator.py

# curl https://raw.githubusercontent.com/open-shade/algos/main/tests/ros_tests.py --output ./ros_tests.py

python3 -m pip install requests

echo "Starting the ROS algo"
/home/shade/shade_ws/start.sh &

echo "Starting testing suite"
result=$(sudo python3 ./ros_tests.py)

echo "Killing the ROS algo"
kill -9 $!

exit "$result"