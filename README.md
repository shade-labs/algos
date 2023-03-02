# We're migrating popular and useful ROS1 packages to ROS2 while also containerizing everything

I think a lot of people who have  tried to start a project using ROS2 are familiar with the pain of trying to find compatible packages. For example, running SLAM on a Jetson Nano using ROS2 is often a difficult setup - finding compatible ROS2  wrappers, compiling on ARM and then maintaining/upgrading the stack is  often a game of luck between what compiles and what ROS version you're  using.

Someone already nailed the state of ROS2 open source with [this meme](https://www.reddit.com/r/ROS/comments/u249az/now_is_the_time_to_migrate_your_ros_1_package_to/?utm_source=share&utm_medium=web2x&context=3).

On our team, we've been working in ROS2 and feel this pain frequently,  this is why we've been steadily putting together a large port of  algorithms including tons of **pre-trained ML algos**, new ROS2 wrappers like **a ROS2 wrapper for ORBSLAM3** and very common robotics algos like **YOLOv5.**

Almost everything we've created is compatible with

- Foxy
- Galactic
- Humble
- Rolling

Everything is already containerized and (almost everything) is compiled for -

- linux/amd64
- linux/arm/v7
- linux/arm/v8
- linux/arm64

**All ML models are also built with Nvidia drivers for GPU acceleration.**

This allows ROS2 to be totally plug and play. Now when our team wants to run YOLOv5 on a Jetson Nano using GPU acceleration on humble we just `docker run -it shaderobotics/yolov5:humble` (or just run it natively using the build steps).

All of it is open source of course so you can build and hack it however you want - each repo follows the same license set by the authors of the the original repo.

We have no  interest in monetization, we just want to make robotics development  faster and easier. This project has already saved our team tons of time  so hopefully it can save you time too.

If there is interest in this project, our hope is that this could be a  community effort for porting old/dead/not containerized projects to the  ROS2 ecosystem. If you find it useful, dropping a star on the /algos  repo really helps gauge interest.

Would love to hear any thoughts on this - Is this a bad way of doing it? How  should we make sure the original authors get the citations? Are we  duplicating other work we don't know about?