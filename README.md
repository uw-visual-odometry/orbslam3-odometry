# orbslam3-odometry
## Introduction
This repository is ROS2 wrapping to use ORB_SLAM3 and to publish ORB_SLAM3 Odometry on ROS2 topic.

## Prerequisites
I have tested on below version: 

* Ubuntu 20.04
* ROS2 Foxy
* Opencv 4.5.0

- Install related ROS2 package
``` bash
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```
## How to build 
1. Clone this repository
``` bash
git clone https://github.com/Il-castor/orbslam3-odometry.git
```
2. Change this [line](src/orbslam3_odometry/CMakeLists.txt#L6) to your own `python site-packages` path
3. Change this [line](src/orbslam3_odometry/cmake/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Build 
``` bash
colcon build --symlink-install
```
## Troubleshootings
If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.

``` bash
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

## How to use
1. Source the workspace 
``` bash
$ source install/local_setup.bash
```

2. Modify parameters in `config/orbslam3_odometry.yaml` according to your system settings. Follow [these instructions](full_documentation.md). 

3. Launch 
``` bash
$ ros2 launch orbslam3_odometry orbslam3-odometry_launch.py
```


To stop the node press `ctrl-c` and this save ORB_SLAM3 statistics txt in your folder. 



# Aaron's notes

The current `orbslam3_odometry.launch.py` is specialized for running the bagfiles from the 4/25/25 Aquarium testing.  In addition to `orbslam3_odometry` is runs two nodes for each of the left and right channels:

* It runs the local script [camera_info_injector](orbslam_helpers/orbslam_helpers/camera_info_injector.py), which listens to an image stream, then republishes that image along with a time-synchronized `camera_info` topic.  This lets us make up for the fact we didn't record `camera_info` at the aquarium.

** I don't know if this node needs to republish or if it can just publish a camera info every time an image is published.  ROS is picky about having images and camera info in sync.    If we don't need to republish the camera info message, things get simpler. **

* It runs an `image_proc::RectifyNode` which rectifies the image using the camera info.

orbslam is configured to take _rectified_ images.  If I read [this line](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L337) correctly, with Rectified images, `Stereo.T_c1_c` does not need to be set (it is effectively redundant with the stereo baseline) but not providing it didn't work TBD.

`orbslam3_odometry.launch.py`, `orblsam3_odometry.yaml`, and `aquarium_nano_stereo.yaml` have all been "tuned" for our cameras and data.

I was testing on the desktop under ROS Jazzy.  I found the system worked much better with [Zenoh as the middleware](https://github.com/ros2/rmw_zenoh)