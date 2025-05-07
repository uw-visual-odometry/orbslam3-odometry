# orbslam3-odometry

## Introduction
This repository is ROS2 warpper around ORBSLAM3.  It publishes ORBSLAM3 Odometry on ROS2 topic.

## Prerequisites

I have tested on below version:

* Desktop
    * Ubuntu 24.04
    * ROS2 Jazzy
    * Opencv 4.6.0 (as installed by apt)
* Jetson Jetpack 6.1
    * Ubuntu 22.04
    * ROS2 Humble
    * OpenCV x.x.x


## To build

First, install [ROS2](https://docs.ros.org/en/jazzy/Installation.html).

1. Create a ROS2 workspace and source ROS

```bash
mkdir -p orbslam_ws/src/ && cd orbslam_ws/src/
source /opt/ros/$ROS_VERSION/setup.bash
```

2. Clone this repository
``` bash
git clone https://github.com/Il-castor/orbslam3-odometry.git
```

2. Change this [line](src/orbslam3_odometry/cmake/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Install dependencies:

```bash
cd ..
rosdep install --ignore-src --from-paths src
```

Then build
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
$ source install/setup.bash
```

2. Modify parameters in `config/orbslam3_odometry.yaml` according to your system settings. Follow [these instructions](full_documentation.md).

3. Launch
``` bash
$ ros2 launch orbslam3_odometry stereo.launch.py
```


To stop the node press `ctrl-c` and this save ORB_SLAM3 statistics txt in your folder.



# Aaron's notes


`orbslam3_odometry.launch.py`, `orblsam3_odometry.yaml`, and `aquarium_nano_stereo.yaml` have all been "tuned" for our cameras and data.

I was testing on the desktop under ROS Jazzy.  I found the system worked much better with [Zenoh as the middleware](https://github.com/ros2/rmw_zenoh)

The current `orbslam3_odometry.launch.py` is specialized for running the bagfiles from the 4/25/25 Aquarium testing.  In addition to `orbslam3_odometry` is runs an `image_proc::RectifyNode` which rectifies the image using the camera info.

orbslam is configured to take _rectified_ images.  If I read [this line](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L337) correctly, with Rectified images, `Stereo.T_c1_c` does not need to be set (it is effectively redundant with the stereo baseline) but not providing it didn't work TBD.

## Publishing supplemental camera info

The bagfiles captured on 4/25/25 at the Aquarium do not contain `camera_info` topics.   The local script [camera_info_injector](orbslam_helpers/orbslam_helpers/camera_info_injector.py) can address this by listens to an image stream, then republishes that image along with a time-synchronized `camera_info` topic.   For stereo data, two instances can be launched with [`inject_camera_info.launch.py`](orbslam3_odometry/launch/inject_camera_info.py).

The injector reads the left and right calibration from this repo.

**NOTE:**  This launchfile assumes the _input_ image topics are in the `/bag/` namespace;  it then publishes its output in the standard locations of `/left/image_raw`, `/left/camera_info` etc.

To run the injector and remap topics appropriately while playing back a bagfile:

```
(in one terminal)     > ros2 launch orbslam_odometry inject_camera_info.launch.py
(in another temrinal) >  ros2 bag play  -m /left/image_raw:=/bag/left/image_raw /right/image_raw:=/bag/right/image_raw
                                     -i <bagfile>
```
