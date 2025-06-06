cmake_minimum_required(VERSION 3.5)
project(orbslam3_odometry)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/usr/lib/python2.7/dist-packages/")
#set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.8/site-packages/")

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS highgui)

include_directories(
  include
  /home/sysop/aaron/orbslam3_ros2_ws/src/ORB_SLAM3/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  )

link_directories(
  include
)

add_executable(orbslam3_odometry
  src/stereo/stereo.cpp
  src/monocular/monocular.cpp
  src/main_node.cpp
  src/stereo/just_check_stereo_calibration.cpp
  src/stereo/stereo_rectification.cpp
)
ament_target_dependencies(orbslam3_odometry 
        cv_bridge 
        message_filters 
        nav_msgs 
        ORB_SLAM3 
        Pangolin 
        rclcpp 
        sensor_msgs 
        tf2_geometry_msgs 
        tf2_ros 
        yaml-cpp
        )
target_link_libraries(orbslam3_odometry
  ${OpenCV_LIBRARIES})


install(TARGETS orbslam3_odometry DESTINATION lib/${PROJECT_NAME})

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch config
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()