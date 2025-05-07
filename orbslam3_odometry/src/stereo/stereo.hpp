#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include <cv_bridge/cv_bridge.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "orbslam3_odometry/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "stereo_rectification.h"

using sensor_msgs::msg::CameraInfo;

class StereoSlamNode : public rclcpp::Node {
 public:
  StereoSlamNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile);

  ~StereoSlamNode();

 private:
  void loadParameters();

  void syncedCallback(const ImageMsg::ConstSharedPtr &left_imagemsg,
                      const ImageMsg::ConstSharedPtr &right_imagemsg,
                      const CameraInfo::ConstSharedPtr &left_info,
                      const CameraInfo::ConstSharedPtr &right_info);

  ORB_SLAM3::System *m_SLAM;

  // Images stuff
  message_filters::Subscriber<ImageMsg> subscription_left;
  message_filters::Subscriber<ImageMsg> subscription_right;
  message_filters::Subscriber<CameraInfo> subscription_left_info;
  message_filters::Subscriber<CameraInfo> subscription_right_info;

  typedef message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                      CameraInfo, CameraInfo> >
      StereoImageSync;
  std::shared_ptr<StereoImageSync> sync_;

  std::mutex bufMutexLeft_, bufMutexRight_;
  double timestamp;

  std::thread *syncThread_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

  // List of all parameters
  std::string camera_left_topic, camera_right_topic, imu, header_id_frame,
      child_id_frame;
  std::string camera_left_info_topic, camera_right_info_topic;
  std::string topic_pub_quat;

  // cropping parameters
  int cropping_x, cropping_y, cropping_width, cropping_height;
  cv::Rect cropping_rect;
  double image_gamma_;

  // Rectification params
  cv::Mat map1_L, map2_L, map1_R, map2_R;
  cv::Rect roi_L, roi_R, common_roi;

  int contImageLeft, contImageRight, contTrackStereo;
  double firstTimeStampLeft, lastTimeStampLeft;

  // Left and right timestamp
  double tImLeft, tImRight;

  // Point cloud and Key points varables/methods
  std::vector<float> depths;
  sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
      std::vector<ORB_SLAM3::MapPoint *> map_points, rclcpp::Time msg_time,
      Eigen::Vector3f actualPosition);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisherPointCloud;
  cv::Scalar interpolateColor(float value, float minDepth, float maxDepth);
};
#endif
