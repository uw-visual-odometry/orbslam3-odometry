#include "stereo.hpp"

#include <chrono>
#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

// If defined the node will print debug information and will show disparity map
#define DEBUG
#define PUBLISH_POINT_CLOUD

using std::placeholders::_1;
using std::placeholders::_2;

void StereoSlamNode::loadParameters() {
  /* ***** DECLARING PARAMETERS ***** */

  declare_parameter("topic_camera_left", "/camera/left_image");
  declare_parameter("topic_camera_right", "/camera/right_image");
  declare_parameter("topic_camera_info_left", "/camera/left_camera_info");
  declare_parameter("topic_camera_info_right", "/camera/right_camera_info");

  declare_parameter("topic_orbslam_odometry", "/Odometry/orbSlamOdom");
  declare_parameter("topic_header_frame_id", "os_track");
  declare_parameter("topic_child_frame_id", "orbslam3");
  declare_parameter("cropping_x", -1);
  declare_parameter("cropping_y", 0);
  declare_parameter("cropping_width", 0);
  declare_parameter("cropping_height", 0);

  declare_parameter("gamma_correction", 1.0);

  /* ******************************** */

  /* ***** READING PARAMETERS ***** */

  get_parameter("topic_camera_left", this->camera_left_topic);
  get_parameter("topic_camera_right", this->camera_right_topic);
  get_parameter("topic_camera_info_left", this->camera_left_info_topic);
  get_parameter("topic_camera_info_right", this->camera_right_info_topic);

  get_parameter("topic_orbslam_odometry", this->topic_pub_quat);
  get_parameter("topic_header_frame_id", this->header_id_frame);
  get_parameter("topic_child_frame_id", this->child_id_frame);
  get_parameter("cropping_x", this->cropping_x);
  get_parameter("cropping_y", this->cropping_y);
  get_parameter("cropping_width", this->cropping_width);
  get_parameter("cropping_height", this->cropping_height);

  get_parameter("gamma_correction", this->image_gamma_);
}

StereoSlamNode::StereoSlamNode()
    : MainNode("orbslam3_odometry"), track_stereo_count_(0), image_count_(0) {
  // Load parameters
  this->loadParameters();

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Loading settings from " << path_settings);

  m_SLAM = std::make_shared<ORB_SLAM3::System>(path_vocabulary, path_settings,
                                               ORB_SLAM3::System::STEREO,
                                               pangolin_visualization);

  RCLCPP_WARN(this->get_logger(), "Created ORB_SLAM3::System");

  // Compute cropping rect
  if (cropping_x != -1) {
    cropping_rect =
        cv::Rect(cropping_x, cropping_y, cropping_width, cropping_height);
  }

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Topic camera left: " << this->camera_left_topic);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Topic camera right: " << this->camera_right_topic);

  RCLCPP_INFO_STREAM(this->get_logger(), "Topic imu: " << this->imu);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "header_id_frame: " << this->header_id_frame);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "child_id_frame: " << this->child_id_frame);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "topic_orbslam_odometry: " << this->topic_pub_quat);

  // Images subscriptions and odometry publication

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  const auto qos_profile = qos.get_rmw_qos_profile();

  subscription_left.subscribe(this, this->camera_left_topic, qos_profile);
  subscription_right.subscribe(this, this->camera_right_topic, qos_profile);

  subscription_left_info.subscribe(this, this->camera_left_info_topic,
                                   qos_profile);
  subscription_right_info.subscribe(this, this->camera_right_info_topic,
                                    qos_profile);

  uint32_t queue_size = 10;
  sync_ = std::make_shared<StereoImageSync>(
      message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                      CameraInfo, CameraInfo>(
          queue_size),
      subscription_left, subscription_right, subscription_left_info,
      subscription_right_info);

  sync_->setAgePenalty(1.00);
  sync_->registerCallback(std::bind(
      &StereoSlamNode::syncedCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  quaternion_pub =
      this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "ORB-SLAM3 STARTED IN STEREO MODE. NODE WILL WAIT FOR IMAGES IN "
      "TOPICS "
          << this->camera_left_topic << " and " << this->camera_right_topic);
  Utility::printCommonInfo(qos);

#ifdef PUBLISH_POINT_CLOUD
  publisherPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/pointCloud", 10);
  // RCLCPP_INFO_STREAM(this->get_logger(), "PUBLISH_POINT_CLOUD");
#endif

  // Depth node
  publisherDepth = this->create_publisher<sensor_msgs::msg::Range>("/Depth", 10);
  // // Starts orbslam3. This thread is used to syncronize the two images
  // syncThread_ = new std::thread(&StereoSlamNode::SyncImg, this);
}

StereoSlamNode::~StereoSlamNode() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Number of images pairs arrived: "
                                             << std::to_string(image_count_));
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Number of TrackStereo calls: " << std::to_string(track_stereo_count_));

  // Stop all threads
  if (m_SLAM) {
    m_SLAM->Shutdown();

    // Save camera and keyframe trajectory
    RCLCPP_INFO(this->get_logger(), "Exit and saving... ");
    m_SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Saved KeyFrameTrajectory.txt ");
    RCLCPP_INFO(this->get_logger(), "Saved CameraTrajectory.txt ");
  }
}

/**
 * Function that returns a string with quaternion, each value is separated by a
 * space.
 */
static std::string quaternionToString(const Eigen::Quaternionf &q) {
  std::stringstream ss;
  ss << setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
     << endl;
  return ss.str();
}

void StereoSlamNode::syncedCallback(
    const ImageMsg::ConstSharedPtr &left_image_msg,
    const ImageMsg::ConstSharedPtr &right_image_msg,
    const CameraInfo::ConstSharedPtr &left_info,
    const CameraInfo::ConstSharedPtr &right_info) {
  cv::Mat left_image, right_image;

  double t = left_image_msg->header.stamp.sec +
             (left_image_msg->header.stamp.nanosec / 1e9);
  try {
    left_image = cv_bridge::toCvCopy(left_image_msg, "bgr8")->image;
    cv::flip(left_image, left_image, -1); //rotates image by 180

    timestamp = Utility::StampToSec(left_image_msg->header.stamp);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  try {
    right_image = cv_bridge::toCvCopy(right_image_msg, "bgr8")->image;
    cv::flip(right_image, right_image, -1); //rotates image by 180
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  image_count_++;

  // Initial time
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  cv::Mat left_rectified_non_cropped, right_rectified_non_cropped;

  // If necessary, perform changes to the images here.
  if (cropping_x != -1) {
    Utility::cropping_image(left_image, cropping_rect);
    Utility::cropping_image(right_image, cropping_rect);
  }

  if (image_gamma_ != 1.0) {
    // If configured, apply gamma correction to incoming images
    cv::Mat left_postproc, right_postproc;

    left_image.convertTo(left_postproc, CV_32FC3, 1 / 255.0);
    right_image.convertTo(right_postproc, CV_32FC3, 1 / 255.0);

    cv::pow(left_postproc, image_gamma_, left_postproc);
    cv::pow(right_postproc, image_gamma_, right_postproc);

    left_postproc.convertTo(left_image, CV_8UC3, 255);
    right_postproc.convertTo(right_image, CV_8UC3, 255);
  }

  // Call ORB-SLAM3 on the 2 original images
  const Sophus::SE3f Tcw =
      m_SLAM->TrackStereo(left_image, right_image, timestamp);

  // Invert the tracking motion to get body motion (?)
  // Obtain the position and the quaternion
  const Sophus::SE3f Twc = Tcw.inverse();
  const Eigen::Vector3f twc = Twc.translation();
  const Eigen::Quaternionf q = Twc.unit_quaternion();

  // publish position and quaternion (rotated)
  auto message = nav_msgs::msg::Odometry();
  geometry_msgs::msg::Pose output_pose{};

  output_pose.position.x = twc.x();
  output_pose.position.y = twc.y();
  output_pose.position.z = twc.z();

  output_pose.orientation.x = q.x();
  output_pose.orientation.y = q.y();
  output_pose.orientation.z = q.z();
  output_pose.orientation.w = q.w();

  // Set Roll and Pitch to Zero
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(output_pose.orientation, tf2_quat);

  double roll, pitch, yaw;
  tf2::Matrix3x3 m(tf2_quat);
  m.getRPY(roll, pitch, yaw);
  tf2_quat.setRPY(0, 0, yaw);
  output_pose.orientation = tf2::toMsg(tf2_quat);

  message.pose.pose = output_pose;
  message.header.frame_id = header_id_frame;
  message.child_frame_id = child_id_frame;
  message.header.stamp = this->now();

  // define twist value
  // Place these as class member variables or static variables in the function
  static Sophus::SE3f last_Twc;
  static rclcpp::Time last_time;
  static bool first_frame = true;
  // Current timestamp
  rclcpp::Time curr_time = this->now();

  // Compute velocity if not the first frame
  if (!first_frame)
  {
      double dt = (curr_time - last_time).seconds();
      if (dt > 0.0)
      {
          // Linear velocity (world frame difference)
          Eigen::Vector3f pos_curr = Twc.translation();
          Eigen::Vector3f pos_prev = last_Twc.translation();
          Eigen::Vector3f v_linear = (pos_curr - pos_prev) / dt;

          // Angular velocity from relative rotation
          Sophus::SO3f R_curr = Twc.so3();
          Sophus::SO3f R_prev = last_Twc.so3();
          Sophus::SO3f dR = R_prev.inverse() * R_curr;
          Eigen::Vector3f rot_vec = dR.log();  // rotation vector (axis-angle)
          Eigen::Vector3f v_angular = rot_vec / dt;

          // Populate the twist fields
          message.twist.twist.linear.x = v_linear.x();
          message.twist.twist.linear.y = v_linear.y();
          message.twist.twist.linear.z = v_linear.z();

          message.twist.twist.angular.x = v_angular.x();
          message.twist.twist.angular.y = v_angular.y();
          message.twist.twist.angular.z = v_angular.z();
      }
  }
  else
  {
      first_frame = false;
  }

  // Update previous values
  last_Twc = Twc;
  last_time = curr_time;

  // if ((image_count_ % 2) == 0) {
  //   double t_1 = timestamp;
  //   Eigen::Vector3d pos_1 = twc;  // From Tcw at t1
  // } else {
  //   double t_2 = timestamp;
  //   Eigen::Vector3d pos_2 = twc;  // From Tcw at t2
  // }
  // double dt = std::abs(t1 - t2);  // In seconds
  // Eigen::Vector3d dpos = std::abs(pos_1 - pos_2);  // In seconds
  // Eigen::Vector3d v_linear = dpos / dt;
  // message.twist.twist.linear.x = v_linear.x();
  // message.twist.twist.linear.y = v_linear.y();
  // message.twist.twist.linear.z = v_linear.z();

  // message.twist.twist.angular.x = 2.0;
  // message.twist.twist.angular.y = 2.0;
  // message.twist.twist.angular.z = 2.0;

  quaternion_pub->publish(message);

  // "End" time and saving times. File with times:
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double tempo =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 -
                                                                            t1)
          .count();
  m_SLAM->InsertTrackTime(tempo);


  //RCLCPP_INFO(this->get_logger(), "prima del mappiont to pointcloud");
  depths.clear();
  sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(
      m_SLAM->GetTrackedMapPoints(), message.header.stamp, twc);
  publisherPointCloud->publish(cloud);
  //RCLCPP_INFO(this->get_logger(), "dopo il mappiont to pointcloud");


  // Depth publish
  sensor_msgs::msg::Range depth = pointcloud_to_depth(
      cloud);
  publisherDepth->publish(depth);
}

// Key point color interpolation
cv::Scalar StereoSlamNode::interpolateColor(float value, float minDepth,
                                            float maxDepth) {
  // Doesn't working
  float range = maxDepth - minDepth;
  float normalized = (value - minDepth) / range;

  // Cold color (blue)
  cv::Vec3b coldColor(255, 0, 0);
  // Warm color (red)
  cv::Vec3b warmColor(0, 0, 255);

  cv::Vec3b color = (1 - normalized) * coldColor + normalized * warmColor;
  return cv::Scalar(color[0], color[1], color[2]);
}

// Converter from MapPoint to Point Cloud
sensor_msgs::msg::PointCloud2 StereoSlamNode::mappoint_to_pointcloud(
    std::vector<ORB_SLAM3::MapPoint *> map_points, rclcpp::Time msg_time,
    Eigen::Vector3f actualPosition) {
  const int num_channels = 3;  // x y z

  // if (map_points.size() == 0) {
  //   std::cout << "Map point vector is empty!" << std::endl;
  // }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id =
      "/camera";  // So it can be shown with lidar's velodyne
  cloud.height = 1;
  cloud.width = map_points.size();
  std::cout << "Size map point: " << map_points.size() << std::endl;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};

  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points[i]) {
      // map_points[i]->SetWorldPos(vectorWorldPos );
      // Eigen::Vector3f tmp = map_points[i]->GetWorldPos();
      // std::cout << "Vector world pos: " << tmp.x() << " " << tmp.y() << " "
      // << tmp.z() << std::endl;
      Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

      tf2::Vector3 point_translation(P3Dw.x() - actualPosition.x(),
                                     P3Dw.y() - actualPosition.y(),
                                     P3Dw.z() - actualPosition.z());

      float data_array[num_channels] = {point_translation.z(),
                                        -point_translation.x(),
                                        -point_translation.y()};

      depths.push_back(data_array[0]);

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
             num_channels * sizeof(float));
    }
  }
  return cloud;
}

// sensor_msgs::msg::Range StereoSlamNode::pointcloud_to_depth_old(
//     sensor_msgs::msg::PointCloud2 cloud) {

//   auto depth = sensor_msgs::msg::Range();
//   pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//   pcl_conversions::toPCL(cloud, pcl_cloud);
//   for (const auto& point : pcl_cloud.points) {
//       float x = point.x;
//       float y = point.y;
//       float z = point.z;
//       // Print x, y, z coordinates
//       std::cout << "x: " << x << "y: " << y << "z: " << z << std::endl;
//   }
  
//   return depth;
//  }

sensor_msgs::msg::Range StereoSlamNode::pointcloud_to_depth(
    sensor_msgs::msg::PointCloud2 cloud) {
  
  auto depth = sensor_msgs::msg::Range();
  
  depth.header = cloud.header;
  depth.radiation_type = sensor_msgs::msg::Range::INFRARED;
  depth.field_of_view = 0.5;
  
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);
  
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float sum_x = 0.0;
  int valid_points = 0;
  
  for (const auto& point : pcl_cloud.points) {
    if (std::isfinite(point.x) && point.x > 0) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      sum_x += point.x;
      valid_points++;
    }
  }
  
  if (valid_points > 0) {
    depth.min_range = min_x;
    depth.max_range = max_x;
    depth.range = sum_x / valid_points;
  } else {
    depth.min_range = 0.0;
    depth.max_range = 0.0;
    depth.range = 0.0;
  }
  
  return depth;
}
