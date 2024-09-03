#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "navigation_interface/ros_params_helper.h"

using namespace navigation_interface;
using namespace message_filters;

typedef pcl::PointXYZI PointType;

const float kMinDepth = 1e-6;
const float kMaxDepth = 30.0;

Eigen::Matrix4d convertTransformToEigenMatrix(
    const tf::StampedTransform &transform) {
  tf::Quaternion q = transform.getRotation();
  Eigen::Quaterniond eigen_quat(q.w(), q.x(), q.y(), q.z());

  tf::Vector3 v = transform.getOrigin();
  Eigen::Vector3d translation(v.x(), v.y(), v.z());

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  transformation_matrix.block<3, 3>(0, 0) = eigen_quat.toRotationMatrix();
  transformation_matrix.block<3, 1>(0, 3) = translation;
  return transformation_matrix;
}

tf::Transform convertEigenMatrixToTransform(const Eigen::Matrix4d &matrix) {
  Eigen::Quaterniond eigen_quat(matrix.block<3, 3>(0, 0));
  eigen_quat.normalize();

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(matrix(0, 3), matrix(1, 3), matrix(2, 3)));
  tf::Quaternion q;
  q.setW(eigen_quat.w());
  q.setX(eigen_quat.x());
  q.setY(eigen_quat.y());
  q.setZ(eigen_quat.z());
  transform.setRotation(q);
  return transform;
}

void CreateDepthCloud(pcl::PointCloud<PointType>::Ptr& depth_cloud, 
                      const cv::Mat& depth_img_float,
                      const Eigen::Matrix3f& camera_intrinsics) {
  if (!depth_img_float.empty()) {
    depth_cloud->reserve(depth_img_float.rows * depth_img_float.cols);
    for (int y = 0; y < depth_img_float.rows; y++) {
      for (int x = 0; x < depth_img_float.cols; x++) {
        const float& depth = depth_img_float.at<float>(y, x);
        if (depth < kMinDepth || depth > kMaxDepth) continue;
        Eigen::Vector3f pts_cam;
        pts_cam[2] = depth;
        pts_cam[0] = depth * (static_cast<float>(x) - camera_intrinsics(0, 2)) / camera_intrinsics(0, 0);
        pts_cam[1] = depth * (static_cast<float>(y) - camera_intrinsics(1, 2)) / camera_intrinsics(1, 1);
        PointType point;
        point.x = pts_cam.x();
        point.y = pts_cam.y();
        point.z = pts_cam.z();
        point.intensity = 0.0;
        depth_cloud->push_back(point);
      }
    }
  }
}

class CMULocalizationInterface {
 public:
  CMULocalizationInterface(ros::NodeHandle &nh, ros::NodeHandle &nhp):
    sub_odometry(nh, "/Odometry", 10),
    sub_pointcloud(nh, "/cloud_registered_body", 10),
    sync(SyncPolicy(10), sub_odometry, sub_pointcloud)
  {
    // clang-format off
    // Parameters
    loc_world_frame_id = get_ros_param(nhp, "loc_world_frame_id", std::string("fastlio_world"));
    loc_sensor_frame_id = get_ros_param(nhp, "loc_pointcloud_frame_id", std::string("fastlio_body"));
    world_frame_id = get_ros_param(nhp, "world_frame_id", std::string("map"));  // the world frame of the base
    sensor_frame_id = get_ros_param(nhp, "pointcloud_frame_id", std::string("imu_link"));
    base_frame_id = get_ros_param(nhp, "base_frame_id", std::string("base"));
    near_point_filter_length = get_ros_param(nhp, "near_point_filter_length", 0.9);
    near_point_filter_width = get_ros_param(nhp, "near_point_filter_width", 0.6);
    // ROS
    sync.registerCallback(boost::bind(&CMULocalizationInterface::callback, this, _1, _2));
    pub_state_estimation = nh.advertise<nav_msgs::Odometry>("/state_estimation", 1);
    pub_registered_scan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/state_estimation_path", 1);
    tf_listener.reset(new tf::TransformListener);
    path_msg.poses.clear();
    path_msg.poses.reserve(10000);
    // clang-format on
  }

  void callback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    if (!init_system) {
      try {
        tf::StampedTransform transform_base_sensor;
        tf_listener->lookupTransform(base_frame_id, sensor_frame_id,
                                     ros::Time(0), transform_base_sensor);
        T_base_sensor = convertTransformToEigenMatrix(transform_base_sensor);
        init_system = true;
      } catch (tf::TransformException &ex) {
        return;
      }
    }
    if (init_system) {
      Eigen::Matrix4d T_world_base = processOdometry(odom_msg, T_base_sensor);
      pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(*cloud_msg, *cloud);      
      processPointCloud(cloud, cloud_msg->header.stamp, T_base_sensor, T_world_base);
    }
  }

  // world_sensor of the localization odometry should be changed to the world_base frame
  // CMULocalizationInterface: world_imu_link is the frame_id of the localization odometry (LIO)
  Eigen::Matrix4d processOdometry(const nav_msgs::Odometry::ConstPtr &msg, 
                                  const Eigen::Matrix4d &T_base_sensor) {
    Eigen::Matrix4d T_world_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    T_world_sensor.block<3, 1>(0, 3) = position;
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    T_world_sensor.block<3, 3>(0, 0) = quat.toRotationMatrix();
    Eigen::Matrix4d T_world_base = T_base_sensor * T_world_sensor * T_base_sensor.inverse();

    nav_msgs::Odometry transformed_odom;
    transformed_odom.header.stamp = msg->header.stamp;
    transformed_odom.header.frame_id = world_frame_id;
    transformed_odom.child_frame_id = base_frame_id;
    transformed_odom.pose.pose.position.x = T_world_base(0, 3);
    transformed_odom.pose.pose.position.y = T_world_base(1, 3);
    transformed_odom.pose.pose.position.z = T_world_base(2, 3);
    Eigen::Quaterniond transformed_quat(T_world_base.block<3, 3>(0, 0));
    transformed_odom.pose.pose.orientation.w = transformed_quat.w();
    transformed_odom.pose.pose.orientation.x = transformed_quat.x();
    transformed_odom.pose.pose.orientation.y = transformed_quat.y();
    transformed_odom.pose.pose.orientation.z = transformed_quat.z();
    for (size_t i = 0; i < 36; i++) transformed_odom.pose.covariance[i] = msg->pose.covariance[i];
    pub_state_estimation.publish(transformed_odom);
    odom_cnt++;

    if (odom_cnt % 10 == 0) {
      odom_cnt = 0;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = msg->header.stamp;
      pose_stamped.header.frame_id = world_frame_id;
      pose_stamped.pose = transformed_odom.pose.pose;

      path_msg.header.stamp = msg->header.stamp;
      path_msg.header.frame_id = world_frame_id;
      path_msg.poses.push_back(pose_stamped);
      pub_path.publish(path_msg);
    }

    Eigen::Matrix4d T_base_world = T_world_base.inverse();
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(convertEigenMatrixToTransform(T_base_world),
                              msg->header.stamp, base_frame_id, world_frame_id));
    return T_world_base;
  }

  void processPointCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud,
                         const ros::Time &timestamp,
                         const Eigen::Matrix4d &T_base_sensor,
                         const Eigen::Matrix4d &T_world_base) {
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
    transformed_cloud->reserve(size_t(cloud->size() / point_skip));

    for (size_t i = 0; i < cloud->size(); i+=point_skip) {
      const auto &p = cloud->points[i];
      Eigen::Vector3d point(p.x, p.y, p.z);
      // conver the point in the lidar frame to the base frame
      Eigen::Vector3d point_base = T_base_sensor.block<3, 3>(0, 0) * point + 
                                   T_base_sensor.block<3, 1>(0, 3);

      if ((abs(point_base.x()) < near_point_filter_length) && 
          (abs(point_base.y()) < near_point_filter_width) &&
          (abs(point_base.z()) < 0.5))
        continue;

      // conver the point in the base frame to the world base frame
      Eigen::Vector3d point_world_base =
        T_world_base.block<3, 3>(0, 0) * point_base + T_world_base.block<3, 1>(0, 3);
      PointType p_world = p;
      p_world.x = static_cast<float>(point_world_base.x());
      p_world.y = static_cast<float>(point_world_base.y());
      p_world.z = static_cast<float>(point_world_base.z());
      p_world.intensity = 0.0;
      transformed_cloud->push_back(p_world);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*transformed_cloud, pc_msg);
    pc_msg.header.stamp = timestamp;
    pc_msg.header.frame_id = world_frame_id;
    pub_registered_scan.publish(pc_msg);
  }

 public:
  // ROS
  // clang-format off
  Subscriber<nav_msgs::Odometry> sub_odometry;
  Subscriber<sensor_msgs::PointCloud2> sub_pointcloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync;  

  ros::Publisher pub_state_estimation;
  ros::Publisher pub_registered_scan;
  ros::Publisher pub_path;
  std::unique_ptr<tf::TransformListener> tf_listener;
  nav_msgs::Path path_msg;

  // Parameters
  std::string loc_world_frame_id, loc_sensor_frame_id;    // not used in processing 
  std::string world_frame_id, base_frame_id, sensor_frame_id;
  double near_point_filter_length = 0.9;
  double near_point_filter_width = 0.6;

  // Others
  bool init_system = false;
  Eigen::Matrix4d T_base_sensor = Eigen::Matrix4d::Identity();
  size_t odom_cnt = 0;
  int point_skip = 1;
  // clang-format on

};

class VISLocalizationInterface {
 public:
  VISLocalizationInterface(ros::NodeHandle &nh, ros::NodeHandle &nhp) :
    sub_odometry(nh, "/odometry", 10),
    sub_camera_info(nh, "/camera_info", 10),
    sub_depth_img(nh, "/depth_image", 10),
    sync(SyncPolicy(10), sub_odometry, sub_camera_info, sub_depth_img)
  {
    // clang-format off
    // Parameters
    loc_world_frame_id = get_ros_param(nhp, "loc_world_frame_id", std::string("world"));
    loc_sensor_frame_id = get_ros_param(nhp, "loc_sensor_frame_id", std::string("camera"));
    world_frame_id = get_ros_param(nhp, "world_frame_id", std::string("map"));  // the world frame of the base
    sensor_frame_id = get_ros_param(nhp, "sensor_frame_id", std::string("camera"));
    base_frame_id = get_ros_param(nhp, "base_frame_id", std::string("base"));
    near_point_filter_length = get_ros_param(nhp, "near_point_filter_length", 0.9);
    near_point_filter_width = get_ros_param(nhp, "near_point_filter_width", 0.6);
    point_skip = get_ros_param(nhp, "point_skip", 50);
    // ROS
    sync.registerCallback(boost::bind(&VISLocalizationInterface::callback, this, _1, _2, _3));
    sub_waypoint = nh.subscribe("/way_point", 10, &VISLocalizationInterface::WaypointCallback, this);
    
    pub_state_estimation = nh.advertise<nav_msgs::Odometry>("/state_estimation", 1);
    pub_registered_scan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/state_estimation_path", 1);
    tf_listener.reset(new tf::TransformListener);
    path_msg.poses.clear();
    path_msg.poses.reserve(10000);
    // clang-format on
  }

  void callback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                const sensor_msgs::CameraInfo::ConstPtr &cam_info_msg, 
                const sensor_msgs::Image::ConstPtr &depth_msg) {
    if (!init_system) {
      try {
        tf::StampedTransform transform_base_sensor;
        tf_listener->lookupTransform(base_frame_id, sensor_frame_id,
                                     depth_msg->header.stamp, transform_base_sensor);
        T_base_sensor = convertTransformToEigenMatrix(transform_base_sensor);
        init_system = true;
      } catch (tf::TransformException &ex) {
        return;
      }
    }

    try {
      cv::Mat depth_img =
          cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
      Eigen::Matrix3d K;
      K << cam_info_msg->K[0], cam_info_msg->K[1], cam_info_msg->K[2], cam_info_msg->K[3],
           cam_info_msg->K[4], cam_info_msg->K[5], cam_info_msg->K[6], cam_info_msg->K[7], cam_info_msg->K[8];
      Eigen::Matrix3f camera_intrinsics = K.cast<float>();

      pcl::PointCloud<PointType>::Ptr depth_cloud(new pcl::PointCloud<PointType>);
      CreateDepthCloud(depth_cloud, depth_img, camera_intrinsics);
      if (init_system) {
        Eigen::Matrix4d T_world_base = processOdometry(odom_msg, T_base_sensor);
        processPointCloud(depth_cloud, depth_msg->header.stamp, T_base_sensor, T_world_base);
      }
    } catch (cv_bridge::Exception& e) {
      std::cout << "cv_bridge exception: " << e.what() << std::endl;
    }
  }

  // world_sensor of the localization odometry should be changed to the world_base frame
  // VISLocalizationInterface: world(_device) (e.g., meta glass, GT sensor) is the frame_id of the localization odometry (VLOC)
  Eigen::Matrix4d processOdometry(const nav_msgs::Odometry::ConstPtr &msg, 
                                  const Eigen::Matrix4d &T_base_sensor) {
    Eigen::Matrix4d T_world_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    T_world_sensor.block<3, 1>(0, 3) = position;
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    T_world_sensor.block<3, 3>(0, 0) = quat.toRotationMatrix();
    Eigen::Matrix4d T_world_base = T_base_sensor * T_world_sensor * T_base_sensor.inverse();

    nav_msgs::Odometry transformed_odom;
    transformed_odom.header.stamp = msg->header.stamp;
    transformed_odom.header.frame_id = world_frame_id;
    transformed_odom.child_frame_id = base_frame_id;
    transformed_odom.pose.pose.position.x = T_world_base(0, 3);
    transformed_odom.pose.pose.position.y = T_world_base(1, 3);
    transformed_odom.pose.pose.position.z = T_world_base(2, 3);
    Eigen::Quaterniond transformed_quat(T_world_base.block<3, 3>(0, 0));
    transformed_odom.pose.pose.orientation.w = transformed_quat.w();
    transformed_odom.pose.pose.orientation.x = transformed_quat.x();
    transformed_odom.pose.pose.orientation.y = transformed_quat.y();
    transformed_odom.pose.pose.orientation.z = transformed_quat.z();
    for (size_t i = 0; i < 36; i++) transformed_odom.pose.covariance[i] = msg->pose.covariance[i];
    pub_state_estimation.publish(transformed_odom);
    odom_cnt++;

    if (odom_cnt % 10 == 0) {
      odom_cnt = 0;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = msg->header.stamp;
      pose_stamped.header.frame_id = world_frame_id;
      pose_stamped.pose = transformed_odom.pose.pose;

      path_msg.header.stamp = msg->header.stamp;
      path_msg.header.frame_id = world_frame_id;
      path_msg.poses.push_back(pose_stamped);
      pub_path.publish(path_msg);
    }

    Eigen::Matrix4d T_base_world = T_world_base.inverse();
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(convertEigenMatrixToTransform(T_base_world),
                              msg->header.stamp, base_frame_id, world_frame_id));
    return T_world_base;
  }

  void processPointCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud,
                         const ros::Time &timestamp,
                         const Eigen::Matrix4d &T_base_sensor,
                         const Eigen::Matrix4d &T_world_base) {
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
    transformed_cloud->reserve(size_t(cloud->size() / point_skip));

    for (size_t i = 0; i < cloud->size(); i+=point_skip) {
      const auto &p = cloud->points[i];
      Eigen::Vector3d point(p.x, p.y, p.z);
      // conver the point in the lidar frame to the base frame
      Eigen::Vector3d point_base = T_base_sensor.block<3, 3>(0, 0) * point + 
                                   T_base_sensor.block<3, 1>(0, 3);

      if ((abs(point_base.x()) < near_point_filter_length) && 
          (abs(point_base.y()) < near_point_filter_width) &&
          (abs(point_base.z()) < 0.5))
        continue;

      // conver the point in the base frame to the world base frame
      Eigen::Vector3d point_world_base =
        T_world_base.block<3, 3>(0, 0) * point_base + T_world_base.block<3, 1>(0, 3);
      PointType p_world = p;
      p_world.x = static_cast<float>(point_world_base.x());
      p_world.y = static_cast<float>(point_world_base.y());
      p_world.z = static_cast<float>(point_world_base.z());
      p_world.intensity = 0.0;
      transformed_cloud->push_back(p_world);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*transformed_cloud, pc_msg);
    pc_msg.header.stamp = timestamp;
    pc_msg.header.frame_id = world_frame_id;
    pub_registered_scan.publish(pc_msg);
  }

  void WaypointCallback(const geometry_msgs::PointStamped::ConstPtr &waypoint_msg) {
    if (init_system) {
      geometry_msgs::PointStamped new_waypoint = *waypoint_msg;
      new_waypoint.header.frame_id = world_frame_id;
      pub_waypoint.publish(new_waypoint);
    }
  }

 public:
  // ROS
  // clang-format off
  Subscriber<nav_msgs::Odometry> sub_odometry;
  Subscriber<sensor_msgs::CameraInfo> sub_camera_info;
  Subscriber<sensor_msgs::Image> sub_depth_img;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::CameraInfo, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync;
  ros::Subscriber sub_waypoint;

  ros::Publisher pub_state_estimation;
  ros::Publisher pub_registered_scan;
  ros::Publisher pub_path;
  ros::Publisher pub_waypoint;

  std::unique_ptr<tf::TransformListener> tf_listener;
  nav_msgs::Path path_msg;

  // Parameters
  std::string loc_world_frame_id, loc_sensor_frame_id;    // not used in processing 
  std::string world_frame_id, base_frame_id, sensor_frame_id;
  double near_point_filter_length = 0.9;
  double near_point_filter_width = 0.6;

  // Others
  bool init_system = false;
  Eigen::Matrix4d T_base_sensor = Eigen::Matrix4d::Identity();
  size_t odom_cnt = 0;
  int point_skip = 1;
  // clang-format on
};
