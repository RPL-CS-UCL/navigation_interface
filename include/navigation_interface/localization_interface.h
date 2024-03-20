#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <iostream>

#include "navigation_interface/ros_params_helper.h"

using namespace navigation_interface;

typedef pcl::PointXYZI PointType;

// #define DEBUG

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

class LocalizationInterface {
 public:
  LocalizationInterface(ros::NodeHandle &nh, ros::NodeHandle &nhp) {
    // clang-format off
    loc_world_frame_id = get_ros_param(nhp,  "loc_world_frame_id", std::string("fastlio_world"));
    loc_pointcloud_frame_id = get_ros_param(nhp, "loc_pointcloud_frame_id", std::string("fastlio_body"));

    world_frame_id = get_ros_param(nhp, "world_frame_id",std::string("map"));
    pointcloud_frame_id = get_ros_param(nhp, "pointcloud_frame_id", std::string("imu_link"));
    base_frame_id = get_ros_param(nhp, "base_frame_id", std::string("base"));
    near_point_filter_radius = get_ros_param(nhp, "near_point_filter_radius", 0.7);

    path_msg.poses.clear();
    path_msg.poses.reserve(10000);

    // NOTE(googjjh): transform from the gravity frame to the world frame of the localization
    T_gravity_transform.block<3, 3>(0, 0) = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).toRotationMatrix();
#ifdef DEBUG
    std::cout << "loc_world_frame_id: " << loc_world_frame_id << std::endl;
    std::cout << "loc_pointcloud_frame_id: " << loc_pointcloud_frame_id << std::endl;
    std::cout << "world_frame_id: " << world_frame_id << std::endl;
    std::cout << "pointcloud_frame_id: " << pointcloud_frame_id << std::endl;
    std::cout << "base_frame_id: " << base_frame_id << std::endl;
    std::cout << "near_point_filter_radius: " << near_point_filter_radius << std::endl;
#endif
    // clang-format on

    // clang-format off
    sub_odometry = nh.subscribe("/Odometry", 10, &LocalizationInterface::odomCallback, this);
    sub_pointcloud = nh.subscribe("/cloud_registered_body", 10, &LocalizationInterface::cloudCallback, this);

    pub_state_estimation = nh.advertise<nav_msgs::Odometry>("/state_estimation", 1);
    pub_registered_scan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/state_estimation_path", 1);

    tf_listener.reset(new tf::TransformListener);
    // clang-format on
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef DEBUG
    std::cout << "odomCallback" << std::endl;
#endif
    if (!init_system) return;

    // Republish Odometry
    Eigen::Matrix4d T_world_pointcloud = Eigen::Matrix4d::Identity();
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    T_world_pointcloud.block<3, 1>(0, 3) = position;
    Eigen::Quaterniond quat(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    T_world_pointcloud.block<3, 3>(0, 0) = quat.toRotationMatrix();
    Eigen::Matrix4d T_world_base =
        T_gravity_transform * T_world_pointcloud * T_base_pointcloud.inverse();

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
    pub_state_estimation.publish(transformed_odom);
    odom_cnt++;

    // Republish path
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

    // Republish TF
    static tf::TransformBroadcaster br;
    br.sendTransform(
      tf::StampedTransform(convertEigenMatrixToTransform(T_world_base), 
      msg->header.stamp,
      world_frame_id, 
      base_frame_id));
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
#ifdef DEBUG
    std::cout << "Received pointcloud with " << cloud_msg->width << " points"
              << std::endl;
#endif
    // Load extrinsics
    if (!init_system) {
      try {
        tf::StampedTransform transform_base_lidar;
        tf_listener->lookupTransform(base_frame_id, pointcloud_frame_id,
                                     ros::Time(0), transform_base_lidar);
        T_base_pointcloud = convertTransformToEigenMatrix(transform_base_lidar);
        init_system = true;
#ifdef DEBUG
        std::cout << "Initial system initialized" << std::endl;
        std::cout << "T_base_pointcloud:\n" << T_base_pointcloud << std::endl;
#endif
      } catch (tf::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        return;
      }
    }

    // Load localization poses
    tf::StampedTransform transform_world_pointcloud;
    Eigen::Matrix4d T_world_pointcloud = Eigen::Matrix4d::Identity();
    try {
      tf_listener->lookupTransform(loc_world_frame_id, loc_pointcloud_frame_id,
                                   cloud_msg->header.stamp,
                                   transform_world_pointcloud);
      T_world_pointcloud =
          convertTransformToEigenMatrix(transform_world_pointcloud);
#ifdef DEBUG
      std::cout << "T_world_pointcloud:\n" << T_world_pointcloud << std::endl;
#endif
    } catch (tf::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        return;
    }
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *cloud);


    // Transform the pointcloud and filter out points that are too close
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
    transformed_cloud->reserve(cloud->size());
    Eigen::Matrix4d T_world_base = T_gravity_transform * T_world_pointcloud * T_base_pointcloud.inverse();
    for (const auto &p : cloud->points) {
      Eigen::Vector3d point(p.x, p.y, p.z);
      Eigen::Vector3d point_base = T_base_pointcloud.block<3, 3>(0, 0) * point +
                                   T_base_pointcloud.block<3, 1>(0, 3);
      if (point_base.norm() > near_point_filter_radius) {
        Eigen::Vector3d point_world =
            T_world_base.block<3, 3>(0, 0) * point_base +
            T_world_base.block<3, 1>(0, 3);
        PointType p_world = p;
        p_world.x = static_cast<float>(point_world.x());
        p_world.y = static_cast<float>(point_world.y());
        p_world.z = static_cast<float>(point_world.z());
        transformed_cloud->push_back(p_world);
      }
    }
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    transformed_cloud_msg.header.stamp = cloud_msg->header.stamp;
    transformed_cloud_msg.header.frame_id = world_frame_id;
    pub_registered_scan.publish(transformed_cloud_msg);
  }

 private:
  // ROS
  ros::Subscriber sub_odometry;
  ros::Subscriber sub_pointcloud;

  ros::Publisher pub_state_estimation;
  ros::Publisher pub_registered_scan;
  ros::Publisher pub_path;

  std::unique_ptr<tf::TransformListener> tf_listener;

  nav_msgs::Path path_msg;

  // Parameters
  std::string loc_world_frame_id, loc_pointcloud_frame_id;
  std::string world_frame_id, base_frame_id, pointcloud_frame_id;
  double near_point_filter_radius = 0.7;

  // Others
  bool init_system = false;
  int odom_cnt = 0;
  Eigen::Matrix4d T_base_pointcloud = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_gravity_transform = Eigen::Matrix4d::Identity();
};