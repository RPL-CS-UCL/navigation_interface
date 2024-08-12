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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <iostream>

#include "navigation_interface/ros_params_helper.h"

using namespace navigation_interface;
using namespace message_filters;

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

class CMULocalizationInterface {
 public:
  CMULocalizationInterface(ros::NodeHandle &nh, ros::NodeHandle &nhp)
      : sub_odometry(nh, "/Odometry", 10),
        sub_pointcloud(nh, "/cloud_registered_body", 10),
        sync(SyncPolicy(10), sub_odometry, sub_pointcloud) {
    // clang-format off
    loc_world_frame_id = get_ros_param(nhp, "loc_world_frame_id", std::string("fastlio_world"));
    loc_pointcloud_frame_id = get_ros_param(nhp, "loc_pointcloud_frame_id", std::string("fastlio_body"));

    world_frame_id = get_ros_param(nhp, "world_frame_id", std::string("map"));  // the world frame of the base
    pointcloud_frame_id = get_ros_param(nhp, "pointcloud_frame_id", std::string("imu_link"));
    base_frame_id = get_ros_param(nhp, "base_frame_id", std::string("base"));
    near_point_filter_length = get_ros_param(nhp, "near_point_filter_length", 0.9);
    near_point_filter_width = get_ros_param(nhp, "near_point_filter_width", 0.6);

    path_msg.poses.clear();
    path_msg.poses.reserve(10000);

#ifdef DEBUG
    std::cout << "loc_world_frame_id: " << loc_world_frame_id << std::endl;
    std::cout << "loc_pointcloud_frame_id: " << loc_pointcloud_frame_id << std::endl;
    std::cout << "world_frame_id: " << world_frame_id << std::endl;
    std::cout << "pointcloud_frame_id: " << pointcloud_frame_id << std::endl;
    std::cout << "base_frame_id: " << base_frame_id << std::endl;
    std::cout << "near_point_filter_length: " << near_point_filter_length << std::endl;
    std::cout << "near_point_filter_width: " << near_point_filter_width << std::endl;
#endif

    pub_state_estimation = nh.advertise<nav_msgs::Odometry>("/state_estimation", 1);
    pub_registered_scan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/state_estimation_path", 1);

    tf_listener.reset(new tf::TransformListener);

    sync.registerCallback(boost::bind(&CMULocalizationInterface::callback, this, _1, _2));
    // clang-format on
  }

  void callback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
#ifdef DEBUG
    std::cout << "Synchronized callback triggered" << std::endl;
#endif
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
        return;
      }
    }

    if (init_system) {
      Eigen::Matrix4d T_world_base_base = processOdometry(odom_msg);
      processPointCloud(cloud_msg, T_world_base_base);
    }
  }

 private:
  Eigen::Matrix4d processOdometry(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef DEBUG
    std::cout << "Processing odometry" << std::endl;
#endif
    Eigen::Matrix4d T_world_pc_pc = Eigen::Matrix4d::Identity();
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    T_world_pc_pc.block<3, 1>(0, 3) = position;
    Eigen::Quaterniond quat(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    T_world_pc_pc.block<3, 3>(0, 0) = quat.toRotationMatrix();
    Eigen::Matrix4d T_world_base_base =
        T_base_pointcloud * T_world_pc_pc * T_base_pointcloud.inverse();

    nav_msgs::Odometry transformed_odom;
    transformed_odom.header.stamp = msg->header.stamp;
    transformed_odom.header.frame_id = world_frame_id;
    transformed_odom.child_frame_id = base_frame_id;
    transformed_odom.pose.pose.position.x = T_world_base_base(0, 3);
    transformed_odom.pose.pose.position.y = T_world_base_base(1, 3);
    transformed_odom.pose.pose.position.z = T_world_base_base(2, 3);
    Eigen::Quaterniond transformed_quat(T_world_base_base.block<3, 3>(0, 0));
    transformed_odom.pose.pose.orientation.w = transformed_quat.w();
    transformed_odom.pose.pose.orientation.x = transformed_quat.x();
    transformed_odom.pose.pose.orientation.y = transformed_quat.y();
    transformed_odom.pose.pose.orientation.z = transformed_quat.z();
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

    Eigen::Matrix4d T_base_world_base = T_world_base_base.inverse();
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(convertEigenMatrixToTransform(T_base_world_base),
                             msg->header.stamp, base_frame_id, world_frame_id));
    return T_world_base_base;
  }

  void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                         const Eigen::Matrix4d &T_world_base_base) {
#ifdef DEBUG
    std::cout << "Received pointcloud with " << cloud_msg->width << " points"
              << std::endl;
#endif
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(
        new pcl::PointCloud<PointType>());
    transformed_cloud->reserve(cloud->size());

    for (const auto &p : cloud->points) {
      Eigen::Vector3d point(p.x, p.y, p.z);
      // conver the point in the lidar frame to the base frame
      Eigen::Vector3d point_base = T_base_pointcloud.block<3, 3>(0, 0) * point +
                                   T_base_pointcloud.block<3, 1>(0, 3);

      if ((abs(point_base.x()) > near_point_filter_length) &&
          (abs(point_base.y()) > near_point_filter_width) &&
          (abs(point_base.z()) > 0.5)) {
        // conver the point in the base frame to the world base frame
        Eigen::Vector3d point_world_base =
            T_world_base_base.block<3, 3>(0, 0) * point_base +
            T_world_base_base.block<3, 1>(0, 3);
        PointType p_world = p;
        p_world.x = static_cast<float>(point_world_base.x());
        p_world.y = static_cast<float>(point_world_base.y());
        p_world.z = static_cast<float>(point_world_base.z());
        transformed_cloud->push_back(p_world);
      }
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*transformed_cloud, pc_msg);
    pc_msg.header.stamp = cloud_msg->header.stamp;
    pc_msg.header.frame_id = world_frame_id;
    pub_registered_scan.publish(pc_msg);
  }

  // ROS
  // clang-format off
  Subscriber<nav_msgs::Odometry> sub_odometry;
  Subscriber<sensor_msgs::PointCloud2> sub_pointcloud;
  typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;
  Synchronizer<SyncPolicy> sync;
  // clang-format on

  ros::Publisher pub_state_estimation;
  ros::Publisher pub_registered_scan;
  ros::Publisher pub_path;

  std::unique_ptr<tf::TransformListener> tf_listener;

  nav_msgs::Path path_msg;

  // Parameters
  std::string loc_world_frame_id, loc_pointcloud_frame_id;
  std::string world_frame_id, base_frame_id, pointcloud_frame_id;
  double near_point_filter_length = 0.9;
  double near_point_filter_width = 0.6;

  // Others
  bool init_system = false;
  int odom_cnt = 0;
  Eigen::Matrix4d T_base_pointcloud = Eigen::Matrix4d::Identity();
};

class VISLocalizationInterface {
 public:
  VISLocalizationInterface(ros::NodeHandle &nh, ros::NodeHandle &nhp){
    // clang-format off
    sub_odometry = nh.subscribe("/Odometry", 10, &VISLocalizationInterface::OdomCallback, this);

    loc_world_frame_id = get_ros_param(nhp, "loc_world_frame_id", std::string("vloc_map"));
    loc_sensor_frame_id = get_ros_param(nhp, "loc_sensor_frame_id", std::string("sensor"));

    world_frame_id = get_ros_param(nhp, "world_frame_id", std::string("map"));      // the world frame of the base
    sensor_frame_id = get_ros_param(nhp, "sensor_frame_id", std::string("sensor")); // the frame id of the sensor
    base_frame_id = get_ros_param(nhp, "base_frame_id", std::string("base"));

    path_msg.poses.clear();
    path_msg.poses.reserve(10000);

#ifdef DEBUG
    std::cout << "loc_world_frame_id: " << loc_world_frame_id << std::endl;
    std::cout << "loc_sensor_frame_id: " << loc_sensor_frame_id << std::endl;
    std::cout << "world_frame_id: " << world_frame_id << std::endl;
    std::cout << "sensor_frame_id: " << sensor_frame_id << std::endl;
    std::cout << "base_frame_id: " << base_frame_id << std::endl;
#endif

    pub_state_estimation = nh.advertise<nav_msgs::Odometry>("/vloc/state_estimation", 1);
    pub_path = nh.advertise<nav_msgs::Path>("/vloc/state_estimation_path", 1);

    tf_listener.reset(new tf::TransformListener);
    // clang-format on
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
#ifdef DEBUG
    std::cout << "Odometry callback triggered" << std::endl;
#endif
    if (!init_system) {
      try {
        tf::StampedTransform transform_base_sensor;
        tf_listener->lookupTransform(base_frame_id, sensor_frame_id,
                                     ros::Time(0), transform_base_sensor);
        T_base_sensor = convertTransformToEigenMatrix(transform_base_sensor);
        init_system = true;
#ifdef DEBUG
        std::cout << "Initial system initialized" << std::endl;
        std::cout << "T_base_sensor:\n" << T_base_sensor << std::endl;
#endif
      } catch (tf::TransformException &ex) {
        return;
      }
    }
    if (init_system) {
      Eigen::Matrix4d T_world_base_base = processOdometry(odom_msg);
    }
  }

 private:
  Eigen::Matrix4d processOdometry(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef DEBUG
    std::cout << "Processing odometry" << std::endl;
#endif
    // This is the difference from the CMULocalizationInterface
    // CMULocalizationInterface: world_imu_link of the localization odometry should be changed to the world_base frame
    // VISLocalizationInterface: the localization odometry is already based on the world_base frame
    // NOTE(gogojjh): T_world_base_sensor: the transformation from the world frame to the sensor frame
    Eigen::Matrix4d T_world_base_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector3d position(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    T_world_base_sensor.block<3, 1>(0, 3) = position;
    Eigen::Quaterniond quat(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    T_world_base_sensor.block<3, 3>(0, 0) = quat.toRotationMatrix();
    // NOTE(gogojjh): T_world_base_base: the transformation from the world_base frame to the base frame
    Eigen::Matrix4d T_world_base_base = T_world_base_sensor * T_base_sensor.inverse();
    // Publish the odometry
    // TODO(gogojjh): convert the covariance to the odometry
    nav_msgs::Odometry transformed_odom;
    transformed_odom.header.stamp = msg->header.stamp;
    transformed_odom.header.frame_id = world_frame_id;
    transformed_odom.child_frame_id = base_frame_id;
    transformed_odom.pose.pose.position.x = T_world_base_base(0, 3);
    transformed_odom.pose.pose.position.y = T_world_base_base(1, 3);
    transformed_odom.pose.pose.position.z = T_world_base_base(2, 3);
    Eigen::Quaterniond transformed_quat(T_world_base_base.block<3, 3>(0, 0));
    transformed_odom.pose.pose.orientation.w = transformed_quat.w();
    transformed_odom.pose.pose.orientation.x = transformed_quat.x();
    transformed_odom.pose.pose.orientation.y = transformed_quat.y();
    transformed_odom.pose.pose.orientation.z = transformed_quat.z();
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

    Eigen::Matrix4d T_base_world_base = T_world_base_base.inverse();
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(convertEigenMatrixToTransform(T_base_world_base),
                             msg->header.stamp, base_frame_id, world_frame_id));
    return T_world_base_base;
  }

  // ROS
  // clang-format off
  ros::Subscriber sub_odometry;
  // clang-format on

  ros::Publisher pub_state_estimation;
  ros::Publisher pub_path;
  nav_msgs::Path path_msg;

  std::unique_ptr<tf::TransformListener> tf_listener;

  // Parameters
  std::string loc_world_frame_id, loc_sensor_frame_id;
  std::string world_frame_id, base_frame_id, sensor_frame_id;

  // Others
  bool init_system = false;
  int odom_cnt = 0;
  Eigen::Matrix4d T_base_sensor = Eigen::Matrix4d::Identity();
};
