#ifndef SENSOR_SCAN_GENERATION_H
#define SENSOR_SCAN_GENERATION_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

class SensorScan : public rclcpp::Node
{
public:
  SensorScan();

private:
  void laserCloudAndOdometryHandler(
    const nav_msgs::msg::Odometry::ConstSharedPtr odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_in_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_in_sensor_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_laser_cloud_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_chassis_odometry_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

#endif  // SENSOR_SCAN_GENERATION_H
