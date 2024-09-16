#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/transform_datatypes.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize publishers
  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  // Initialize message filters subscribers with QoS settings
  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));

  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  auto laser_cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto laser_cloud_in_sensor_frame = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::fromROSMsg(*pcd_msg, *laser_cloud_in);

  geometry_msgs::msg::TransformStamped chassis_to_lidar;
  try {
    chassis_to_lidar = tf_buffer_->lookupTransform(
      "right_mid360", "chassis", pcd_msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_chassis_to_lidar, tf_odom_to_lidar, tf_odom_to_chassis;
  tf2::fromMsg(chassis_to_lidar.transform, tf_chassis_to_lidar);
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  // Calculate odom -> chassis transform
  tf_odom_to_chassis = tf_odom_to_lidar * tf_chassis_to_lidar;

  // Publish transform (odom -> chassis)
  geometry_msgs::msg::TransformStamped odom_to_chassis_msg;
  odom_to_chassis_msg.header.stamp = pcd_msg->header.stamp;
  odom_to_chassis_msg.header.frame_id = "odom";
  odom_to_chassis_msg.child_frame_id = "chassis";
  odom_to_chassis_msg.transform = tf2::toMsg(tf_odom_to_chassis);
  br_->sendTransform(odom_to_chassis_msg);

  // Publish odometry message (odom -> chassis)
  nav_msgs::msg::Odometry chassis_odometry_msg;
  chassis_odometry_msg.header.stamp = pcd_msg->header.stamp;
  chassis_odometry_msg.header.frame_id = "odom";
  chassis_odometry_msg.child_frame_id = "chassis";
  chassis_odometry_msg.pose.pose.position.x = tf_odom_to_chassis.getOrigin().x();
  chassis_odometry_msg.pose.pose.position.y = tf_odom_to_chassis.getOrigin().y();
  chassis_odometry_msg.pose.pose.position.z = tf_odom_to_chassis.getOrigin().z();
  chassis_odometry_msg.pose.pose.orientation = tf2::toMsg(tf_odom_to_chassis.getRotation());
  pub_chassis_odometry_->publish(chassis_odometry_msg);

  // Convert tf_odom_to_lidar to Eigen::Matrix4f
  Eigen::Matrix4f transform_matrix;
  pcl_ros::transformAsMatrix(tf_odom_to_lidar.inverse(), transform_matrix);

  // Transform point cloud (odom -> lidar)
  pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_in_sensor_frame, transform_matrix);

  // Publish the point cloud in lidar frame
  sensor_msgs::msg::PointCloud2 scan_data;
  pcl::toROSMsg(*laser_cloud_in_sensor_frame, scan_data);
  scan_data.header.stamp = pcd_msg->header.stamp;
  scan_data.header.frame_id = "right_mid360";  // lidar frame
  pub_laser_cloud_->publish(scan_data);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
