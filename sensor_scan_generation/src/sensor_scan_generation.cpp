#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/transform_datatypes.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options), gimbal_to_lidar_initialized_(false)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("vehicle_base_frame", "");
  this->declare_parameter<std::string>("vel_ref_frame", "");

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("vehicle_base_frame", vehicle_base_frame_);
  this->get_parameter("vel_ref_frame", vel_ref_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

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
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  auto laser_cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto laser_cloud_in_sensor_frame = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::fromROSMsg(*pcd_msg, *laser_cloud_in);

  if (!gimbal_to_lidar_initialized_) {
    try {
      tf_gimbal_to_lidar_ = getTransform(lidar_frame_, vel_ref_frame_, pcd_msg->header.stamp);
      gimbal_to_lidar_initialized_ = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  tf2::Transform tf_chassis_to_lidar;
  try {
    tf_chassis_to_lidar = getTransform(lidar_frame_, vehicle_base_frame_, pcd_msg->header.stamp);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  tf2::Transform tf_odom_to_chassis = tf_odom_to_lidar * tf_chassis_to_lidar;
  tf2::Transform tf_odom_to_gimbal = tf_odom_to_lidar * tf_gimbal_to_lidar_;

  // Publish transformations and odometry
  publishTransform(tf_odom_to_chassis, odometry_msg->header.frame_id, vehicle_base_frame_, pcd_msg->header.stamp);
  publishOdometry(tf_odom_to_gimbal, odometry_msg->header.frame_id, vel_ref_frame_, pcd_msg->header.stamp);

  // Transform point cloud (odom -> lidar)
  Eigen::Matrix4f transform_matrix;
  pcl_ros::transformAsMatrix(tf_odom_to_lidar.inverse(), transform_matrix);
  pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_in_sensor_frame, transform_matrix);

  // Publish point cloud in lidar frame
  sensor_msgs::msg::PointCloud2 scan_data;
  pcl::toROSMsg(*laser_cloud_in_sensor_frame, scan_data);
  scan_data.header.stamp = pcd_msg->header.stamp;
  scan_data.header.frame_id = lidar_frame_;
  pub_laser_cloud_->publish(scan_data);
}


tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    throw;
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, const std::string & frame_id, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame;
  odom_msg.pose.pose.position.x = transform.getOrigin().x();
  odom_msg.pose.pose.position.y = transform.getOrigin().y();
  odom_msg.pose.pose.position.z = transform.getOrigin().z();
  odom_msg.pose.pose.orientation = tf2::toMsg(transform.getRotation());
  pub_chassis_odometry_->publish(odom_msg);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
