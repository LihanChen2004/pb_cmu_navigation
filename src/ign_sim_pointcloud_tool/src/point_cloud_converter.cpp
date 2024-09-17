#include "ign_sim_pointcloud_tool/point_cloud_converter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ign_sim_pointcloud_tool
{
PointCloudConverter::PointCloudConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("point_cloud_converter", options)
{
  this->declare_parameter<std::string>("pcd_topic", "livox/lidar");
  this->declare_parameter<int>("n_scan", 32);
  this->declare_parameter<int>("horizon_scan", 1875);
  this->declare_parameter<float>("ang_bottom", 7.0);
  this->declare_parameter<float>("ang_res_y", 1.0);

  pcd_topic_ = this->get_parameter("pcd_topic").as_string();
  n_scan_ = this->get_parameter("n_scan").as_int();
  horizon_scan_ = this->get_parameter("horizon_scan").as_int();
  ang_bottom_ = this->get_parameter("ang_bottom").as_double();
  ang_res_y_ = this->get_parameter("ang_res_y").as_double();

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pcd_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudConverter::lidarHandle, this, std::placeholders::_1));

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "Listening to lidar topic: %s", pcd_topic_.c_str());
}

void PointCloudConverter::lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<PointXYZIRT>::Ptr converted_pc(new pcl::PointCloud<PointXYZIRT>());

  pcl::fromROSMsg(*pc_msg, *original_pc);

  for (size_t point_id = 0; point_id < original_pc->points.size(); ++point_id) {
    PointXYZIRT new_point;
    const auto & old_point = original_pc->points[point_id];

    new_point.x = old_point.x;
    new_point.y = old_point.y;
    new_point.z = old_point.z;
    new_point.intensity = 0;

    float vertical_angle =
      std::atan2(new_point.z, std::sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) *
      180 / M_PI;
    int row_id = static_cast<int>((vertical_angle + ang_bottom_) / ang_res_y_);

    if (row_id >= 0 && row_id < n_scan_) {
      new_point.ring = row_id;
    } else {
      continue;
    }

    new_point.time = (point_id % horizon_scan_) * 0.1f / horizon_scan_;

    converted_pc->points.push_back(new_point);
  }

  publishPoints<PointXYZIRT>(converted_pc, *pc_msg);
}

template <typename T>
void PointCloudConverter::publishPoints(
  const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg)
{
  new_pc->is_dense = false;

  sensor_msgs::msg::PointCloud2 pc_new_msg;
  pcl::toROSMsg(*new_pc, pc_new_msg);
  pc_new_msg.header = old_msg.header;

  pcd_pub_->publish(pc_new_msg);
}

}  // namespace ign_sim_pointcloud_tool

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ign_sim_pointcloud_tool::PointCloudConverter)
