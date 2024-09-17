#ifndef POINT_CLOUD_CONVERTER_HPP_
#define POINT_CLOUD_CONVERTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                 uint16_t, ring, ring)(float, time, time))

namespace ign_sim_pointcloud_tool
{
class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);

  template <typename T>
  void publishPoints(
    const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  int n_scan_;
  int horizon_scan_;
  float ang_bottom_;
  float ang_res_y_;
  std::string pcd_topic_;
};
}  // namespace ign_sim_pointcloud_tool

#endif  // POINT_CLOUD_CONVERTER_HPP_
