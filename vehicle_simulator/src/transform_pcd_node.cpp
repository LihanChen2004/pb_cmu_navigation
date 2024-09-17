#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer() : Node("point_cloud_transformer")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, std::bind(&PointCloudTransformer::odomCallback, this, std::placeholders::_1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", 10,
      std::bind(&PointCloudTransformer::pointcloudCallback, this, std::placeholders::_1));
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = *msg; }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << latest_odom_.pose.pose.position.x, latest_odom_.pose.pose.position.y,
      latest_odom_.pose.pose.position.z;
    Eigen::Quaterniond q(
      latest_odom_.pose.pose.orientation.w, latest_odom_.pose.pose.orientation.x,
      latest_odom_.pose.pose.orientation.y, latest_odom_.pose.pose.orientation.z);
    transform.rotate(q);

    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(transformed_cloud, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = "odom";

    pointcloud_pub_->publish(output);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  nav_msgs::msg::Odometry latest_odom_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}