#include <iostream>
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;

class PatchworkppDemo : public rclcpp::Node
{
public:
  PatchworkppDemo() : Node("ground_segmentation")
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/kitti/velo/pointcloud");
    output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "/ground_segmentation/cloud");
    output_ground_topic_ = this->declare_parameter<std::string>("output_ground_topic", "/ground_segmentation/ground");
    output_nonground_topic_ = this->declare_parameter<std::string>("output_nonground_topic", "/ground_segmentation/nonground");
    log_every_n_ = this->declare_parameter<int>("log_every_n", 30);
    warn_empty_ground_ = this->declare_parameter<bool>("warn_empty_ground", true);

    RCLCPP_INFO(this->get_logger(), "Operating Patchwork++...");
    patchworkpp_ground_seg_ = std::make_shared<PatchWorkpp<PointType>>(this);

    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, rclcpp::QoS(100).transient_local());
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_ground_topic_, rclcpp::QoS(100).transient_local());
    pub_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_nonground_topic_, rclcpp::QoS(100).transient_local());

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PatchworkppDemo::callbackCloud, this, std::placeholders::_1));
  }

private:
  template<typename T>
  sensor_msgs::msg::PointCloud2 cloud2msg(
    const pcl::PointCloud<T> & cloud,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.stamp = stamp;
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
  }

  void callbackCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    double time_taken = 0.0;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    pcl::fromROSMsg(*cloud_msg, pc_curr);
    pc_curr.header.frame_id = cloud_msg->header.frame_id;

    patchworkpp_ground_seg_->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    frame_count_++;
    if (warn_empty_ground_ && !pc_curr.empty() && pc_ground.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Patchwork++ produced empty ground cloud. Check lidar mounting height and point cloud axis convention.");
    }
    if (log_every_n_ > 0 && frame_count_ % static_cast<uint64_t>(log_every_n_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Input PointCloud: %zu -> Ground: %zu / NonGround: %zu (running_time: %.6f sec)",
        pc_curr.size(), pc_ground.size(), pc_non_ground.size(), time_taken);
    }

    pub_cloud_->publish(cloud2msg(pc_curr, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_ground_->publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_non_ground_->publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
  }

  std::string cloud_topic_;
  std::string output_cloud_topic_;
  std::string output_ground_topic_;
  std::string output_nonground_topic_;
  int log_every_n_;
  bool warn_empty_ground_;
  uint64_t frame_count_ = 0;
  std::shared_ptr<PatchWorkpp<PointType>> patchworkpp_ground_seg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatchworkppDemo>());
  rclcpp::shutdown();
  return 0;
}
