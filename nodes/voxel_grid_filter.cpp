#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "points_downsampler.h"

#define MAX_MEASUREMENT_RANGE 120.0

class VoxelGridFilterNode : public rclcpp::Node {
public:
  VoxelGridFilterNode() : Node("voxel_grid_filter") {
    this->declare_parameter<std::string>("points_topic", "points_raw");
    this->declare_parameter<double>("leaf_size", 2.0);
    this->declare_parameter<bool>("output_log", false);
    this->declare_parameter<double>("measurement_range", MAX_MEASUREMENT_RANGE);

    this->get_parameter("points_topic", points_topic_);
    this->get_parameter("leaf_size", voxel_leaf_size_);
    this->get_parameter("output_log", output_log_);
    this->get_parameter("measurement_range", measurement_range_);

    RCLCPP_INFO(this->get_logger(), "Voxel leaf size is: %f", voxel_leaf_size_);

    if (output_log_) {
      char buffer[80];
      std::time_t now = std::time(NULL);
      std::tm *pnow = std::localtime(&now);
      std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
      filename_ = "voxel_grid_filter_" + std::string(buffer) + ".csv";
      ofs_.open(filename_.c_str(), std::ios::app);
    }

    filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, 10,
      std::bind(&VoxelGridFilterNode::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);

    if (measurement_range_ != MAX_MEASUREMENT_RANGE) {
      scan = removePointsByRange(scan, 0, measurement_range_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    sensor_msgs::msg::PointCloud2 filtered_msg;

    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (voxel_leaf_size_ >= 0.1) {
      // Downsampling the velodyne scan using VoxelGrid filter
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_grid_filter.setInputCloud(scan_ptr);
      voxel_grid_filter.filter(*filtered_scan_ptr);

      // define the heading direction
      pcl::PointXYZ x_axis(1, 0, 0);

      // keep front view points
      pcl::PointIndices::Ptr front_index(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      RCLCPP_INFO(this->get_logger(), "before filtering %zu", filtered_scan_ptr->size());

      for (size_t i = 0; i < filtered_scan_ptr->size(); i++) {
        auto point = filtered_scan_ptr->points[i];
        double mod = sqrt(point.x * point.x + point.y * point.y);
        point.x /= mod;
        point.y /= mod;
        double angle = acos(point.x * x_axis.x + point.y * x_axis.y);
        if (abs(angle) > M_PI / 180.0 * 60) {
          front_index->indices.push_back(i);
        }
      }

      extract.setInputCloud(filtered_scan_ptr);
      extract.setIndices(front_index);
      extract.setNegative(true);
      //extract.filter(*filtered_scan_ptr);

      RCLCPP_INFO(this->get_logger(), "after filtering %zu", filtered_scan_ptr->size());

      pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
    } else {
      pcl::toROSMsg(*scan_ptr, filtered_msg);
    }

    filtered_msg.header = input->header;
    filtered_points_pub_->publish(filtered_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  double voxel_leaf_size_ = 2.0;
  bool output_log_ = false;
  std::ofstream ofs_;
  std::string filename_;
  std::string points_topic_;
  double measurement_range_ = MAX_MEASUREMENT_RANGE;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridFilterNode>());
  rclcpp::shutdown();
  return 0;
}
