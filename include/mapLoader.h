#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <pcl/common/transforms.h>

class MapLoader : public rclcpp::Node {
public:
    MapLoader();

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_map_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ndt_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    std::vector<std::string> file_list_;
    double tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_;
    geometry_msgs::msg::Pose curr_pose_, pre_pose_;
    double submap_size_xy_, submap_size_z_;
    double traversal_dist_ = 0.0;
    double map_switch_thres_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;

    void createPcd();
    void transformMap();
    void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);
    void callbackRobotPose(const nav_msgs::msg::Odometry::SharedPtr ndt_odom_msg);
    void callbackInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_ptr);
    sensor_msgs::msg::PointCloud2 switchSubmap(const geometry_msgs::msg::Pose &pose);
};

#endif