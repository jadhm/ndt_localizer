#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NdtLocalizer : public rclcpp::Node {
public:
    NdtLocalizer();
    ~NdtLocalizer();

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ndt_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr exe_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iteration_num_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub_;
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Member variables
    std::map<std::string, std::string> key_value_stdmap_;
    geometry_msgs::msg::PolygonStamped poly;
    std::string path_file;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    double converged_param_transform_probability_;
    std::mutex ndt_map_mtx_;
    bool init_pose = false;
    bool is_ndt_published = false;
    Eigen::Matrix4f pre_trans, odom_trans, pre_odom_trans, map_to_odom_matrix, delta_trans, pre_corr_trans, base_to_sensor_matrix_, initial_pose_matrix;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    // Updated function signatures
    void callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_ptr);
    void callback_pointsmap(const sensor_msgs::msg::PointCloud2::SharedPtr map_points_msg_ptr);
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr sensor_points_sensorTF_msg_ptr);
    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                      const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped,
                      const rclcpp::Time & time_stamp);
    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                      const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                   const geometry_msgs::msg::PoseStamped & pose_msg);
    bool loadPath(std::string path);
    double getNearestHeight(const geometry_msgs::msg::Pose p);
    void getXYZRPYfromMat(const Eigen::Matrix4f mat, Pose &p);
    void timer_diagnostic();
    void init_params();
};