#include "mapLoader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <iostream>

MapLoader::MapLoader() : Node("map_loader") {
    this->declare_parameter<std::string>("pcd_path", "");
    this->declare_parameter<std::string>("map_topic", "point_map");
    this->declare_parameter<std::string>("init_pose_topic", "initialpose");
    this->declare_parameter<std::string>("robot_pose_topic", "ndt_pose");
    this->declare_parameter<float>("submap_size_xy", 100.0);
    this->declare_parameter<float>("submap_size_z", 50.0);
    this->declare_parameter<float>("map_switch_thres", 10.0);

    std::string pcd_file_path, map_topic, robot_pose_topic, init_pose_topic;
    this->get_parameter("pcd_path", pcd_file_path);
    this->get_parameter("map_topic", map_topic);
    this->get_parameter("init_pose_topic", init_pose_topic);
    this->get_parameter("robot_pose_topic", robot_pose_topic);
    this->get_parameter("submap_size_xy", submap_size_xy_);
    this->get_parameter("submap_size_z", submap_size_z_);
    this->get_parameter("map_switch_thres", map_switch_thres_);

    RCLCPP_INFO(this->get_logger(), "init_pose_topic: %s", init_pose_topic.c_str());

    init_tf_params();

    pc_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 10);
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        init_pose_topic, 1, std::bind(&MapLoader::callbackInitPose, this, std::placeholders::_1));
    ndt_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        robot_pose_topic, 10, std::bind(&MapLoader::callbackRobotPose, this, std::placeholders::_1));

    file_list_.push_back(pcd_file_path);

    createPcd();
    transformMap();
}

void MapLoader::init_tf_params() {
    this->declare_parameter<float>("x", 0.0);
    this->declare_parameter<float>("y", 0.0);
    this->declare_parameter<float>("z", 0.0);
    this->declare_parameter<float>("roll", 0.0);
    this->declare_parameter<float>("pitch", 0.0);
    this->declare_parameter<float>("yaw", 0.0);
    this->get_parameter("x", tf_x_);
    this->get_parameter("y", tf_y_);
    this->get_parameter("z", tf_z_);
    this->get_parameter("roll", tf_roll_);
    this->get_parameter("pitch", tf_pitch_);
    this->get_parameter("yaw", tf_yaw_);
    RCLCPP_INFO(this->get_logger(), "x: %f y: %f z: %f roll: %f pitch: %f yaw: %f", tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_);
}

void MapLoader::transformMap() {
    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();
    pcl::transformPointCloud(*global_map_ptr_, *global_map_ptr_, tf_m2w);
    SaveMap(global_map_ptr_);
}

sensor_msgs::msg::PointCloud2 MapLoader::switchSubmap(const geometry_msgs::msg::Pose &pose) {
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(pose.position.x - submap_size_xy_, pose.position.y - submap_size_xy_, pose.position.z - submap_size_z_, 1.0));
    box_filter.setMax(Eigen::Vector4f(pose.position.x + submap_size_xy_, pose.position.y + submap_size_xy_, pose.position.z + submap_size_z_, 1.0));
    RCLCPP_INFO(this->get_logger(), "min: %f, %f, %f", pose.position.x - submap_size_xy_, pose.position.y - submap_size_xy_, pose.position.z - submap_size_z_);
    RCLCPP_INFO(this->get_logger(), "max: %f, %f, %f", pose.position.x + submap_size_xy_, pose.position.y + submap_size_xy_, pose.position.z + submap_size_z_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    box_filter.setInputCloud(global_map_ptr_);
    box_filter.filter(*submap_ptr);
    sensor_msgs::msg::PointCloud2 submap_msg;
    pcl::toROSMsg(*submap_ptr, submap_msg);
    return submap_msg;
}

void MapLoader::callbackInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_ptr) {
    curr_pose_ = initial_pose_msg_ptr->pose.pose;
    auto sub_map_msg = switchSubmap(curr_pose_);
    if (sub_map_msg.width != 0) {
        sub_map_msg.header.frame_id = "map";
        pc_map_pub_->publish(sub_map_msg);
        RCLCPP_INFO(this->get_logger(), "submap is published!");
    }
    pre_pose_ = curr_pose_;
}

void MapLoader::callbackRobotPose(const nav_msgs::msg::Odometry::SharedPtr ndt_odom_msg) {
    curr_pose_ = ndt_odom_msg->pose.pose;
    traversal_dist_ += std::sqrt(std::pow(curr_pose_.position.x - pre_pose_.position.x, 2) + std::pow(curr_pose_.position.y - pre_pose_.position.y, 2));
    if (traversal_dist_ >= map_switch_thres_) {
        auto sub_map_msg = switchSubmap(curr_pose_);
        if (sub_map_msg.width != 0) {
            sub_map_msg.header.frame_id = "map";
            pc_map_pub_->publish(sub_map_msg);
            RCLCPP_INFO(this->get_logger(), "new submap is published!");
        }
        traversal_dist_ = 0;
    }
    pre_pose_ = curr_pose_;
}

void MapLoader::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr) {
    pcl::io::savePCDFile("/tmp/transformed_map.pcd", *map_pc_ptr);
}

void MapLoader::createPcd() {
    sensor_msgs::msg::PointCloud2 pcd, part;
    for (const std::string &path : file_list_) {
        if (pcd.width == 0) {
            if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
                RCLCPP_ERROR(this->get_logger(), "load failed %s", path.c_str());
            }
        } else {
            if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
                RCLCPP_ERROR(this->get_logger(), "load failed %s", path.c_str());
            }
            pcd.width += part.width;
            pcd.row_step += part.row_step;
            pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
        }
        RCLCPP_INFO(this->get_logger(), "load %s", path.c_str());
    }
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcd, *global_map_ptr_);
    pcd.header.frame_id = "map";
    pc_map_pub_->publish(pcd);
    RCLCPP_INFO(this->get_logger(), "done!");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLoader>());
    rclcpp::shutdown();
    return 0;
}
