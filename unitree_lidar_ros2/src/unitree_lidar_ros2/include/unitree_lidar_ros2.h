/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// SDK
#include "unitree_lidar_sdk_pcl.h"


using std::placeholders::_1;

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
    explicit UnitreeLidarSDKNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~UnitreeLidarSDKNode();

    void timer_callback();

protected:
    // ROS
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // Unitree Lidar Reader
    UnitreeLidarReader *lsdk_;

    // Config params
    int work_mode_;
    int initialize_type_;
    int local_port_;
    std::string local_ip_;
    int lidar_port_;
    std::string lidar_ip_;
    std::string serial_port_;
    int baudrate_;
    
    int cloud_scan_num_;
    bool use_system_timestamp_;
    bool start_lidar_rotation_;
    bool reset_lidar_after_set_mode_;
    bool publish_tf_;
    std::string imu_quaternion_order_;
    double imu_angular_velocity_scale_;
    double imu_linear_acceleration_scale_;
    double range_min_;
    double range_max_;

    std::string cloud_frame_;
    std::string cloud_topic_;

    std::string imu_frame_;
    std::string imu_topic_;

    bool save_cloud_txt_;
    std::string cloud_txt_save_mode_;
    std::string cloud_txt_path_;
    std::string cloud_txt_dir_;
    int cloud_txt_save_every_n_;
    uint64_t cloud_txt_frame_count_;

    void savePointCloudToTxt(const PointCloudUnitree &cloud);
    bool writePointCloudTxtFile(const std::string &file_path, const PointCloudUnitree &cloud, uint64_t frame_count);
};

///////////////////////////////////////////////////////////////////

UnitreeLidarSDKNode::~UnitreeLidarSDKNode()
{
}

UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions &options)
    : Node("unitre_lidar_sdk_node", options)
{
    // load config parameters
    declare_parameter<int>("initialize_type", 2);
    declare_parameter<int>("work_mode", 0);
    declare_parameter<bool>("use_system_timestamp", true);
    declare_parameter<bool>("start_lidar_rotation", true);
    declare_parameter<bool>("reset_lidar_after_set_mode", true);
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::string>("imu_quaternion_order", "wxyz");
    declare_parameter<double>("imu_angular_velocity_scale", 0.017453292519943295);
    declare_parameter<double>("imu_linear_acceleration_scale", 1.0);
    declare_parameter<double>("range_min", 0);
    declare_parameter<double>("range_max", 50);
    declare_parameter<int>("cloud_scan_num", 18);

    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 4000000);

    declare_parameter<int>("lidar_port", 6101);
    declare_parameter<std::string>("lidar_ip", "192.168.1.2");
    declare_parameter<int>("local_port", 6201);
    declare_parameter<std::string>("local_ip", "192.168.1.62");

    declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
    declare_parameter<std::string>("cloud_topic", "unilidar/cloud");

    declare_parameter<std::string>("imu_frame", "unilidar_imu");
    declare_parameter<std::string>("imu_topic", "unilidar/imu");

    declare_parameter<bool>("save_cloud_txt", false);
    declare_parameter<std::string>("cloud_txt_save_mode", "overwrite_one_file");
    declare_parameter<std::string>("cloud_txt_path", "/tmp/unitree_lidar_cloud.txt");
    declare_parameter<std::string>("cloud_txt_dir", "/tmp/unitree_lidar_cloud_frames");
    declare_parameter<int>("cloud_txt_save_every_n", 1);

    work_mode_ = get_parameter("work_mode").as_int();
    initialize_type_ = get_parameter("initialize_type").as_int();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    lidar_port_ = get_parameter("lidar_port").as_int();
    lidar_ip_ = get_parameter("lidar_ip").as_string();
    local_port_ = get_parameter("local_port").as_int();
    local_ip_ = get_parameter("local_ip").as_string();

    cloud_scan_num_ = get_parameter("cloud_scan_num").as_int();
    use_system_timestamp_ = get_parameter("use_system_timestamp").as_bool();
    start_lidar_rotation_ = get_parameter("start_lidar_rotation").as_bool();
    reset_lidar_after_set_mode_ = get_parameter("reset_lidar_after_set_mode").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    imu_quaternion_order_ = get_parameter("imu_quaternion_order").as_string();
    imu_angular_velocity_scale_ = get_parameter("imu_angular_velocity_scale").as_double();
    imu_linear_acceleration_scale_ = get_parameter("imu_linear_acceleration_scale").as_double();
    range_max_ = get_parameter("range_max").as_double();
    range_min_ = get_parameter("range_min").as_double();

    cloud_frame_ = get_parameter("cloud_frame").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    
    imu_frame_ = get_parameter("imu_frame").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    save_cloud_txt_ = get_parameter("save_cloud_txt").as_bool();
    cloud_txt_save_mode_ = get_parameter("cloud_txt_save_mode").as_string();
    cloud_txt_path_ = get_parameter("cloud_txt_path").as_string();
    cloud_txt_dir_ = get_parameter("cloud_txt_dir").as_string();
    cloud_txt_save_every_n_ = get_parameter("cloud_txt_save_every_n").as_int();
    cloud_txt_frame_count_ = 0;

    RCLCPP_INFO(
        this->get_logger(),
        "timestamp/imu config: use_system_timestamp=%s angular_velocity_scale=%.9f linear_acceleration_scale=%.9f",
        use_system_timestamp_ ? "true" : "false",
        imu_angular_velocity_scale_,
        imu_linear_acceleration_scale_);

    if (cloud_txt_save_every_n_ < 1)
    {
        RCLCPP_WARN(this->get_logger(), "cloud_txt_save_every_n must be >= 1, reset to 1");
        cloud_txt_save_every_n_ = 1;
    }

    // Initialize UnitreeLidarReader
    lsdk_ = createUnitreeLidarReader();

    std::cout << "initialize_type_ = " << initialize_type_ << std::endl;

    if (initialize_type_ == 1)
    {
        if (lsdk_->initializeSerial(serial_port_, baudrate_,
                                    cloud_scan_num_, use_system_timestamp_, range_min_, range_max_))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "failed to initialize Unitree lidar over serial: port=%s baudrate=%d",
                         serial_port_.c_str(), baudrate_);
            throw std::runtime_error("Unitree lidar serial initialization failed");
        }
        RCLCPP_INFO(this->get_logger(),
                    "initialized Unitree lidar over serial: port=%s baudrate=%d",
                    serial_port_.c_str(), baudrate_);
    }
    else if (initialize_type_ == 2)
    {
        if (lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,
                                 cloud_scan_num_, use_system_timestamp_, range_min_, range_max_))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "failed to initialize Unitree lidar over UDP: lidar=%s:%d local=%s:%d",
                         lidar_ip_.c_str(), lidar_port_, local_ip_.c_str(), local_port_);
            throw std::runtime_error("Unitree lidar UDP initialization failed");
        }
        RCLCPP_INFO(this->get_logger(),
                    "initialized Unitree lidar over UDP: lidar=%s:%d local=%s:%d",
                    lidar_ip_.c_str(), lidar_port_, local_ip_.c_str(), local_port_);
    }
    else
    {
        std::cout << "initialize_type is not right! exit now ...\n";
        exit(0);
    }

    if (start_lidar_rotation_)
    {
        RCLCPP_INFO(this->get_logger(), "starting lidar rotation");
        lsdk_->startLidarRotation();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "setting lidar work mode to: %d", work_mode_);
    lsdk_->setLidarWorkMode(work_mode_);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (reset_lidar_after_set_mode_)
    {
        RCLCPP_INFO(this->get_logger(), "resetting lidar after setting work mode");
        lsdk_->resetLidar();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (save_cloud_txt_)
    {
        if (cloud_txt_save_mode_ != "overwrite_one_file" && cloud_txt_save_mode_ != "separate_files")
        {
            RCLCPP_ERROR(this->get_logger(),
                         "cloud_txt_save_mode must be overwrite_one_file or separate_files, got: %s",
                         cloud_txt_save_mode_.c_str());
            save_cloud_txt_ = false;
        }
        else if (cloud_txt_save_mode_ == "separate_files")
        {
            std::error_code error_code;
            std::filesystem::create_directories(cloud_txt_dir_, error_code);
            if (error_code)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "failed to create point cloud txt directory: %s, error: %s",
                             cloud_txt_dir_.c_str(),
                             error_code.message().c_str());
                save_cloud_txt_ = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "saving each point cloud frame to txt files in: %s", cloud_txt_dir_.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "saving latest point cloud frame to txt file: %s", cloud_txt_path_.c_str());
        }
    }

    // ROS2
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&UnitreeLidarSDKNode::timer_callback, this));
}

void UnitreeLidarSDKNode::timer_callback()
{
    int result = 0;
    try
    {
        result = lsdk_->runParse();
    }
    catch (const std::exception &exc)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Unitree lidar parse exception: %s",
            exc.what());
        return;
    }

    static pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    // RCLCPP_INFO(this->get_logger(), "result = %d", result);

    if (result == LIDAR_IMU_DATA_PACKET_TYPE)
    {
        LidarImuData imu;
        lsdk_->getImuData(imu);

        if (lsdk_->getImuData(imu))
        {
            // publish imu message
            rclcpp::Time timestamp = use_system_timestamp_
                ? this->now()
                : rclcpp::Time(imu.info.stamp.sec, imu.info.stamp.nsec);

            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = imu_frame_;
            imuMsg.header.stamp = timestamp;

            if (imu_quaternion_order_ == "xyzw")
            {
                imuMsg.orientation.x = imu.quaternion[0];
                imuMsg.orientation.y = imu.quaternion[1];
                imuMsg.orientation.z = imu.quaternion[2];
                imuMsg.orientation.w = imu.quaternion[3];
            }
            else
            {
                imuMsg.orientation.w = imu.quaternion[0];
                imuMsg.orientation.x = imu.quaternion[1];
                imuMsg.orientation.y = imu.quaternion[2];
                imuMsg.orientation.z = imu.quaternion[3];
            }

            imuMsg.angular_velocity.x = imu.angular_velocity[0] * imu_angular_velocity_scale_;
            imuMsg.angular_velocity.y = imu.angular_velocity[1] * imu_angular_velocity_scale_;
            imuMsg.angular_velocity.z = imu.angular_velocity[2] * imu_angular_velocity_scale_;

            imuMsg.linear_acceleration.x = imu.linear_acceleration[0] * imu_linear_acceleration_scale_;
            imuMsg.linear_acceleration.y = imu.linear_acceleration[1] * imu_linear_acceleration_scale_;
            imuMsg.linear_acceleration.z = imu.linear_acceleration[2] * imu_linear_acceleration_scale_;

            pub_imu_->publish(imuMsg);

            if (publish_tf_)
            {
                // Vendor demo TFs. For robot integration, prefer disabling these and
                // publishing fixed sensor extrinsics from the robot bringup.
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = imu_frame_ + "_initial";
                transformStamped.child_frame_id = imu_frame_;
                transformStamped.transform.translation.x = 0;
                transformStamped.transform.translation.y = 0;
                transformStamped.transform.translation.z = 0;
                transformStamped.transform.rotation.x = imu.quaternion[1];
                transformStamped.transform.rotation.y = imu.quaternion[2];
                transformStamped.transform.rotation.z = imu.quaternion[3];
                transformStamped.transform.rotation.w = imu.quaternion[0];
                broadcaster_->sendTransform(transformStamped);

                transformStamped.header.frame_id = imu_frame_;
                transformStamped.child_frame_id = cloud_frame_;
                transformStamped.transform.translation.x = 0.007698;
                transformStamped.transform.translation.y = 0.014655;
                transformStamped.transform.translation.z = -0.00667;
                transformStamped.transform.rotation.x = 0;
                transformStamped.transform.rotation.y = 0;
                transformStamped.transform.rotation.z = 0;
                transformStamped.transform.rotation.w = 1;
                broadcaster_->sendTransform(transformStamped);
            }
        }
    }
    else if (result == LIDAR_POINT_DATA_PACKET_TYPE)
    {
        // RCLCPP_INFO(this->get_logger(), "POINT_CLOUD");
        PointCloudUnitree cloud;
        if (lsdk_->getPointCloud(cloud))
        {
            savePointCloudToTxt(cloud);

            transformUnitreeCloudToPCL(cloud, cloudOut);

            rclcpp::Time timestamp = use_system_timestamp_
                ? this->now()
                : rclcpp::Time(
                    static_cast<int32_t>(cloud.stamp),
                    static_cast<uint32_t>((cloud.stamp - static_cast<int32_t>(cloud.stamp)) * 1e9));

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloudOut, cloud_msg);
            cloud_msg.header.frame_id = cloud_frame_;
            cloud_msg.header.stamp = timestamp;

            pub_cloud_->publish(cloud_msg);
        }
    }
}

void UnitreeLidarSDKNode::savePointCloudToTxt(const PointCloudUnitree &cloud)
{
    if (!save_cloud_txt_)
    {
        return;
    }

    cloud_txt_frame_count_++;
    if ((cloud_txt_frame_count_ - 1) % static_cast<uint64_t>(cloud_txt_save_every_n_) != 0)
    {
        return;
    }

    std::string file_path = cloud_txt_path_;
    if (cloud_txt_save_mode_ == "separate_files")
    {
        std::ostringstream file_name;
        file_name << "cloud_" << std::setw(9) << std::setfill('0') << cloud_txt_frame_count_ << ".txt";
        file_path = (std::filesystem::path(cloud_txt_dir_) / file_name.str()).string();
    }

    if (!writePointCloudTxtFile(file_path, cloud, cloud_txt_frame_count_))
    {
        RCLCPP_ERROR(this->get_logger(), "failed to write point cloud txt file: %s", file_path.c_str());
        save_cloud_txt_ = false;
    }
}

bool UnitreeLidarSDKNode::writePointCloudTxtFile(
    const std::string &file_path,
    const PointCloudUnitree &cloud,
    uint64_t frame_count)
{
    std::ofstream file(file_path, std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        return false;
    }

    file << "# Unitree L2 point cloud txt\n";
    file << "# columns: cloud_stamp cloud_id point_index x y z intensity relative_time ring\n";
    file << "# frame " << frame_count
         << " stamp " << std::fixed << std::setprecision(9) << cloud.stamp
         << " id " << cloud.id
         << " points " << cloud.points.size()
         << " ring_num " << cloud.ringNum << '\n';

    file << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        const PointUnitree &point = cloud.points[i];
        file << cloud.stamp << ' '
             << cloud.id << ' '
             << i << ' '
             << point.x << ' '
             << point.y << ' '
             << point.z << ' '
             << point.intensity << ' '
             << point.time << ' '
             << point.ring << '\n';
    }

    return static_cast<bool>(file);
}
