#include <csignal>
#include <ctime>
#include <iostream>
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "patchworkpp/patchworkpp.hpp"
#include "tools/kitti_loader.hpp"

using PointType = PointXYZILID;
using namespace std;

void signal_callback_handler(int signum)
{
    cout << "Caught Ctrl + c " << endl;
    exit(signum);
}

template<typename T>
sensor_msgs::msg::PointCloud2 cloud2msg(const pcl::PointCloud<T> &cloud, const std::string &frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("offline_kitti");

    auto cloud_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/cloud", rclcpp::QoS(100).transient_local());
    auto tp_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/TP", rclcpp::QoS(100).transient_local());
    auto fp_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FP", rclcpp::QoS(100).transient_local());
    auto fn_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FN", rclcpp::QoS(100).transient_local());

    const std::string algorithm = node->declare_parameter<std::string>("algorithm", "patchworkpp");
    const std::string seq = node->declare_parameter<std::string>("sequence", "00");
    std::string data_path = node->declare_parameter<std::string>("data_path", "/");
    const std::string output_csvpath = node->declare_parameter<std::string>("output_csvpath", "/data/");
    const int init_idx = node->declare_parameter<int>("init_idx", 0);
    const bool save_csv_file = node->declare_parameter<bool>("save_csv_file", false);
    const bool stop_per_each_frame = node->declare_parameter<bool>("stop_per_each_frame", false);

    (void)algorithm;
    signal(SIGINT, signal_callback_handler);

    auto patchworkpp_ground_seg = std::make_shared<PatchWorkpp<PointType>>(node.get());
    data_path = data_path + "/" + seq;
    KittiLoader loader(data_path);

    const int n_frames = loader.size();
    for (int n = init_idx; rclcpp::ok() && n < n_frames; ++n) {
        cout << n << "th node come" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);

        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;
        double time_taken = 0.0;

        cout << "Operating patchwork++..." << endl;
        patchworkpp_ground_seg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

        double precision, recall, precision_wo_veg, recall_wo_veg;
        calculate_precision_recall(pc_curr, pc_ground, precision, recall);
        calculate_precision_recall_without_vegetation(pc_curr, pc_ground, precision_wo_veg, recall_wo_veg);

        cout << "\033[1;32m" << n << "th, takes : " << time_taken << " | "
             << pc_curr.size() << " -> " << pc_ground.size() << "\033[0m" << endl;
        cout << "\033[1;32m P: " << precision_wo_veg << " | R: " << recall_wo_veg << "\033[0m" << endl;

        if (save_csv_file) {
            ofstream sc_output(output_csvpath + seq + ".csv", ios::app);
            sc_output << n << "," << time_taken << "," << precision << "," << recall << ","
                      << precision_wo_veg << "," << recall_wo_veg << std::endl;
        }

        pcl::PointCloud<PointType> tp;
        pcl::PointCloud<PointType> fp;
        pcl::PointCloud<PointType> fn;
        pcl::PointCloud<PointType> tn;
        discern_ground_without_vegetation(pc_ground, tp, fp);
        discern_ground_without_vegetation(pc_non_ground, fn, tn);

        auto stamp = node->now();
        auto cloud_msg = cloud2msg(pc_curr);
        auto tp_msg = cloud2msg(tp);
        auto fp_msg = cloud2msg(fp);
        auto fn_msg = cloud2msg(fn);
        cloud_msg.header.stamp = stamp;
        tp_msg.header.stamp = stamp;
        fp_msg.header.stamp = stamp;
        fn_msg.header.stamp = stamp;

        cloud_publisher->publish(cloud_msg);
        tp_publisher->publish(tp_msg);
        fp_publisher->publish(fp_msg);
        fn_publisher->publish(fn_msg);
        rclcpp::spin_some(node);

        if (stop_per_each_frame) cin.ignore();
    }

    rclcpp::shutdown();
    return 0;
}
