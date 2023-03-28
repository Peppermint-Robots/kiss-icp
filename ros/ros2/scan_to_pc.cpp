// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// KISS-ICP
#include "laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.h"
#include "tf2_ros/create_timer_ros.h"

namespace kiss_icp_ros {

ScanToPCL::ScanToPCL() : rclcpp::Node("scan_to_pcl_node") {
    std::cout << "here3\n";

    child_frame_ = declare_parameter<std::string>("child_frame", child_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);

    // Intialize subscribers
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&ScanToPCL::ScanToPC, this, std::placeholders::_1));

    // Intialize publishers
    rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});
    pc_from_laser_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("pc_from_laser", qos);

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                                     get_node_timers_interface());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    tf_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    RCLCPP_INFO(this->get_logger(), "scan to pc initialized");
}

void ScanToPCL::ScanToPC(const sensor_msgs::msg::LaserScan::SharedPtr msg_ptr) {
    // ROS2::Foxy can't handle a callback to const MessageT&, so we hack it here
    // https://github.com/ros2/rclcpp/pull/1598

    const sensor_msgs::msg::LaserScan &msg = *msg_ptr;
    auto cloud_out = std::make_shared<sensor_msgs::msg::PointCloud2>();

    try {
        auto transform =
            tf_buffer_->lookupTransform("laser", msg_ptr->header.frame_id, tf2::TimePointZero);
        projector_.transformLaserScanToPointCloud("laser", *msg_ptr, *cloud_out, *tf_buffer_);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "laser",
                    msg_ptr->header.frame_id.c_str(), ex.what());
        return;
    }

    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_out, *pcl_cloud);
    pc_from_laser_publisher_->publish(*cloud_out);
}

}  // namespace kiss_icp_ros

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    std::cout << "here1\n";

    rclcpp::init(argc, argv);

    std::cout << "here2\n";
    rclcpp::spin(std::make_shared<kiss_icp_ros::ScanToPCL>());
    rclcpp::shutdown();
    return 0;
}