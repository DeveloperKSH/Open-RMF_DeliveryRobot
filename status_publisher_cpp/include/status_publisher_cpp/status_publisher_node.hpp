#ifndef STATUS_PUBLISHER_NODE_HPP
#define STATUS_PUBLISHER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class StatusPublisherNode : public rclcpp::Node
{
public:
    StatusPublisherNode();

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    double normalize_yaw(double yaw);
    void publish_status_data();

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    sensor_msgs::msg::NavSatFix::SharedPtr gps_data_;
    nav_msgs::msg::Path::SharedPtr path_data_;
    float current_yaw_;
    bool received_path_data_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // STATUS_PUBLISHER_NODE_HPP
