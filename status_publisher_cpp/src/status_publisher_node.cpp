#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <nlohmann/json.hpp>

#include "status_publisher_cpp/status_publisher_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

using json = nlohmann::json;

StatusPublisherNode::StatusPublisherNode()
    : Node("status_publisher_node"), current_yaw_(0.0), received_path_data_(false)
{
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&StatusPublisherNode::gps_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/global", 10, std::bind(&StatusPublisherNode::odom_callback, this, std::placeholders::_1));

    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/converted_plan", 10, std::bind(&StatusPublisherNode::path_callback, this, std::placeholders::_1));

    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/status_data", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&StatusPublisherNode::publish_status_data, this));
}

void StatusPublisherNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gps_data_ = msg;
}

void StatusPublisherNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 오일러 각으로 변환
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Yaw 값 저장
    current_yaw_ = normalize_yaw(yaw);
}

double StatusPublisherNode::normalize_yaw(double yaw)
{
    // Yaw 값을 0에서 2π 사이로 변환
    double two_pi = 2.0 * M_PI;
    yaw = std::fmod(yaw, two_pi);
    if (yaw < 0.0)
    {
        yaw += two_pi;
    }
    return yaw;
}

void StatusPublisherNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_data_ = msg;
    received_path_data_ = true;  // Set the flag when path data is received
}

void StatusPublisherNode::publish_status_data()
{
    if (gps_data_ && !std::isnan(current_yaw_) && received_path_data_)
    {
        // Create JSON object
        json status_json;

        status_json["gps"] = {
            {"latitude", gps_data_->latitude},
            {"longitude", gps_data_->longitude},
            {"altitude", gps_data_->altitude}
        };

        status_json["heading"] = current_yaw_;
        
        // Create path data
        json path_json = json::array();
        for (const auto &pose_stamped : path_data_->poses)
        {
            json pose_json = {
                {"latitude", pose_stamped.pose.position.y},
                {"longitude", pose_stamped.pose.position.x},
                {"altitude", pose_stamped.pose.position.z}
            };
            path_json.push_back(pose_json);
        }

        status_json["path"] = path_json;
        
        // Publish JSON string
        auto msg = std_msgs::msg::String();
        msg.data = status_json.dump();
        status_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for all required data: GPS, IMU, and Path");
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
