#include <string>
#include <memory>
#include <chrono>

#include "nav2_behavior_tree/plugins/action/wait_until_clear_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree
{

WaitUntilClearAction::WaitUntilClearAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf),  
  timer_initialized_(false),
  wait_obstacle_(false), 
  obstacle_detected_(false), 
  obstacle_cleared_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  rclcpp::QoS qos_settings(10);
  qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  obstacle_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "obstacle_topic", qos_settings, std::bind(&WaitUntilClearAction::obstacleCallback, this, std::placeholders::_1));

  clear_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "alert_topic", qos_settings, std::bind(&WaitUntilClearAction::clearCallback, this, std::placeholders::_1));

  delay_publisher_ = node_->create_publisher<std_msgs::msg::String>("delay_status_topic", qos_settings);
  delay_threshold_ = std::chrono::seconds(10);
}

BT::NodeStatus WaitUntilClearAction::tick()
{
  rclcpp::spin_some(node_);

  // 타이머 초기화
  if (!timer_initialized_) {
    initializeTimer();
    if (obstacle_detected_) {
      wait_obstacle_ = true;
    }
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - delay_start_time_);

  // 경과 시간이 임계 시간(10s)을 지나면 시간 경과 메시지를 토픽으로 발행
  if (elapsed_time >= delay_threshold_) {
    auto msg = std_msgs::msg::String();
    msg.data = "Delay exceeded " + std::to_string(elapsed_time.count()) + " seconds";
    delay_publisher_->publish(msg);
    initializeTimer();
  }

  // 
  if (!wait_obstacle_) {
    RCLCPP_INFO(node_->get_logger(), "Obstacle Unknown");
    timer_initialized_ = false;
    RCLCPP_INFO(node_->get_logger(), "WaitUntilClear Finish (Obstacle Unknown)");
    return BT::NodeStatus::SUCCESS;
  }

  // 장애물이 사라졌다는 토픽을 수신받으면 정지 상태 유지를 종료
  if (obstacle_cleared_) {
    RCLCPP_INFO(node_->get_logger(), "Clear Message Received");
    timer_initialized_ = false;
    wait_obstacle_ = false;
    obstacle_cleared_ = false;
    RCLCPP_INFO(node_->get_logger(), "WaitUntilClear Finish (Obstacle Clear)");
    return BT::NodeStatus::SUCCESS;
  }

  // 정지 상태 후 제어 명령 또는 장애물이 사라졌다는 메시지를 수신받지 못하면 정지 상태 유지
  return BT::NodeStatus::RUNNING;
}

// 타이머 초기화 함수
void WaitUntilClearAction::initializeTimer()
{
  delay_start_time_ = std::chrono::steady_clock::now();
  timer_initialized_ = true;
}

// 'obstacle_topic' 토픽에 대한 콜백 함수
void WaitUntilClearAction::obstacleCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string detect_msg = "Detected";

  if (msg->data == detect_msg) {
    obstacle_detected_ = true;
  }
}

// 'clear_topic' 토픽에 대한 콜백 함수
void WaitUntilClearAction::clearCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string obstacle_clear_msg = "Obstacle Clear";

  if (msg->data == obstacle_clear_msg) {
    obstacle_cleared_ = true;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::WaitUntilClearAction>("WaitUntilClear");
}