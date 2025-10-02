#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_UNTIL_CLEAR_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_UNTIL_CLEAR_ACTION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree
{

class WaitUntilClearAction : public BT::CoroActionNode
{
public:
  WaitUntilClearAction(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus tick() override;

  void initializeTimer();
  void obstacleCallback(const std_msgs::msg::String::SharedPtr msg);
  void clearCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr clear_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delay_publisher_;

  std::chrono::steady_clock::time_point delay_start_time_;
  std::chrono::seconds delay_threshold_;

  bool timer_initialized_;
  bool wait_obstacle_;
  bool obstacle_detected_;
  bool obstacle_cleared_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_UNTIL_CLEAR_ACTION_HPP_