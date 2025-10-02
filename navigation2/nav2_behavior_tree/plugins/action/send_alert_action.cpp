#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/send_alert_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

SendAlertAction::SendAlertAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node_->create_publisher<std_msgs::msg::String>("alert_topic", 10);
}

BT::NodeStatus SendAlertAction::tick()
{
  std_msgs::msg::String msg;
  if (!getInput("message", msg.data))
  {
    throw BT::RuntimeError("missing required input [message]");
  }
  publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "SendAlert Finish");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SendAlertAction>("SendAlert");
}