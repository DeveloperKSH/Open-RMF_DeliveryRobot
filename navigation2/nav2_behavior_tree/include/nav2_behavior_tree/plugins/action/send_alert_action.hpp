#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ALERT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ALERT_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_behavior_tree
{

class SendAlertAction : public BT::SyncActionNode
{
public:
  SendAlertAction(
    const std::string & xml_tag_name, 
    const BT::NodeConfiguration & conf);
  
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>(
            "message",
            "Topic message")
    };
  }

  //BT::NodeStatus tick() override;

private:
  BT::NodeStatus tick() override;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ALERT_ACTION_HPP_