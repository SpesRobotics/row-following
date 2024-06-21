#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "row_following_msgs/action/follow_row.hpp"

using FollowRowAction = row_following_msgs::action::FollowRow;

class FollowRowActionNode : public BT::RosActionNode<FollowRowAction>
{
public:
  FollowRowActionNode(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosActionNode<FollowRowAction>(name, conf, params) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("mode"),
    });
  }

  bool setGoal(Goal& goal) override
  {
    std::string mode;
    if (!getInput<std::string>("mode", mode))
    {
      throw BT::RuntimeError("missing required input [mode]");
    }

    RCLCPP_INFO(this->logger(), "Mode: %s", mode.c_str());
    goal.mode = mode;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    auto node = node_.lock();
    if (!node)
    {
      throw BT::RuntimeError("Node has expired");
    }

    if (wr.result->success)
    {
      RCLCPP_INFO(node->get_logger(), "%s: onResultReceived with success", name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "%s: onResultReceived with failure", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    auto node = node_.lock();
    if (!node)
    {
      throw BT::RuntimeError("Node has expired");
    }
    RCLCPP_ERROR(node->get_logger(), "%s: %s", name().c_str(), BT::toStr(error));
    RCLCPP_ERROR(node->get_logger(), "%s: onFailure %d", name().c_str(), static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }
};
