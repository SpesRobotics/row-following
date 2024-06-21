#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include "behaviortree_ros2/bt_action_node.hpp"

using namespace BT;

using FollowPathAction = nav2_msgs::action::FollowPath;

class FollowPath : public BT::RosActionNode<FollowPathAction>
{
public:
    FollowPath(const std::string &name, const BT::NodeConfig &config, const BT::RosNodeParams &params)
        : BT::RosActionNode<FollowPathAction>(name, config, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
        });
    }

    bool setGoal(Goal &goal) override
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", waypoints))
        {
            throw BT::RuntimeError("missing required input [waypoints]");
        }

        if (!waypoints.empty())
        {
            goal.path.poses = waypoints;
            goal.path.header.frame_id = waypoints.front().header.frame_id;
            RCLCPP_INFO(rclcpp::get_logger("FollowPath"), "Goal set with %zu waypoints.", waypoints.size());
        }
        else
        {
            throw BT::RuntimeError("input port [waypoints] is empty");
        }

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        auto node = node_.lock();
        if (!node)
        {
            throw BT::RuntimeError("Node has expired");
        }

        switch (wr.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "%s: onResultReceived with success", name().c_str());
            return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "%s: onResultReceived with failure", name().c_str());
            return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node->get_logger(), "%s: onResultReceived with canceled", name().c_str());
            return BT::NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(node->get_logger(), "%s: onResultReceived with unknown result code", name().c_str());
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
