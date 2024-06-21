#pragma once

#include <string>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;

class FollowRow : public RosTopicPubNode<std_msgs::msg::String>
{
public:
    FollowRow(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
        : BT::RosTopicPubNode<std_msgs::msg::String>(name, conf, params)
    {
        RCLCPP_INFO(rclcpp::get_logger("FollowRowTopicNode"), "FollowRow node initialized");
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("mode")
        });
    }

    bool setMessage(std_msgs::msg::String &goal) override
    {
        std::string mode;
        if (!getInput<std::string>("mode", mode))
        {
            RCLCPP_ERROR(rclcpp::get_logger("FollowRowTopicNode"), "Failed to get input port 'mode'");
            return false;
        }

        goal.data = mode;
        return true;
    }
};
