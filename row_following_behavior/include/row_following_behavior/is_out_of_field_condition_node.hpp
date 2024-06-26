#pragma once

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace BT;

class IsOutOfField : public RosTopicSubNode<tf2_msgs::msg::TFMessage>
{
public:
    IsOutOfField(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
        : BT::RosTopicSubNode<tf2_msgs::msg::TFMessage>(name, conf, params),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->logger(), "IsOutOfField node initialized");
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("field_coordinates"),
            InputPort<std::string>("turn")
        });
    }
    BT::NodeStatus onTick(const std::shared_ptr<tf2_msgs::msg::TFMessage>& last_msg) override
    {
        double field_coordinates;
        if (!getInput<double>("field_coordinates", field_coordinates))
        {
            throw BT::RuntimeError("missing required input [field_coordinates]");
        }

        std::string turn;
        if (!getInput<std::string>("turn", turn))
        {
            throw BT::RuntimeError("missing required input [turn]");
        }

        RCLCPP_INFO(this->logger(), "Field Coordinates: %f", field_coordinates);
        RCLCPP_INFO(this->logger(), "Turn: %s", turn.c_str());
        
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->logger(), "Could not transform %s to %s: %s", "map", "base_link", ex.what());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(this->logger(), "Current Coordinates: %f", transform.transform.translation.x);

        if (turn == "left" && transform.transform.translation.x >= field_coordinates)
            return BT::NodeStatus::SUCCESS;
        if (turn == "right" && transform.transform.translation.x <= field_coordinates)
        {
            RCLCPP_INFO(this->logger(), "HERE!");
            return BT::NodeStatus::SUCCESS;
        }
        // if (transform.transform.translation.x >= field_coordinates)
        //     return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
