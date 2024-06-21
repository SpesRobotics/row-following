#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "row_following_msgs/srv/generate_path.hpp"

class GeneratePath : public BT::RosServiceNode<row_following_msgs::srv::GeneratePath>
{
public:
  GeneratePath(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosServiceNode<row_following_msgs::srv::GeneratePath>(name, conf, params)
  {
    setServiceName("generate_path");
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("arc_radius"),
      BT::InputPort<double>("line_length"),
      BT::InputPort<std::string>("reference_frame"),
      BT::InputPort<std::string>("anchor_frame"),
      BT::InputPort<std::string>("turn"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints")
    });
  }

  bool setRequest(typename Request::SharedPtr& request) override
  {
    double arc_radius;
    double line_length;
    std::string reference_frame;
    std::string anchor_frame;
    std::string turn;

    if (!getInput<double>("arc_radius", arc_radius) ||
        !getInput<double>("line_length", line_length) ||
        !getInput<std::string>("reference_frame", reference_frame) ||
        !getInput<std::string>("anchor_frame", anchor_frame) ||
        !getInput<std::string>("turn", turn))
    {
      throw BT::RuntimeError("missing required input [arc_radius, line_length, reference_frame, anchor_frame, turn]");
    }

    request->arc_radius = arc_radius;
    request->line_length = line_length;
    request->reference_frame = reference_frame;
    request->anchor_frame = anchor_frame;
    request->turn = turn;

    RCLCPP_INFO(rclcpp::get_logger("GeneratePath"), "arc_radius: %f", arc_radius);
    RCLCPP_INFO(rclcpp::get_logger("GeneratePath"), "line_length: %f", line_length);
    RCLCPP_INFO(rclcpp::get_logger("GeneratePath"), "reference_frame: %s", reference_frame.c_str());
    RCLCPP_INFO(rclcpp::get_logger("GeneratePath"), "anchor_frame: %s", anchor_frame.c_str());
    RCLCPP_INFO(rclcpp::get_logger("GeneratePath"), "turn: %s", turn.c_str());

    return true;
  }

  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
  {
    if (!response->waypoints.empty())
    {
      setOutput("waypoints", response->waypoints);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};
