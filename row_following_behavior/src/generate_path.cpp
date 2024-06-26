#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include "row_following_msgs/srv/generate_path.hpp"

class GeneratePathServiceServer : public rclcpp::Node
{
public:
    GeneratePathServiceServer()
        : Node("generate_path_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        service_ = this->create_service<row_following_msgs::srv::GeneratePath>(
            "generate_path",
            std::bind(&GeneratePathServiceServer::handle_set_path_params, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service server is ready to set path parameters.");
    }

private:
    void handle_set_path_params(const std::shared_ptr<row_following_msgs::srv::GeneratePath::Request> request,
                                std::shared_ptr<row_following_msgs::srv::GeneratePath::Response> response)
    {
        arc_radius_ = request->arc_radius;
        line_length_ = request->line_length * 100;
        reference_frame_ = request->reference_frame;
        anchor_frame_ = request->anchor_frame;
        turn_direction_ = request->turn;

        RCLCPP_INFO(this->get_logger(), "Path parameters set: arc_radius=%.2f, line_length=%.2f, reference_frame=%s, anchor_frame=%s",
                    arc_radius_, line_length_, reference_frame_.c_str(), anchor_frame_.c_str());

        response->waypoints = generate_waypoints(); // Generate waypoints and set in the response
    }

    geometry_msgs::msg::Pose convertTf2TransformToPose(const tf2::Transform& transform) {
        geometry_msgs::msg::Pose pose;

        // Set position
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();

        // Set orientation
        tf2::Quaternion quat = transform.getRotation();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        return pose;
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_waypoints()
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = anchor_frame_;

        int step = 1;
        bool left_turn = true;
        double x_offset = 0.0;

        geometry_msgs::msg::TransformStamped anchor_reference_tf_msg;
        tf2::Transform anchor_reference_tf;

        while (rclcpp::ok())
        {
            try
            {
                anchor_reference_tf_msg = tf_buffer_.lookupTransform(anchor_frame_, reference_frame_, tf2::TimePointZero);
                tf2::fromMsg(anchor_reference_tf_msg.transform, anchor_reference_tf);
                break;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
                rclcpp::spin_some(this->get_node_base_interface());
            }
        }
        int STARTING_ANGLE;
        int ENDING_ANGLE;
        if (turn_direction_ == "left")
        {
            STARTING_ANGLE = -90;
            ENDING_ANGLE = 100;
        }
        else
        {
            STARTING_ANGLE = 90;
            ENDING_ANGLE = -100;
        }
        double translation_y;
        if (turn_direction_ == "left")
        {
            for (int angle = STARTING_ANGLE; angle < ENDING_ANGLE; angle += step)
            {
                double angle_rad = angle * M_PI / 180.0;
                double x = x_offset + arc_radius_ * std::cos(angle_rad);
                double y = arc_radius_ * std::sin(angle_rad);
                double yaw = angle_rad + M_PI / 2.0;

                Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                tf2::Matrix3x3 tf_rotation_matrix(
                    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

                tf2::Transform tf_base_link_point(tf_rotation_matrix);

                translation_y = y + arc_radius_;

                if (angle == -90)
                {
                    std::cout << "HERE" << std::endl;
                    for (int line = 0; line < line_length_; line += step)
                    {
                        x_offset += static_cast<double>(step) / 100.0;
                        tf_base_link_point.setOrigin(tf2::Vector3(x_offset, translation_y, 0.0));
                        tf2::Transform waypoint = anchor_reference_tf * tf_base_link_point;
                        geometry_msgs::msg::Pose waypoint_msg_pose = convertTf2TransformToPose(waypoint);

                        geometry_msgs::msg::PoseStamped waypoint_msg_pose_stamped;
                        waypoint_msg_pose_stamped.header.frame_id = anchor_frame_;
                        waypoint_msg_pose_stamped.pose = waypoint_msg_pose;

                        path_msg->poses.push_back(waypoint_msg_pose_stamped);
                        waypoints.push_back(waypoint_msg_pose_stamped);
                    }
                }
                else
                {
                    tf_base_link_point.setOrigin(tf2::Vector3(x, translation_y, 0.0));
                    tf2::Transform waypoint = anchor_reference_tf * tf_base_link_point;
                    geometry_msgs::msg::Pose waypoint_msg_pose = convertTf2TransformToPose(waypoint);

                    geometry_msgs::msg::PoseStamped waypoint_msg_pose_stamped;
                    waypoint_msg_pose_stamped.header.frame_id = anchor_frame_;
                    waypoint_msg_pose_stamped.pose = waypoint_msg_pose;

                    path_msg->poses.push_back(waypoint_msg_pose_stamped);
                    waypoints.push_back(waypoint_msg_pose_stamped);
                }
            }
        }
        else
        {
            for (int angle = STARTING_ANGLE; angle > ENDING_ANGLE; angle -= step)
            {
                double angle_rad = angle * M_PI / 180.0;
                double x = x_offset + arc_radius_ * std::cos(angle_rad);
                double y = arc_radius_ * std::sin(angle_rad);
                double yaw = angle_rad + M_PI / 2.0;

                Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                tf2::Matrix3x3 tf_rotation_matrix(
                    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

                tf2::Transform tf_base_link_point(tf_rotation_matrix);

                translation_y = y - arc_radius_;

                if (angle == 80)
                {
                    std::cout << "HERE" << std::endl;
                    for (int line = 0; line < line_length_; line += step)
                    {
                        x_offset += static_cast<double>(step) / 100.0;
                        tf_base_link_point.setOrigin(tf2::Vector3(x_offset, translation_y, 0.0));
                        tf2::Transform waypoint = anchor_reference_tf * tf_base_link_point;
                        geometry_msgs::msg::Pose waypoint_msg_pose = convertTf2TransformToPose(waypoint);

                        geometry_msgs::msg::PoseStamped waypoint_msg_pose_stamped;
                        waypoint_msg_pose_stamped.header.frame_id = anchor_frame_;
                        waypoint_msg_pose_stamped.pose = waypoint_msg_pose;

                        path_msg->poses.push_back(waypoint_msg_pose_stamped);
                        waypoints.push_back(waypoint_msg_pose_stamped);
                    }
                }
                else
                {
                    tf_base_link_point.setOrigin(tf2::Vector3(x, translation_y, 0.0));
                    tf2::Transform waypoint = anchor_reference_tf * tf_base_link_point;
                    geometry_msgs::msg::Pose waypoint_msg_pose = convertTf2TransformToPose(waypoint);

                    geometry_msgs::msg::PoseStamped waypoint_msg_pose_stamped;
                    waypoint_msg_pose_stamped.header.frame_id = anchor_frame_;
                    waypoint_msg_pose_stamped.pose = waypoint_msg_pose;

                    path_msg->poses.push_back(waypoint_msg_pose_stamped);
                    waypoints.push_back(waypoint_msg_pose_stamped);
                }
            }  
        }

        path_publisher_->publish(*path_msg);
        return waypoints;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Service<row_following_msgs::srv::GeneratePath>::SharedPtr service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double arc_radius_;
    double line_length_;
    std::string reference_frame_;
    std::string anchor_frame_;
    std::string turn_direction_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeneratePathServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
