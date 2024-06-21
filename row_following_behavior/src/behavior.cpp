#include "row_following_behavior/generate_path_service_node.hpp"
#include "row_following_behavior/follow_path_action_node.hpp"
#include "row_following_behavior/is_out_of_field_condition_node.hpp"
#include "row_following_behavior/follow_row_action_node.hpp"
#include "row_following_behavior/follow_row_topic_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_ros2/plugins.hpp"

#include <string>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>

// using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("row_following_behavior");
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

    node->declare_parameter<std::string>("set_bt_tree", "main_tree");
    std::string test_twist = node->get_parameter("set_bt_tree").as_string();

    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;

    params.default_port_value = "tf";
    factory.registerNodeType<IsOutOfField>("IsOutOfField", params);

    params.default_port_value = "out/of/field";
    factory.registerNodeType<FollowRow>("FollowRowTopicNode", params);

    params.default_port_value = "generate_path";
    factory.registerNodeType<GeneratePath>("GeneratePath", params);

    params.default_port_value = "follow_path";
    factory.registerNodeType<FollowPath>("FollowPath", params);

    using std::filesystem::directory_iterator;
    for (auto const &entry : directory_iterator(BEHAVIOR_DIRECTORY))
        if (entry.path().extension() == ".xml")
            factory.registerBehaviorTreeFromFile(entry.path().string());
    BT::Tree tree = factory.createTree(test_twist, blackboard);
    BT::StdCoutLogger logger_cout(tree);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.tickOnce() == BT::NodeStatus::SUCCESS;
        tree.sleep(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}