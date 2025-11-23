#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_between_states");

    // Initialize MoveGroup for your robot
    moveit::planning_interface::MoveGroupInterface move_group(node, "Klaw");

    // Optional: set planner parameters
    move_group.setPlanningTime(5.0);

    // Define two named targets (states) you want to move between
    std::string first_state = "Turned";
    std::string second_state = "Flexxed";

    // ---------- Move to first state ----------
    move_group.setNamedTarget(first_state);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (move_group.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success1) {
        std::cout << "Moving to " << first_state << "...\n";
        move_group.execute(plan1);
    } else {
        std::cerr << "Failed to plan to " << first_state << std::endl;
        return 1;
    }

    // ---------- Move to second state ----------
    move_group.setNamedTarget(second_state);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (move_group.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success2) {
        std::cout << "Moving to " << second_state << "...\n";
        move_group.execute(plan2);
    } else {
        std::cerr << "Failed to plan to " << second_state << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
