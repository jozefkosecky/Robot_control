#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_control/RobotControlNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto robot_control_node = std::make_shared<RobotControlNode>();

    
    robot_control_node->initMoveGroup();

    // rclcpp::Rate rate(10); // 10 Hz, adjust the frequency as needed

    // while (rclcpp::ok()) {
    //     // robot_control_node->main_loop();

    //     // Handle callbacks
    //     rclcpp::spin_some(robot_control_node);

    //     // Sleep for the rest of the cycle
    //     rate.sleep();
    // }

    rclcpp::spin(robot_control_node);
    rclcpp::shutdown();
    return 0;
}