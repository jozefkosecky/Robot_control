#ifndef ROBOT_CONTROL_NODE_HPP
#define ROBOT_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode();

    //Var Moveit
    const std::string PLANNING_GROUP = "ur_manipulator";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;


    // functions
    void publishCheckerboard();
    void move();
    void initMoveGroup();

private:

    // const std::string PLANNING_GROUP;
    // std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;


    // publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_chessBoardPub;
    rclcpp::TimerBase::SharedPtr timer_;
};



#endif // ROBOT_CONTROL_NODE_HPP
