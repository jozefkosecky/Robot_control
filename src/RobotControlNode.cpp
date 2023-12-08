#include "robot_control/RobotControlNode.hpp"


using namespace std::chrono_literals;

RobotControlNode::RobotControlNode() : Node("robot_control_node") {

    // publisher
    m_chessBoardPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publishCheckerboard();

    // Create a temporary shared pointer with a null-deleter
        // to prevent the instance from being destroyed when it
        // goes out of scope:
    // auto ptr = std::shared_ptr<RobotControlNode>( this, [](RobotControlNode*){} );

    // // We can now call shared_from_this() in the constructor:
    // move();

    // timer_ = this->create_wall_timer(
    //     500ms, 
    //     [this]() {
    //         this->move();
    //         this->timer_->cancel(); // Cancel the timer after it fires once
    //     }
    // );
}

void RobotControlNode::initMoveGroup() {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");

    move();
}

void RobotControlNode::move() {
    // using moveit::planning_interface::MoveGroupInterface;

    // auto move_group_interface = MoveGroupInterface(shared_from_this(), "ur_manipulator");

    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;
    move_group_interface->setPoseTarget(target_pose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface->plan(plan));

    // Execute the plan
    if (success) {
        move_group_interface->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
}

void RobotControlNode::publishCheckerboard()
{
    visualization_msgs::msg::MarkerArray marker_array;
        int rows = 8;
        int cols = 8;
        float square_size = 0.1;  // Size of each square

        // Create the checkerboard squares and pieces
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                // Add square
                visualization_msgs::msg::Marker square_marker;
                square_marker.header.frame_id = "world";
                square_marker.type = visualization_msgs::msg::Marker::CUBE;
                square_marker.action = visualization_msgs::msg::Marker::ADD;

                square_marker.pose.position.x = row * square_size;
                square_marker.pose.position.y = col * square_size;
                square_marker.pose.position.z = 0.0;
                square_marker.pose.orientation.w = 1.0;

                square_marker.scale.x = square_size;
                square_marker.scale.y = square_size;
                square_marker.scale.z = 0.01;

                if ((row + col) % 2 == 0)
                {
                    square_marker.color.r = 1.0;
                    square_marker.color.g = 1.0;
                    square_marker.color.b = 1.0;
                }
                else
                {
                    square_marker.color.r = 0.0;
                    square_marker.color.g = 0.0;
                    square_marker.color.b = 0.0;
                }

                square_marker.color.a = 1.0;  // Alpha
                square_marker.id = row * cols + col;

                marker_array.markers.push_back(square_marker);
            }
        }

        m_chessBoardPub->publish(marker_array);
}
