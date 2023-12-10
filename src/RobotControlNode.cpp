#include "robot_control/RobotControlNode.hpp"


using namespace std::chrono_literals;

RobotControlNode::RobotControlNode() : Node("robot_control_node") {

    // publisher
    chessBoardPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publishCheckerboard();

    geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.x = 0.5;
    // target_pose1.orientation.y = 0.5;
    // target_pose1.orientation.z = -0.5;
    // target_pose1.orientation.w = 0.5;
    target_pose1.orientation.w = -0.021076412871479988;
    target_pose1.orientation.x = 0.9997775554656982;
    target_pose1.orientation.y = 4.4367559894453734e-05;
    target_pose1.orientation.z = -0.0007643926655873656;
    target_pose1.position.x = 0.49955984950065613;
    target_pose1.position.y = 0.4997904896736145;
    target_pose1.position.z = 0.2755698561668396;

    // Second Pose
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.w = -0.021076412871479988;
    target_pose2.orientation.x = 0.9997775554656982;
    target_pose2.orientation.y = 4.4367559894453734e-05;
    target_pose2.orientation.z = -0.0007643926655873656;
    target_pose2.position.x = 0.49955984950065613;
    target_pose2.position.y = 0.4997904896736145;
    target_pose2.position.z = 0.5055698561668396;

    pose_list.push_back(target_pose1);
    pose_list.push_back(target_pose2);
    target_pose = pose_list[target_pose_index];
}

void RobotControlNode::initMoveGroup() {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_interface->startStateMonitor();

    // createStone();
    // move();
    // attachStone();
    // move2();
    // detachStone();
    // move();
    
    // move2();
    move(target_pose);
}

void RobotControlNode::mainLoop() {
    tool0_pose = getPose();

    auto result = checkPosition(tool0_pose, target_pose);
    bool isCloseEnough = result.first;
    // double distance = result.second;

    if(isCloseEnough == false) {
        return;
    }

    target_pose_index++;
    if(target_pose_index == 2) {
        target_pose_index = 0;
    }
    target_pose = pose_list[target_pose_index];
    move(target_pose);

}

std::pair<bool, double> RobotControlNode::checkPosition(const geometry_msgs::msg::Pose& current_local_pos, const geometry_msgs::msg::Pose& target_position) {
    double threshold = 0.02;

    double current_x = current_local_pos.position.x;
    double current_y = current_local_pos.position.y;
    double current_z = current_local_pos.position.z;
    if (current_z < 0) {
        current_z = 0.0;
    }

    double target_x = target_position.position.x;
    double target_y = target_position.position.y;
    double target_z = target_position.position.z;

    double distance = euclideanDistance(current_x, current_y, current_z, target_x, target_y, target_z);

    if (distance < threshold) {
        std::cout << "We are close enough to the target!" << std::endl;
        return std::make_pair(true, distance);
    } else {
        std::cout << "Still on the way to the target." << std::endl;
        return std::make_pair(false, distance);
    }
}

double RobotControlNode::euclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}



geometry_msgs::msg::Pose RobotControlNode::getPose() {
    // Get the current state of the robot
    move_group_interface->setStartStateToCurrentState();
    auto currentPosition = move_group_interface->getCurrentPose().pose;
    
    // Get the pose of the end-effector
    // const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("tool0");

    
    // // Convert to a PoseStamped message if needed
    // geometry_msgs::msg::PoseStamped tool_pose;
    // tf2::convert(end_effector_state, tool_pose.pose);

    return currentPosition;
}

void RobotControlNode::move(geometry_msgs::msg::Pose targetPose) {
    // Set a target Pose
    move_group_interface->setPoseTarget(targetPose);

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

void RobotControlNode::move2() {
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = -0.021076412871479988;
    target_pose.orientation.x = 0.9997775554656982;
    target_pose.orientation.y = 4.4367559894453734e-05;
    target_pose.orientation.z = -0.0007643926655873656;
    target_pose.position.x = 0.49955984950065613;
    target_pose.position.y = 0.4997904896736145;
    target_pose.position.z = 0.4555698561668396;
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

void RobotControlNode::createStone() {
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.03;
    primitive.dimensions[primitive.BOX_Y] = 0.03;
    primitive.dimensions[primitive.BOX_Z] = 0.03;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::attachStone() {
    move_group_interface->attachObject(collision_object.id, "tool0"); 
}

void RobotControlNode::detachStone() {
    move_group_interface->detachObject(collision_object.id); 
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

        chessBoardPub->publish(marker_array);
}
