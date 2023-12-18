#include "robot_control/RobotControlNode.hpp"


using namespace std::chrono_literals;

RobotControlNode::RobotControlNode() : Node("robot_control_node") {

    // publisher
    chessBoardPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    // subcriber
    checkersBoardSub = this->create_subscription<checkers_msgs::msg::Board>(
        "board_topic", 10, std::bind(&RobotControlNode::checkers_board_callback, this, std::placeholders::_1));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    

    square_size = 0.047;  // Size of each square
    boardOffsetX = 0.475;
    boardOffsetY = -(square_size * 4)+ (square_size / 2);

    geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.x = 0.5;
    // target_pose1.orientation.y = 0.5;
    // target_pose1.orientation.z = -0.5;
    // target_pose1.orientation.w = 0.5;

    target_pose1.orientation.w = 0.0;
    target_pose1.orientation.x = 1.0;
    target_pose1.orientation.y = 0.0; //-5.563795639318414e-06
    target_pose1.orientation.z = 0.0; //-0.000838900392409414
    target_pose1.position.x = 0.4765383303165436;
    target_pose1.position.y = -0.16665436327457428;
    target_pose1.position.z = 0.04123930260539055;

    // Second Pose
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation.x = 0.0;
    target_pose2.orientation.y = 1.0;
    target_pose2.orientation.z = 0.0;
    target_pose2.orientation.w = 0.0;
    target_pose2.position.x = 0.4765383303165436;
    target_pose2.position.y = -0.16665436327457428 + (square_size*7);
    target_pose2.position.z = 0.04123930260539055;

    pose_list.push_back(target_pose1);
    pose_list.push_back(target_pose2);
    target_pose = pose_list[target_pose_index];


    
}

void RobotControlNode::initMoveGroup() {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    // move_group_interface->setEndEffectorLink("tool0");
    move_group_interface->startStateMonitor();

    publishCheckerboard();
    
    // move(target_pose);
}

void RobotControlNode::mainLoop() {
    tool0_pose = getPose();

    auto result = checkPosition(tool0_pose, target_pose);
    bool isCloseEnough = result.first;
    // double distance = result.second;

    if(isCloseEnough == false) {
        // move(target_pose);
        return;
    }

    if(target_pose_index == 0) {
        attachStone();
    }
    else {
        detachStone();
    }

    target_pose_index++;
    if(target_pose_index == 2) {
        target_pose_index = 0;
    }
 
    target_pose = pose_list[target_pose_index];
    move(target_pose);

}


std::pair<bool, double> RobotControlNode::checkPosition(const geometry_msgs::msg::Pose& current_local_pos, const geometry_msgs::msg::Pose& target_position) {
    double threshold = 0.05;

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

void RobotControlNode::checkers_board_callback(const checkers_msgs::msg::Board::SharedPtr msg)
{
    for (const auto& element : piecesInRviz) {
        std::string objectID= element.first;
        bool isOnTheBoard = element.second;

        if(isOnTheBoard) {
            removeObjectById(objectID);
        }
        // Do something with theString and theBool
    }

    for (const auto& piece : msg->pieces) {
        int row = piece.row;
        int col = piece.col;
        std::string color = piece.color;
        // bool isKing = piece.king;

        std::string pieceID = "piece" + std::to_string(row) + std::to_string(col);
        
        createPiece(pieceID, row, col, color);
    }

    if(startProgram) {
        // move(target_pose);
        startProgram = false;
    }
}

std::tuple<float, float, float> RobotControlNode::getColorFromName(const std::string& colorName) {
    if (colorName == "white") {
        return std::make_tuple(1.0f, 1.0f, 1.0f); // RGB for white
    } else if (colorName == "red") {
        return std::make_tuple(1.0f, 0.0f, 0.0f); // RGB for red
    }

    // Default color (black) if no match is found
    return std::make_tuple(0.0f, 0.0f, 0.0f);
}

void RobotControlNode::createPiece(const std::string& object_id, int row, int col, const std::string& colorName) {
    piecesInRviz.push_back(std::make_pair(object_id, true));

    std::tuple<float, float, float> color = getColorFromName(colorName);

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = object_id;
    shape_msgs::msg::SolidPrimitive primitive;


    float posX = (row * square_size) + boardOffsetX;
    float posY = (col * square_size) + boardOffsetY;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.005; // Height of the cylinder
    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.028/2; // Radius of the cylinder

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = posX;
    cylinder_pose.position.y = posY;
    cylinder_pose.position.z = 0.0075;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    

    // Set the color for the object (this part is optional and depends on your RViz setup)
    moveit_msgs::msg::ObjectColor object_color;
    object_color.id = object_id;
    object_color.color.r = std::get<0>(color);
    object_color.color.g = std::get<1>(color);
    object_color.color.b = std::get<2>(color);
    object_color.color.a = 1.0; // Alpha value for opacity (1.0 is fully opaque)

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::removeObjectById(const std::string& object_id) {
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = object_id;
    remove_object.header.frame_id = move_group_interface->getPlanningFrame();
    remove_object.operation = remove_object.REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(remove_object);
}

void RobotControlNode::attachStone() {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "piece01";
    move_group_interface->attachObject(collision_object.id, "tool0"); 
}

void RobotControlNode::detachStone() {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "piece01";
    move_group_interface->detachObject(collision_object.id); 
}

void RobotControlNode::publishCheckerboard()
{
    visualization_msgs::msg::MarkerArray marker_array;
        int rows = 8;
        int cols = 8;
        

        // Create the checkerboard squares and pieces
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                // Add square
                visualization_msgs::msg::Marker square_marker;
                square_marker.header.frame_id = move_group_interface->getPlanningFrame();
                square_marker.type = visualization_msgs::msg::Marker::CUBE;
                square_marker.action = visualization_msgs::msg::Marker::ADD;

                square_marker.pose.position.x = (row * square_size) + boardOffsetX;
                square_marker.pose.position.y = (col * square_size) + boardOffsetY;
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
