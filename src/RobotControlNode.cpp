#include "robot_control/RobotControlNode.hpp"


using namespace std::chrono_literals;

RobotControlNode::RobotControlNode() : Node("robot_control_node") {

    // publisher
    chessBoardPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    // subcriber
    checkersBoardSub = this->create_subscription<checkers_msgs::msg::Board>(
        "board_topic", 10, std::bind(&RobotControlNode::checkers_board_callback, this, std::placeholders::_1));

    checkersMoveSub = this->create_subscription<checkers_msgs::msg::Move>(
        "move_topic", 10, std::bind(&RobotControlNode::checkers_move_callback, this, std::placeholders::_1));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    

    square_size = 0.047;  // Size of each square

    // DON'T FORGET. BOAR IS ROTATED ABOUT 90 degrees
    boardOffsetX = (square_size * 12);
    boardOffsetY = 0.3;


    // geometry_msgs::msg::Pose target_pose1;
    // // target_pose1.orientation.x = 0.5;
    // // target_pose1.orientation.y = 0.5;
    // // target_pose1.orientation.z = -0.5;
    // // target_pose1.orientation.w = 0.5;

    // target_pose1.orientation.w = 0.0;
    // target_pose1.orientation.x = 1.0;
    // target_pose1.orientation.y = 0.0; //-5.563795639318414e-06
    // target_pose1.orientation.z = 0.0; //-0.000838900392409414
    // target_pose1.position.x = 0.4765383303165436;
    // target_pose1.position.y = -0.16665436327457428;
    // target_pose1.position.z = 0.04123930260539055;

    // // Second Pose
    // geometry_msgs::msg::Pose target_pose2;
    // target_pose2.orientation.x = 0.0;
    // target_pose2.orientation.y = 1.0;
    // target_pose2.orientation.z = 0.0;
    // target_pose2.orientation.w = 0.0;
    // target_pose2.position.x = 0.4765383303165436;
    // target_pose2.position.y = -0.16665436327457428 + (square_size*7);
    // target_pose2.position.z = 0.04123930260539055;

    // pose_list.push_back(target_pose1);
    // pose_list.push_back(target_pose2);
    // target_pose = pose_list[target_pose_index];
    

}

void RobotControlNode::initMoveGroup() {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_interface->setEndEffectorLink("tool0");
    move_group_interface->startStateMonitor();

    publishCheckerboard();
    
    // createPiece(0, 0);
    // attachPiece();
    // usleep(2000000);
    // detachPiece();
    // removePiece();


    // targetPositions.push_back(Mission(0, 0, Task::ATTACH));
    // targetPositions.push_back(Mission(0, 5, Task::DETACH));

    // trajectory_list = getPoseList(targetPositions[target_pose_index]);
    // target_pose = trajectory_list[trajectory_pose_index].first;
    // move(target_pose);

    // move(target_pose);
}

void RobotControlNode::mainLoop() {
    if((target_pose_index >= 0 && static_cast<std::size_t>(target_pose_index) >= targetPositions.size()) || doingTask) {
        std::cout << "Return FIRST"<< std::endl;
        std::cout << "Size: " << targetPositions.size() << std::endl;
        std::cout << "target_pose_index: " << target_pose_index << std::endl;
        std::cout << "doingTask: " << doingTask << std::endl;
        std::cout << "-------------------------------" << std::endl;
        
        return;
    }

    tool0_pose = getPose();

    auto result = checkPosition(tool0_pose, target_pose);
    bool isCloseEnough = result.first;
    // double distance = result.second;

    if(isCloseEnough == false) {
        std::cout << "Return SECOND" << std::endl;
        std::cout << "-------------------------------" << std::endl;
        attempts++;

        if(attempts == 3) {
            move(target_pose);
            attempts = 0;
        }
        return;
    }

    
    
    if(trajectory_pose_index >= 0 && static_cast<std::size_t>(trajectory_pose_index + 1) < trajectory_list.size()) {
        RCLCPP_WARN(shared_from_this()->get_logger(), "\n\nDalsi bod na trajektori.");
        std::cout << "Size: " << trajectory_list.size() << std::endl;
        std::cout << "trajectory_pose_index: " << trajectory_pose_index << std::endl;
    
        // Taking task at the end of the move.
        Task task = trajectory_list[trajectory_pose_index].second;
        trajectory_pose_index++;
        if(task != Task::NONE) {
            std::cout << "Robim TASK" << std::endl;
            Mission mission = targetPositions[target_pose_index];
            makeTask(mission);  
            usleep(500000);
        }
        
        std::cout << "trajectory_pose_index: " << trajectory_pose_index << std::endl;
        std::cout << "-------------------------------" << std::endl;
        target_pose = trajectory_list[trajectory_pose_index].first;
        move(target_pose);
        return;
    }

    
    if(target_pose_index >= 0 && static_cast<std::size_t>(target_pose_index + 1) < targetPositions.size()) {
        target_pose_index++;

        RCLCPP_ERROR(shared_from_this()->get_logger(), "\n\nDalsia misia.");
        std::cout << "Size: " << targetPositions.size() << std::endl;
        std::cout << "target_pose_index: " << target_pose_index << std::endl;
        std::cout << "-------------------------------" << std::endl;;

        trajectory_list = getPoseList(targetPositions[target_pose_index]);

        target_pose = trajectory_list[trajectory_pose_index].first;
        usleep(500000);
        move(target_pose);
    }
   
    // usleep(2000000); // Sleep for 2000000 microseconds (2 seconds)
}

void RobotControlNode::makeTask(Mission mission) {
    doingTask = true;

    Task task = mission.task;
    int row = mission.row;
    int col = mission.col;
    std::string color = mission.color;

    std::string pieceID = "piece" + std::to_string(row) + std::to_string(col);

    if(task == Task::ATTACH) {
        removeFakePiece(pieceID);

        createPiece(row, col);
        attachPiece();
    }
    else {
        detachPiece();
        removePiece();
        createFakePieceWithColor(pieceID, row, col, color);
        chessBoardPub->publish(marker_array_fake_pieces);
    }

    targetPositions[target_pose_index].task = Task::NONE;
    doingTask = false;
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
        RCLCPP_DEBUG(shared_from_this()->get_logger(), "\n\nWe are close enough to the target!\n\n");
        // std::cout << "We are close enough to the target!" << std::endl;
        return std::make_pair(true, distance);
    } else {
        RCLCPP_DEBUG(shared_from_this()->get_logger(), "\n\nStill on the way to the target.\n\n");
        // std::cout << "Still on the way to the target." << std::endl;
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

void RobotControlNode::checkers_move_callback(const checkers_msgs::msg::Move::SharedPtr msg) {
    targetPositions.clear();
    target_pose_index = 0;
    

    // First Mission: Attach at the start position
    auto [startRow, startCol] = rotate90DegreesCounterClockwise(msg->piece_for_moving.row, msg->piece_for_moving.col);

    targetPositions.push_back(Mission(startRow, startCol, msg->piece_for_moving.color, Task::ATTACH));

    // Second Mission: Detach at the target position
    auto [targetRow, targetCol] = rotate90DegreesCounterClockwise(msg->target_row, msg->target_col);

    targetPositions.push_back(Mission(targetRow, targetCol, msg->piece_for_moving.color, Task::DETACH));

    // Additional Missions for removed pieces
    for (const auto& piece : msg->removed_pieces) {
        auto [row, col] = rotate90DegreesCounterClockwise(piece.row, piece.col);

        targetPositions.push_back(Mission(row, col, piece.color, Task::ATTACH));

        targetPositions.push_back(Mission(0, -1, piece.color, Task::DETACH));
    }

    trajectory_list = getPoseList(targetPositions[target_pose_index]);
    target_pose = trajectory_list[trajectory_pose_index].first;
    move(target_pose);
}

std::vector<std::pair<geometry_msgs::msg::Pose, Task>> RobotControlNode::getPoseList(Mission mission) {
    trajectory_pose_index = 0;
    std::vector<std::pair<geometry_msgs::msg::Pose, Task>> poses;

    
    // float posX = (mission.row * square_size) + boardOffsetX + (square_size/2);
    // float posY = (mission.col * square_size) + boardOffsetY + (square_size/2);

    std::cout << "-------------------------------" << std::endl;
    std::cout << "mission.row: " << mission.row << std::endl;
    std::cout << "mission.col: " << mission.col << std::endl;
    std::cout << "square_size: " << square_size << std::endl;
    std::cout << "boardOffsetX: " << boardOffsetX << std::endl;
    std::cout << "boardOffsetY: " << boardOffsetY << std::endl;
    std::cout << "-------------------------------" << std::endl;

    float posX = (mission.row * square_size) + boardOffsetX;
    float posY = (mission.col * square_size) + boardOffsetY;

    std::cout << "-------------------------------" << std::endl;
    std::cout << "posX: " << posX << std::endl;
    std::cout << "posY: " << posY << std::endl;
    std::cout << "-------------------------------" << std::endl;

    geometry_msgs::msg::Pose pose1;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 1.0;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 0.0;
    pose1.position.x = posX;
    pose1.position.y = posY;
    pose1.position.z = 0.04123930260539055 + 0.30;
    poses.push_back(std::make_pair(pose1, Task::NONE));
    

    geometry_msgs::msg::Pose pose2;
    pose2.orientation.x = 0.0;
    pose2.orientation.y = 1.0;
    pose2.orientation.z = 0.0;
    pose2.orientation.w = 0.0;
    pose2.position.x = posX;
    pose2.position.y = posY;
    pose2.position.z = 0.04123930260539055;
    poses.push_back(std::make_pair(pose2, mission.task));

    geometry_msgs::msg::Pose pose3;
    pose3.orientation.x = 0.0;
    pose3.orientation.y = 1.0;
    pose3.orientation.z = 0.0;
    pose3.orientation.w = 0.0;
    pose3.position.x = posX;
    pose3.position.y = posY;
    pose3.position.z = 0.042 + 0.30;
    poses.push_back(std::make_pair(pose3, Task::NONE));

    return poses;
}


void RobotControlNode::checkers_board_callback(const checkers_msgs::msg::Board::SharedPtr msg)
{
    removeAllFakePieces();

    for (const auto& piece : msg->pieces) {

        auto [row, col] = rotate90DegreesCounterClockwise(piece.row, piece.col);
        
        std::string color = piece.color;
        // bool isKing = piece.king;

        std::string pieceID = "piece" + std::to_string(row) + std::to_string(col);
        
        createFakePieceWithColor(pieceID, row, col, color);
    }

    chessBoardPub->publish(marker_array_fake_pieces);

    // removeFakePiece("piece01");
    
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



void RobotControlNode::createFakePieceWithColor(const std::string& object_id, int row, int col, const std::string& colorName) {
    int objectIDLong = convertStringToInt(object_id);

    piecesInRviz[objectIDLong] = true;

    std::tuple<float, float, float> color = getColorFromName(colorName);
    
    visualization_msgs::msg::Marker fakePiece;
    // fakePiece.header.frame_id = move_group_interface->getPlanningFrame();
    fakePiece.header.frame_id = "base_link";
    fakePiece.id = objectIDLong;

    fakePiece.type = visualization_msgs::msg::Marker::CYLINDER;
    fakePiece.action = visualization_msgs::msg::Marker::ADD;

    fakePiece.pose.position.x = (row * square_size) + boardOffsetX;
    fakePiece.pose.position.y = (col * square_size) + boardOffsetY;
    fakePiece.pose.position.z = 0.0075;
    fakePiece.pose.orientation.w = 1.0;

    fakePiece.scale.x = 0.028;
    fakePiece.scale.y = 0.028;
    fakePiece.scale.z = 0.005;

    fakePiece.color.r = std::get<0>(color);
    fakePiece.color.g = std::get<1>(color);
    fakePiece.color.b = std::get<2>(color);
    fakePiece.color.a = 1.0;  // Alpha

    marker_array_fake_pieces.markers.push_back(fakePiece);

    
}

int RobotControlNode::convertStringToInt(const std::string& stringID){
    long long concatenatedNumber = 0;

    for (char c : stringID) {
        concatenatedNumber = concatenatedNumber * 1000 + static_cast<int>(c);
    }

    return concatenatedNumber;
}

void RobotControlNode::removeAllFakePieces() {
    for (auto& marker : marker_array_fake_pieces.markers) {
        int markerID = marker.id;
        if (piecesInRviz.find(markerID) != piecesInRviz.end()) {
            bool isOnTheBoard = piecesInRviz[markerID];
            if(isOnTheBoard) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
            }
        }
    }

    chessBoardPub->publish(marker_array_fake_pieces);
    piecesInRviz.clear();
    marker_array_fake_pieces.markers.clear();
}

void RobotControlNode::removeFakePiece(const std::string& object_id) {
    int objectIDLong = convertStringToInt(object_id);
    // visualization_msgs::msg::MarkerArray marker_array;

    // visualization_msgs::msg::Marker removePiece;
    // removePiece.header.frame_id = move_group_interface->getPlanningFrame();
    // removePiece.id = objectIDLong;
    // removePiece.action = visualization_msgs::msg::Marker::DELETE;

    // marker_array.markers.push_back(removePiece);

    // chessBoardPub->publish(marker_array);

    // visualization_msgs::msg::Marker marker_for_delete;
    // for (auto& marker : marker_array_fake_pieces.markers) {
    //     int markerID = marker.id;
    //     if(objectIDLong == markerID){
    //         marker.action = visualization_msgs::msg::Marker::DELETE;
    //         marker_for_delete = marker;
    //         break;
    //     }
    // }


    visualization_msgs::msg::MarkerArray updated_marker_array;

    for (auto& marker : marker_array_fake_pieces.markers) {
        if (marker.id != objectIDLong) {  // Keep all markers except the one to remove
            updated_marker_array.markers.push_back(marker);
        }
        else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
            if (piecesInRviz.find(objectIDLong) != piecesInRviz.end()) {
                piecesInRviz.erase(objectIDLong);
            }
        }
    }

    chessBoardPub->publish(marker_array_fake_pieces);

    marker_array_fake_pieces = updated_marker_array;
}

void RobotControlNode::createPiece(int row, int col) {
    collision_object = moveit_msgs::msg::CollisionObject();
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "collisionObjectID";
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
    

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::removePiece() {
    collision_object.operation = collision_object.REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::attachPiece() {
    move_group_interface->attachObject(collision_object.id, "tool0"); 
}

void RobotControlNode::detachPiece() {
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

                auto [rotatedRow, rotatedCol] = rotate90DegreesCounterClockwise(row, col);

                // Add square
                visualization_msgs::msg::Marker square_marker;
                square_marker.header.frame_id = move_group_interface->getPlanningFrame();
                square_marker.type = visualization_msgs::msg::Marker::CUBE;
                square_marker.action = visualization_msgs::msg::Marker::ADD;

                square_marker.pose.position.x = (rotatedRow * square_size) + boardOffsetX;
                square_marker.pose.position.y = (rotatedCol * square_size) + boardOffsetY;
                square_marker.pose.position.z = 0.0;
                square_marker.pose.orientation.w = 1.0;

                square_marker.scale.x = square_size;
                square_marker.scale.y = square_size;
                square_marker.scale.z = 0.01;

                if ((row + col) == 0)
                {
                    square_marker.color.r = 0.0;
                    square_marker.color.g = 1.0;
                    square_marker.color.b = 0.0;
                }
                else if ((rotatedRow + rotatedCol) % 2 == 0)
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
                square_marker.id = rotatedRow * cols + rotatedCol;

                marker_array.markers.push_back(square_marker);
            }
        }

        chessBoardPub->publish(marker_array);
}


std::pair<int, int> RobotControlNode::rotate90DegreesCounterClockwise(int x, int y) {
    return {-y, x};
}