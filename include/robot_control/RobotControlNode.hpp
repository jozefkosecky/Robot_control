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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <cmath>
#include <utility>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>  // For Eigen conversions
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For geometry_msgs conversions
#include "checkers_msgs/msg/board.hpp" 
#include "checkers_msgs/msg/piece.hpp" 
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <std_msgs/msg/color_rgba.hpp>


class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode();

    // functions
    void publishCheckerboard();
    void mainLoop();
    void initMoveGroup();
    void move(geometry_msgs::msg::Pose targetPose);
    void attachStone();
    void detachStone();
    void createPiece(const std::string& object_id, int row, int col, const std::string& colorName);
    std::tuple<float, float, float> getColorFromName(const std::string& colorName);
    void removeObjectById(const std::string& object_id);



    geometry_msgs::msg::Pose getPose();
    std::pair<bool, double> checkPosition(const geometry_msgs::msg::Pose& current_local_pos, const geometry_msgs::msg::Pose& target_position);
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    


private:

    // functions
    double euclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2);
    
    
    //Var Moveit
    const std::string PLANNING_GROUP = "ur_manipulator";
    

    // publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr chessBoardPub;
    rclcpp::TimerBase::SharedPtr timer;

    // Subcriber
    rclcpp::Subscription<checkers_msgs::msg::Board>::SharedPtr checkersBoardSub;
    void checkers_board_callback(const checkers_msgs::msg::Board::SharedPtr msg);


    // Variables
    std::vector<geometry_msgs::msg::Pose> pose_list;
    geometry_msgs::msg::Pose tool0_pose;
    geometry_msgs::msg::Pose target_pose;
    int target_pose_index = 0;
    bool startProgram = true;
    std::vector<std::pair<std::string, bool>> piecesInRviz;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    float square_size;  // Size of each square
    float boardOffsetX;
    float boardOffsetY;


};



#endif // ROBOT_CONTROL_NODE_HPP
