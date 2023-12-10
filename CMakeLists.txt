cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include C++ headers
include_directories(include)

# Declare a C++ executable
add_executable(robot_control_node src/RobotControlMain.cpp src/RobotControlNode.cpp)
# Specify libraries to link a library or executable target against
ament_target_dependencies(robot_control_node 
  moveit_core
  moveit_ros_planning_interface 
  rclcpp 
  visualization_msgs 
  geometry_msgs 
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)


add_executable(another_node src/robot_control.cpp)
ament_target_dependencies(another_node moveit_ros_planning_interface rclcpp)

# # Include directories for the build and install phase
target_include_directories(robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(another_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_compile_features(robot_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# Install the executable
install(TARGETS
  robot_control_node
  another_node
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
launch
rviz
DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
