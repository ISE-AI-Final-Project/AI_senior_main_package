cmake_minimum_required(VERSION 3.8)
project(colliding_links_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(moveit_core REQUIRED)
find_package(custom_srv_pkg REQUIRED)


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

add_executable(colliding_links_node src/colliding_links_node.cpp)

ament_target_dependencies(colliding_links_node
  rclcpp
  moveit_ros_planning
  moveit_msgs
  moveit_ros_planning_interface

)

install(TARGETS
  colliding_links_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(robot_model_loader_node src/robot_model_loader_node.cpp)

ament_target_dependencies(robot_model_loader_node
  rclcpp
  std_msgs
  urdf
  srdfdom
  moveit_core
)

install(TARGETS robot_model_loader_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(ur_colliding_boxes_pub src/ur_colliding_boxes_pub.cpp)

ament_target_dependencies(ur_colliding_boxes_pub
  rclcpp
  std_msgs
  urdf
  srdfdom
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  custom_srv_pkg
)

install(TARGETS ur_colliding_boxes_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()