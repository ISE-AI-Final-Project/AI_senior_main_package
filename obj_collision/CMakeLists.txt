cmake_minimum_required(VERSION 3.8)
project(obj_collision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

add_executable(obj_collision src/obj_collision.cpp)
target_include_directories(obj_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(obj_collision PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  obj_collision
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "tf2_geometry_msgs"
  "tf2_ros"
  "moveit_core"
  "rviz_visual_tools"
  "moveit_visual_tools"
  "interactive_markers"
  "tf2_geometry_msgs"
  "moveit_ros_planning"
  "pluginlib"
  "Eigen3"
  "Boost"
  "control_msgs"
  "moveit_servo"
  "moveit_task_constructor_core"
  "PCL"

)

install(TARGETS obj_collision
  DESTINATION lib/${PROJECT_NAME})

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
