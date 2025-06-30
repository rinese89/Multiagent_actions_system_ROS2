cmake_minimum_required(VERSION 3.8)
project(controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tb3_actions REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs)  
find_package(visualization_msgs REQUIRED)
find_package(std_msgs)


#---- Reference Msg Dependencies -----------

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Reference.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
 )

#---- Executables & Dependencies -----

add_executable(control_follower_server_node
"src/control_follower_server_node.cpp")

ament_target_dependencies(
  control_follower_server_node
  "rclcpp"
  "rclcpp_action"
  "tb3_actions"
  "geometry_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
  "nav_msgs"
  "std_msgs"
)

add_executable(control_leader_server_node
"src/control_leader_server_node.cpp")

ament_target_dependencies(
  control_leader_server_node
  "rclcpp"
  "rclcpp_action"
  "tb3_actions"
  "geometry_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
  "nav_msgs"
  "std_msgs"
)

#---- Executables & Dependencies -----

add_executable(reference_server_node
"src/reference_server_node.cpp")

ament_target_dependencies(
  reference_server_node
  "rclcpp"
  "rclcpp_action"
  "tb3_actions"
  "geometry_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "std_msgs"

)

#---- Executables & Dependencies -----

add_executable(
  control_lc_client_node
"src/control_lc_client_node.cpp")

ament_target_dependencies(
  control_lc_client_node
  "rclcpp"
  "rclcpp_action"
  "tb3_actions"
  "geometry_msgs"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
)

add_executable(
  odom_transform_inverter_node
"src/odom_transform_inverter_node.cpp")

ament_target_dependencies(
  odom_transform_inverter_node
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

add_executable(
  odom_publish_tb30_node
"src/odom_publish_tb30_node.cpp")

ament_target_dependencies(
  odom_publish_tb30_node
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
)

add_executable(
  odom_publish_tb31_node
"src/odom_publish_tb31_node.cpp")

ament_target_dependencies(
  odom_publish_tb31_node
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
)

#------ linked against msg library

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(reference_server_node "${cpp_typesupport_target}")
target_link_libraries(control_follower_server_node "${cpp_typesupport_target}")
target_link_libraries(control_leader_server_node "${cpp_typesupport_target}")
target_link_libraries(control_lc_client_node "${cpp_typesupport_target}")

install(TARGETS
control_follower_server_node
control_leader_server_node
reference_server_node
control_lc_client_node
odom_publish_tb30_node
odom_publish_tb31_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg launch
DESTINATION share/${PROJECT_NAME})

ament_package()
