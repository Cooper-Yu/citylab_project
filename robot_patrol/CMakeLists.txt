cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(patrol_node src/patrol.cpp)
add_executable(direction_service_server src/direction_service.cpp)
add_executable(test_service_client src/test_service.cpp)
add_executable(patrol_with_service_node src/patrol_with_service.cpp)
add_executable(go_to_pose_action_server src/go_to_pose_action.cpp)

ament_target_dependencies(patrol_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(direction_service_server
  rclcpp
  sensor_msgs
)

ament_target_dependencies(test_service_client
  rclcpp
  sensor_msgs
)

ament_target_dependencies(patrol_with_service_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(go_to_pose_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
)

target_link_libraries(direction_service_server "${cpp_typesupport_target}")
target_link_libraries(test_service_client "${cpp_typesupport_target}")
target_link_libraries(patrol_with_service_node "${cpp_typesupport_target}")
target_link_libraries(go_to_pose_action_server "${cpp_typesupport_target}")

install(TARGETS
  patrol_node
  direction_service_server
  test_service_client
  patrol_with_service_node
  go_to_pose_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()