cmake_minimum_required(VERSION 3.8)
project(go2_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(inekf REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(go2_description REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build msgs
set(msg_files
  "msg/OdometryVector.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

add_executable(state_converter_node src/go2_state_converter_node.cpp)
ament_target_dependencies(state_converter_node rclcpp unitree_go sensor_msgs)

add_executable(inekf_odometry src/inekf_odometry.cpp)
target_link_libraries(inekf_odometry inekf::inekf)
ament_target_dependencies(inekf_odometry rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros unitree_go)

add_executable(feet_to_odom src/feet_to_odom.cpp)
target_link_libraries(feet_to_odom pinocchio::pinocchio go2_description::go2_description)
ament_target_dependencies(feet_to_odom rclcpp sensor_msgs nav_msgs geometry_msgs unitree_go)

rosidl_target_interfaces(state_converter_node 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(inekf_odometry 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  rosidl_target_interfaces(feet_to_odom 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  state_converter_node
  inekf_odometry
  feet_to_odom
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
file(GLOB SCRIPTS_LIST scripts/*.py)

install(PROGRAMS
  ${SCRIPTS_LIST}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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
