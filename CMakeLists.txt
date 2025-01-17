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
find_package(Eigen3 REQUIRED)

add_executable(imu_preint_node src/go2_imu_preint.cpp)
add_executable(fake_imu_node src/fake_imu_data.cpp)
ament_target_dependencies(fake_imu_node rclcpp sensor_msgs std_msgs)

target_include_directories(imu_preint_node PUBLIC
                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/equivariant-preintegration/include>
							            $<INSTALL_INTERFACE:third_party/equivariant-preintegration/include>  
)

target_include_directories(imu_preint_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/Lie-plusplus/include>
							              $<INSTALL_INTERFACE:third_party/Lie-plusplus/include> 
)

target_link_libraries(imu_preint_node Eigen3::Eigen rclcpp)
ament_target_dependencies(imu_preint_node rclcpp sensor_msgs std_msgs)

add_executable(state_converter_node src/go2_state_converter_node.cpp)
ament_target_dependencies(state_converter_node rclcpp unitree_go sensor_msgs)

install(TARGETS
  fake_imu_node
  imu_preint_node
  state_converter_node
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
