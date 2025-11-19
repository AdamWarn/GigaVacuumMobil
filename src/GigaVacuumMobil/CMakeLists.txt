cmake_minimum_required(VERSION 3.16)
project(GigaVacuumMobil)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_toolbox REQUIRED)

# Create the hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/diffdrive_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

# Export the hardware interface as a plugin
pluginlib_export_plugin_description_file(hardware_interface GigaVacuumMobil.xml)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install the include directory
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(
  PROGRAMS
    scripts/cmd_vel_relay.py
    scripts/odom_frame_fixer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds directory for Gazebo
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz configs
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps directory
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Arduino code (for reference)
install(
  DIRECTORY arduino
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(
  include/${PROJECT_NAME}
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

ament_package()