cmake_minimum_required(VERSION 3.20)
project(swervebot_controllers)

# Suppress Boost Find module policy warning (CMP0167)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate the C++ parameter library from the YAML file
generate_parameter_library(
  swerve_controller_parameters
  "config/swerve_controller_parameters.yaml"
)

# --- CONSOLIDATED SINGLE TARGET ---
# We use 'swerve_controller' since it matches your export statements below.
add_library(swerve_controller SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/swerve_controller.cpp)

add_dependencies(swerve_controller swerve_controller_parameters)

target_include_directories(swerve_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link EVERYTHING to our single controller target
target_link_libraries(swerve_controller PUBLIC
  swerve_controller_parameters
  ${controller_interface_TARGETS}
  ${hardware_interface_TARGETS}   # Added missing hardware interface
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${realtime_tools_TARGETS}
  ${pluginlib_TARGETS}
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
  angles::angles
)

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 1. Install local asset folders 
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install C++ header files (.hpp) so other packages can find them
install(
  DIRECTORY include/
  DESTINATION include
)

# 3. Install your compiled Shared Library (.so plugin binary)
install(
  TARGETS swerve_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 4. Install the plugin XML file so the controller_manager can discover it
install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ensure internal include mapping handles dependencies perfectly
ament_export_include_directories(include)
ament_export_libraries(swerve_controller)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  angles
  pluginlib
)

ament_package()
