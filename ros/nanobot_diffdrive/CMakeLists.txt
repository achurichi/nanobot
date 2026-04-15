cmake_minimum_required(VERSION 3.8)
project(nanobot_diffdrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
  dynamixel_sdk
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  nanobot_diffdrive
  SHARED
  hardware/nanobot_system.cpp
  hardware/wheel.cpp
  hardware/dynamixel_comms.cpp
)
target_compile_features(nanobot_diffdrive PUBLIC cxx_std_17)
target_include_directories(nanobot_diffdrive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/nanobot_diffdrive>
)
ament_target_dependencies(
  nanobot_diffdrive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface nanobot_diffdrive.xml)

# INSTALL
install(
  DIRECTORY config description hardware
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS nanobot_diffdrive
  EXPORT export_nanobot_diffdrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_nanobot_diffdrive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
