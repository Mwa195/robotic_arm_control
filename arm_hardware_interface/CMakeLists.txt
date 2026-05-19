cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Build the shared library that pluginlib will load at runtime
add_library(arm_hardware_interface SHARED
  src/hardware_translator.cpp
)

target_include_directories(arm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Export the pluginlib plugin description XML
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_interface_plugin.xml)

install(TARGETS arm_hardware_interface
  EXPORT export_arm_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(arm_hardware_interface)
ament_export_targets(export_arm_hardware_interface)

ament_package()