cmake_minimum_required(VERSION 3.16)
project(myactuator_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(myactuator_broadcaster_parameters
  src/myactuator_broadcaster_parameters.yaml
)

add_library(myactuator_broadcaster SHARED
  src/myactuator_broadcaster.cpp
)
target_compile_features(myactuator_broadcaster PUBLIC cxx_std_17)
target_include_directories(myactuator_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/myactuator_controllers>
)
target_link_libraries(myactuator_broadcaster PUBLIC
  myactuator_broadcaster_parameters
)
ament_target_dependencies(myactuator_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(myactuator_broadcaster PRIVATE "FORCE_TORQUE_SENSOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface myactuator_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/myactuator_controllers
)
install(
  TARGETS
    myactuator_broadcaster
    myactuator_broadcaster_parameters
  EXPORT export_myactuator_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_myactuator_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()