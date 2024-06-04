cmake_minimum_required(VERSION 3.8)
project(myactuator_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add myactuator_broadcaster library related compile commands
generate_parameter_library(myactuator_broadcaster_parameters
  src/myactuator_broadcaster.yaml
  include/myactuator_broadcaster/validate_myactuator_broadcaster_parameters.hpp
)
add_library(
  myactuator_broadcaster
  SHARED
  src/myactuator_broadcaster.cpp
)
target_include_directories(myactuator_broadcaster PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(myactuator_broadcaster myactuator_broadcaster_parameters)
ament_target_dependencies(myactuator_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(myactuator_broadcaster PRIVATE "MYACTUATOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface myactuator_broadcaster.xml)

install(
  TARGETS
  myactuator_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_myactuator_broadcaster test/test_load_myactuator_broadcaster.cpp)
  target_include_directories(test_load_myactuator_broadcaster PRIVATE include)
  ament_target_dependencies(
    test_load_myactuator_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_myactuator_broadcaster test/test_myactuator_broadcaster.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/myactuator_broadcaster_params.yaml)
  target_include_directories(test_myactuator_broadcaster PRIVATE include)
  target_link_libraries(test_myactuator_broadcaster myactuator_broadcaster)
  ament_target_dependencies(
    test_myactuator_broadcaster
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_myactuator_broadcaster_preceeding test/test_myactuator_broadcaster_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/myactuator_broadcaster_preceeding_params.yaml)
  target_include_directories(test_myactuator_broadcaster_preceeding PRIVATE include)
  target_link_libraries(test_myactuator_broadcaster_preceeding myactuator_broadcaster)
  ament_target_dependencies(
    test_myactuator_broadcaster_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  myactuator_broadcaster
)

ament_package()
