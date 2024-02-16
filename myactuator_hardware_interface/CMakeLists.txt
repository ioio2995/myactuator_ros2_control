cmake_minimum_required(VERSION 3.20)
project(myactuator_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(myactuator_rmd REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  myactuator_control SHARED
  src/myactuator_control.cpp
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/myactuator_hardware_interface.cpp
)


target_link_libraries(myactuator_hardware_interface  
  myactuator_rmd::myactuator_rmd
)

pluginlib_export_plugin_description_file(hardware_interface myactuator_hardware_interface.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
