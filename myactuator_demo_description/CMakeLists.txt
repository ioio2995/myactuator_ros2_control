cmake_minimum_required(VERSION 3.5)
project(myactuator_demo_description)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE urdf ros2_control)
