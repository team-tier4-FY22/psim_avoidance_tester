cmake_minimum_required(VERSION 3.8)
project(psim_avoidance_tester)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  param
)

install(PROGRAMS
  scripts/dummy_publisher.py
)
