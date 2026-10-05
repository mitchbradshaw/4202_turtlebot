cmake_minimum_required(VERSION 3.5)
project(metr4202_test_worlds NONE)

# Data-only package: worlds, launch files and ArUco materials. Nothing to compile.
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds resource docs
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
