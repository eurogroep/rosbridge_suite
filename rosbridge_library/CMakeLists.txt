cmake_minimum_required(VERSION 3.5)
project(rosbridge_library)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()
