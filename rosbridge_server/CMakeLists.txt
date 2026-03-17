cmake_minimum_required(VERSION 3.5)
project(rosbridge_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")

ament_package()

install(PROGRAMS
  scripts/rosbridge_websocket.py
  scripts/rosbridge_websocket
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/rosbridge_websocket_launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)
