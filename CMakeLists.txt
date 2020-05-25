cmake_minimum_required(VERSION 2.8.3)
project(jimmbot_esp32_firmware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS rosserial_esp32/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)