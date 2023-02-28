cmake_minimum_required(VERSION 3.0.2)
project(jimmbot_boards_firmware)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_subdirectory(can_atmega328p)

install(DIRECTORY
  ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS rosserial_esp32/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
