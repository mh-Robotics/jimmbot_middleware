cmake_minimum_required(VERSION 3.8)
project(jimmbot_middleware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  ros_lib
  DESTINATION share/${PROJECT_NAME}/src
)

install(PROGRAMS
  rosserial_esp32/make_libraries.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
