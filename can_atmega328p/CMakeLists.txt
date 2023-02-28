cmake_minimum_required(VERSION 3.15)
project(MotorController
        VERSION 0.5.0
        DESCRIPTION "Firmware to control the motor hub of jimmBOT"
        LANGUAGES C CXX
)

set(CPU_FREQ "16000000")
set(CPU "atmega328p")
set(PROGRAMMING_BAUD "57600")
set(USB_PORT "/dev/ttyUSB0")
set(AVRDUDE_BIN "${CMAKE_CURRENT_SOURCE_DIR}/conf/avrdude")
set(AVRDUDE_CONF "${CMAKE_CURRENT_SOURCE_DIR}/conf/avrdude.conf")

set(CMAKE_C_COMPILER "avr-gcc")
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_COMPILER "avr-g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-error=narrowing -Os -mmcu=${CPU} -DF_CPU=${CPU_FREQ}")
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-error=narrowing -Os -mmcu=${CPU} -DF_CPU=${CPU_FREQ}")

include_directories(
  src/drivers/include
)

set(${PROJECT_NAME}_drivers_src
  src/drivers/src/mcp2515.cpp
)

set(${PROJECT_NAME}_src
  src/main.cpp
  src/can_wrapper.cpp
  src/iwheel_controller.cpp
  src/wheel_controller.cpp
  src/wheel.cpp
)

add_library(${PROJECT_NAME}_drivers STATIC ${${PROJECT_NAME}_drivers_src})

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_src})

target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_drivers)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  add_custom_target(doc_doxygen
    ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else()
  message("Doxygen not found, unable to generate documentation")
endif()

add_custom_target(hex ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
  COMMAND avr-size -C -x ${PROJECT_NAME}
  DEPENDS ${PROJECT_NAME}
)

add_custom_target(flash
  COMMAND sudo avrdude -v -p ${CPU} -c usbasp -P ${USB_PORT} -b ${PROGRAMMING_BAUD} -D -U flash:w:${PROJECT_NAME}.hex:i
  DEPENDS hex
)

add_custom_target(arduino_flash
  COMMAND sudo avrdude -v -p ${CPU} -c arduino -P ${USB_PORT} -b ${PROGRAMMING_BAUD} -D -U flash:w:${PROJECT_NAME}.hex:i
  DEPENDS hex
)

add_custom_target(bin_arduino_flash
  COMMAND sudo ${AVRDUDE_BIN} -C ${AVRDUDE_CONF} -v -p ${CPU} -c arduino -P ${USB_PORT} -b ${PROGRAMMING_BAUD} -D -U flash:w:${PROJECT_NAME}.hex:i
  DEPENDS hex
)
