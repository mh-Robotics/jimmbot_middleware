
cmake_minimum_required(VERSION 3.10...3.15)
project(MotorController
        VERSION 0.1.0
        DESCRIPTION "MotorController firmware for jimmBOT wheel"
        LANGUAGES C CXX
)

set(CPU_FREQ "16000000")
set(CPU "atmega328p")
set(PROGRAMMING_BAUD "57600")
set(USB_PORT "/dev/ttyUSB0")
set(AVRDUDE_BIN "${CMAKE_CURRENT_SOURCE_DIR}/conf/avrdude") 
set(AVRDUDE_CONF "${CMAKE_CURRENT_SOURCE_DIR}/conf/avrdude.conf")

set(CMAKE_C_COMPILER "avr-gcc") 
set(CMAKE_C_FLAGS "-D__DEBUG__ -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -std=c17 -Os -Wl,--relax,--gc-sections -DF_CPU=${CPU_FREQ} -mmcu=${CPU}")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

set(CMAKE_CXX_COMPILER "avr-g++") 
set(CMAKE_CXX_FLAGS "-D__DEBUG__ -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -std=c++17 -Os -Wl,--relax,--gc-sections -DF_CPU=${CPU_FREQ} -mmcu=${CPU}")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include_directories(
  src/drivers/include
)

set(
  ${PROJECT_NAME}_drivers_src
  src/drivers/src/spi.cpp
  src/drivers/src/usart.cpp
  src/drivers/src/millis.cpp
  src/drivers/src/mcp2515.cpp
)

set(
  ${PROJECT_NAME}_src
  src/main.cpp
  src/can_wrapper.cpp
  src/iwheel_controller.cpp
  src/wheel_controller.cpp
  src/wheel.cpp
)

add_library(
  ${PROJECT_NAME}_drivers STATIC
  ${${PROJECT_NAME}_drivers_src}
)

target_link_libraries(
  ${PROJECT_NAME}_drivers
)

add_executable(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_src}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_drivers
)

add_custom_target(hex
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
