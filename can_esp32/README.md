# MotorController ROS Node

This repository holds the firmware for MotorController ROS Node. It will communicate with ROS Master through serial communication, will get ros msgs array, and it will transfer the 
messages in CAN Network for the motors to listen and to perform.

ESP-IDF needs to be installed in order for this to compile

source esp-idf/export.sh

## motor_controller_ros_esp32

This code subscribes to `/esp32/command/can_msg`, the ESP32 will transmit can messages, receive the feedback and publish on `/esp32/feedback/can_msg`
The data received from Can as Feedback will be published on `/esp32/feedback/can_msg`. The ID that will be received are only the ID's of the feedback messages from the wheels 4->7.
The data published to `/esp32/command/can_msg` from the main computer, will be subscribed from ESP32 and transmitted into CanNetwork

### Build and Flash

Generate the ROS libraries prior to building this example as instructed in the [README](../README.md) of root directory (If done already, ignore)

Default mode of rosserial communication is over UART.

To use WiFi:
1. Enable rosserial over WiFi

`idf.py menuconfig` -> `Component config` -> `rosserial` ->`rosserial over WiFi using TCP`

2. Enter WiFi and server details

```
$ export ESPPORT=/dev/esp32_ros_node
$ idf.py build flash
```