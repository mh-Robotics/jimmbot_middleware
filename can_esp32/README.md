# WheelController ROS Node
This repository holds the firmware for the WheelController ROS Node, which allows communication between ROS Master and the motors through serial communication and the CAN Network.

## Prerequisites
To compile this code, the ESP-IDF must be installed on your system. You can install ESP-IDF by following the instructions on the Espressif website.

## Usage

This code subscribes to the topic `/esp32/command/can_msg`, which receives messages from the main computer and transmits them to the motors through the CAN Network. Feedback from the motors is published on the topic `/esp32/feedback/can_msg`. Only feedback messages from wheels 4 to 7 are received by this node.

## Hardware Setup
Connect your ESP32 to your ROS Master computer via USB, and the CanBus connections in the CanBus wires.

### Pinout
The pinout and conenctions are as follows:
```bash
        +----------------+             +-----------------------+
        |  RosMaster PC  +--+  USB  +--+         ESP32         |
        +----------------+             +-----------------------+
                                  CANH | GPIO21         GPIO22 | CANL
                                       |                       |
                                       |                       |
                                   +---|---+               +---|---+
                                   |  TJA  |               |  TJA  |
                                   +---|---+               +---|---+
                                       |                       |
                                       |                       |
                    GPIO32 --o---------o                       o---------o-- GPIO33
                    Relay (Left Light)                          Relay (Right Light)
```

### Build and Flash

Before building this code, ensure that you have generated the ROS libraries as instructed in the [README](../README.md) of the root directory. If you haven't done this yet, please refer to the instructions in that file.

```
$ export ESPPORT=/dev/esp32_ros_node
$ idf.py build flash
```
Once you have flashed the code onto the ESP32, you can start using the WheelController ROS Node to control your motors.

To use WiFi instead of the default UART communication mode, follow these steps:
1. Enable rosserial over WiFi

`idf.py menuconfig` -> `Component config` -> `rosserial` ->`rosserial over WiFi using TCP`

2. Enter WiFi and server details
