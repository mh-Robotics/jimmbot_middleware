# jimmbot_middleware

Jimmbot Boards Firmware is a package designed to control wheel controllers and ESP32 modules that connect the ROS master
with the CAN network to control the wheels. The firmware can be used to command the bldc wheel drive ZS-X11H V2, which
calculates odometry and constructs a wheel status. The status is then sent to the CAN network, retrieved by the ESP32,
and sent back to the ROS master.

## rosserial_esp32

The Rosserial ESP32 package is used to enable communication between ROS and ESP32 using ESP-IDF. This package is based
on [rosserial](http://wiki.ros.org/rosserial) and supports rosserial communication over UART and WiFi.

## Generate ROS libraries

To generate and include ROS libraries, follow the steps below. This creates a component in IDF_PATH and needs to be
generated only once.

```bash
cd path/to/catkin_ws/src/
git clone 
rosrun jimmbot_middleware make_libraries.py $IDF_PATH/components/ jimmbot_msgs
```

Once the above commands have been executed, all the necessary ROS files and custom messages will have been generated in
`$IDF_PATH/components/jimmbot_middleware/`.

## Firmware

* [middleware_firmware](firmware/README.md)

These firmware types enable communication between the ESP32 module and the CAN network. The CAN ATmega328P firmware
is designed for controlling the ATmega328P microcontroller-based boards, while the CAN ESP32 firmware is designed for
controlling the ESP32 module-based boards.

## Additional information

The connection between modules is as follows:

| ROS Master  | <-> | ESP32 | <-> | CAN Network  | <-> | Wheel Controller | <-> | ZS-X11H V2 Motor Driver | <->  | Wheel
 Hub |

In this diagram, the ROS Master is connected to the ESP32 module, which is in turn connected to the CAN network. The
Wheel Controller retrieves information from the CAN network and sends commands to the ZS-X11H V2 motor driver, which in
turn controls the Wheel Hub. The Wheel Hub calculates the odometry and constructs the wheel status, which is then sent
back to the ROS Master via the ESP32 and CAN network.
