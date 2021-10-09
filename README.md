## rosserial_esp32

This package is based on [rosserial](http://wiki.ros.org/rosserial) to enable communication between ROS and ESP32 using ESP-IDF.

Supports rosserial communication over **UART** and **WiFi**

### Generate ROS libraries
Follow the steps below in order to generate and include ROS libraries
(This will create a component in IDF_PATH and need to generate it only once)

```
$ cd path/to/catkin_ws/src/
$ git clone 
$ rosrun jimmbot_boards_firmware make_libraries.py $IDF_PATH/components/ jimmbot_msgs
```

After execution of above commands, all the necessary ROS files and custom messages would have been generated in `$IDF_PATH/components/jimmbot_boards_firmware/`

### Firmware
* [can_atmega328p_firmware](can_atmega328p/README.md)
* [can_esp32_firmware](can_esp32/README.md)
