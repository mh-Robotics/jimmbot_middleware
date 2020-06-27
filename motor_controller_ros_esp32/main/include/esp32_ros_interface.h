/**
 * @file esp32_ros_interface.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the definitions for initializations, callbacks and types used in this program
 * @version 0.1
 * @date 2020-06-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef ___ESP32_ROS_INTERFACE_H___
#define ___ESP32_ROS_INTERFACE_H___

#include "stdio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/can.h"
#include "driver/gpio.h"

#define PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY "/esp32/feedback/can_msg_array"
#define SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY "/esp32/command/can_msg_array"

#define WHEEL_MSG_COUNT 4u
#define LIGHT_MSG_COUNT 1u
#define CAN_MSG_LENGTH 8u

#define GPIO_OUTPUT_LIGHT_LEFT    18
#define GPIO_OUTPUT_LIGHT_RIGHT   19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_LIGHT_LEFT) | (1ULL<<GPIO_OUTPUT_LIGHT_RIGHT))

#ifdef __cplusplus
extern "C" {
#endif //end __cplusplus

/**
 * @brief Initialise the Output GPIO configuration
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t output_gpio_init();

/**
 * @brief Initialise the ROS Node, subscriber and publisher
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t rosserial_setup();

/**
 * @brief Spin the ROS Node callback, and publish feedback
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t rosserial_spinonce();

#ifdef __cplusplus
}
#endif //end __cplusplus

#endif //end ___ESP32_ROS_INTERFACE_H___
