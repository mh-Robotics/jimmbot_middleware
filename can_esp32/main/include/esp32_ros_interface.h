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
#include "esp_err.h"

#define PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY "/esp32/feedback/can_msg_array"
#define SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY "/esp32/command/can_msg_array"

#define CAN_MSG_COUNT 4U
#define LIGHT_MSG_ID 0x31

#define GPIO_CAN0_TRANSMIT 5U
#define GPIO_CAN0_RECEIVE  4U
#define GPIO_CAN1_TRANSMIT 21U
#define GPIO_CAN1_RECEIVE  22U
#define GPIO_OUTPUT_LIGHT_LEFT  32U
#define GPIO_OUTPUT_LIGHT_RIGHT 33U
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_LIGHT_LEFT) | (1ULL<<GPIO_OUTPUT_LIGHT_RIGHT))

#ifdef __cplusplus
extern "C" {
#endif //end __cplusplus

/**
 * @brief Installs and starts the Can Driver
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t can_init(void);

/**
 * @brief Uninstalls and stops the Can Driver
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t can_destroy(void);

/**
 * @brief Initialise the Output GPIO configuration
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t output_gpio_init(void);

/**
 * @brief Uninstalls and stops output gpios
 * 
 * @return esp_err_t 
 */
esp_err_t output_gpio_destroy(void);

/**
 * @brief Initialise the ROS Node, subscriber and publisher
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t rosserial_setup(void);

/**
 * @brief Spin the ROS Node callback, and publish feedback
 * 
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t rosserial_spinonce(void);

#ifdef __cplusplus
}
#endif //end __cplusplus

#endif //end ___ESP32_ROS_INTERFACE_H___
