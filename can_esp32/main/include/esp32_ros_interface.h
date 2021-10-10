/**
 * @file esp32_ros_interface.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the definitions for initializations, callbacks and
 * types used in this program
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * @license This project is released under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#ifndef CAN_ESP32_ROS_INTERFACE_H_
#define CAN_ESP32_ROS_INTERFACE_H_
#include <esp_err.h>

#define PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY "/esp32/feedback/can_msg_array"
#define SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY "/esp32/command/can_msg_array"

#define CAN_MSG_COUNT 4U
#define LIGHT_MSG_ID 0x31

#define GPIO_CAN_TRANSMIT 21U
#define GPIO_CAN_RECEIVE 22U
#define GPIO_OUTPUT_LIGHT_LEFT 32U
#define GPIO_OUTPUT_LIGHT_RIGHT 33U
#define GPIO_OUTPUT_PIN_SEL                                                    \
  ((1ULL << GPIO_OUTPUT_LIGHT_LEFT) | (1ULL << GPIO_OUTPUT_LIGHT_RIGHT))

#ifdef __cplusplus
extern "C" {
#endif // end __cplusplus

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

/**
 * @brief Spin the ROS Node callback, and publish feedback
 *
 * @return esp_err_t Return error code with success or fail
 */
esp_err_t rosserial_spin(void);

#ifdef __cplusplus
}
#endif // end __cplusplus

#endif // CAN_ESP32_ROS_INTERFACE_H_
