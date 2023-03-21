/**
 * @file pin_configuration.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Pin Configuration structure that holds all the pin numbers and
 * connections for this firmware to work.
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
#ifndef JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_H_
#define JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_H_

#include <ArduinoSTL.h> // for ArduinoSTL containers
#include <avr/io.h>     // for avr pins P*
#include <cstdint>      // for uint8_t

/**
 * @brief Pin Configuration structure that holds all the pin numbers and
 * connections for this firmware to work.
 *
 */
typedef struct PinConfiguration {
  /**
   * @brief Holds the motor brake pin number
   *
   */
  uint8_t motor_brake;

  /**
   * @brief Holds the motor stop pin number
   *
   */
  uint8_t motor_stop;

  /**
   * @brief Holds the motor encoder pin number
   *
   */
  uint8_t motor_signal;

  /**
   * @brief Holds the motor direction pin number
   *
   */
  uint8_t motor_direction;

  /**
   * @brief Holds the motor speed pin number
   *
   */
  uint8_t motor_speed;

  /**
   * @brief Holds the CanBus board interrupt pin number
   *
   */
  uint8_t can_mcp_irq;

  /**
   * @brief Holds the CanBus board receive pin number
   *
   */
  uint8_t can_mcp_rcv;

  /**
   * @brief Holds the CanBus board mosi pin number
   *
   */
  uint8_t can_mcp_mosi;

  /**
   * @brief Holds the CanBus board miso pin number
   *
   */
  uint8_t can_mcp_miso;

  /**
   * @brief Holds the CanBus board serial clock pin number
   *
   */
  uint8_t can_mcp_sck;

  /**
   * @brief Holds the FrontLeft wheel active pin number
   *
   */
  uint8_t wheel_front_left;

  /**
   * @brief Holds the FrontRight wheel active pin number
   *
   */
  uint8_t wheel_front_right;

  /**
   * @brief Holds the BackLeft wheel active pin number
   *
   */
  uint8_t wheel_back_left;

  /**
   * @brief Holds the BackRight wheel active pin number
   *
   */
  uint8_t wheel_back_right;

  /**
   * @brief Construct a new Pin Configuration object with default pin numbers
   *
   *     ATMEL ATMEGA328P / ARDUINO
   *
   *                   +-\/-+
   *             PC6  1|    |28  PC5 (AI 5)
   *       (D 0) PD0  2|    |27  PC4 (AI 4)
   *       (D 1) PD1  3|    |26  PC3 (AI 3)
   *       (D 2) PD2  4|    |25  PC2 (AI 2)
   *  PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
   *       (D 4) PD4  6|    |23  PC0 (AI 0)
   *             VCC  7|    |22  GND
   *             GND  8|    |21  AREF
   *             PB6  9|    |20  AVCC
   *             PB7 10|    |19  PB5 (D 13)
   *  PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
   *  PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
   *       (D 7) PD7 13|    |16  PB2 (D 10) PWM
   *       (D 8) PB0 14|    |15  PB1 (D 9) PWM
   *                   +----+
   *
   *  (PWM+ indicates the additional PWM pins on the ATmega168.)
   */
  PinConfiguration()
      : motor_brake{PD7}, motor_stop{PD4}, motor_signal{PD3},
        motor_direction{PD5}, motor_speed{PD6}, can_mcp_irq{PD2},
        can_mcp_rcv{PB2}, can_mcp_mosi{PB3}, can_mcp_miso{PB4},
        can_mcp_sck{PB5}, wheel_front_left{PC0}, wheel_front_right{PC1},
        wheel_back_left{PC2}, wheel_back_right{PC3} {}

  /**
   * @brief Construct a new Pin Configuration object with specified pin numbers
   *
   * @param motorBrake uint8_t PinNumber passed when called
   * @param motorStop uint8_t PinNumber passed when called
   * @param motorSignal uint8_t PinNumber passed when called
   * @param motorDirection uint8_t PinNumber passed when called
   * @param motorSpeed uint8_t PinNumber passed when called
   * @param canMcpIrq uint8_t PinNumber passed when called
   * @param canMcpRcv uint8_t PinNumber passed when called
   * @param canMcpMosi uint8_t PinNumber passed when called
   * @param canMcpMiso uint8_t PinNumber passed when called
   * @param canMcpSck uint8_t PinNumber passed when called
   * @param wheelFrontLeft uint8_t PinNumber passed when called
   * @param wheelFrontRight uint8_t PinNumber passed when called
   * @param wheelBackLeft uint8_t PinNumber passed when called
   * @param wheelBackRight uint8_t PinNumber passed when called
   */
  PinConfiguration(const uint8_t &motorBrake, const uint8_t &motorStop,
                   const uint8_t &motorSignal, const uint8_t &motorDirection,
                   const uint8_t &motorSpeed, const uint8_t &canMcpIrq,
                   const uint8_t &canMcpRcv, const uint8_t &canMcpMosi,
                   const uint8_t &canMcpMiso, const uint8_t &canMcpSck,
                   const uint8_t &wheelFrontLeft,
                   const uint8_t &wheelFrontRight, const uint8_t &wheelBackLeft,
                   const uint8_t &wheelBackRight)
      : motor_brake{motorBrake}, motor_stop{motorStop},
        motor_signal{motorSignal}, motor_direction{motorDirection},
        motor_speed{motorSpeed}, can_mcp_irq{canMcpIrq}, can_mcp_rcv{canMcpRcv},
        can_mcp_mosi{canMcpMosi}, can_mcp_miso{canMcpMiso},
        can_mcp_sck{canMcpSck}, wheel_front_left{wheelFrontLeft},
        wheel_front_right{wheelFrontRight}, wheel_back_left{wheelBackLeft},
        wheel_back_right{wheelBackRight} {}
} pin_configuration_t;
#endif // JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_H_
