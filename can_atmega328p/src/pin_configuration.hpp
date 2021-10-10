/**
 * @file pin_configuration.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
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
#ifndef CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_
#define CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_

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
   * @brief Holds the motor enable pin number
   *
   */
  uint8_t motor_enable;

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
   */
  PinConfiguration()
      : motor_brake{PD7}, motor_enable{PD4}, motor_signal{PD3},
        motor_direction{PD5}, motor_speed{PD6}, can_mcp_irq{PD2},
        can_mcp_rcv{PB2}, can_mcp_mosi{PB3}, can_mcp_miso{PB4},
        can_mcp_sck{PB5}, wheel_front_left{PC0}, wheel_front_right{PC1},
        wheel_back_left{PC2}, wheel_back_right{PC3} {}

  /**
   * @brief Construct a new Pin Configuration object with specified pin numbers
   *
   * @param motorBrake uint8_t PinNumber passed when called
   * @param motorEnable uint8_t PinNumber passed when called
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
  PinConfiguration(const uint8_t &motorBrake, const uint8_t &motorEnable,
                   const uint8_t &motorSignal, const uint8_t &motorDirection,
                   const uint8_t &motorSpeed, const uint8_t &canMcpIrq,
                   const uint8_t &canMcpRcv, const uint8_t &canMcpMosi,
                   const uint8_t &canMcpMiso, const uint8_t &canMcpSck,
                   const uint8_t &wheelFrontLeft,
                   const uint8_t &wheelFrontRight, const uint8_t &wheelBackLeft,
                   const uint8_t &wheelBackRight)
      : motor_brake{motorBrake}, motor_enable{motorEnable},
        motor_signal{motorSignal}, motor_direction{motorDirection},
        motor_speed{motorSpeed}, can_mcp_irq{canMcpIrq}, can_mcp_rcv{canMcpRcv},
        can_mcp_mosi{canMcpMosi}, can_mcp_miso{canMcpMiso},
        can_mcp_sck{canMcpSck}, wheel_front_left{wheelFrontLeft},
        wheel_front_right{wheelFrontRight}, wheel_back_left{wheelBackLeft},
        wheel_back_right{wheelBackRight} {}
} pin_configuration_t;

/**
 * @brief @todo Get rid of this macro
 *
 */
#define SPEED_CONTROL_PWM(speed) ((OCR0A) = (speed))

/**
 * @brief Index constant for direction byte in CanBus message
 *
 */
constexpr uint8_t kDirectionByteIndex = 6;

/**
 * @brief Index constant for speed byte in CanBus message
 *
 */
constexpr uint8_t kSpeedByteIndex = 7;

/**
 * @brief Timeout constant [ms] if no CanBus message is received
 *
 */
constexpr uint8_t kTimeoutMs = 255;

#include "drivers/include/millis.h"
#include "drivers/include/spi.h"
#include "drivers/include/usart.h"

#endif // CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_
