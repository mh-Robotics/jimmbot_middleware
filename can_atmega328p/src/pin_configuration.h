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
#ifndef CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_
#define CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_

#include "Arduino.h" // for digital*, analogWrite()
#include "stdint.h"  // for uint8_t

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
   */
  PinConfiguration()
      : motor_brake{7}, motor_stop{4}, motor_signal{3}, motor_direction{5},
        motor_speed{6}, can_mcp_irq{2}, can_mcp_rcv{10}, can_mcp_mosi{11},
        can_mcp_miso{12}, can_mcp_sck{13}, wheel_front_left{A0},
        wheel_front_right{A1}, wheel_back_left{A2}, wheel_back_right{A3} {}

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

/**
 * @brief Timeout constant [ms] if no CanBus message is received
 *
 */
constexpr auto kTimeoutMillis{250};
constexpr auto kTimeoutMicros{kTimeoutMillis * 1000};
constexpr auto kWheelPowerInWatt{300};
constexpr auto kMinVelocityToEffort{0.5};
#endif // CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_
