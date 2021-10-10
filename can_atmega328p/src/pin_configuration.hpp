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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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

typedef struct PinConfiguration {
  uint8_t motor_brake;
  uint8_t motor_enable;
  uint8_t motor_signal;
  uint8_t motor_direction;
  uint8_t motor_speed;

  uint8_t can_mcp_irq;
  uint8_t can_mcp_rcv;
  uint8_t can_mcp_mosi;
  uint8_t can_mcp_miso;
  uint8_t can_mcp_sck;

  uint8_t wheel_front_left;
  uint8_t wheel_front_right;
  uint8_t wheel_back_left;
  uint8_t wheel_back_right;

  PinConfiguration()
      : motor_brake{PD7}, motor_enable{PD4}, motor_signal{PD3},
        motor_direction{PD5}, motor_speed{PD6}, can_mcp_irq{PD2},
        can_mcp_rcv{PB2}, can_mcp_mosi{PB3}, can_mcp_miso{PB4},
        can_mcp_sck{PB5}, wheel_front_left{PC0}, wheel_front_right{PC1},
        wheel_back_left{PC2}, wheel_back_right{PC3} {}

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

#define SPEED_CONTROL_PWM(speed) ((OCR0A) = (speed))
#define DIRECTION_BIT_INDEX 6
#define SPEED_BIT_INDEX 7

constexpr int kTimeoutMs = 250;

#include "drivers/include/millis.h"
#include "drivers/include/spi.h"
#include "drivers/include/usart.h"

#endif // CAN_ATMEGA328P_SRC_PIN_CONFIGURATION_HPP_
