/**
 * @file wheel.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Wheel class implementation that initializes the wheel and its
 * properties.
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
 */
#include "wheel.h"             // for Wheel
#include "pin_configuration.h" // for PinConfiguration

#include <ArduinoSTL.h> // for ArduinoSTL containers
#include <avr/io.h>     // for DDR*, PORT* and PIN*
#include <cstdint>      // for uint8_t

bool Wheel::Init() {
  const uint8_t dirBrkStopSpeedMask = (1 << Configuration().motor_direction) |
                                      (1 << Configuration().motor_brake) |
                                      (1 << Configuration().motor_stop) |
                                      (1 << Configuration().motor_speed);
  DDRD |= dirBrkStopSpeedMask; // Set direction, brake, stop, and speed pins as
                               // outputs
  DDRD &= ~(1 << Configuration().motor_signal); // Set signal pin as input
  PORTD |= (1 << Configuration().motor_signal); // Enable internal Pull Up
  DDRC &= ~0x0F;                                // Set wheel pins as inputs
  PORTC |= 0x0F; // Enable internal Pull Ups on wheel pins

  return EnumToCanId(DetermineWheel());
}

Wheel::properties Wheel::Properties() const { return properties_; }

PinConfiguration Wheel::Configuration() const { return pin_configuration_; }

bool Wheel::EnumToCanId(const Wheel::Wheel_Enum &wheelEnum) {
  switch (wheelEnum) {
  case Wheel::Wheel_Enum::kFrontLeft: {
    properties_.ReceiveId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontLeft));
    properties_.TransmitId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontLeft));

    return true;
  }

  case Wheel::Wheel_Enum::kFrontRight: {
    properties_.ReceiveId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontRight));
    properties_.TransmitId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontRight));

    return true;
  }

  case Wheel::Wheel_Enum::kBackLeft: {
    properties_.ReceiveId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackLeft));
    properties_.TransmitId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackLeft));

    return true;
  }

  case Wheel::Wheel_Enum::kBackRight: {
    properties_.ReceiveId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackRight));
    properties_.TransmitId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackRight));

    return true;
  }

  default: {
    properties_.ReceiveId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));
    properties_.TransmitId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));

    return true;
  }
  }

  return false;
}

Wheel::Wheel_Enum Wheel::DetermineWheel() {
  const uint8_t pinState =
      ((PINC & ((1 << Configuration().wheel_front_left) |
                (1 << Configuration().wheel_front_right) |
                (1 << Configuration().wheel_back_left) |
                (1 << Configuration().wheel_back_right))) >>
       2);

  // Use the bitmask to determine the selected wheel
  switch (pinState) {
  case 0b0001:
    return Wheel::Wheel_Enum::kFrontLeft;
  case 0b0010:
    return Wheel::Wheel_Enum::kFrontRight;
  case 0b0100:
    return Wheel::Wheel_Enum::kBackLeft;
  case 0b1000:
    return Wheel::Wheel_Enum::kBackRight;
  default:
    return Wheel::Wheel_Enum::kUnspecified;
  }
}
