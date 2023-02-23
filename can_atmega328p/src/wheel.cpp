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
 *
 */
#include "wheel.hpp"

bool Wheel::Init() {
  pinMode(Configuration().motor_direction, OUTPUT);
  pinMode(Configuration().motor_brake, OUTPUT);
  pinMode(Configuration().motor_enable, OUTPUT);
  pinMode(Configuration().motor_signal, INPUT_PULLUP);
  pinMode(Configuration().motor_speed, OUTPUT);

  pinMode(Configuration().wheel_front_left, INPUT_PULLUP);
  pinMode(Configuration().wheel_front_right, INPUT_PULLUP);
  pinMode(Configuration().wheel_back_left, INPUT_PULLUP);
  pinMode(Configuration().wheel_back_right, INPUT_PULLUP);

  return EnumToCanId(DetermineWheel());
}

Wheel::properties_t Wheel::Properties() const { return properties_; }

pin_configuration_t Wheel::Configuration() const { return pin_configuration_; }

bool Wheel::EnumToCanId(const Wheel::Wheel_Enum &wheelEnum) {
  switch (wheelEnum) {
    case Wheel::Wheel_Enum::kFrontLeft: {
      properties_.CommandId(
          static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontLeft));
      properties_.FeedbackId(
          static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontLeft));
      properties_.Inverse(false);

      return true;
    }

    case Wheel::Wheel_Enum::kFrontRight: {
      properties_.CommandId(
          static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontRight));
      properties_.FeedbackId(
          static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontRight));
      properties_.Inverse(true);

      return true;
    }

    case Wheel::Wheel_Enum::kBackLeft: {
      properties_.CommandId(
          static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackLeft));
      properties_.FeedbackId(
          static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackLeft));
      properties_.Inverse(false);

      return true;
    }

    case Wheel::Wheel_Enum::kBackRight: {
      properties_.CommandId(
          static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackRight));
      properties_.FeedbackId(
          static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackRight));
      properties_.Inverse(true);

      return true;
    }

    default: {
      properties_.CommandId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));
      properties_.FeedbackId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));
      properties_.Inverse(false);

      return true;
    }
  }

  return false;
}

Wheel::Wheel_Enum Wheel::DetermineWheel() {
  if (!(digitalRead(Configuration().wheel_front_left))) {
    return Wheel::Wheel_Enum::kFrontLeft;
  } else if (!(digitalRead(Configuration().wheel_front_right))) {
    return Wheel::Wheel_Enum::kFrontRight;
  } else if (!(digitalRead(Configuration().wheel_back_left))) {
    return Wheel::Wheel_Enum::kBackLeft;
  } else {
    return Wheel::Wheel_Enum::kBackRight;
  }

  return Wheel::Wheel_Enum::kUnspecified;
}
