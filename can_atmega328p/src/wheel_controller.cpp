/**
 * @file wheel_controller.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief WheelController class implementation that initializes the controller
 * and instantiates a Wheel. It controls and sets the logic as requested from
 * CanBus message.
 * @version 0.1
 * @date 2021-10-10
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
#include "wheel_controller.h"  // for WheelController
#include "constants.h"         // for k*
#include "pin_configuration.h" // for PinConfiguration and k*

#include <ArduinoSTL.h> // for ArduinoSTL containers
#include <cmath>        // for M_PI

#include "Arduino.h"

bool WheelController::Init(const Wheel &wheel) {
  wheel_ = &wheel;

  // The logic is here, but the pin is not used in the drive.
  Stop(false);
  Drive(false);

  return true;
}

void WheelController::WheelSignalIrqHandler() {
  if (is_reverse_) {
    if (signal_counter_ == INT32_MIN) {
      signal_counter_ = 0;
    } else {
      signal_counter_--;
    }
  } else {
    if (signal_counter_ == INT32_MAX) {
      signal_counter_ = 0;
    } else {
      signal_counter_++;
    }
  }

  CalculateWheelOdometry();
}

bool WheelController::CalculateWheelOdometry() {
  std::call_once(first_odometry_tick_, [this]() { old_time_ = micros(); });

  unsigned long current_time = micros();
  if (current_time < old_time_) {
    time_taken_ =
        static_cast<double>(current_time + (UINT32_MAX - old_time_ + 1)) /
        1000000.0;
  } else {
    time_taken_ = static_cast<double>(current_time - old_time_) / 1000000.0;
  }

  int sign = is_reverse_ ? -1 : 1;
  double circumference_meters = 2 * M_PI * (internal::wheel::kRadius / 100.0);

  wheel_feedback_status_.Position(
      (circumference_meters * 100 * signal_counter_) /
      (100 * internal::wheel::kPulsePerRevolution)); // Pose in Meters

  wheel_feedback_status_.Rpm(
      1.0 / (time_taken_ * internal::wheel::kPulsePerRevolution) * 60.0);

  wheel_feedback_status_.Velocity(sign * circumference_meters *
                                  wheel_feedback_status_.Rpm() / 60.0);

  // // @todo(jimmyhalimi): This slows down the position calculation somehow.
  // const double kVelocity = wheel_feedback_status_.Velocity();
  // if (kVelocity >= -internal::kMinVelocityToEffort &&
  //     kVelocity <= internal::kMinVelocityToEffort) {
  //   wheel_feedback_status_.Effort(sign * 63);
  // } else {
  //   wheel_feedback_status_.Effort(sign * internal::wheel::kPowerInWatt /
  //                                 kVelocity);
  // }

  old_time_ = current_time;
  return true;
}

void WheelController::UpdateTimeout() { timeout_ = micros(); }

bool WheelController::TimeoutCheck() const {
  return (micros() - timeout_ >= internal::kTimeoutMicros);
}

void WheelController::SetDirection(const bool &direction) {
  is_reverse_ = !direction;

  // Active Low: Direction clockwise
  if (direction) {
    PORTD |= (1 << wheel_->Configuration().motor_direction);
  } else {
    PORTD &= ~(1 << wheel_->Configuration().motor_direction);
  }
}

void WheelController::SetSpeed(const uint8_t &speed) const {
  if (speed == 0) {
    Drive(false);
  } else {
    Drive(true);
  }

  analogWrite(wheel_->Configuration().motor_speed, speed);
}

void WheelController::SetSpeedAndDirection(const uint8_t &speed,
                                           const bool &direction) const {
  SetDirection(direction);
  SetSpeed(speed);
}

WheelController::wheel_status_t WheelController::WheelFeedbackStatus() const {

  const auto wheel_feedback_status = wheel_feedback_status_;
  // Reset all the values after the feedback is requested except the position
  wheel_feedback_status_.CommandId(0);
  wheel_feedback_status_.Rpm(0);
  wheel_feedback_status_.Velocity(0.0);

  return wheel_feedback_status;
}

void WheelController::Brake(const bool &brake) const {
  // Active High: Brake applied
  if (brake) {
    PORTD |= (1 << wheel_->Configuration().motor_brake);
  } else {
    PORTD &= ~(1 << wheel_->Configuration().motor_brake);
  }
}

void WheelController::Stop(const bool &stop) const {
  // Active Low: Drive disabled
  if (!stop) {
    PORTD |= (1 << wheel_->Configuration().motor_stop);
  } else {
    PORTD &= ~(1 << wheel_->Configuration().motor_stop);
  }
}

void WheelController::Drive(const bool &drive) const { Brake(!drive); }
