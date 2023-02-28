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

bool WheelController::Init(const Wheel &wheel) {
  wheel_ = &wheel;

  Drive(false);

  return true;
}

void WheelController::WheelSignalIrqHandler(void) {
  if (wheel_->Properties().Reverse()) {
    signal_counter_--;
  } else {
    signal_counter_++;
  }

  CalculateWheelOdometry();
}

bool WheelController::CalculateWheelOdometry(void) {
  std::call_once(first_odometry_tick_, [this]() { old_time_ = ::micros(); });
  time_taken_ = ::micros() - old_time_;
  wheel_status_.Effort(10);
  double circumference = 2 * M_PI * wheel_->Properties().Radius();
  wheel_status_.Position(
      (circumference * signal_counter_) /
      (100 * wheel_->Properties().PulsePerRevolution()));  // Pose in CM

  wheel_status_.Rpm(
      (1 / (time_taken_ * wheel_->Properties().PulsePerRevolution())) * 60.0);

  wheel_status_.Velocity((circumference * wheel_status_.Rpm() * 60) / 3600);

  old_time_ = ::micros();
  return true;
}

void WheelController::UpdateTimeout(void) { timeout_ = ::micros(); }

bool WheelController::TimeoutCheck(void) {
  return (::micros() - timeout_ >= kTimeoutMs);
}

void WheelController::SetDirection(bool direction) {
  wheel_->Properties().Reverse(!direction);

  // Active Low: Direction clockwise
  digitalWrite(wheel_->Configuration().motor_direction, direction);
}

void WheelController::SetSpeed(int speed) {
  int abs_speed = abs(speed);
  if (abs_speed == 0) {
    Drive(false);
  } else {
    Drive(true);
    SetDirection(speed > 0);
    analogWrite(wheel_->Configuration().motor_speed, abs_speed);
  }
}

WheelController::wheel_status_t WheelController::WheelStatus(void) {
  return wheel_status_;
}

void WheelController::Brake(bool kBrake) {
  // Active High: Brake applied
  digitalWrite(wheel_->Configuration().motor_brake, kBrake);
}

void WheelController::Stop(bool kStop) {
  // Active Low: Drive disabled
  digitalWrite(wheel_->Configuration().motor_stop, !kStop);
}

void WheelController::Drive(bool drive) { Brake(!drive); }
