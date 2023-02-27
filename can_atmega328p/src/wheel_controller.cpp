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
#include "wheel_controller.hpp"

bool WheelController::Init(const Wheel &wheel) {
  wheel_ = &wheel;

  return WheelController::Setup();
}

bool WheelController::Setup() {
  Drive(false);

  return true;
}

void WheelController::WheelSignalIrqHandler(void) {
  wheel_->Properties().Reverse() ? signal_counter_-- : signal_counter_++;
  CalculateWheelOdometry();
}

// https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
bool WheelController::CalculateWheelOdometry(void) {
  std::call_once(first_odometry_tick_, [this]() { old_time_ = ::millis(); });
  time_taken_ = ::millis() - old_time_;
  wheel_status_.Effort(10);
  double circumference = 2 * M_PI * wheel_->Properties().Radius();
  wheel_status_.Position(
      (circumference / wheel_->Properties().PulsePerRevolution()) *
      signal_counter_ / 100);  // Pose in CM
  wheel_status_.Rpm(
      (1 / (time_taken_ * wheel_->Properties().PulsePerRevolution())) * 60.0);
  wheel_status_.Velocity(wheel_status_.Position() / time_taken_);

  old_time_ = ::millis();
  return true;
}

void WheelController::UpdateTimeout(void) { timeout_ = ::millis(); }

bool WheelController::TimeoutCheck(void) {
  return (::millis() - timeout_ >= kTimeoutMs);
}

// True = forward, False = reverse
void WheelController::SetDirection(const bool &direction) {
  wheel_->Properties().Reverse(!direction);

  digitalWrite(wheel_->Configuration().motor_direction, direction);
}

void WheelController::SetSpeed(const int &speed) {
  if (speed == 0) {
    analogWrite(wheel_->Configuration().motor_speed, abs(speed));
    Drive(false);
  } else if (speed > 0) {
    Drive(true);
    SetDirection(true);
    analogWrite(wheel_->Configuration().motor_speed, abs(speed));
  } else if (speed < 0) {
    Drive(true);
    SetDirection(false);
    analogWrite(wheel_->Configuration().motor_speed, abs(speed));
  }
}

WheelController::wheel_status_t WheelController::WheelStatus(void) {
  return wheel_status_;
}

void WheelController::Break(const bool &kBreak) {
  digitalWrite(wheel_->Configuration().motor_brake, kBreak);
}

void WheelController::Drive(const bool &drive) { Break(!drive); }
