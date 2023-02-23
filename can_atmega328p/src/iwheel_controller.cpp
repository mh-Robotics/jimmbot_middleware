/**
 * @file iwheel_controller.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
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
#include "iwheel_controller.hpp"

bool IWheelController::Init(const WheelController &wheel_controller,
                            const CanWrapper &can_wrapper) {
  wheel_controller_ = &wheel_controller;
  can_wrapper_ = &can_wrapper;

  return true;
}

bool IWheelController::Start(void) {
  return can_wrapper_->ConfigureCanIdFilterMask(
      wheel_controller_->MotorStatus().CommandId());
}

bool IWheelController::CommandReady(void) { return update_flag_; }

void IWheelController::CommandReady(const bool &flag) { update_flag_ = flag; }

bool IWheelController::FeedbackReady(void) { return feedback_flag_; }

void IWheelController::FeedbackReady(const bool &flag) {
  feedback_flag_ = flag;
}

bool IWheelController::UpdateCanMessage(void) {
  UpdateTimeout();
  CommandReady(true);
  return can_wrapper_->CommandHandler();
}

void IWheelController::ResetCanInterrupts(void) {
  can_wrapper_->resetCanInterrupts();
}

bool IWheelController::UpdateEmptyCanMessage(void) {
  return can_wrapper_->cleanCanMessage();
}

void IWheelController::UpdateTimeout(void) {
  wheel_controller_->UpdateTimeout();
}

void IWheelController::UpdateWheelSignal(void) {
  wheel_controller_->WheelSignalIrqHandler();
}

bool IWheelController::CommandCallback(void) {
  wheel_controller_->SetSpeed(can_wrapper_->SpeedPwm());
  CommandReady(false);
  FeedbackReady(true);

  return true;
}

bool IWheelController::FeedbackCallback(void) {
  can_wrapper_->FeedbackHandler(
      canpressor_.ParseToCan(wheel_controller_->MotorStatus()));
  FeedbackReady(false);

  return true;
}

bool IWheelController::TimeoutCheckCallback(void) {
  return wheel_controller_->TimeoutCheck();
}
