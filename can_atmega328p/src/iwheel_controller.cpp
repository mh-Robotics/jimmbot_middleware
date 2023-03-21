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
#include "iwheel_controller.h" // for IWheelController

bool IWheelController::Init(WheelController &wheel_controller,
                            CanWrapper &can_wrapper) {
  wheel_controller_ = &wheel_controller;
  can_wrapper_ = &can_wrapper;
  return true;
}

bool IWheelController::CommandReady() const { return update_flag_; }

void IWheelController::CommandReady(const bool &flag) { update_flag_ = flag; }

bool IWheelController::FeedbackReady() const { return feedback_flag_; }

void IWheelController::FeedbackReady(const bool &flag) {
  feedback_flag_ = flag;
}

bool IWheelController::UpdateCanMessage() {
  UpdateTimeout();
  CommandReady(true);
  return can_wrapper_->CommandHandler();
}

bool IWheelController::UpdateEmptyCanMessage() const {
  return can_wrapper_->cleanCanMessage();
}

void IWheelController::UpdateTimeout() const {
  wheel_controller_->UpdateTimeout();
}

void IWheelController::UpdateWheelSignal() const {
  wheel_controller_->WheelSignalIrqHandler();
}

bool IWheelController::CommandCallback() {
  wheel_controller_->SetSpeedAndDirection(can_wrapper_->SpeedPwm(),
                                          can_wrapper_->Direction());
  CommandReady(false);
  FeedbackReady(true);

  return true;
}

bool IWheelController::FeedbackCallback() {
  can_wrapper_->FeedbackHandler(wheel_controller_->WheelFeedbackStatus());
  FeedbackReady(false);

  return true;
}

bool IWheelController::TimeoutCheckCallback() const {
  return wheel_controller_->TimeoutCheck();
}
