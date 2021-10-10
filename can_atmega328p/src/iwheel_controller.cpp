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

bool IWheelController::Init(const pin_configuration_t &pinConfiguration) {
  return (IWheelController::wheel_controller_.Init(pinConfiguration) &
          IWheelController::can_wrapper_.Init(pinConfiguration));
}

bool IWheelController::Start(void) {
  return can_wrapper_.ConfigureCanIdFilterMask(
      wheel_controller_.MotorStatus().CommandId());
}

bool IWheelController::CommandReady(void) { return update_flag_; }

void IWheelController::CommandReady(const bool &flag) { update_flag_ = flag; }

bool IWheelController::FeedbackReady(void) { return feedback_flag_; }

void IWheelController::FeedbackReady(const bool &flag) {
  feedback_flag_ = flag;
}

bool IWheelController::UpdateCanMessage(void) {
  CommandReady(true);
  return can_wrapper_.CommandHandler();
}

void IWheelController::ResetCanInterrupts(void) {
  can_wrapper_.resetCanInterrupts();
}

bool IWheelController::UpdateEmptyCanMessage(void) {
  return can_wrapper_.cleanCanMessage();
}

void IWheelController::UpdateTimeout(void) {
  wheel_controller_.UpdateTimeout();
}

bool IWheelController::UpdateWheelSignal(void) {
  return wheel_controller_.WheelSignalIrqHandler();
}

bool IWheelController::CommandCallback(void) {
  wheel_controller_.SetSpeed(can_wrapper_.SpeedPwm());
  CommandReady(false);
  FeedbackReady(true);
  UpdateTimeout();
  return true;
}

bool IWheelController::FeedbackCallback(void) {
  can_wrapper_.FeedbackHandler(
      canpressor_.ParseToCan(wheel_controller_.MotorStatus()));
  FeedbackReady(false);

  return true;
}

#ifndef NDEBUG
#include <stdio.h>
#endif

void IWheelController::DiagnosticsCallback(void) {
#ifndef NDEBUG
  char _nr_to_str[1024];
  usart_transmit("\r\n::millis_get(): ");
  sprintf(_nr_to_str, "%lu", ::millis_get());
  usart_transmit(_nr_to_str);
  usart_transmit("\r\nwheel_.Properties().CommandId(): ");
  sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().CommandId());
  usart_transmit(_nr_to_str);

  usart_transmit("\r\ncan_wrapper_.CanMessage(): ");
  sprintf(_nr_to_str, "%lu",
          can_wrapper_.CanMessage()
              .can_id); // Maybe we need to omit using CanMessage
  usart_transmit(_nr_to_str);
  usart_transmit(" ");
  sprintf(_nr_to_str, "%d", can_wrapper_.CanMessage().can_dlc);
  usart_transmit(_nr_to_str);
  usart_transmit(" ");

  for (int i = 0; i < can_wrapper_.CanMessage().can_dlc; i++) {
    sprintf(_nr_to_str, "%d", can_wrapper_.CanMessage().data[i]);
    usart_transmit(_nr_to_str);
    usart_transmit(" ");
  }

  usart_transmit("\r\nWheelControllerDiagnostics: ");

  usart_transmit("\r\n  WheelController inversity: ");
  sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().Inverse());
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n  WheelController direction: ");
  sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().Reverse());
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n  WheelController velocity (divide by 10): ");
  sprintf(_nr_to_str, "%d",
          static_cast<int>(wheel_controller_.MotorStatus().Velocity() * 10));
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n  WheelController rpm: ");
  sprintf(_nr_to_str, "%d",
          static_cast<int>(wheel_controller_.MotorStatus().Rpm()));
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n  WheelController position: ");
  sprintf(_nr_to_str, "%d",
          static_cast<int>(wheel_controller_.MotorStatus().Position()));
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n  WheelController effort: ");
  sprintf(_nr_to_str, "%f", wheel_controller_.MotorStatus().Effort());
  usart_transmit(_nr_to_str);
  usart_transmit("\r\n");
#endif
}

bool IWheelController::TimeoutCheckCallback(void) {
  UpdateEmptyCanMessage(); //@todo
  return wheel_controller_.TimeoutCheck();
}