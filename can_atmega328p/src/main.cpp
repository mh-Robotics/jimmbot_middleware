/**
 * @file main.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Main file that initiates the control of the Hub Motor for jimmBOT
 * robot. It automatically instantiates the configuration for specific wheel and
 * continues the logic where it requires a CanBus message from the HOST and
 * responds with a feedback CanBus message.
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
#include "can_wrapper.h"        // for CanWrapper
#include "iwheel_controller.h"  // for IWheelController
#include "wheel.h"              // for Wheel
#include "wheel_controller.h"   // for WheelController

Wheel wheel;
WheelController wheel_controller;
CanWrapper can_wrapper;

IWheelController i_wheel_controller;

void TimeoutCheckLoop(void) {
  if (i_wheel_controller.TimeoutCheckCallback()) {
    i_wheel_controller.UpdateEmptyCanMessage();
    i_wheel_controller.CommandCallback();
    i_wheel_controller.FeedbackCallback();
  }
}

void CanReceiveHandler() {
  if (i_wheel_controller.UpdateCanMessage() &&
      i_wheel_controller.CommandReady()) {
    i_wheel_controller.CommandCallback();
  }

  if (i_wheel_controller.FeedbackReady()) {
    i_wheel_controller.FeedbackCallback();
  }
}

void IsrEncoderPulse() { i_wheel_controller.UpdateWheelSignal(); }

void setup() {
  wheel.Init();
  wheel_controller.Init(wheel);
  can_wrapper.Init(wheel.Properties().ReceiveId(),
                   wheel.Properties().TransmitId());
  i_wheel_controller.Init(wheel_controller, can_wrapper);

  attachInterrupt(digitalPinToInterrupt(wheel.Configuration().motor_signal),
                  IsrEncoderPulse, RISING);
}

void loop() {
  CanReceiveHandler();
  TimeoutCheckLoop();
  delay(10);
}
