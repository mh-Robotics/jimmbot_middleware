/**
 * @file can_packt.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Implementation of the canpackt class used to parse can message to
 * motor status and vice versa
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
#include "can_packt.hpp"

can_frame_t
CanPackt::ParseToCan(const WheelController::motor_status_t &motorStatus) {
  can_frame _msg;

  _msg.can_id = motorStatus.FeedbackId();
  _msg.can_dlc = CAN_MAX_DLEN;
  _msg.data[0] = static_cast<uint8_t>(Wheel::Command::kMotorStatus);
  _msg.data[1] = (addPrecision(motorStatus.Effort()));
  _msg.data[2] = ((addPrecision(motorStatus.Position()) >> 8) & 0xff);
  _msg.data[3] = (addPrecision(motorStatus.Position()) & 0xff);
  _msg.data[4] = ((addPrecision(motorStatus.Rpm()) >> 8) & 0xff);
  _msg.data[5] = (addPrecision(motorStatus.Rpm()) & 0xff);
  _msg.data[6] = ((addPrecision(motorStatus.Velocity()) >> 8) & 0xff);
  _msg.data[7] = (addPrecision(motorStatus.Velocity()) & 0xff);

  return _msg;
}

WheelController::motor_status_t
CanPackt::ParseFromCan(const can_frame_t &canFrame) {
  WheelController::motor_status_t _ms;

  _ms.FeedbackId(canFrame.can_id);
  _ms.Effort(removePrecision(canFrame.data[1]));
  _ms.Position(removePrecision((canFrame.data[2] << 8) | canFrame.data[3]));
  _ms.Rpm(removePrecision((canFrame.data[4] << 8) | canFrame.data[5]));
  _ms.Velocity(removePrecision((canFrame.data[6] << 8) | canFrame.data[7]));

  return _ms;
}