/**
 * @file can_packt.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Implementation of the canpackt class used to parse can message to
 * motor status and vice versa
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
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

  _ms.Effort(removePrecision(canFrame.data[1]));
  _ms.Position(removePrecision((canFrame.data[2] << 8) | canFrame.data[3]));
  _ms.Rpm(removePrecision((canFrame.data[4] << 8) | canFrame.data[5]));
  _ms.Velocity(removePrecision((canFrame.data[6] << 8) | canFrame.data[7]));

  return _ms;
}