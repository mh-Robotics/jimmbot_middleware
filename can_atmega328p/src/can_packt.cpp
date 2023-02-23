/**
 * @file can_packt.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief CanPackt class implementation that is used to parse ::can_frame
 * message to WheelController::MotorStatus and vice-versa.
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
#include "can_packt.hpp"

can_frame_t PackCompressedMotorStatus(const ros_motor_status_t& motorStatus) {
    can_frame_t canFrame;
    canFrame.can_id = motorStatus.can_id;
    canFrame.can_dlc = CAN_MAX_DLEN;

    // Compress the motor status data into a bit-field struct
    compressed_motor_status_t compressedStatus;
    compressedStatus.command_id = motorStatus.command_id;
    compressedStatus.effort = motorStatus.effort;
    compressedStatus.position = static_cast<uint32_t>(motorStatus.position * 100);
    compressedStatus.rpm = motorStatus.rpm;
    compressedStatus.velocity = static_cast<uint32_t>(motorStatus.velocity * 100);
    compressedStatus.can_id = motorStatus.can_id & 0x7FF;

    // Copy the compressed data into the CAN frame
    memcpy(canFrame.data, &compressedStatus, sizeof(compressed_motor_status_t));

    return canFrame;
}

// Unpack a motor status struct from a compressed CAN frame
ros_motor_status_t UnpackCompressedMotorStatus(const can_frame_t& canFrame) {
    ros_motor_status_t motorStatus;

    // Extract the compressed data from the CAN frame
    compressed_motor_status_t compressedStatus;
    memcpy(&compressedStatus, canFrame.data, sizeof(compressed_motor_status_t));

    // Unpack the compressed data into the motor status struct
    motorStatus.can_id = canFrame.can_id;
    motorStatus.command_id = compressedStatus.command_id;
    motorStatus.effort = compressedStatus.effort;
    motorStatus.position = static_cast<double>(compressedStatus.position) / 100;
    motorStatus.rpm = compressedStatus.rpm;
    motorStatus.velocity = static_cast<double>(compressedStatus.velocity) / 100;

    return motorStatus;
}

can_frame_t
CanPackt::ParseToCan(const WheelController::motor_status_t &motorStatus) {
  ros_motor_status_t _msg = {motorStatus.FeedbackId(), static_cast<int>(Wheel::Command::kMotorStatus), motorStatus.Effort(), motorStatus.Position(), motorStatus.Rpm(), motorStatus.Velocity()};
  return PackCompressedMotorStatus(_msg);
}

WheelController::motor_status_t
CanPackt::ParseFromCan(const can_frame_t &canFrame) {
  WheelController::motor_status_t _ms;
  ros_motor_status_t _ros_ms = UnpackCompressedMotorStatus(canFrame);

  _ms.FeedbackId(canFrame.can_id);
  _ms.Effort(_ros_ms.effort);
  _ms.Position(_ros_ms.position);
  _ms.Rpm(_ros_ms.rpm);
  _ms.Velocity(_ros_ms.velocity);

  return _ms;
}