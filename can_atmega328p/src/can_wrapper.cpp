/**
 * @file can_wrapper.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
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
#include "can_wrapper.h" // for CanWrapper

#include "wheel.h"
#include "wheel_controller.h"

namespace {
/**
 * @brief Max speed in m/s of the 300 Watt wheel hub measured with the formulas
 * already implemeted to calculate the speed when analogWrite is set to 255 and
 * the voltage in the robot is 29.4V DC.
 */
constexpr auto kWheelMaxSpeed{7.65};
} // namespace

bool CanWrapper::Init(uint8_t transmit_id, uint8_t receive_id) {
  canpressor_ = new CanPackt(transmit_id, receive_id);
  return Setup(receive_id);
}

bool CanWrapper::Setup(int receive_id) {
  MCP2515::ERROR err = MCP2515::ERROR_FAIL;
  mcp_can_ = new MCP2515();

  err = mcp_can_->reset();
  err = mcp_can_->setBitrate(CAN_500KBPS, MCP_8MHZ);
  err = mcp_can_->setConfigMode();
  err = mcp_can_->setFilterMask(MCP2515::MASK0, false, 0x03);
  err = mcp_can_->setFilter(MCP2515::RXF0, false, receive_id);
  err = mcp_can_->setFilter(MCP2515::RXF1, false, receive_id);
  err = mcp_can_->setFilterMask(MCP2515::MASK1, false, 0x03);
  err = mcp_can_->setFilter(MCP2515::RXF2, false, receive_id);
  err = mcp_can_->setFilter(MCP2515::RXF3, false, receive_id);
  err = mcp_can_->setFilter(MCP2515::RXF4, false, receive_id);
  err = mcp_can_->setFilter(MCP2515::RXF5, false, receive_id);
  err = mcp_can_->setNormalMode();

  return err == MCP2515::ERROR_OK;
}

bool CanWrapper::CommandHandler(void) {
  return mcp_can_->readMessage(&can_msg_) != MCP2515::ERROR_OK;
}
// creating a loopback can message
void CanWrapper::FeedbackHandler(
    const WheelController::wheel_status_t &wheel_status) {
  mcp_can_->sendMessage(
      &canpressor_->PackCompressed<WheelController::wheel_status_t,
                                   can_frame_t>(wheel_status));

  // // DEBUG Only when we want to test loopback msg.
  // mcp_can_->sendMessage(&CanMessage());

  // // DEBUG Only when we want to test loopback msg.
  // mcp_can_->sendMessage(
  //     &canpressor_->PackCompressed<WheelController::wheel_status_t,
  //                                  can_frame_t>(WheelCommandStatus()));

  // Reset speed after is sent to clear the last value if the wheel is stopped
}

// @todo(jimmyhalimi): Since no mutex, we need to use volatile for this
can_frame_t CanWrapper::CanMessage(void) const { return can_msg_; }

WheelController::wheel_status_t CanWrapper::WheelCommandStatus(void) const {
  return canpressor_
      ->UnpackCompressed<can_frame_t, WheelController::wheel_status_t>(
          CanMessage());
}

const uint8_t CanWrapper::convertSpeedToUint8(const double &speed) const
    volatile {
  return (abs(speed) * UINT8_MAX / kWheelMaxSpeed);
}

uint8_t CanWrapper::SpeedPwm(void) const {
  return convertSpeedToUint8(WheelCommandStatus().Velocity());
}

bool CanWrapper::Direction(void) const {
  return WheelCommandStatus().Velocity() >= 0;
}

bool CanWrapper::cleanCanMessage(void) {
  can_msg_ = {0, 8, {0}};

  return true;
}
