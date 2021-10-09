/**
 * @file can_wrapper.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 *
 */
#include "can_wrapper.hpp"

bool CanWrapper::Init(const pin_configuration_t &pinConfiguration) {
  return CanWrapper::Setup(pinConfiguration);
}

bool CanWrapper::Setup(const pin_configuration_t &pinConfiguration) {
  mcp_can_ = MCP2515(pinConfiguration.can_mcp_rcv);

  mcp_can_.reset();
  mcp_can_.setBitrate(CAN_500KBPS, MCP_8MHZ);

  return true;
}

bool CanWrapper::CommandHandler(void) {
  return (mcp_can_.readMessage(&can_msg_) != MCP2515::ERROR_OK);
}

void CanWrapper::FeedbackHandler(const can_frame_t &CanMessage) {
  mcp_can_.sendMessage(&CanMessage);
}

bool CanWrapper::ConfigureCanIdFilterMask(const int &canId) {
  MCP2515::ERROR err = MCP2515::ERROR_FAIL;

  err = mcp_can_.setConfigMode();
  err = mcp_can_.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  err = mcp_can_.setFilter(MCP2515::RXF0, false, canId);
  err = mcp_can_.setFilter(MCP2515::RXF1, false, canId);
  err = mcp_can_.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  err = mcp_can_.setFilter(MCP2515::RXF2, false, canId);
  err = mcp_can_.setFilter(MCP2515::RXF3, false, canId);
  err = mcp_can_.setFilter(MCP2515::RXF4, false, canId);
  err = mcp_can_.setFilter(MCP2515::RXF5, false, canId);
  err = mcp_can_.setNormalMode();

  return ((err == MCP2515::ERROR_OK) ? true : false);
}

can_frame_t CanWrapper::CanMessage(void) { return can_msg_; }

uint8_t CanWrapper::SpeedPwm(void) volatile {
  return (can_msg_.data[DIRECTION_BIT_INDEX]
              ? can_msg_.data[SPEED_BIT_INDEX] * -1
              : can_msg_.data[SPEED_BIT_INDEX]);
}

bool CanWrapper::cleanCanMessage(void) {
  can_msg_ = {0, 8, {0}};

  return true;
}

void CanWrapper::resetCanInterrupts(void) {
  mcp_can_.clearRXnOVRFlags();
  mcp_can_.clearInterrupts();
}
