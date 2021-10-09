/**
 * @file can_wrapper.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * 
 */
#ifndef CAN_ATMEGA328P_SRC_CAN_WRAPPER_H_
#define CAN_ATMEGA328P_SRC_CAN_WRAPPER_H_

#include "mcp2515.h"
#include "pin_configuration.hpp"

class CanWrapper {
public:
  CanWrapper(void) = default;

  bool init(const pin_configuration_t &pinConfiguration);
  bool canIrqHandler(void);
  void canFeedbackHandler(const can_frame_t &canMsg);
  bool setCanIdFilterMask(int canId);
  can_frame_t getCanMsg(void);
  uint8_t getSpeed(void);
  bool getWheelDirection(void);
  bool cleanCanMsg(void);
  void resetCanInterrupts(void);

  ~CanWrapper() = default;

private:
  bool setup(const pin_configuration_t &pinConfiguration);
  uint8_t getSpeedFromCanMsg(void);
  bool getDirectionFromCanMsg(void);

  MCP2515 mcp_can_;
  can_frame_t can_msg_{0, 8, {0}};
  volatile uint8_t speed_pwm_;
  volatile bool direction_;
};
#endif // CAN_WRAPPER_H_
