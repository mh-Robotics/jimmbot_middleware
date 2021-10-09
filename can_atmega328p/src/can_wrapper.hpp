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

/**
 * @brief @todo Add doxy doc
 *
 */
class CanWrapper {
public:
  /**
   * @brief Construct a new Can Wrapper object
   *
   */
  CanWrapper(void) = default;

  /**
   * @brief @todo Add doxy doc
   *
   * @param pinConfiguration
   * @return true
   * @return false
   */
  bool Init(const pin_configuration_t &pinConfiguration);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool CommandHandler(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @param CanMessage
   */
  void FeedbackHandler(const can_frame_t &CanMessage);

  /**
   * @brief @todo Add doxy doc
   *
   * @param canId
   * @return true
   * @return false
   */
  bool ConfigureCanIdFilterMask(const int &canId);

  /**
   * @brief @todo Add doxy doc
   *
   * @return can_frame_t
   */
  can_frame_t CanMessage(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return uint8_t
   */
  uint8_t SpeedPwm(void) volatile;

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool cleanCanMessage(void);

  /**
   * @brief @todo Add doxy doc
   *
   */
  void resetCanInterrupts(void);

  /**
   * @brief Destroy the Can Wrapper object
   *
   */
  ~CanWrapper() = default;

private:
  /**
   * @brief @todo Add doxy doc
   *
   * @param pinConfiguration
   * @return true
   * @return false
   */
  bool Setup(const pin_configuration_t &pinConfiguration);

  /**
   * @brief @todo Add doxy doc
   *
   */
  MCP2515 mcp_can_;
  can_frame_t can_msg_{0, 8, {0}};
};
#endif // CAN_WRAPPER_H_
