/**
 * @file iwheel_controller.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 *
 */
#ifndef CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_
#define CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_

#include "can_packt.hpp"
#include "can_wrapper.hpp"
#include "wheel_controller.hpp"

/**
 * @brief @todo Add doxy doc
 *
 */
class IWheelController {
public:
  /**
   * @brief Construct a new IWheelController object
   *
   */
  IWheelController() = default;

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
  bool Start(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool CommandReady(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @param flag
   */
  void CommandReady(const bool &flag);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool FeedbackReady(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @param flag
   */
  void FeedbackReady(const bool &flag);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool UpdateCanMessage(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool UpdateEmptyCanMessage(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool UpdateWheelSignal(void);

  /**
   * @brief @todo Add doxy doc
   *
   */
  void ResetCanInterrupts(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool CommandCallback(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool FeedbackCallback(void);

  /**
   * @brief @todo Add doxy doc
   *
   */
  void DiagnosticsCallback(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool TimeoutCheckCallback(void);

  /**
   * @brief Destroy the IWheelController object
   *
   */
  ~IWheelController(void) = default;

private:
  /**
   * @brief @todo Add doxy doc
   *
   */
  void UpdateTimeout(void);

  /**
   * @brief @todo Add doxy doc
   *
   */
  WheelController wheel_controller_;
  CanWrapper can_wrapper_;
  CanPackt canpressor_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile bool update_flag_;
  volatile bool feedback_flag_;
};
#endif // CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_