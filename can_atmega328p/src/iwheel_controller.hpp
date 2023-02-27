/**
 * @file iwheel_controller.hpp
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
#ifndef CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_
#define CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_

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
   * @return true
   * @return false
   */
  bool Init(const WheelController &wheel_controller,
            const CanWrapper &can_wrapper);

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
  void UpdateWheelSignal(void);

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
  WheelController *wheel_controller_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  CanWrapper *can_wrapper_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile bool update_flag_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile bool feedback_flag_;
};
#endif  // CAN_ATMEGA328P_SRC_INTERFACE_WHEEL_CONTROLLER_HPP_