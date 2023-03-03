/**
 * @file iwheel_controller.h
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
#ifndef CAN_ATMEGA328P_SRC_IWHEEL_CONTROLLER_HPP_
#define CAN_ATMEGA328P_SRC_IWHEEL_CONTROLLER_HPP_

#include "can_wrapper.h"       // for CanWrapper
#include "wheel_controller.h"  // for WheelController

/**
 * @brief Interface for a wheel controller.
 *
 */
class IWheelController {
 public:
  /**
   * @brief Construct a new IWheelController object.
   *
   */
  IWheelController() = default;

  /**
   * @brief Initializes the wheel controller with a specified WheelController
   * object and CanWrapper object.
   *
   * @param wheel_controller The WheelController object.
   * @param can_wrapper The CanWrapper object.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init(WheelController& wheel_controller, CanWrapper& can_wrapper);

  /**
   * @brief Returns the status of the CommandReady flag.
   *
   * @return true if the CommandReady flag is set, false otherwise.
   */
  bool CommandReady(void) const;

  /**
   * @brief Sets the CommandReady flag to the specified value.
   *
   * @param flag The value to set the CommandReady flag to.
   */
  void CommandReady(bool flag);

  /**
   * @brief Returns the status of the FeedbackReady flag.
   *
   * @return true if the FeedbackReady flag is set, false otherwise.
   */
  bool FeedbackReady(void) const;

  /**
   * @brief Sets the FeedbackReady flag to the specified value.
   *
   * @param flag The value to set the FeedbackReady flag to.
   */
  void FeedbackReady(bool flag);

  /**
   * @brief Updates the CAN message for the wheel controller.
   *
   * @return true if the update is successful, false otherwise.
   */
  bool UpdateCanMessage(void);

  /**
   * @brief Updates the empty CAN message for the wheel controller.
   *
   * @return true if the update is successful, false otherwise.
   */
  bool UpdateEmptyCanMessage(void) const;

  /**
   * @brief Updates the wheel signal for the wheel controller.
   */
  void UpdateWheelSignal(void) const;

  /**
   * @brief Callback function for the command.
   *
   * @return true if the callback is successful, false otherwise.
   */
  bool CommandCallback(void);

  /**
   * @brief Callback function for the feedback.
   *
   * @return true if the callback is successful, false otherwise.
   */
  bool FeedbackCallback(void);

  /**
   * @brief Callback function for the timeout check.
   *
   * @return true if the callback is successful, false otherwise.
   */
  bool TimeoutCheckCallback(void) const;

  /**
   * @brief Destroy the IWheelController object.
   */
  ~IWheelController(void) = default;

 private:
  /**
   * @brief Updates the timeout for the wheel controller.
   */
  void UpdateTimeout(void) const;

  /**
   * @brief The WheelController object for the wheel controller.
   */
  WheelController* wheel_controller_{nullptr};

  /**
   * @brief The CanWrapper object for the wheel controller.
   */
  CanWrapper* can_wrapper_{nullptr};

  /**
   * @brief Flag for updating the wheel controller.
   */
  volatile bool update_flag_;

  /**
   * @brief Flag for feedback from the wheel controller.
   */
  volatile bool feedback_flag_;
};
#endif  // CAN_ATMEGA328P_SRC_IWHEEL_CONTROLLER_HPP_
