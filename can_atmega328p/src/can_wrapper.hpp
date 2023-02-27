/**
 * @file can_wrapper.hpp
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
#ifndef CAN_ATMEGA328P_SRC_CAN_WRAPPER_H_
#define CAN_ATMEGA328P_SRC_CAN_WRAPPER_H_

#include "can_packt.hpp"  // for PackCompressed<> and UnpackCompressed<>
#include "drivers/include/mcp2515.h"  // for MCP2515
#include "pin_configuration.hpp"      // for PinConfiguration
#include "wheel_controller.hpp"       // for WheelController::wheel_status_t

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
   * @return true
   * @return false
   */
  bool Init(int transmit_id, int receive_id);

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
  void FeedbackHandler(const WheelController::wheel_status_t &wheel_status);

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
   * @return int
   */
  int SpeedPwm(void) volatile;

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool cleanCanMessage(void);

  /**
   * @brief Destroy the Can Wrapper object
   *
   */
  ~CanWrapper() = default;

 private:
  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool Setup(int receive_id);

  /**
   * @brief @todo Add doxy doc
   *
   */
  MCP2515 mcp_can_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  CanPackt *canpressor_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  can_frame_t can_msg_{0, 8, {0}};
};
#endif  // CAN_WRAPPER_H_
