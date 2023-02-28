/**
 * @file can_wrapper.h
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

#include "can_packt.h"  // for PackCompressed<> and UnpackCompressed<>
#include "drivers/include/mcp2515.h"  // for MCP2515
#include "pin_configuration.h"        // for PinConfiguration
#include "wheel_controller.h"         // for WheelController::wheel_status_t

/**
 * @brief A wrapper class for the MCP2515 CAN controller
 */
class CanWrapper {
 public:
  /**
   * @brief Construct a new Can Wrapper object
   */
  CanWrapper(void) = default;

  /**
   * @brief Initializes the CAN controller with the given transmit and receive
   * IDs
   *
   * @param transmit_id The transmit ID for the CAN controller
   * @param receive_id The receive ID for the CAN controller
   * @return true if initialization was successful, false otherwise
   */
  bool Init(uint8_t transmit_id, uint8_t receive_id);

  /**
   * @brief Reads a message from the CAN bus
   *
   * @return true if a message was successfully read, false otherwise
   */
  bool CommandHandler(void);

  /**
   * @brief Sends a message to the CAN bus
   *
   * @param wheel_status The wheel status data to be sent
   */
  void FeedbackHandler(const WheelController::wheel_status_t &wheel_status);

  /**
   * @brief Configures the CAN ID filter mask for the given CAN ID
   *
   * @param canId The CAN ID to configure the filter mask for
   * @return true if the configuration was successful, false otherwise
   */
  bool ConfigureCanIdFilterMask(const int &canId);

  /**
   * @brief Returns the latest CAN message received
   *
   * @return The latest CAN message received
   */
  can_frame_t CanMessage(void);

  /**
   * @brief Returns the speed value from the latest received CAN message
   *
   * @return The speed value from the latest received CAN message
   */
  int SpeedPwm(void) volatile;

  /**
   * @brief Clears the current CAN message
   *
   * @return true if the message was cleared successfully, false otherwise
   */
  bool cleanCanMessage(void);

  /**
   * @brief Destroy the Can Wrapper object
   */
  ~CanWrapper() = default;

 private:
  /**
   * @brief Sets up the MCP2515 CAN controller with the given receive ID
   *
   * @param receive_id The receive ID for the CAN controller
   * @return true if setup was successful, false otherwise
   */
  bool Setup(int receive_id);

  /**
   * @brief The MCP2515 CAN controller instance
   */
  MCP2515 mcp_can_;

  /**
   * @brief The CanPackt instance for compressed message packing
   */
  CanPackt *canpressor_{nullptr};

  /**
   * @brief The latest CAN message received
   */
  can_frame_t can_msg_{0, 8, {0}};
};
#endif  // CAN_WRAPPER_H_
