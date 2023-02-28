/**
 * @file can_packt.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for packing and unpacking compressed CAN messages.
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
#include "drivers/include/can.h"  // for can_frame_t
#include "wheel_controller.h"     // for WheelController::wheel_status_t

#ifndef CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_
#define CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_

/**
 * @brief Defines a bit-field struct to represent the compressed motor status
 * data
 */
typedef struct __attribute__((packed)) {
  uint32_t command_id : 8; /**< Command ID */
  uint32_t effort : 12;    /**< Effort */
  int32_t position : 19;   /**< Position: 1 sign bit + 18 bits for magnitude */
  uint32_t rpm : 10;       /**< RPM */
  int32_t velocity : 23;   /**< Velocity: 1 sign bit + 22 bits for magnitude */
} compressed_wheel_status_t;

/**
 * @brief A class for packing and unpacking compressed CAN messages
 */
class CanPackt {
 public:
  /**
   * @brief Construct a new Can Packt object
   *
   * @param transmit_id the ID of the transmitter node
   * @param receive_id the ID of the receiver node
   */
  CanPackt(uint8_t transmit_id, uint8_t receive_id)
      : transmit_id_(transmit_id), receive_id_(receive_id){};

  /**
   * @brief Packs a compressed wheel status data structure into a CAN frame
   *
   * @tparam inType the input data type (must be a wheel_status_t)
   * @tparam outType the output data type (must be a can_frame_t)
   * @param data the input data to pack
   * @return the packed CAN frame
   */
  template <typename inType, typename outType>
  outType PackCompressed(const inType &data) {
    static_assert(sizeof(inType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    can_frame_t canFrame;
    canFrame.can_id = transmit_id_;
    canFrame.can_dlc = CAN_MAX_DLEN;

    ::memcpy(canFrame.data, &data, sizeof(inType));

    return canFrame;
  }

  /**
   * @brief Unpacks a compressed wheel status CAN frame into a wheel status data
   * structure
   *
   * @tparam inType the input data type (must be a can_frame_t)
   * @tparam outType the output data type (must be a wheel_status_t)
   * @param msg the CAN frame to unpack
   * @return the unpacked wheel status data structure
   */
  template <typename inType, typename outType>
  outType UnpackCompressed(const inType &can_frame) {
    static_assert(sizeof(outType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    outType data;
    ::memcpy(&data, can_frame.data, sizeof(outType));

    return data;
  }

 private:
  uint8_t transmit_id_{0x00}; /**< The ID of the transmitter node */
  uint8_t receive_id_{0x00};  /**< The ID of the receiver node */
};

/**
 * @brief Specialization of the PackCompressed template function for packing a
 * wheel_status_t into a can_frame_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param wheel_status The input data to pack.
 * @return The packed data as a CAN frame.
 */
template <>
inline can_frame_t
CanPackt::PackCompressed<WheelController::wheel_status_t, can_frame_t>(
    const WheelController::wheel_status_t &wheel_status) {
  can_frame_t canFrame;
  canFrame.can_id = transmit_id_;
  canFrame.can_dlc = CAN_MAX_DLEN;

  // Compress the motor status data into a bit-field struct
  compressed_wheel_status_t compressed_status;
  compressed_status.command_id = wheel_status.CommandId();
  compressed_status.effort = wheel_status.Effort() & 0xFFF;
  compressed_status.position =
      static_cast<int32_t>(wheel_status.Position() * 100);
  compressed_status.rpm = wheel_status.Rpm() & 0x3FF;
  compressed_status.velocity =
      static_cast<int32_t>(wheel_status.Velocity() * 100);

  // Copy the compressed data into the CAN frame
  ::memcpy(canFrame.data, &compressed_status,
           sizeof(compressed_wheel_status_t));

  return canFrame;
}

/**
 * @brief Specialization of the PackCompressed template function for unpacking a
 * can_frame_t into a wheel_status_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param can_frame_t The input data to pack.
 * @return The packed data as a wheel status data structure.
 */
template <>
inline WheelController::wheel_status_t
CanPackt::UnpackCompressed<can_frame_t, WheelController::wheel_status_t>(
    const can_frame_t &can_frame) {
  WheelController::wheel_status_t wheel_status;

  // Extract the compressed data from the CAN frame
  compressed_wheel_status_t compressed_status;
  ::memcpy(&compressed_status, can_frame.data,
           sizeof(compressed_wheel_status_t));

  // Unpack the compressed data into the motor status struct
  wheel_status.CommandId(compressed_status.command_id);
  wheel_status.Effort(compressed_status.effort);
  wheel_status.Position(static_cast<double>(compressed_status.position) / 100);
  wheel_status.Rpm(compressed_status.rpm);
  wheel_status.Velocity(static_cast<double>(compressed_status.velocity) / 100);

  return wheel_status;
}
#endif  // CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_