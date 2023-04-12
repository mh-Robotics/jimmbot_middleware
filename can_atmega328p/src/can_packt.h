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
 */
#ifndef JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CAN_PACKT_H_
#define JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CAN_PACKT_H_
#include "drivers/can.h"      // for can_frame_t
#include "wheel_controller.h" // for WheelController::WheelStatus

#include <ArduinoSTL.h> // for ArduinoSTL containers
#include <cstdint>      // for int Data Types
#include <cstring>      // for std::memcpy

/**
 * @brief Defines a bit-field struct to represent the compressed motor status
 * data
 */
using CompressedWheelStatus =
    struct __attribute__((packed)) CompressedWheelStatus {
  uint8_t command_id : 7; /**< Command ID (7 bits), Range 0 to 127 */
  int8_t effort : 7;      /**< Effort (7 bits), Range -64 to 63 */
  int32_t position : 22;  /**< Position (1 sign bit + 21 bits for magnitude),
                             Range  -9999.99 to 9999.99 */
  uint16_t rpm : 10;      /**< RPM (10 bits), Range 0 to 1023  */
  int8_t velocity : 8;    /**< Velocity (signed 8-bit magnitude), Range -5.95
                             to 5.95 */
};

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
   * @tparam inType the input data type (must be a WheelStatus)
   * @tparam outType the output data type (must be a can_frame_t)
   * @param data the input data to pack
   * @return the packed CAN frame
   */
  template <typename inType, typename outType>
  outType PackCompressed(const inType &wheel_status) {
    static_assert(sizeof(inType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    can_frame_t canFrame;
    canFrame.can_id = transmit_id_;
    canFrame.can_dlc = CAN_MAX_DLEN;

    std::memcpy(canFrame.data, &wheel_status, sizeof(inType));

    return canFrame;
  }

  /**
   * @brief Unpacks a compressed wheel status CAN frame into a wheel status data
   * structure
   *
   * @tparam inType the input data type (must be a can_frame_t)
   * @tparam outType the output data type (must be a WheelStatus)
   * @param msg the CAN frame to unpack
   * @return the unpacked wheel status data structure
   */
  template <typename inType, typename outType>
  outType UnpackCompressed(const inType &can_frame) {
    static_assert(sizeof(outType) <= CAN_MAX_DLEN,
                  "Struct is larger than CAN message data field size");

    outType data;
    std::memcpy(&data, can_frame.data, sizeof(outType));

    return data;
  }

private:
  uint8_t transmit_id_{0x00}; /**< The ID of the transmitter node */
  uint8_t receive_id_{0x00};  /**< The ID of the receiver node */
};

/**
 * @brief Specialization of the PackCompressed template function for packing a
 * WheelStatus into a can_frame_t
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param wheel_status The input data to pack.
 * @return The packed data as a CAN frame.
 */
template <>
inline can_frame_t
CanPackt::PackCompressed<WheelController::WheelStatus, can_frame_t>(
    const WheelController::WheelStatus &wheel_status) {
  static_assert(sizeof(CompressedWheelStatus) <= CAN_MAX_DLEN,
                "Struct is larger than CAN message data field size");

  can_frame_t canFrame;
  canFrame.can_id = transmit_id_;
  canFrame.can_dlc = CAN_MAX_DLEN;

  // Compress the motor status data into a bit-field struct
  CompressedWheelStatus compressed_status;
  compressed_status.command_id = static_cast<int8_t>(wheel_status.CommandId());
  compressed_status.effort = static_cast<int>(wheel_status.Effort()) & 0xFFF;
  compressed_status.position =
      static_cast<int32_t>(wheel_status.Position() * 100);
  compressed_status.rpm = static_cast<uint16_t>(wheel_status.Rpm()) & 0x3FF;
  compressed_status.velocity =
      static_cast<int8_t>(wheel_status.Velocity() * 20);

  // Copy the compressed data into the CAN frame
  std::memcpy(canFrame.data, &compressed_status, sizeof(CompressedWheelStatus));

  return canFrame;
}

/**
 * @brief Specialization of the PackCompressed template function for unpacking a
 * can_frame_t into a WheelStatus
 *
 * @tparam inType The type of the input data.
 * @tparam outType The type of the output data.
 * @param can_frame_t The input data to pack.
 * @return The packed data as a wheel status data structure.
 */
template <>
inline WheelController::WheelStatus
CanPackt::UnpackCompressed<can_frame_t, WheelController::WheelStatus>(
    const can_frame_t &can_frame) {
  static_assert(sizeof(can_frame_t::data) <= CAN_MAX_DLEN,
                "Struct is larger than CAN message data field size");

  WheelController::WheelStatus wheel_status;

  // Extract the compressed data from the CAN frame
  CompressedWheelStatus compressed_status;
  std::memcpy(&compressed_status, can_frame.data,
              sizeof(CompressedWheelStatus));

  // Unpack the compressed data into the motor status struct
  wheel_status.CommandId(static_cast<int>(compressed_status.command_id));
  wheel_status.Effort(static_cast<double>(compressed_status.effort));
  wheel_status.Position(static_cast<double>(compressed_status.position) / 100);
  wheel_status.Rpm(static_cast<int>(compressed_status.rpm));
  wheel_status.Velocity(static_cast<double>(compressed_status.velocity) / 20);

  return wheel_status;
}
#endif // JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CAN_PACKT_H_
