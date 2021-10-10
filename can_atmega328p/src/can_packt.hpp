/**
 * @file can_packt.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Declaration of the canpackt class used to parse can message to
 * motor status and vice versa
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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
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
#include "can.h"
#include "wheel_controller.hpp"

#ifndef CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_
#define CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_

#define PRECISION 100
#define REVERSE_PRECISION 0.01

/**
 * @brief @todo Add doxy doc
 *
 */
class CanPackt {
public:
  /**
   * @brief Construct a new Can Packt object
   *
   */
  CanPackt(void) = default;

  /**
   * @brief @todo Add doxy doc
   *
   * @param motorStatus
   * @return can_frame_t
   */
  can_frame_t ParseToCan(const WheelController::motor_status_t &motorStatus);

  /**
   * @brief @todo Add doxy doc
   *
   * @param canFrame
   * @return WheelController::motor_status_t
   */
  WheelController::motor_status_t ParseFromCan(const can_frame_t &canFrame);

  /**
   * @brief @todo Add doxy doc
   *
   * @tparam inType
   * @tparam outType
   * @param data
   * @return outType
   */
  template <typename inType, typename outType> outType compress(inType &data) {
    outType _temp;

    return outType{_temp};
  }

  /**
   * @brief @todo Add doxy doc
   *
   * @tparam inType
   * @tparam outType
   * @param msg
   * @return outType
   */
  template <typename inType, typename outType> outType decompress(inType &msg) {
    outType _temp;

    return _temp;
  }

private:
  /**
   * @brief
   *
   * @tparam T
   * @param number
   * @return int
   */
  template <typename T> int addPrecision(T number) {
    return (number * PRECISION);
  }

  /**
   * @brief @todo Add doxy doc
   *
   * @tparam T
   * @param number
   * @return double
   */
  template <typename T> double removePrecision(T number) {
    return (number * REVERSE_PRECISION);
  }
};

#endif // CAN_ATMEGA328P_SRC_CAN_PACKT_HPP_