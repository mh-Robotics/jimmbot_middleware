/**
 * @file wheel_controller.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief WheelController class declaration that initializes the controller and
 * instantiates a Wheel. It controls and sets the logic as requested from CanBus
 * message.
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
#ifndef CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_
#define CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_

#include <math.h>
#include <stdlib.h>

#include "stl_helper_functions.h"
#include "wheel.hpp"

/**
 * @brief @todo Add doxy doc
 *
 */
class WheelController {
 public:
  /**
   * @brief @todo Add doxy doc
   *
   */
  typedef struct WheelStatus {
   public:
    /**
     * @brief @todo Add doxy doc
     *
     * @return int
     */
    int CommandId(void) const { return command_id; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param command_id
     */
    void CommandId(const int &command_id) { this->command_id = command_id; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return double
     */
    int Effort(void) const { return effort; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param effort
     */
    void Effort(const double &effort) { this->effort = effort; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return double
     */
    double Position(void) const { return position; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param position
     */
    void Position(const double &position) { this->position = position; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return int
     */
    int Rpm(void) const { return rpm; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param rpm
     */
    void Rpm(const int &rpm) { this->rpm = rpm; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return double
     */
    double Velocity(void) const { return velocity; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param velocity
     */
    void Velocity(const double &velocity) { this->velocity = velocity; }

   private:
    int command_id{0};
    int effort{10};
    double position{0.0};
    int rpm{0};
    double velocity{0.0};
  } wheel_status_t;

  /**
   * @brief Construct a new Wheel Controller object
   *
   */
  WheelController(void) = default;

  /**
   * @brief @todo Add doxy doc
   *
   * @param pinConfiguration
   * @return true
   * @return false
   */
  bool Init(const Wheel &wheel);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool Setup(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  void WheelSignalIrqHandler(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool CalculateWheelOdometry(void);

  /**
   * @brief @todo Add doxy doc
   *
   */
  void UpdateTimeout(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @return true
   * @return false
   */
  bool TimeoutCheck(void);

  /**
   * @brief Set the Direction object
   *
   * @param direction
   * @return true
   * @return false
   */
  void SetDirection(const bool &direction);

  /**
   * @brief Set the Speed object
   *
   * @param speed
   */
  void SetSpeed(const int &speed);

  /**
   * @brief @todo Add doxy doc
   *
   * @return wheel_status_t
   */
  wheel_status_t WheelStatus(void);

  /**
   * @brief @todo Add doxy doc
   *
   * @param drive
   */
  void Drive(const bool &drive);

  /**
   * @brief Destroy the Wheel Controller object
   *
   */
  ~WheelController() = default;

 private:
  /**
   * @brief @todo Add doxy doc
   *
   * @param kBreak
   */
  void Break(const bool &kBreak);

  /**
   * @brief @todo Add doxy doc
   *
   */
  wheel_status_t wheel_status_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  Wheel *wheel_;

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile unsigned long timeout_{0};

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile double old_time_{0};

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile double time_taken_{0};

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile signed long signal_counter_{0};

  /**
   * @brief @todo Add doxy doc
   *
   */
  volatile double last_pulse_time_{0};

  /**
   * @brief @todo Add doxy doc
   *
   */
  std::once_flag first_odometry_tick_;
};
#endif  // CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_
