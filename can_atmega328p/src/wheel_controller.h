/**
 * @file wheel_controller.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Declaration of the WheelController class that controls a Wheel object
 * according to messages received over CanBus.
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics
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

#include "stl_helper_functions.h"  // for std::call_once
#include "wheel.h"                 // for Wheel

/**
 * @brief Controls a Wheel object according to messages received over CanBus.
 *
 */
class WheelController {
 public:
  /**
   * @brief Struct that contains the status of the Wheel.
   *
   */
  typedef struct WheelStatus {
   public:
    /**
     * @brief Gets the command ID associated with the WheelStatus.
     */
    int CommandId(void) const { return command_id; }

    /**
     * @brief Sets the command ID associated with the WheelStatus.
     *
     * @param command_id The command ID to set.
     */
    void CommandId(const int& command_id) { this->command_id = command_id; }

    /**
     * @brief Gets the effort value associated with the WheelStatus.
     */
    int Effort(void) const { return effort; }

    /**
     * @brief Sets the effort value associated with the WheelStatus.
     *
     * @param effort The effort value to set.
     */
    void Effort(const int& effort) { this->effort = effort; }

    /**
     * @brief Gets the position value associated with the WheelStatus.
     */
    double Position(void) const { return position; }

    /**
     * @brief Sets the position value associated with the WheelStatus.
     *
     * @param position The position value to set.
     */
    void Position(const double& position) { this->position = position; }

    /**
     * @brief Gets the RPM value associated with the WheelStatus.
     */
    int Rpm(void) const { return rpm; }

    /**
     * @brief Sets the RPM value associated with the WheelStatus.
     *
     * @param rpm The RPM value to set.
     */
    void Rpm(const int& rpm) { this->rpm = rpm; }

    /**
     * @brief Gets the velocity value associated with the WheelStatus.
     */
    double Velocity(void) const { return velocity; }

    /**
     * @brief Sets the velocity value associated with the WheelStatus.
     *
     * @param velocity The velocity value to set.
     */
    void Velocity(const double& velocity) { this->velocity = velocity; }

   private:
    int command_id{0};
    int effort{10};
    double position{0.0};
    int rpm{0};
    double velocity{0.0};
  } wheel_status_t;

  /**
   * @brief Construct a new Wheel Controller object
   */
  WheelController(void) = default;

  /**
   * @brief Initializes the wheel controller with the given wheel.
   *
   * @param wheel The wheel to be controlled.
   * @return true if initialization was successful, false otherwise.
   */
  bool Init(const Wheel& wheel);

  /**
   * @brief Handles the wheel signal IRQ.
   */
  void WheelSignalIrqHandler(void);

  /**
   * @brief Calculates the wheel odometry.
   *
   * @return true if odometry was calculated, false otherwise.
   */
  bool CalculateWheelOdometry(void);

  /**
   * @brief Updates the timeout value.
   */
  void UpdateTimeout(void);

  /**
   * @brief Checks if the timeout has occurred.
   *
   * @return true if the timeout has occurred, false otherwise.
   */
  bool TimeoutCheck(void);

  /**
   * @brief Sets the direction of the wheel.
   *
   * @param direction The direction of the wheel (true for forward, false for
   * backward).
   */
  void SetDirection(bool direction);

  /**
   * @brief Sets the speed of the wheel.
   *
   * @param speed The speed of the wheel (in PWM 0-255).
   */
  void SetSpeed(uint8_t speed);

  /**
   * @brief Sets the speed and direction of the wheel.
   *
   * @param speed The speed of the wheel (in PWM 0-255).
   * @param direction The direction of the wheel (true for forward, false for
   * backward).
   */
  void SetSpeedAndDirection(uint8_t speed, bool direction);

  /**
   * @brief Gets the current status of the wheel.
   *
   * @return The current status of the wheel.
   */
  wheel_status_t WheelStatus(void);

  /**
   * @brief Drives the wheel.
   *
   * @param drive true to drive the wheel, false to stop it.
   */
  void Drive(bool drive);

  /**
   * @brief Destroys the Wheel Controller object.
   */
  ~WheelController() = default;

 private:
  /**
   * @brief Applies the brake to the wheel.
   *
   * @param kBrake true to apply the brake, false to release it.
   */
  void Brake(bool kBrake);

  /**
   * @brief Disables the drive of the wheel.
   *
   * @param kStop true to disable the drive, false to enable it.
   */
  void Stop(bool kStop);

  wheel_status_t wheel_status_{};
  Wheel* wheel_{nullptr};
  volatile unsigned long timeout_{0};
  volatile double old_time_{0};
  volatile double time_taken_{0};
  volatile signed long signal_counter_{0};
  volatile double last_pulse_time_{0};
  std::once_flag first_odometry_tick_;
};
#endif  // CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_
