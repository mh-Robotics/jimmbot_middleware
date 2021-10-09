/**
 * @file wheel_controller.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 *
 */
#ifndef CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_
#define CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdlib.h>

#include "wheel.hpp"

// todo Check inherit wheel into wheel controller, so when we instantiate
// wheelcontroller,
// we have a ready wheel instantiated

namespace std {
struct once_flag {
  bool is_called = false;
};

template <class Callable, class... Args>
void inline call_once(once_flag &flag, Callable &&f, Args &&... args) {
  if (!flag.is_called) {
    f();
    flag.is_called = true;
  }
}
} // namespace std

class WheelController : private Wheel {
public:
  typedef struct MotorStatus {
  public:
    int CommandId(void) const { return command_id; }
    void CommandId(const int &command_id) { this->command_id = command_id; }
    int FeedbackId(void) const { return feedback_id; }
    void FeedbackId(const int &feedback_id) { this->feedback_id = feedback_id; }
    double Effort(void) const { return effort; }
    void Effort(const double &effort) { this->effort = effort; }
    double Position(void) const { return position; }
    void Position(const double &position) { this->position = position; }
    int Rpm(void) const { return rpm; }
    void Rpm(const int &rpm) { this->rpm = rpm; }
    double Velocity(void) const { return velocity; }
    void Velocity(const double &velocity) { this->velocity = velocity; }
    bool Inverse(void) const { return inverse; }
    void Inverse(const bool &inverse) { this->inverse = inverse; }
    bool Reverse(void) const { return reverse; }
    void Reverse(const bool &reverse) { this->reverse = reverse; }

  private:
    int command_id;
    int feedback_id;
    double effort;
    double position;
    int rpm;
    double velocity;
    bool inverse;
    bool reverse;
  } motor_status_t;

  WheelController(void) = default;

  bool Init(const pin_configuration_t &pinConfiguration);
  bool Setup(void);

  bool WheelSignalIrqHandler(void);
  bool CalculateWheelOdometry(void);
  void UpdateTimeout(void);
  unsigned long Millis(void);
  bool TimeoutCheck(void);

  bool SetDirection(const bool &direction);
  void SetSpeed(const int &speed);
  motor_status_t MotorStatus(void);

  void Drive(const bool &drive);

  ~WheelController() = default;

private:
  void EnableDrive(const bool &state);
  void Break(const bool &kBreak);

  pin_configuration_t pin_configuration_;
  motor_status_t motor_status_;
  Wheel wheel_;

  volatile unsigned long timeout_{0}, old_time_{0}, time_taken_{0},
      signal_counter_{0};
  std::once_flag first_odometry_tick_;
};
#endif // CAN_ATMEGA328P_SRC_WHEEL_CONTROLLER_HPP_
