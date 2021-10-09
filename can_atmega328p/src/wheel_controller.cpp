#include "wheel_controller.hpp"
#include <limits.h>
#include <util/delay.h>

bool WheelController::Init(const pin_configuration_t &pinConfiguration) {
  pin_configuration_ = pinConfiguration;

  wheel_.Init(pinConfiguration);

  motor_status_.CommandId(wheel_.Properties().CommandId());
  motor_status_.FeedbackId(wheel_.Properties().FeedbackId());
  motor_status_.Inverse(wheel_.Properties().Inverse());

  return WheelController::Setup(); // todo send pins as argument
}

bool WheelController::Setup(void) {
  DDRD |= (1 << pin_configuration_.motor_direction);
  DDRD |= (1 << pin_configuration_.motor_brake);
  DDRD |= (1 << pin_configuration_.motor_enable);
  DDRD &= ~(1 << pin_configuration_.motor_signal);
  PORTD |= (1 << pin_configuration_.motor_signal);
  DDRD |= (1 << pin_configuration_.motor_speed);

  Drive(false);

  return true;
}

bool WheelController::WheelSignalIrqHandler(void) {
  motor_status_.Reverse() ? signal_counter_-- : signal_counter_++;

  //@todo Change logic here, call differently
  return CalculateWheelOdometry();
}

// https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
bool WheelController::CalculateWheelOdometry(void) {
  std::call_once(first_odometry_tick_, [this]() { old_time_ = Millis(); });
  // todo Fix magic numbers, make constants
  time_taken_ = Millis() - old_time_;
  motor_status_.Effort(25);
  motor_status_.Position(((2 * M_PI * wheel_.Properties().Radius()) /
                          wheel_.Properties().PulsePerRevolution()) *
                         signal_counter_); // Pose in CM
  motor_status_.Rpm(
      ((1000.0 * 60.0) / wheel_.Properties().PulsePerRevolution()) /
      time_taken_);
  motor_status_.Velocity(((2 * M_PI * wheel_.Properties().Radius()) /
                          wheel_.Properties().PulsePerRevolution()) /
                         time_taken_);
  old_time_ = Millis();

  return true;
}

void WheelController::UpdateTimeout(void) { timeout_ = Millis(); }

unsigned long WheelController::Millis(void) { return ::millis_get(); }

bool WheelController::TimeoutCheck(void) {
  return (Millis() - timeout_ > kTimeoutMs);
}

bool WheelController::SetDirection(const bool &direction) {
  motor_status_.Reverse(
      direction); // Todo, setDirection makes no sense, change name

  if (!direction) {
    PORTD |= (1 << pin_configuration_.motor_direction);
  } else {
    PORTD &= ~(1 << pin_configuration_.motor_direction);
  }

  return true;
}

void WheelController::SetSpeed(const int &speed) {
  if (speed == 0) {
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
    Drive(false);
  } else if (speed > 0) {
    Drive(true);
    motor_status_.Inverse() ? SetDirection(true) : SetDirection(false);
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
  } else if (speed < 0) {
    Drive(true);
    motor_status_.Inverse() ? SetDirection(false) : SetDirection(true);
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
  }
}

WheelController::motor_status_t WheelController::MotorStatus(void) {
  return motor_status_;
}

void WheelController::EnableDrive(const bool &enable) {
  if (enable) {
    PORTD |= (1 << pin_configuration_.motor_enable);
  } else {
    PORTD &= ~(1 << pin_configuration_.motor_enable);
  }
}

void WheelController::Break(const bool &kBreak) {
  if (kBreak) {
    PORTD |= (1 << pin_configuration_.motor_brake);
  } else {
    PORTD &= ~(1 << pin_configuration_.motor_brake);
  }
}

void WheelController::Drive(const bool &drive) {
  if (drive) {
    Break(false);
    _delay_ms(5);
    EnableDrive(true);
  } else {
    EnableDrive(false);
    _delay_ms(5);
    Break(true);
  }
}