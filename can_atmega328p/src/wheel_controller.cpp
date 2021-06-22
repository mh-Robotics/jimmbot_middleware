#include "wheel_controller.hpp"
#include <limits.h>

bool WheelController::init(const pin_configuration_t &pinConfiguration)
{
  this->_pin_configuration = pinConfiguration;

  return WheelController::setup();
}

bool WheelController::setup()
{
  DDRD |= (1 << this->_pin_configuration._motor_direction);
  DDRD |= (1 << this->_pin_configuration._motor_enable);
  DDRD &= ~(1 << this->_pin_configuration._motor_signal);
  PORTD |= (1 << this->_pin_configuration._motor_signal);
  DDRD |= (1 << this->_pin_configuration._motor_speed);

  return true;
}

void WheelController::wheelSignalIrqHandler(void)
{
  if(this->getWheelDirection())
  {
    this->_signal_counter++;
  }
  else
  {
    this->_signal_counter--;
  }

  //@todo Change logic here, call differently
  this->calculateWheelOdometry();
}

//https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
void WheelController::calculateWheelOdometry()
{
  if(!this->_first_tick)
  {
    this->_old_time = this->getMillis();
    this->_first_tick = true;
  }

  this->_time_taken = this->getMillis() - this->_old_time;
  this->_wheel_rpm = (((1000.0 * 60.0) / this->_wheel_pulses_per_revolution) / this->_time_taken);
  this->_wheel_velocity = (((2 * M_PI * this->_wheel_radius) / this->_wheel_pulses_per_revolution) / this->_time_taken);
  this->_wheel_position = (((2 * M_PI * this->_wheel_radius) / this->_wheel_pulses_per_revolution) * this->_signal_counter); //Pose in CM
  this->_wheel_effort = 25;
  this->_old_time = this->getMillis();
}

void WheelController::updateTimeout(void)
{
  this->_timeout = this->getMillis();
}

unsigned long WheelController::getMillis(void)
{
  return ::millis_get();
}

bool WheelController::timeoutCheck(void)
{
  if(this->getMillis() - this->_timeout > TIME_OUT_MS) //no inetrrupt found for 250ms
  {
    return true;
  }

  return false;
}

bool WheelController::setIsInverse(bool isInverse)
{
  this->_is_inverse = isInverse;

  return true;
}

bool WheelController::getIsInverse(void)
{
  return this->_is_inverse;
}

bool WheelController::setDirection(bool direction)
{
  if(direction)
  {
    PORTD |= (1 << this->_pin_configuration._motor_direction);
  }
  else
  {
    PORTD &= ~(1 << this->_pin_configuration._motor_direction);
  }
  
  this->_direction = direction;

  return true;
}

void WheelController::setSpeed(const int speed)
{
  if(speed == 0)
  {
    this->brk(true);
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
    //@todo If we add a drive with brake, we need to add the pin here.
  }
  else if(speed > 0)
  {
    this->getIsInverse() ? this->setDirection(true) : this->setDirection(false);
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
    this->brk(false);
  }
  else if(speed < 0)
  {
    this->getIsInverse() ? this->setDirection(false) : this->setDirection(true);
    SPEED_CONTROL_PWM(static_cast<uint8_t>(abs(speed)));
    this->brk(false);
  }
}

bool WheelController::getWheelDirection(void)
{
  return this->_direction;
}

long double WheelController::getWheelVelocity(void)
{
  return this->_wheel_velocity;
}

int WheelController::getWheelRpm(void)
{
  return this->_wheel_rpm;
}

long double WheelController::getWheelPosition(void)
{
  return this->_wheel_position;
}

int WheelController::getWheelEffort(void)
{
  return this->_wheel_effort;
}

int WheelController::getWheelSignalCounter(void)
{
  return this->_signal_counter;
}

void WheelController::brk(const bool brk)
{
  if(brk)
  {
    PORTD &= ~(1 << this->_pin_configuration._motor_enable);
  }
  else
  {
    PORTD |= (1 << this->_pin_configuration._motor_enable);
  }
}