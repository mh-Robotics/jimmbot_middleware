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

#include <Arduino.h> // TODO remove after a proper ::millis() implementation

void WheelController::wheelSignalIrqHandler(void)
{
  if(this->getDirection())
  {
    this->_signal_counter++;
  }
  else
  {
    this->_signal_counter--;
  }

  this->_time_taken = this->getMillis() - this->_old_time;
  this->_old_time = this->getMillis();

  this->_wheel_rpm = (1000 / this->_time_taken++) * 60;

  this->_wheel_velocity = this->_wheel_radius * this->_wheel_rpm * 0.104;
  this->_wheel_distance = (2 * M_PI * this->_wheel_radius * this->_signal_counter);
  this->_wheel_position = 9.9;
}

void WheelController::updateTimeout(void)
{
  this->_timeout = this->getMillis();
}

void WheelController::setMillisIrqHandler(void)
{
  this->_timer1_millis += MILLIS_INC;
  this->_timer1_fract += FRACT_INC;

  if (this->_timer1_fract >= FRACT_MAX) 
  {
    this->_timer1_fract -= FRACT_MAX;
    this->_timer1_millis += 1;
  }

  this->_timer1_overflow_count++;
}

unsigned long WheelController::getMillis(void)
{
  //1000/30
  // return this->_timer1_millis * 33.3;
  return ::millis();
}

bool WheelController::timeoutCheck(void)
{
  if(this->getMillis() - this->_timeout > TIME_OUT_MS) //no inetrrupt found for 250ms
  {
    this->_wheel_rpm = this->_wheel_velocity = 0;
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

bool WheelController::getDirection(void)
{
  return this->_direction;
}

void WheelController::setSpeed(const int speed)
{
  if(speed == 0)
  {
    this->brk(true);
    this->setDirection(false);
    OCR0A = static_cast<uint8_t>(abs(speed));
  }
  else if(speed > 0)
  {
    this->getIsInverse() ? this->setDirection(false) : this->setDirection(true);
    OCR0A = static_cast<uint8_t>(abs(speed));
    this->brk(false);
  }
  else if(speed < 0)
  {
    this->getIsInverse() ? this->setDirection(true) : this->setDirection(false);
    OCR0A = static_cast<uint8_t>(abs(speed));
    this->brk(false);
  }
}

double WheelController::getWheelVelocity(void)
{
  return this->_wheel_velocity;
}

int WheelController::getWheelRpm(void)
{
  return this->_wheel_rpm;
}

double WheelController::getWheelDistance(void)
{
  return this->_wheel_distance;
}

double WheelController::getWheelPosition(void)
{
  return this->_wheel_position;
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