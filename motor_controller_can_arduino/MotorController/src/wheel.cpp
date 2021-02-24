#include "wheel.hpp"

bool Wheel::init(const pin_configuration_t &pinConfiguration)
{
  this->_pin_configuration = pinConfiguration;

  return this->setup();
}

bool Wheel::setup(void)
{

  DDRC &= ~(1 << this->_pin_configuration._wheel_front_left); // Configuring PC0 as Input
  PORTC |= (1 << this->_pin_configuration._wheel_front_left); // Enable internal Pull Up

  DDRC &= ~(1 << this->_pin_configuration._wheel_front_right); // Configuring PC1 as Input
  PORTC |= (1 << this->_pin_configuration._wheel_front_right); // Enable internal Pull Up

  DDRC &= ~(1 << this->_pin_configuration._wheel_back_left); // Configuring PC2 as Input
  PORTC |= (1 << this->_pin_configuration._wheel_back_left); // Enable internal Pull Up

  DDRC &= ~(1 << this->_pin_configuration._wheel_back_right); // Configuring PC3 as Input
  PORTC |= (1 << this->_pin_configuration._wheel_back_right); // Enable internal Pull Up

  this->setCanIdFromEnum(this->determineWheel());

  return true;
}

Wheel::Wheel_Enum Wheel::determineWheel(void)
{
  if(!(PINC & (1 << this->_pin_configuration._wheel_front_left)))
  {
    return Wheel::Wheel_Enum::FrontLeft;
  }
  else if(!(PINC & (1 << this->_pin_configuration._wheel_front_right)))
  {
    return Wheel::Wheel_Enum::FrontRight;
  }
  else if(!(PINC & (1 << this->_pin_configuration._wheel_back_left)))
  {
    return Wheel::Wheel_Enum::BackLeft;
  }
  else
  {
    return Wheel::Wheel_Enum::BackRight;
  }

  return Wheel::Wheel_Enum::UNSPECIFIED;
}

bool Wheel::setCanIdFromEnum(Wheel::Wheel_Enum wheelEnum)
{
  switch(wheelEnum)
  {
    case Wheel::Wheel_Enum::FrontLeft : 
    {
      this->_can_id = static_cast<uint8_t>(Wheel::CanId::COMMAND_WHEEL_FRONT_LEFT);
      this->_feedback_id = static_cast<uint8_t>(Wheel::CanId::FEEDBACK_WHEEL_FRONT_LEFT);
      this->_is_inverse = false;
      break;
    }

    case Wheel::Wheel_Enum::FrontRight : 
    {
      this->_can_id = static_cast<uint8_t>(Wheel::CanId::COMMAND_WHEEL_FRONT_RIGHT);
      this->_feedback_id = static_cast<uint8_t>(Wheel::CanId::FEEDBACK_WHEEL_FRONT_RIGHT);
      this->_is_inverse = true;
      break;
    }

    case Wheel::Wheel_Enum::BackLeft : 
    {
      this->_can_id = static_cast<uint8_t>(Wheel::CanId::COMMAND_WHEEL_BACK_LEFT);
      this->_feedback_id = static_cast<uint8_t>(Wheel::CanId::FEEDBACK_WHEEL_BACK_LEFT);
      this->_is_inverse = false;
      break;
    }

    case Wheel::Wheel_Enum::BackRight : 
    {
      this->_can_id = static_cast<uint8_t>(Wheel::CanId::COMMAND_WHEEL_BACK_RIGHT);
      this->_feedback_id = static_cast<uint8_t>(Wheel::CanId::FEEDBACK_WHEEL_FRONT_RIGHT);
      this->_is_inverse = true;
      break;
    }

    default : 
    {
      this->_can_id = static_cast<uint8_t>(Wheel::CanId::UNSPECIFIED);
      this->_feedback_id = static_cast<uint8_t>(Wheel::CanId::UNSPECIFIED);
    }
  }

  return true;
}

uint8_t Wheel::getCanId()
{
  return this->_can_id;
}

uint8_t Wheel::getFeedbackId()
{
  return this->_feedback_id;
}

bool Wheel::isInverse(void)
{
  return this->_is_inverse;
}