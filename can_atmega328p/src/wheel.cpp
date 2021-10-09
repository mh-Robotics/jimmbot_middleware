/**
 * @file wheel.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Wheel class implementation that initializes the wheel and its
 * properties.
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "wheel.hpp"

bool Wheel::Init(const pin_configuration_t &pinConfiguration) {
  return Wheel::Setup(pinConfiguration);
}

bool Wheel::Setup(const pin_configuration_t &pinConfiguration) {
  DDRC &= ~(1 << pinConfiguration.wheel_front_left); // PC0 as Input
  PORTC |= (1 << pinConfiguration.wheel_front_left); // Enable internal Pull Up

  DDRC &= ~(1 << pinConfiguration.wheel_front_right); // PC1 as Input
  PORTC |= (1 << pinConfiguration.wheel_front_right); // Enable internal Pull Up

  DDRC &= ~(1 << pinConfiguration.wheel_back_left); // PC2 as Input
  PORTC |= (1 << pinConfiguration.wheel_back_left); // Enable internal Pull Up

  DDRC &= ~(1 << pinConfiguration.wheel_back_right); // PC3 as Input
  PORTC |= (1 << pinConfiguration.wheel_back_right); // Enable internal Pull Up

  return EnumToCanId(DetermineWheel(pinConfiguration));
}

Wheel::Wheel_Enum
Wheel::DetermineWheel(const pin_configuration_t &pinConfiguration) {
  if (!(PINC & (1 << pinConfiguration.wheel_front_left))) {
    return Wheel::Wheel_Enum::kFrontLeft;
  } else if (!(PINC & (1 << pinConfiguration.wheel_front_right))) {
    return Wheel::Wheel_Enum::kFrontRight;
  } else if (!(PINC & (1 << pinConfiguration.wheel_back_left))) {
    return Wheel::Wheel_Enum::kBackLeft;
  } else {
    return Wheel::Wheel_Enum::kBackRight;
  }

  return Wheel::Wheel_Enum::kUnspecified;
}

bool Wheel::EnumToCanId(const Wheel::Wheel_Enum &wheelEnum) {
  switch (wheelEnum) {
  case Wheel::Wheel_Enum::kFrontLeft: {
    properties_.CommandId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontLeft));
    properties_.FeedbackId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontLeft));
    properties_.Inverse(false);

    return true;
  }

  case Wheel::Wheel_Enum::kFrontRight: {
    properties_.CommandId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelFrontRight));
    properties_.FeedbackId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelFrontRight));
    properties_.Inverse(true);

    return true;
  }

  case Wheel::Wheel_Enum::kBackLeft: {
    properties_.CommandId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackLeft));
    properties_.FeedbackId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackLeft));
    properties_.Inverse(false);

    return true;
  }

  case Wheel::Wheel_Enum::kBackRight: {
    properties_.CommandId(
        static_cast<uint8_t>(Wheel::CanId::kCommandWheelBackRight));
    properties_.FeedbackId(
        static_cast<uint8_t>(Wheel::CanId::kFeedbackWheelBackRight));
    properties_.Inverse(true);

    return true;
  }

  default: {
    properties_.CommandId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));
    properties_.FeedbackId(static_cast<uint8_t>(Wheel::CanId::kUnspecified));
    properties_.Inverse(false);

    return true;
  }
  }

  return false;
}

Wheel::properties_t Wheel::Properties() { return properties_; }
