/**
 * @file wheel.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Wheel class declaration that initializes the wheel and its properties.
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef CAN_ATMEGA328P_SRC_WHEEL_HPP_
#define CAN_ATMEGA328P_SRC_WHEEL_HPP_

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "pin_configuration.hpp"

/**
 * @brief @todo Add Doxy doc
 *
 */
class Wheel {
  /**
   * @brief @todo Add Doxy doc
   *
   */
  enum class Wheel_Enum : uint8_t {
    kBegin = 0,
    kFrontLeft = kBegin,
    kFrontRight,
    kBackLeft,
    kBackRight,
    kUnspecified,
    kEnd
  };

  /**
   * @brief @todo Add Doxy doc
   *
   */
  enum class CanId : uint8_t {
    kBegin = 0,
    kCommandWheelFrontLeft = kBegin,
    kCommandWheelFrontRight,
    kCommandWheelBackLeft,
    kCommandWheelBackRight,
    kFeedbackWheelFrontLeft,
    kFeedbackWheelFrontRight,
    kFeedbackWheelBackLeft,
    kFeedbackWheelBackRight,
    kUnspecified,
    kEnd
  };

public:
  /**
   * @brief @todo Add Doxy doc
   *
   */
  enum class Command : uint8_t {
    kBegin = 0,
    kMotorOff = kBegin,
    kMotorOn,
    kMotorStatus,
    kMotorStatusUpdate,
    kMotorSpeed,
    kUnspecified,
    kEnd
  };

  /**
   * @brief Properties strucutre that hold the wheel stats
   *
   */
  typedef struct Properties {
  public:
    uint8_t CommandId(void) { return command_id; }
    void CommandId(uint8_t command_id) { this->command_id = command_id; }
    uint8_t FeedbackId(void) { return feedback_id; }
    void FeedbackId(uint8_t feedback_id) { this->feedback_id = feedback_id; }
    bool Inverse(void) { return inverse; }
    void Inverse(bool inverse) { this->inverse = inverse; }

    double Radius(void) { return kRadius; }
    int PulsePerRevolution(void) { return kPulsePerRevolution; }

  private:
    uint8_t command_id;
    uint8_t feedback_id;
    bool inverse;
    const double kRadius{8.25};
    const int kPulsePerRevolution{90};
  } properties_t;

  /**
   * @brief Construct a new Wheel object
   *
   */
  Wheel(void) = default;

  /**
   * @brief @todo Add Doxy doc
   *
   * @param pinConfiguration
   * @return true
   * @return false
   */
  bool Init(const pin_configuration_t &pinConfiguration);

  /**
   * @brief Returns the properties object
   *
   * @return properties_t
   */
  properties_t Properties(void);

  /**
   * @brief Destroy the Wheel object
   *
   */
  ~Wheel() = default;

private:
  /**
   * @brief Sets the specific properties for wheel
   *
   * @param wheelEnum Specific wheel
   * @return true Set finished sucessfully
   * @return false Set failed
   */
  bool EnumToCanId(Wheel::Wheel_Enum wheelEnum);

  /**
   * @brief @todo Add Doxy doc
   *
   * @param pinConfiguration
   * @return true
   * @return false
   */
  bool Setup(const pin_configuration_t &pinConfiguration);

  /**
   * @brief Determines which wheel does this object belongs too
   *
   * @param pinConfiguration
   * @return Wheel_Enum
   */
  Wheel_Enum DetermineWheel(const pin_configuration_t &pinConfiguration);

  /**
   * @brief Wheel properties variable
   *
   */
  properties_t properties_;
};
#endif // CAN_ATMEGA328P_SRC_WHEEL_HPP_
