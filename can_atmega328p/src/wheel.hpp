/**
 * @file wheel.hpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Wheel class declaration that initializes the wheel and its properties.
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
#ifndef CAN_ATMEGA328P_SRC_WHEEL_HPP_
#define CAN_ATMEGA328P_SRC_WHEEL_HPP_

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
    /**
     * @brief @todo Add doxy doc
     *
     * @return uint8_t
     */
    uint8_t CommandId(void) const { return command_id; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param command_id
     */
    void CommandId(const uint8_t &command_id) { this->command_id = command_id; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return uint8_t
     */
    uint8_t FeedbackId(void) const { return feedback_id; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param feedback_id
     */
    void FeedbackId(const uint8_t &feedback_id) {
      this->feedback_id = feedback_id;
    }

    /**
     * @brief @todo Add doxy doc
     *
     * @return true
     * @return false
     */
    bool Inverse(void) const { return inverse; }

    /**
     * @brief @todo Add doxy doc
     *
     * @param inverse
     */
    void Inverse(const bool &inverse) { this->inverse = inverse; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return double
     */
    double Radius(void) const { return kRadius; }

    /**
     * @brief @todo Add doxy doc
     *
     * @return int
     */
    int PulsePerRevolution(void) const { return kPulsePerRevolution; }

   private:
    /**
     * @brief @todo Add doxy doc
     *
     */
    uint8_t command_id;

    /**
     * @brief @todo Add doxy doc
     *
     */
    uint8_t feedback_id;

    /**
     * @brief @todo Add doxy doc
     *
     */
    bool inverse;

    /**
     * @brief @todo Add doxy doc
     *
     */
    const double kRadius{8.25};

    /**
     * @brief @todo Add doxy doc
     *
     */
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
  bool Init(void);

  /**
   * @brief Returns the properties object
   *
   * @return properties_t
   */
  properties_t Properties(void) const;

  /**
   * @brief Returns the pin_configuration_t object
   *
   * @return pin_configuration_t
   */
  pin_configuration_t Configuration(void) const;

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
  bool EnumToCanId(const Wheel::Wheel_Enum &wheelEnum);

  /**
   * @brief Determines which wheel does this object belongs too
   *
   * @return Wheel_Enum
   */
  Wheel_Enum DetermineWheel(void);

  /**
   * @brief Wheel properties_t object
   *
   */
  properties_t properties_;

  /**
   * @brief Wheel pin_configuration_t object
   *
   */
  pin_configuration_t pin_configuration_;
};
#endif  // CAN_ATMEGA328P_SRC_WHEEL_HPP_
