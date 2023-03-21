/**
 * @file speed_to_pwm.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief A class for mapping linear velocity to PWM value.
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
#include "constants.h"            // for k*
#include "stl_helper_functions.h" // for std::prev

#include <ArduinoSTL.h> // for ArduinoSTL containers
#include <cmath>        // for std::abs
#include <cstdint>      // for uint8_t
#include <map>          // for std::map

#ifndef JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_SPEED_TO_PWM_H_
#define JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_SPEED_TO_PWM_H_

/**
 * @brief The SpeedToPWM class provides a lookup table for mapping speed values
 * to PWM values. @todo(jimmyhalimi): This class needs to update to PID Control.
 */
class SpeedToPWM {
public:
  /**
   * @brief Constructor to initialize the lookup table with speed to PWM
   * mappings.
   */
  SpeedToPWM() {
    // Initialize the lookup table with speed to PWM mappings
    map_[0.00] = 0;
    map_[0.10] = 15;
    map_[0.22] = 30;
    map_[0.36] = 45;
    map_[0.50] = 60;
    map_[0.64] = 75;
    map_[0.78] = 90;
    map_[0.90] = 105;
    map_[1.00] = 120;
    map_[1.15] = 135;
    map_[1.25] = 150;
    map_[1.40] = 165;
    map_[1.50] = 180;
    map_[1.70] = 195;
    map_[1.80] = 210;
    map_[1.95] = 225;
    map_[2.10] = 240;
    map_[internal::wheel::kMaxSpeed] = 255;
  }

  /**
   * @brief Get the PWM value that corresponds to the given speed value.
   *
   * @param speed The speed value to get the PWM value for.
   * @return The PWM value that corresponds to the given speed value.
   */
  uint8_t GetPWM(double speed) const {
    // Ensure that speed is within the range of the map
    if (speed < 0.0) {
      speed = std::abs(speed);
    }
    if (speed > internal::wheel::kMaxSpeed) {
      speed = internal::wheel::kMaxSpeed;
    }

    // Find the closest speed value in the map
    auto it = map_.lower_bound(speed);
    if (it == map_.begin()) {
      // If speed is less than the first value in the map, return the first PWM
      // value
      return it->second;
    } else if (it == map_.end()) {
      // If speed is greater than the last value in the map, return the last PWM
      // value
      return std::prev(it)->second;
    } else {
      // Interpolate between the two closest speed values in the map
      double prev_speed = std::prev(it)->first;
      double next_speed = it->first;
      int prev_pwm = std::prev(it)->second;
      int next_pwm = it->second;
      double alpha = (speed - prev_speed) / (next_speed - prev_speed);
      return (1.0 - alpha) * prev_pwm + alpha * next_pwm;
    }
  }

private:
  std::map<double, int> map_;
};
#endif // JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CAN_PACKT_H_
