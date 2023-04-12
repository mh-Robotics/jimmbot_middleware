/**
 * @file constants.h
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file contains the constants that are used in the project.
 * @version 0.1
 * @date 2023-03-21
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
 */
#ifndef JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CONSTANTS_H_
#define JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CONSTANTS_H_
namespace internal {
/**
 * @brief Timeout constant [ms] if no CanBus message is received
 */
constexpr auto kTimeoutMillis{250};

/**
 * @brief Timeout constant [micros] if no CanBus message is received
 */
constexpr auto kTimeoutMicros{kTimeoutMillis * 1000};

/**
 * @brief The HUB Wheel power [Watt]
 */
constexpr auto kMinVelocityToEffort{0.5};

namespace wheel {
/**
 * @brief The HUB Wheel power [Watt]
 */
constexpr auto kPowerInWatt{300};

/**
 * @brief Max speed in m/s of the 300 Watt wheel hub measured with the formulas
 * already implemeted to calculate the speed when analogWrite is set to 255 and
 * the voltage in the robot is 29.4V DC.
 */
constexpr auto kMaxSpeed{2.20};

/**
 * @brief The radius of the wheel in centimeters
 */
const auto kRadius{8.25};

/**
 * @brief The number of encoder pulses per revolution of the wheel
 */
const auto kPulsePerRevolution{90};
} // namespace wheel
} // namespace internal
#endif // JIMMBOT_BOARDS_FIRMWARE_CAN_ATMEGA328P_SRC_CONSTANTS_H_
