/**
 * @file main.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief Main file that initiates the control of the Hub Motor for jimmBOT
 * robot. It automatically instantiates the configuration for specific wheel and
 * continues the logic where it requires a CanBus message from the HOST and
 * responds with a feedback CanBus message.
 * @version 0.1
 * @date 2021-10-10
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
#include "iwheel_controller.hpp"

/**
 * @brief Software reset microcontroller. Just call: SoftwareReset(); to reset.
 *
 */
void (*SoftwareReset)(void) = 0;

/**
 * @brief Wheel Controller Interface instance
 *
 */
IWheelController interface_wheel_controller;

/**
 * @brief Pin Configuration instance with default values
 *
 */
pin_configuration_t pin_configuration;

/**
 * @brief Holds the value if motor is started. It is set when called once
 *
 */
std::once_flag motor_started;

/**
 * @brief Holds the value if interrupts are configured. It is set when called
 * once
 *
 */
std::once_flag interrupts_configured;

/**
 * @brief Initializes the necessary drivers used in this firmware
 *
 */
void InitDrivers(void) {
  millis_init();
  spi_init();
  usart_init();
}

/**
 * @brief Configures the Timer 0 for Atmega328P
 *
 */
void ConfigureTimer0(void) {
  cli();

  TCCR0A |= (1 << COM0A1);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // for ::millis_get(), configured in millis_init()

  sei();
}

/**
 * @brief Configures the Timer 1 for Atmega328P
 *
 */
void ConfigureTimer1(void) {
  cli();

  TCCR1A = TCCR1B = TCNT1 = TIMSK1 = 0;
  OCR1A = 31250; // todo constant magic number
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

/**
 * @brief Configures the External Interrupt 0 for Atmega328P
 *
 */
void ConfigureExternalInt0(void) {
  cli();

  EIMSK |= (1 << INT0);
  EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (1 << ISC00);

  sei();
}

/**
 * @brief Configures the External Interrupt 1 for Atmega328P
 *
 */
void ConfigureExternalInt1(void) {
  cli();

  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC10);
  EICRA &= ~(0 << ISC11);

  sei();
}

/**
 * @brief Function that calls necessary functions and checks if there is no
 * CanBus message received for specified Tiemout constant
 *
 * @todo Moved CleanCanMsg inside CommandCallback, check if motor acts strange
 * and if by any chance this is called while driving
 *
 */
void TimeoutCheckLoop(void) {
  if (interface_wheel_controller.TimeoutCheckCallback()) {
    interface_wheel_controller.CommandCallback();
    interface_wheel_controller.FeedbackCallback();
  }
}

/**
 * @brief Initializes all the Hardware Related configurations and calls driver
 * initialization
 *
 */
void HalInit(void) {
  std::call_once(interrupts_configured, [&] {
    InitDrivers();
    ConfigureTimer0();
    ConfigureTimer1();
    ConfigureExternalInt0();
    ConfigureExternalInt1();
  });
}

/**
 * @brief Initializes and starts the motor
 *
 */
void ControllerInit(void) {
  std::call_once(motor_started, [&] {
    if (interface_wheel_controller.Init(::pin_configuration)) {
      interface_wheel_controller.Start();
    }
  });
}

/**
 * @brief Main function for the entire program
 *
 * @return int Exit status
 */
int main() {
  while (true) {
    HalInit();
    ControllerInit();

    TimeoutCheckLoop();
  }

  return EXIT_SUCCESS;
}

/**
 * @brief Interrupt service routine to trigger diagnostics data to serial.
 *
 */
ISR(TIMER1_COMPA_vect) {
  if (interrupts_configured.is_called && motor_started.is_called) {
#ifndef NDEBUG
    interface_wheel_controller.DiagnosticsCallback();
#endif
  }
}

/**
 * @brief Interrupt service routine when a can message is received.
 *
 */
ISR(INT0_vect) {
  if (interrupts_configured.is_called && motor_started.is_called) {
    if (interface_wheel_controller.UpdateCanMessage() &&
        interface_wheel_controller.CommandReady()) {
      interface_wheel_controller.CommandCallback();
    }

    if (interface_wheel_controller.FeedbackReady()) {
      interface_wheel_controller.FeedbackCallback();
    }

    interface_wheel_controller.ResetCanInterrupts();
  }
}

/**
 * @brief Interrupt service routine when a tick is received from encoder.
 *
 * @todo Uncomment the call to updateWheelSignal and check if the extensive call
 * to the interrupt causes a lag in main loop.
 */
ISR(INT1_vect) {
  if (interrupts_configured.is_called && motor_started.is_called) {
    // interface_wheel_controller.updateWheelSignal();
  }
}