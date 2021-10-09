#include "iwheel_controller.hpp"

/**
 * @brief Software reset microcontroller
 * Just call: softwareReset(); to reset.
 *
 */
void (*softwareReset)(void) = 0;

IWheelController iWheelController;
pin_configuration_t pinConfiguration;
std::once_flag motor_started_, interrupts_configured_;

void initDrivers(void) {
  millis_init();
  spi_init();
  usart_init();
}

void configureTimer0(void) {
  cli();

  TCCR0A |= (1 << COM0A1);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // for ::millis_get(), configured in millis_init()

  sei();
}

void configureTimer1(void) {
  cli();

  TCCR1A = TCCR1B = TCNT1 = TIMSK1 = 0;
  OCR1A = 31250;
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void configureExternalInt0(void) {
  cli();

  EIMSK |= (1 << INT0);
  EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (1 << ISC00);

  sei();
}

void configureExternalInt1(void) {
  cli();

  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC10);
  EICRA &= ~(0 << ISC11);

  sei();
}

// moved clean cam nsg inside command callbackk. Check when to call, or if it is
// called Check if motor does, bam bam bam.
void timeoutCheckLoop(void) {
  if (iWheelController.timeoutCheckCallback()) {
    iWheelController.commandCallback();
    iWheelController.feedbackCallback();
  }
}

void HalInit(void) {
  std::call_once(interrupts_configured_, []() {
    initDrivers();
    configureTimer0();
    configureTimer1();
    configureExternalInt0();
    configureExternalInt1();
  });
}

void ControllerInit(void) {
  std::call_once(motor_started_, [&]() {
    if (iWheelController.Init(::pinConfiguration)) {
      iWheelController.Start();
    }
  });
}

int main() {
  HalInit();
  ControllerInit();

  while (true) {
    timeoutCheckLoop();
  }

  return EXIT_SUCCESS;
}

/**
 * @brief Interrupt service routine to trigger diagnostics data to serial.
 *
 */
ISR(TIMER1_COMPA_vect) {
  if (interrupts_configured_.is_called && motor_started_.is_called) {
#ifdef __DEBUG__
    iWheelController.diagnosticsCallback();
#endif
  }
}

/**
 * @brief Interrupt service routine when a can message is received.
 *
 */
ISR(INT0_vect) {
  if (interrupts_configured_.is_called && motor_started_.is_called) {
    if (iWheelController.updateCanMessage() &&
        iWheelController.CommandReady()) {
      iWheelController.commandCallback();
    }

    if (iWheelController.FeedbackReady()) {
      iWheelController.feedbackCallback();
    }

    iWheelController.resetCanInterrupts();
  }
}

/**
 * @brief Interrupt service routine when a tick is received from encoder.
 *
 */
ISR(INT1_vect) {
  if (interrupts_configured_.is_called && motor_started_.is_called) {
    // iWheelController.updateWheelSignal(); //todo check the interrupt for
    // signal of odometry
  }
}