#include "iwheel_controller.hpp"

constexpr short motorEnable    = PD4;
constexpr short motorSignal    = PD3;
constexpr short motorDirection = PD5;
constexpr short motorSpeed     = PD6;

constexpr short canMcpIrq      = PD2;
constexpr short canMcpRcv      = PB2;
constexpr short canMcpMosi     = PB3;
constexpr short canMcpMiso     = PB4;
constexpr short canMcpSck      = PB5;

constexpr short wheelFrontLeft   = PC0;
constexpr short wheelFrontRight  = PC1;
constexpr short wheelBackLeft    = PC2;
constexpr short wheelBackRight   = PC3;

/**
 * @brief Software reset microcontroller
 * Just call: softwareReset(); to reset.
 * 
 */
void(* softwareReset) (void) = 0;

namespace std
{
  struct once_flag 
  {
    bool is_called = false;
  };

  template< class Callable, class... Args >
  void inline call_once( once_flag& flag, Callable&& f, Args&&... args )
  {
    if(!flag.is_called)
    {
      f();
      flag.is_called = true;
    }
  }
}

pin_configuration_t pinConfiguration{motorEnable,
                                     motorSignal,
                                     motorDirection,
                                     motorSpeed,
                                     canMcpIrq,
                                     canMcpRcv,
                                     canMcpMosi,
                                     canMcpMiso,
                                     canMcpSck,
                                     wheelFrontLeft,
                                     wheelFrontRight,
                                     wheelBackLeft,
                                     wheelBackRight};

IWheelController iWheelController;
std::once_flag motor_started_, interrupts_configured_;

void initDrivers(void)
{
  millis_init();
  spi_init();
  usart_init();
}

void configureTimer0(void)
{
  cli();

  TCCR0A |= (1 << COM0A1);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS01) | (1 << CS00);
  //for ::millis_get(), configured in millis_init()

  sei();
}

void configureTimer1(void)
{
  cli();

  TCCR1A = TCCR1B = TCNT1 = TIMSK1 = 0;
  OCR1A = 31250;
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void configureExternalInt0(void)
{
  cli();

  EIMSK |= (1 << INT0);
  EICRA  = ( EICRA & ~( (1 << ISC00) | (1 << ISC01) ) ) | (1 << ISC00);

  sei();
}

void configureExternalInt1(void)
{
  cli();

  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC10);
  EICRA &= ~(0 << ISC11);

  sei();
}

int main()
{
  while(true)
  {
    std::call_once(interrupts_configured_, [](){
                                                initDrivers();
                                                configureTimer0();
                                                configureTimer1();
                                                configureExternalInt0();
                                                configureExternalInt1();
                                               });

    std::call_once(motor_started_, [](){ if(iWheelController.init(::pinConfiguration))
                                         {
                                            iWheelController.start();
                                         }
                                       });

    if(iWheelController.updateReady())
    {
      iWheelController.updateCallback();
      iWheelController.updateTimeout();
    }
    else if(iWheelController.timeoutCheckCallback())
    {
      iWheelController.updateEmptyCanMessage();
      iWheelController.setUpdateReadyFlag(true);
    }

    if(iWheelController.feedbackReady())
    {
      iWheelController.feedbackCallback();
    }
  }

  return EXIT_SUCCESS;
}

/**
 * @brief Interrupt service routine to trigger diagnostics data to serial.
 * 
 */
ISR(TIMER1_COMPA_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called)
  {
    #ifdef __DEBUG__
      iWheelController.diagnosticsCallback();
    #endif

    //todo: setFeedbackReadyFlag(true);
    //todo: mutex the can feedback send and receive
  }
}

/**
 * @brief Interrupt service routine when a can message is received.
 * 
 */
ISR(INT0_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called)
  {
    iWheelController.updateCanMessage();
    iWheelController.setUpdateReadyFlag(true);
  }
}

/**
 * @brief Interrupt service routine when a tick is received from encoder.
 * 
 */
ISR(INT1_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called)
  {
    iWheelController.updateWheelSignal();
  }
}