#include "iwheel_controller.hpp"

// #define __DEBUG__

constexpr short motorEnable    = PD4;
constexpr short motorSignal    = PD3;
constexpr short motorDirection = PD5;
constexpr short motorSpeed     = PD6;

constexpr short canMcpIrq      = PD2;
constexpr short canMcpRcv      = 10; //PB2
constexpr short canMcpMosi     = PB3;
constexpr short canMcpMiso     = PB4;
constexpr short canMcpSck      = PB5;

constexpr short wheelFrontLeft   = PC0;
constexpr short wheelFrontRight  = PC1;
constexpr short wheelBackLeft    = PC2;
constexpr short wheelBackRight   = PC3;

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

void configureTimer0(void)
{
  cli();

  TCCR0A = TCCR0B = TCNT0 = 0;

  OCR0A = 0;
  OCR0B = 0;

  TCCR0A |= (1 << COM0A1);

  TCCR0A |= (1 << WGM01) | (1 << WGM00);

  TCCR0B |= (1 << CS01);

  sei();
}

void configureTimer1(void)
{
  // cli();

  // TCCR1A = TCCR1B = TCNT1 = TIMSK1 = 0;

  // OCR1A = 8332;
  // OCR1B = OCR1A / 2;

  // TCCR1A |= (1 << WGM11) | (1 << WGM10);

  // TCCR1B |= (1 << WGM13) | (1 << WGM12);
  
  // TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);

  // TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1A) | (1<<TOIE0);

  // sei();
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

void setup() { /* I hate using this */}

void(* resetFunc) (void) = 0; //declare reset function @ address 0
bool startChecker = false;

void loop()
{
  std::call_once(interrupts_configured_, [](){ configureTimer0();
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
    iWheelController.feedbackCallback();
    iWheelController.updateTimeout();
    startChecker = true;
  }
  else if(iWheelController.timeoutCheckCallback() && startChecker)
  {
    resetFunc();
  }
}

static int counter_ = 0;
ISR(TIMER1_COMPA_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called && ++counter_)
  {
    #ifdef __DEBUG__
      iWheelController.diagnosticsCallback();
    #endif  
  }
}

ISR(INT0_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called)
  {
    iWheelController.updateCanMessage();
    iWheelController.setUpdateReadyFlag(true);
  }
}

static int counter1_ = 0;
ISR(INT1_vect)
{
  if(interrupts_configured_.is_called && motor_started_.is_called && ++counter1_ == 2)
  {
    counter1_ = 0;
    iWheelController.updateWheelSignal();
  }
}