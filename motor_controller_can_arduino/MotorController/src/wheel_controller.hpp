#ifndef ___WHEEL_CONTROLLER_H___
#define ___WHEEL_CONTROLLER_H___

#ifndef __AVR_ATmega328P__
// #define __AVR_ATmega328P__
#endif

#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define _clockCyclesPerMicrosecond() (F_CPU / 1000000L)
#define _clockCyclesToMicroseconds(a) (((a) * 1000L) / (F_CPU / 1000L))
#define MICROSECONDS_PER_TIMER0_OVERFLOW (_clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

#include "pin_configuration.hpp"

constexpr int TIME_OUT_MS = 250; // Check timer TIMER0_COMPB_vect prescaler for correct number

class WheelController
{
  public:
    WheelController(void) = default;

    bool init(const pin_configuration_t &pinConfiguration);
    bool setup(void);

    void wheelSignalIrqHandler(void);
    void setMillisIrqHandler(void);
    unsigned long getMillis(void);
    bool timeoutCheck(void);

    bool setIsInverse(bool isInverse);
    bool getIsInverse(void);
    bool setDirection(bool direction);
    bool getDirection(void);

    void setSpeed(const int speed);
    double getWheelVelocity(void);
    int getWheelRpm(void);
    double getWheelDistance(void);
    double getWheelPosition(void);

    void brk(const bool brk);

    ~WheelController() = default;

  private:
    pin_configuration_t _pin_configuration;
    int _signal_counter{0};
    unsigned long _millis;
    unsigned long _old_time{0}, _time_taken{0}, _timeout{0};
    long double _wheel_radius{0.00825}, _wheel_velocity{0}, _wheel_distance{0}, _wheel_position{0};  //Measure the radius of your wheel and enter it here in cm
    unsigned long _wheel_rpm{0};
    unsigned long _timer1_millis{0};
    unsigned long _timer1_overflow_count{0};
    unsigned char _timer1_fract{0};
    bool _is_inverse{false};
    bool _roll_over{false};
    bool _direction{false};
};
#endif //___MOTOR_CONTROLLER_H___
