#ifndef ___WHEEL_CONTROLLER_H___
#define ___WHEEL_CONTROLLER_H___

#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pin_configuration.hpp"

class WheelController
{
  public:
    WheelController(void) = default;

    bool init(const pin_configuration_t &pinConfiguration);
    bool setup(void);

    void wheelSignalIrqHandler(void);
    void calculateWheelOdometry(void);
    void updateTimeout(void);
    unsigned long getMillis(void);
    bool timeoutCheck(void);

    bool setIsInverse(bool isInverse);
    bool getIsInverse(void);
    bool setDirection(bool direction);
    bool getWheelDirection(void);

    void setSpeed(const int speed);
    long double getWheelVelocity(void);
    int getWheelRpm(void);
    int getWheelEffort(void);
    int getWheelSignalCounter(void);
    long double getWheelPosition(void);

    void brk(const bool brk);

    ~WheelController() = default;

  private:
    pin_configuration_t _pin_configuration;
    unsigned long _signal_counter{0};
    unsigned long _millis;
    unsigned long _old_time{0}, _time_taken{0}, _timeout{0};
    unsigned long _wheel_rpm{0};
    long double _wheel_radius{8.25}, _wheel_velocity{0}, _wheel_position{0}; //Values here in CM, Velocity in m/s
    int _wheel_effort{0}, _wheel_pulses_per_revolution{90};
    bool _first_tick{false};
    bool _is_inverse{false};
    bool _roll_over{false};
    bool _direction{false};
};
#endif //___MOTOR_CONTROLLER_H___
