#ifndef ___WHEEL_H___
#define ___WHEEL_H___

#include <avr/pgmspace.h>
#include <avr/io.h>

#include "pin_configuration.hpp"

class Wheel
{
  enum class Wheel_Enum : uint8_t
  {
    BEGIN = 0,
    FrontLeft = BEGIN,
    FrontRight,
    BackLeft,
    BackRight,
    UNSPECIFIED,
    END
  };

  enum class CanId : uint8_t 
  { 
    COMMAND_WHEEL_FRONT_LEFT = 0x00,
    COMMAND_WHEEL_FRONT_RIGHT,
    COMMAND_WHEEL_BACK_LEFT,
    COMMAND_WHEEL_BACK_RIGHT,
    FEEDBACK_WHEEL_FRONT_LEFT,
    FEEDBACK_WHEEL_FRONT_RIGHT,
    FEEDBACK_WHEEL_BACK_LEFT,
    FEEDBACK_WHEEL_BACK_RIGHT,
    UNSPECIFIED,
  };

  public:
    Wheel(void) = default;

    bool init(const pin_configuration_t &pinConfiguration);
    uint8_t getCanId(void);
    uint8_t getFeedbackId(void);
    bool isInverse(void);

    ~Wheel() = default;

  private:
    bool setup(void);
    bool setCanIdFromEnum(Wheel::Wheel_Enum wheelEnum);
    Wheel_Enum determineWheel(void);

    pin_configuration_t _pin_configuration;
    uint8_t _can_id;
    uint8_t _feedback_id;
    bool _is_inverse;
};
#endif //___WHEEL_H___