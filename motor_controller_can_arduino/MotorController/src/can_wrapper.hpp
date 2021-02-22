#ifndef ___CAN_WRAPPER_H___
#define ___CAN_WRAPPER_H___

#include "can/mcp2515.h"

#include "pin_configuration.hpp"

class CanWrapper
{
  public: 
    enum class Command : uint8_t 
    { 
      BEGIN = 0,
      MOTOR_OFF = BEGIN,
      MOTOR_ON,
      MOTOR_STATUS,
      MOTOR_SPEED,
      UNSPECIFIED,
      END
    };

  public:
    CanWrapper(void) = default;

    bool init(const pin_configuration_t &pinConfiguration);
    bool setup(void);
    void canIrqHandler(void);
    void canFeedbackHandler(uint8_t id, uint8_t dlc, uint8_t data[8]);
    bool setCanIdFilterMask(int canId);
    struct can_frame getCanMsg(void);
    int getSpeed(void);
    int getDirection(void);
    void cleanCanMsg(void);

    ~CanWrapper() = default;

  private:
    int getSpeedFromCanMsg(void);
    int getDirectionFromCanMsg(void);

    pin_configuration_t _pin_configuration;
    MCP2515 _mcp_can;
    struct can_frame _can_msg{0, 8, 0};
    struct can_frame _feedback_msg{0, 8, 0};
};
#endif //___CAN_WRAPPER_H___
