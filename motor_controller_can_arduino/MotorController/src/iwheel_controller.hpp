#ifndef ___INTERFACE_WHEEL_CONTROLLER_H___
#define ___INTERFACE_WHEEL_CONTROLLER_H___

#include "wheel.hpp"
#include "wheel_controller.hpp"
#include "can_wrapper.hpp"

class IWheelController
{
  public:
    IWheelController() = default;

    bool init(const pin_configuration_t &pinConfiguration);
    bool start(void);
    bool updateReady(void);
    bool setUpdateReadyFlag(const bool flag);
    bool setFeedbackReadyFlag(const bool flag);

    bool updateMillis(void);
    bool updateCanMessage(void);
    bool updateWheelSignal(void);

    bool updateCallback(void);
    bool feedbackCallback(void);
    bool diagnosticsCallback(void);
    bool timeoutCheckCallback(void);

    ~IWheelController(void) = default;

  private:
    Wheel _wheel;
    WheelController _wheel_controller;
    CanWrapper _can_wrapper;
    bool _update_flag;
    bool _feedback_flag;
};
#endif //___INTERFACE_MOTOR_CONTROLLER_H___