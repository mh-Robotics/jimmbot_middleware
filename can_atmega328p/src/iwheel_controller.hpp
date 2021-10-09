#ifndef ___INTERFACE_WHEEL_CONTROLLER_H___
#define ___INTERFACE_WHEEL_CONTROLLER_H___

#include "wheel.hpp"
#include "wheel_controller.hpp"
#include "can_wrapper.hpp"

#include "can_packt.hpp"

class IWheelController
{
  public:
    IWheelController() = default;

    bool Init(const pin_configuration_t &pinConfiguration);
    bool Start(void);
    bool CommandReady(void);
    void CommandReady(const bool flag);
    bool FeedbackReady(void);
    void FeedbackReady(const bool flag);

    bool updateCanMessage(void);
    void resetCanInterrupts(void);
    bool updateEmptyCanMessage(void);
    
    bool updateWheelSignal(void);

    bool commandCallback(void);
    bool feedbackCallback(void);
    void diagnosticsCallback(void);
    bool timeoutCheckCallback(void);

    ~IWheelController(void) = default;

  private:
    void UpdateTimeout(void);
    WheelController wheel_controller_;
    CanWrapper can_wrapper_; //Inherit canpackt and remove canpackt instance here @todo
    CanPackt canpressor_;

    volatile bool update_flag_;
    volatile bool feedback_flag_;
};
#endif //___INTERFACE_MOTOR_CONTROLLER_H___