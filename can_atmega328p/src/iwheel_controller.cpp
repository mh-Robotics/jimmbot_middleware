#include "iwheel_controller.hpp"

bool IWheelController::Init(const pin_configuration_t &pinConfiguration)
{
  return (IWheelController::wheel_controller_.Init(pinConfiguration) & IWheelController::can_wrapper_.init(pinConfiguration));
}

bool IWheelController::Start(void)
{
  return can_wrapper_.setCanIdFilterMask(wheel_controller_.MotorStatus().CommandId());
}

bool IWheelController::CommandReady(void)
{
  return update_flag_;
}

void IWheelController::CommandReady(const bool flag)
{
  update_flag_ = flag;
}

bool IWheelController::FeedbackReady(void)
{
  return feedback_flag_;
}

void IWheelController::FeedbackReady(const bool flag)
{
  feedback_flag_ = flag;
}

bool IWheelController::updateCanMessage(void)
{
  CommandReady(true);
  return can_wrapper_.canIrqHandler();
}

void IWheelController::resetCanInterrupts(void)
{
  can_wrapper_.resetCanInterrupts();
}

bool IWheelController::updateEmptyCanMessage(void)
{
  return can_wrapper_.cleanCanMsg();
}

void IWheelController::UpdateTimeout(void)
{
  wheel_controller_.UpdateTimeout();
}

bool IWheelController::updateWheelSignal(void)
{
  return wheel_controller_.WheelSignalIrqHandler();
}

bool IWheelController::commandCallback(void)
{
  wheel_controller_.SetSpeed(can_wrapper_.getWheelDirection() ? can_wrapper_.getSpeed() * -1 : can_wrapper_.getSpeed());
  CommandReady(false);
  FeedbackReady(true);
  UpdateTimeout();
  return true;
}

bool IWheelController::feedbackCallback(void)
{
  can_wrapper_.canFeedbackHandler(canpressor_.ParseToCan(wheel_controller_.MotorStatus()));
  FeedbackReady(false);

  return true;
}

#ifdef __DEBUG__
  #include <stdio.h>
#endif

void IWheelController::diagnosticsCallback(void)
{
  #ifdef __DEBUG__
    char _nr_to_str[1024];
    usart_transmit("\r\n::millis_get(): ");
    sprintf(_nr_to_str, "%lu", ::millis_get());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\nwheel_.Properties().CommandId(): ");
    sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().CommandId());
    usart_transmit(_nr_to_str);

    usart_transmit("\r\ncan_wrapper_.getCanMsg(): ");
    sprintf(_nr_to_str, "%lu", can_wrapper_.getCanMsg().can_id); //Maybe we need to omit using getCanMsg
    usart_transmit(_nr_to_str);
    usart_transmit(" ");
    sprintf(_nr_to_str, "%d", can_wrapper_.getCanMsg().can_dlc);
    usart_transmit(_nr_to_str);
    usart_transmit(" ");
    
    for (int i = 0; i < can_wrapper_.getCanMsg().can_dlc; i++)  
    {
      sprintf(_nr_to_str, "%d", can_wrapper_.getCanMsg().data[i]);
      usart_transmit(_nr_to_str);
      usart_transmit(" ");
    }

    usart_transmit("\r\nWheelControllerDiagnostics: ");

    usart_transmit("\r\n  WheelController inversity: ");
    sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().Inverse());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController direction: ");
    sprintf(_nr_to_str, "%d", wheel_controller_.MotorStatus().Reverse());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController velocity (divide by 10): ");
    sprintf(_nr_to_str, "%d", static_cast<int>(wheel_controller_.MotorStatus().Velocity() * 10));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController rpm: ");
    sprintf(_nr_to_str, "%d", static_cast<int>(wheel_controller_.MotorStatus().Rpm()));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController position: ");
    sprintf(_nr_to_str, "%d", static_cast<int>(wheel_controller_.MotorStatus().Position()));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController effort: ");
    sprintf(_nr_to_str, "%f", wheel_controller_.MotorStatus().Effort());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n");
  #endif
}

bool IWheelController::timeoutCheckCallback(void)
{
  return wheel_controller_.TimeoutCheck();
}