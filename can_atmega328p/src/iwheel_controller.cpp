#include "iwheel_controller.hpp"

bool IWheelController::init(const pin_configuration_t &pinConfiguration)
{
  bool boolean_ = false;

  boolean_ = this->_wheel.init(pinConfiguration) & true;
  boolean_ = this->_wheel_controller.init(pinConfiguration) & true;

  if(this->_wheel.isInverse() && boolean_)
  {
    this->_wheel_controller.setIsInverse(true);
  }

  boolean_ = this->_can_wrapper.init(pinConfiguration) & true;

  return boolean_;
}

bool IWheelController::start(void)
{
  this->_can_wrapper.setCanIdFilterMask(this->_wheel.getCanId());
  this->_wheel_controller.brk(false);

  return true;
}

bool IWheelController::updateReady(void)
{
  return this->_update_flag;
}

bool IWheelController::setUpdateReadyFlag(const bool flag)
{
  this->_update_flag = flag;
}

bool IWheelController::feedbackReady(void)
{
  return this->_feedback_flag;
}

bool IWheelController::setFeedbackReadyFlag(const bool flag)
{
  this->_feedback_flag = flag;
}

bool IWheelController::updateCanMessage(void)
{
  this->_can_wrapper.canIrqHandler();
  this->_can_wrapper.resetCanInterrupts();
}

void IWheelController::resetCan(void)
{
  this->_can_wrapper.resetCan();
}

void IWheelController::resetCanInterrupts(void)
{
  this->_can_wrapper.resetCanInterrupts();
}

bool IWheelController::updateEmptyCanMessage(void)
{
  this->_can_wrapper.cleanCanMsg();
}

void IWheelController::updateTimeout(void)
{
  this->_wheel_controller.updateTimeout();
}

bool IWheelController::updateWheelSignal(void)
{
  this->_wheel_controller.wheelSignalIrqHandler();
}

bool IWheelController::updateCallback(void)
{
  this->_wheel_controller.setSpeed(this->_can_wrapper.getWheelDirection() ? this->_can_wrapper.getSpeed() * -1 : this->_can_wrapper.getSpeed());
  this->setUpdateReadyFlag(false);
  this->setFeedbackReadyFlag(true);

  return true;
}

bool IWheelController::feedbackCallback(void)
{
  constexpr uint8_t dlc = 8;
  uint8_t data[dlc];

  data[0] = static_cast<uint8_t>(CanWrapper::Command::MOTOR_STATUS);
  data[1] = static_cast<uint8_t>(this->_wheel_controller.getWheelEffort()); //real number, divide by 10, 25 = 2.5
  data[2] = static_cast<uint8_t>(this->_wheel_controller.getWheelPosition()); //25.5 truncated to 25
  data[3] = static_cast<uint8_t>(this->_wheel_controller.getWheelPosition() - data[3] * 10); //25.5 - 20 = 0.5 * 10 = 5
  data[4] = static_cast<uint8_t>(this->_wheel_controller.getWheelRpm() / 10); //455 / 10 = 45.5 truncated to 45 
  data[5] = static_cast<uint8_t>(this->_wheel_controller.getWheelRpm() - data[4]); //45.5 - 45 = 0.5 * 10 = 5
  data[6] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity()); //2.5 truncated to 2
  data[7] = static_cast<uint8_t>((this->_wheel_controller.getWheelVelocity() - data[6]) * 10); //2.5 - 2 = 0.5 * 10 = 5

  this->_can_wrapper.canFeedbackHandler(this->_wheel.getFeedbackId(), dlc, data);
  this->setFeedbackReadyFlag(false);

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
    usart_transmit("\r\nthis->_wheel.getCanId(): ");
    sprintf(_nr_to_str, "%d", this->_wheel.getCanId());
    usart_transmit(_nr_to_str);

    usart_transmit("\r\nthis->_can_wrapper.getCanMsg(): ");
    sprintf(_nr_to_str, "%d", this->_can_wrapper.getCanMsg().can_id);
    usart_transmit(_nr_to_str);
    usart_transmit(" ");
    sprintf(_nr_to_str, "%d", this->_can_wrapper.getCanMsg().can_dlc);
    usart_transmit(_nr_to_str);
    usart_transmit(" ");
    
    for (int i = 0; i < this->_can_wrapper.getCanMsg().can_dlc; i++)  
    {
      sprintf(_nr_to_str, "%d", this->_can_wrapper.getCanMsg().data[i]);
      usart_transmit(_nr_to_str);
      usart_transmit(" ");
    }

    usart_transmit("\r\nWheelControllerDiagnostics: ");

    usart_transmit("\r\n  Wheel inversity: ");
    sprintf(_nr_to_str, "%d", this->_wheel.isInverse());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController inversity: ");
    sprintf(_nr_to_str, "%d", this->_wheel_controller.getIsInverse());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController direction: ");
    sprintf(_nr_to_str, "%d", this->_wheel_controller.getWheelDirection());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController velocity (divide by 10): ");
    sprintf(_nr_to_str, "%d", static_cast<int>(this->_wheel_controller.getWheelVelocity() * 10));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController rpm: ");
    sprintf(_nr_to_str, "%d", static_cast<int>(this->_wheel_controller.getWheelRpm()));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController position: ");
    sprintf(_nr_to_str, "%d", static_cast<int>(this->_wheel_controller.getWheelPosition()));
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController effort: ");
    sprintf(_nr_to_str, "%d", this->_wheel_controller.getWheelEffort());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n  WheelController signal counter: ");
    sprintf(_nr_to_str, "%d", this->_wheel_controller.getWheelSignalCounter());
    usart_transmit(_nr_to_str);
    usart_transmit("\r\n");
  #endif
}

bool IWheelController::timeoutCheckCallback(void)
{
  if(this->_wheel_controller.timeoutCheck())
  {
    return true;
  }
  
  return false;
}