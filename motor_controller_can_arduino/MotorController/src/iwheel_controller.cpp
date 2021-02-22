#include "iwheel_controller.hpp"
// #define __DEBUG__

bool IWheelController::init(const pin_configuration_t &pinConfiguration)
{
  #ifdef __DEBUG__
    Serial.begin(115200);
  #endif

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

bool IWheelController::setFeedbackReadyFlag(const bool flag)
{
  this->_feedback_flag = flag;
}

bool IWheelController::updateMillis(void)
{
  this->_wheel_controller.setMillisIrqHandler();
}

bool IWheelController::updateCanMessage(void)
{
  this->_can_wrapper.canIrqHandler();
}

bool IWheelController::updateWheelSignal(void)
{
  this->_wheel_controller.wheelSignalIrqHandler();
}

bool IWheelController::updateCallback(void)
{
  this->setUpdateReadyFlag(false);
  this->_wheel_controller.setSpeed(this->_can_wrapper.getDirection() ? this->_can_wrapper.getSpeed() * -1 : this->_can_wrapper.getSpeed());
  this->setFeedbackReadyFlag(true);

  return true;
}

bool IWheelController::feedbackCallback(void)
{
  this->setFeedbackReadyFlag(false);

  constexpr uint8_t dlc = 8;
  uint8_t data[dlc];

  data[0] = static_cast<uint8_t>(CanWrapper::Command::MOTOR_STATUS);
  data[1] = static_cast<uint8_t>(this->_wheel_controller.getWheelDistance());
  data[2] = static_cast<uint8_t>(this->_wheel_controller.getWheelDistance());
  data[3] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity());
  data[4] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity());
  data[5] = static_cast<uint8_t>(this->_wheel_controller.getWheelRpm()); //Saved for effort, but returning rpm
  data[6] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity());
  data[7] = static_cast<uint8_t>(this->_wheel_controller.getWheelVelocity());

  this->_can_wrapper.canFeedbackHandler(this->_wheel.getFeedbackId(), dlc, data);

  return true;
}

bool IWheelController::diagnosticsCallback(void)
{
  #ifdef __DEBUG__
    Serial.print("\nthis->_wheel.getCanId(): ");
    Serial.println(this->_wheel.getCanId(), HEX);

    Serial.print("this->_can_wrapper.getCanMsg(): ");
    Serial.print(this->_can_wrapper.getCanMsg().can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(this->_can_wrapper.getCanMsg().can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i < this->_can_wrapper.getCanMsg().can_dlc; i++)  
    {
      Serial.print(this->_can_wrapper.getCanMsg().data[i]);
      Serial.print(" ");
    }

    Serial.print("\n");

    Serial.println("WheelControllerDiagnostics: ");

    Serial.print("  Wheel inversity: ");
    Serial.println(this->_wheel.isInverse());
    Serial.print("  WheelController inversity: ");
    Serial.println(this->_wheel_controller.getIsInverse());
    Serial.print("  Wheel velocity: ");
    Serial.println(this->_wheel_controller.getWheelVelocity());
    Serial.print("  Wheel rpm: ");
    Serial.println(this->_wheel_controller.getWheelRpm());
    Serial.print("  Wheel distance: ");
    Serial.println(this->_wheel_controller.getWheelDistance());
  #endif
  return true;
}

bool IWheelController::timeoutCheckCallback(void)
{
  this->_can_wrapper.cleanCanMsg();
  return this->_wheel_controller.timeoutCheck();
}