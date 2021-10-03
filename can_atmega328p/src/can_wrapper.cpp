#include "can_wrapper.hpp"

bool CanWrapper::init(const pin_configuration_t &pinConfiguration)
{
  this->_pin_configuration = pinConfiguration;

  return CanWrapper::setup();
}

bool CanWrapper::setup()
{
  this->_mcp_can = MCP2515(this->_pin_configuration._can_mcp_rcv);

  this->_mcp_can.reset();
  this->_mcp_can.setBitrate(CAN_500KBPS, MCP_8MHZ);

  return true;
}

void CanWrapper::canIrqHandler(void)
{
  if(this->_mcp_can.readMessage(&this->_can_msg) != MCP2515::ERROR_OK) { }
}

void CanWrapper::canFeedbackHandler(uint8_t id, uint8_t dlc, uint8_t data[8])
{
  this->_feedback_msg.can_id = id;
  this->_feedback_msg.can_dlc = dlc;
  memcpy(this->_feedback_msg.data, data, dlc);

  if(this->_mcp_can.sendMessage(&this->_feedback_msg) != MCP2515::ERROR_OK) { }
}

bool CanWrapper::setCanIdFilterMask(int canId)
{
  this->_mcp_can.setConfigMode();
  this->_mcp_can.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  this->_mcp_can.setFilter(MCP2515::RXF0, false, canId);
  this->_mcp_can.setFilter(MCP2515::RXF1, false, canId);
  this->_mcp_can.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  this->_mcp_can.setFilter(MCP2515::RXF2, false, canId);
  this->_mcp_can.setFilter(MCP2515::RXF3, false, canId);
  this->_mcp_can.setFilter(MCP2515::RXF4, false, canId);
  this->_mcp_can.setFilter(MCP2515::RXF5, false, canId);
  this->_mcp_can.setNormalMode();
}

struct can_frame CanWrapper::getCanMsg(void)
{
  return this->_can_msg;
}

int CanWrapper::getWheelDirection(void)
{
  return this->getDirectionFromCanMsg();
}

int CanWrapper::getDirectionFromCanMsg(void)
{
  return this->_can_msg.data[DIRECTION_BIT_INDEX];
}

int CanWrapper::getSpeed(void)
{
  return this->getSpeedFromCanMsg();
}

void CanWrapper::cleanCanMsg(void)
{
  this->_can_msg = { 0, 8, { 0 } };
  this->_feedback_msg = { 0, 8, { 0 } };
}

void CanWrapper::resetCan(void)
{
  this->_mcp_can.reset();
}

void CanWrapper::resetCanInterrupts(void)
{
  this->_mcp_can.clearInterrupts();
}

int CanWrapper::getSpeedFromCanMsg(void)
{
  return this->_can_msg.data[SPEED_BIT_INDEX];
}
