#ifndef ___PIN_CONFIGURATION_H___
#define ___PIN_CONFIGURATION_H___

typedef struct PinConfiguration
{
  short _motor_enable;
  short _motor_signal;
  short _motor_direction;
  short _motor_speed;

  short _can_mcp_irq;
  short _can_mcp_rcv;
  short _can_mcp_mosi;
  short _can_mcp_miso;
  short _can_mcp_sck;

  short _wheel_front_left;
  short _wheel_front_right;
  short _wheel_back_left;
  short _wheel_back_right;

  PinConfiguration() : _motor_enable{5},
                       _motor_signal{3},
                       _motor_direction{4},
                       _motor_speed{1},
                       _can_mcp_irq{2},
                       _can_mcp_rcv{10},
                       _can_mcp_mosi{11},
                       _can_mcp_miso{12},
                       _can_mcp_sck{13},
                       _wheel_front_left{6},
                       _wheel_front_right{7},
                       _wheel_back_left{8},
                       _wheel_back_right{9} { }

  PinConfiguration(short motorEnable,
                   short motorSignal,
                   short motorDirection,
                   short motorSpeed,
                   short canMcpIrq,
                   short canMcpRcv,
                   short canMcpMosi,
                   short canMcpMiso,
                   short canMcpSck,
                   short wheelFrontLeft,
                   short wheelFrontRight,
                   short wheelBackLeft,
                   short wheelBackRight) : _motor_enable{motorEnable},
                                           _motor_signal{motorSignal},
                                           _motor_direction{motorDirection},
                                           _motor_speed{motorSpeed},
                                           _can_mcp_irq{canMcpIrq},
                                           _can_mcp_rcv{canMcpRcv},
                                           _can_mcp_mosi{canMcpMosi},
                                           _can_mcp_miso{canMcpMiso},
                                           _can_mcp_sck{canMcpSck},
                                           _wheel_front_left{wheelFrontLeft},
                                           _wheel_front_right{wheelFrontRight},
                                           _wheel_back_left{wheelBackLeft},
                                           _wheel_back_right{wheelBackRight} { }
}pin_configuration_t;
#endif //___PIN_CONFIGURATION_H___
