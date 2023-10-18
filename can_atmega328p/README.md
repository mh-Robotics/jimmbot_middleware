# MotorController Firmware

This is the firmware code for the motor hub of jimmBOT. This README file provides information on how to build the code
and what to expect.

## Building the code

### Release build

A release build disables UART.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Debug build

A debug build enables UART and prints data to serial. The baud rate for the serial communication is not specified in the
code, so it needs to be set manually in your terminal program.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```

### Additional build options

To build the documentation and tests, use the following options:

```bash
cmake -DCMAKE_BUILD_DOC=On ..
cmake -DCMAKE_BUILD_TESTS=On ..
```

## Hardware Setup

Connect your motor hub to your computer via USB. If the firmware is running, it should enumerate as a virtual serial
port. The port name varies depending on your system. On Linux, it will typically be /dev/ttyACM0 or /dev/ttyACM1. On
Windows, it will typically be COM3 or COM4. You can use a terminal program such as PuTTY or TeraTerm to communicate with
the motor hub.

### Pinout

The pinout and conenctions are as follows:

```bash
       +-------------+
       | Arduino Pro |
       |    Micro    |
       +-------------+
              |
              |
              +-----------+
              |  ZS-X11H  |
              |  Motor    |
              |  Driver   |
              +-----------+
        motor_brake{7}    | <---> BRAKE
        motor_stop{4}     | <---> STOP
        motor_signal{3}   | <---> S
        motor_direction{5}| <---> DIR
        motor_speed{6}    | <---> SPEED
              |
              |
              +-------------+
              | MCP2515 CAN |
              | Controller  |
              +-------------+
          can_mcp_irq{2}    | <---> INT
          can_mcp_rcv{10}   | <---> SO
          can_mcp_mosi{11}  | <---> SI
          can_mcp_miso{12}  | <---> SCK
          can_mcp_sck{13}   | <---> CS
              |
              |
       +--------------------+   +---------------------+   +-------------------+  +--------------------+
       |  Wheel Front Left  |   |  Wheel Front Right  |   |  Wheel Back Left  |  |  Wheel Back Right  |
       +--------------------+   +---------------------+   +-------------------+  +--------------------+
        wheel_front_left{A0}     wheel_front_right{A1}     wheel_back_left{A2}    wheel_back_right{A3}
```

## Communication Protocol

The motor hub communicates with a host computer over a serial connection. The protocol is ASCII-based, with each command
and response terminated by a newline character (\n).

### Command Format

The command frame format is:

```bash
| ID (2 bytes) | Length (1 byte) | Data (up to 8 bytes) |
```

#### Command ID

The command ID is a 2-byte field that identifies the type of command being sent. The lower byte specifies the command
type, while the upper byte specifies the target motor ID. For example, the command ID 0x0123 might indicate a speed
command for motor ID 0x23.

#### Command Length

The command length field is a 1-byte value that specifies the number of bytes of data in the command frame. This can be
between 0 and 8 bytes.

#### Command Data

The command data field contains the actual data for the command. The contents of this field depend on the command type
and may be empty for some commands.

## License

This code is licensed under the MIT License. See the LICENSE file for details.
