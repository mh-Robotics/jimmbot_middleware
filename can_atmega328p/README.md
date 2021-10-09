# MotorController Firmware
## TODO Add readme info

### Release build disables UART
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Debug build enables UART and prints data to serial
#### Todo: add serial information, baud rate 
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```