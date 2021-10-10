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

todo - add these options too, and cleanup CMakeLists
-DCMAKE_BUILD_DOC=On
-DCMAKE_BUILD_TESTS=On