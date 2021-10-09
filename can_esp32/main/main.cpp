/**
 * @file main.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the main function call
 * @version 0.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * 
 */
#include <esp32_ros_interface.h>

#ifdef __cplusplus
extern "C" {
#endif  //  end __cplusplus

  int app_main(void);

#ifdef __cplusplus
}
#endif  //  end __cplusplus

int app_main(void) {
  can_init();
  output_gpio_init();

  {
    rosserial_setup();
    rosserial_spin();
  }

  output_gpio_destroy();
  can_destroy();

  return EXIT_SUCCESS;
}
