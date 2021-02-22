/**
 * @file main.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the main function call
 * @version 0.1
 * @date 2020-05-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "esp32_ros_interface.h"

#ifdef __cplusplus
extern "C" {
#endif //end __cplusplus

	int app_main(void);

#ifdef __cplusplus
}
#endif //end __cplusplus

int app_main(void)
{
  ESP_ERROR_CHECK(can_init());
  ESP_ERROR_CHECK(output_gpio_init());

  {
    rosserial_setup();
    while(true) 
    {
      rosserial_spinonce();
    }
  }

  ESP_ERROR_CHECK(can_destroy());

  return EXIT_SUCCESS;
}
