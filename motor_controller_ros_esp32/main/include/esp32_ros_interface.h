#ifndef ___ESP32_ROS_INTERFACE_H___
#define ___ESP32_ROS_INTERFACE_H___

#include "stdio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/can.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif //end __cplusplus

esp_err_t rosserial_setup();

esp_err_t rosserial_spinonce();

#ifdef __cplusplus
}
#endif //end __cplusplus

#endif //end ___ESP32_ROS_INTERFACE_H___
