/**
 * @file esp32_ros_interface.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the implementation for initializations, callbacks and types used in this program
 * @version 0.1
 * @date 2020-06-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ros.h"
#include "jimmbot_msgs/canFrame.h"
#include "jimmbot_msgs/canFrameArray.h"

#include "esp32_ros_interface.h"

#include "driver/can.h"
#include "driver/gpio.h"

constexpr int CAN_RX_TX_DELAY_MS = 25;

static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

static const can_filter_config_t f_config =
{
  .acceptance_code = 0x00000004,
  .acceptance_mask = 0xFFFFFFF8,
  .single_filter = true
};

static const can_general_config_t g_config = 
{
 .mode = CAN_MODE_NORMAL,
 .tx_io = (gpio_num_t)GPIO_CAN_TRANSMIT,
 .rx_io = (gpio_num_t)GPIO_CAN_RECEIVE,
 .clkout_io = (gpio_num_t)(-1),
 .bus_off_io = (gpio_num_t)(-1),
 .tx_queue_len = 100,
 .rx_queue_len = 100,
 .alerts_enabled = false,
 .clkout_divider = 0
};

jimmbot_msgs::canFrameArray feedback_msg;

constexpr int ENABLE_LIGHT_LEFT_MSG_INDEX = 3;
constexpr int ENABLE_LIGHT_RIGHT_MSG_INDEX = 7;
constexpr int LIGHT_CAN_MSG_INDEX = 4;

ros::NodeHandle nh;
void canFrameArrayFeedback(int index);
ros::Publisher canFrameArrayPublisher(PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY, &feedback_msg);
void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg);
ros::Subscriber<jimmbot_msgs::canFrameArray> canFrameArraySubscriber(SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY, &canFrameArrayCallback);

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg)
{
  gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, data_msg.can_frames[LIGHT_CAN_MSG_INDEX].data[ENABLE_LIGHT_LEFT_MSG_INDEX]);
  gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, data_msg.can_frames[LIGHT_CAN_MSG_INDEX].data[ENABLE_LIGHT_RIGHT_MSG_INDEX]);

  can_message_t tx_msg;
  for(int index = 0; index < CAN_MSG_COUNT; index++)
  {
    tx_msg.extd = data_msg.can_frames[index].is_extended;
    tx_msg.rtr = data_msg.can_frames[index].is_rtr;
    tx_msg.identifier = data_msg.can_frames[index].id;
    tx_msg.data_length_code = data_msg.can_frames[index].dlc;
    tx_msg.ss = 1;
    memcpy(tx_msg.data, data_msg.can_frames[index].data, data_msg.can_frames[index].dlc);

    esp_err_t err = can_transmit(&tx_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
    if(err != ESP_OK)
    { 
      return;
    }

    canFrameArrayFeedback(index);
  }
}

void canFrameArrayFeedback(int index)
{
  can_message_t rx_msg;
  feedback_msg.header.stamp = nh.now();

  esp_err_t err = can_receive(&rx_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
  if(err != ESP_OK)
  {
    {
      const uint8_t dummy_values[8] = {255, 255, 255, 255, 255, 255, 255, 255};
      feedback_msg.can_frames[index].id = 0xFF;
      memcpy(feedback_msg.can_frames[index].data, dummy_values, rx_msg.data_length_code);
    }

    return;
  }

  feedback_msg.can_frames[index].is_extended = rx_msg.extd;
  feedback_msg.can_frames[index].is_rtr = rx_msg.rtr;
  feedback_msg.can_frames[index].id = rx_msg.identifier;
  feedback_msg.can_frames[index].dlc = rx_msg.data_length_code;
  memcpy(feedback_msg.can_frames[index].data, rx_msg.data, rx_msg.data_length_code);

  canFrameArrayPublisher.publish(&feedback_msg);
}

esp_err_t can_init(void)
{
  esp_err_t err;
  
  err = can_driver_install(&g_config, &t_config, &f_config);
  err = can_start();

  return err;
}

esp_err_t can_destroy(void)
{
  esp_err_t err;
  
  err = can_stop();
  err = can_driver_uninstall();

  return err;
}

esp_err_t output_gpio_init(void)
{
  esp_err_t err;
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  io_conf.pull_up_en = (gpio_pullup_t)0;
  
  err = gpio_config(&io_conf);

  return err;
}

esp_err_t rosserial_setup(void)
{
  nh.initNode();
  nh.subscribe(canFrameArraySubscriber);
  nh.advertise(canFrameArrayPublisher);

  return ESP_OK;
}

esp_err_t rosserial_spinonce(void)
{
  nh.spinOnce();

  return ESP_OK;
}
