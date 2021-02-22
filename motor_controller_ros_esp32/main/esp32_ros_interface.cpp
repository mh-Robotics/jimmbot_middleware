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

static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static const can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)GPIO_CAN_TRANSMIT, (gpio_num_t)GPIO_CAN_RECEIVE, CAN_MODE_NORMAL);

jimmbot_msgs::canFrameArray feedback_msg;

ros::NodeHandle nh;
void canFrameArrayFeedback(void);
ros::Publisher canFrameArrayPublisher(PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY, &feedback_msg);
void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg);
ros::Subscriber<jimmbot_msgs::canFrameArray> canFrameArraySubscriber(SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY, &canFrameArrayCallback);

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg)
{
  can_message_t tx_msg;

  // //Range based for loops does not work with ESP-IDF
  // for(const auto& can_msg : data_msg)
  // {
  //   data_msg.can_frames.size(); //This does not return the size, even 'thou we know it is 5
  // }

  for(int i = 0; i < CAN_MSG_COUNT; i++)
  {
    if(data_msg.can_frames[i].id == LIGHT_MSG_ID)
    {
      gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, data_msg.can_frames[i].data[3]);
      gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, data_msg.can_frames[i].data[7]);

      continue;
    }

    tx_msg.extd = data_msg.can_frames[i].is_extended;
    tx_msg.rtr = data_msg.can_frames[i].is_rtr;
    tx_msg.identifier = data_msg.can_frames[i].id;
    tx_msg.data_length_code = data_msg.can_frames[i].dlc;
    tx_msg.ss = 1;
    memcpy(tx_msg.data, data_msg.can_frames[i].data, data_msg.can_frames[i].dlc);

    esp_err_t err = can_transmit(&tx_msg, portMAX_DELAY);
    if(err != ESP_OK)
    {
      nh.logerror(__PRETTY_FUNCTION__);
      nh.logerror("can_transmit: ");
      nh.logerror(esp_err_to_name(err));
      return;
    }
  }

  // canFrameArrayFeedback();
}

void canFrameArrayFeedback(void)
{
  can_message_t rx_msg;

  feedback_msg.header.stamp = nh.now();
  for(int i = 0; i < 2; i++)
  {
    esp_err_t err = can_receive(&rx_msg, portMAX_DELAY);
    if(err != ESP_OK)
    {
      nh.logerror(__PRETTY_FUNCTION__);
      nh.logerror("can_receive: ");
      nh.logerror(esp_err_to_name(err));
      return;
    }

    feedback_msg.can_frames[i].is_extended = rx_msg.extd;
    feedback_msg.can_frames[i].is_rtr = rx_msg.rtr;
    feedback_msg.can_frames[i].id = rx_msg.identifier;
    feedback_msg.can_frames[i].dlc = rx_msg.data_length_code;
    memcpy(feedback_msg.can_frames[i].data, rx_msg.data, rx_msg.data_length_code);

    // feedback_msg.can_frames[0].is_extended = rx_msg.extd;
    // feedback_msg.can_frames[0].is_rtr = rx_msg.rtr;
    // feedback_msg.can_frames[0].id = rx_msg.identifier;
    // feedback_msg.can_frames[0].dlc = rx_msg.data_length_code;
    // memcpy(feedback_msg.can_frames[0].data, rx_msg.data, rx_msg.data_length_code);
  }

  canFrameArrayPublisher.publish(&feedback_msg);
}

esp_err_t can_init(void)
{
  esp_err_t err;
  
  err = can_driver_install(&g_config, &t_config, &f_config);
  nh.loginfo(__PRETTY_FUNCTION__);
  nh.loginfo("can_driver_install: ");
  nh.loginfo(esp_err_to_name(err));

  err = can_start();
  nh.loginfo(__PRETTY_FUNCTION__);
  nh.loginfo("can_start: ");
  nh.loginfo(esp_err_to_name(err));

  return err;
}

esp_err_t can_destroy(void)
{
  esp_err_t err;
  
  err = can_stop();
  nh.loginfo(__PRETTY_FUNCTION__);
  nh.loginfo("can_stop: ");
  nh.loginfo(esp_err_to_name(err));

  err = can_driver_uninstall();
  nh.loginfo(__PRETTY_FUNCTION__);
  nh.loginfo("can_driver_uninstall: ");
  nh.loginfo(esp_err_to_name(err));

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
  nh.loginfo(__PRETTY_FUNCTION__);
  nh.loginfo("gpio_config: ");
  nh.loginfo(esp_err_to_name(err));

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
