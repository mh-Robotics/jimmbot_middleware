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

ros::NodeHandle nh;

jimmbot_msgs::canFrame frame;
jimmbot_msgs::canFrameArray feedback_msg;

ros::Publisher canFrameArrayPublisher(PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY, &feedback_msg);

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg)
{
  can_message_t tx_msg;

  for(int i = 0; i < WHEEL_MSG_COUNT + LIGHT_MSG_COUNT; i++)
  {

    if(i == (WHEEL_MSG_COUNT + LIGHT_MSG_COUNT - 1))
    {
      gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, data_msg.can_frames[(WHEEL_MSG_COUNT + LIGHT_MSG_COUNT - 1)].data[3]);
      gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, data_msg.can_frames[(WHEEL_MSG_COUNT + LIGHT_MSG_COUNT - 1)].data[7]);

      break;
    }

    tx_msg.data_length_code = CAN_MSG_LENGTH;
    tx_msg.identifier = data_msg.can_frames[i].id;

    tx_msg.data[0] = data_msg.can_frames[i].data[0];
    tx_msg.data[1] = data_msg.can_frames[i].data[1];
    tx_msg.data[2] = data_msg.can_frames[i].data[2];
    tx_msg.data[3] = data_msg.can_frames[i].data[3];
    tx_msg.data[4] = data_msg.can_frames[i].data[4];
    tx_msg.data[5] = data_msg.can_frames[i].data[5];
    tx_msg.data[6] = data_msg.can_frames[i].data[6];
    tx_msg.data[7] = data_msg.can_frames[i].data[7];

    tx_msg.flags = CAN_MSG_FLAG_SS;

    //send can message
  }
}

ros::Subscriber<jimmbot_msgs::canFrameArray> canFrameArraySubscriber(SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY, &canFrameArrayCallback);

esp_err_t output_gpio_init(void)
{
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  io_conf.pull_up_en = (gpio_pullup_t)0;
  gpio_config(&io_conf);

  return ESP_OK;
}

esp_err_t rosserial_setup()
{
  nh.initNode();
  nh.subscribe(canFrameArraySubscriber);
  nh.advertise(canFrameArrayPublisher);

  return ESP_OK;
}

esp_err_t rosserial_spinonce()
{
  // Publish feedback
  
  nh.spinOnce();

  return ESP_OK;
}
