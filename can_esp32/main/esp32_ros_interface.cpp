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

// #define __DEBUG_EMPTY_PUB_SUB__

constexpr int STACK_SIZE = 4096;
constexpr int RX_TASK_PRIO = 8;
constexpr int TX_TASK_PRIO = 9;
constexpr int CAN_RX_TX_DELAY_MS = 100;

static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

static const can_filter_config_t f_config =
{
  .acceptance_code = 0x00000004,
  .acceptance_mask = 0xFFFFFFF7,
  .single_filter = true
};

static const can_general_config_t g_config = 
{
 .mode = CAN_MODE_NORMAL,
 .tx_io = (gpio_num_t)GPIO_CAN1_TRANSMIT,
 .rx_io = (gpio_num_t)GPIO_CAN1_RECEIVE,
 .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
 .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
 .tx_queue_len = 10,
 .rx_queue_len = 10,
 .alerts_enabled = CAN_ALERT_NONE,
 .clkout_divider = 0
};

jimmbot_msgs::canFrameArray feedback_msg;

constexpr int ENABLE_LIGHT_LEFT_MSG_INDEX = 3;
constexpr int ENABLE_LIGHT_RIGHT_MSG_INDEX = 7;
constexpr int LIGHT_CAN_MSG_INDEX = 4;

ros::NodeHandle nh;
void canFrameArrayFeedback(void);
ros::Publisher canFrameArrayPublisher(PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY, &feedback_msg);
void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg);
ros::Subscriber<jimmbot_msgs::canFrameArray> canFrameArraySubscriber(SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY, &canFrameArrayCallback);
static QueueHandle_t tx_task_queue;
TaskHandle_t rxHandle = NULL;
TaskHandle_t txHandle = NULL;

static void can_transmit_task(void *arg)
{
  while(true) 
  {
    jimmbot_msgs::canFrameArray data_msg;

    if(xQueueReceive(tx_task_queue, &data_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS)) == pdPASS)
    {
      can_message_t tx_msg;
      for(int index = 0; index < CAN_MSG_COUNT; index++)
      {
        tx_msg.extd = data_msg.can_frames[index].is_extended;
        tx_msg.rtr = data_msg.can_frames[index].is_rtr;
        tx_msg.identifier = data_msg.can_frames[index].id;
        tx_msg.data_length_code = data_msg.can_frames[index].dlc;
        tx_msg.ss = 1;
        memcpy(tx_msg.data, data_msg.can_frames[index].data, data_msg.can_frames[index].dlc);
        can_transmit(&tx_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
      }
    }
  }
}

static void can_receive_task(void *arg)
{
  while (true) 
  {
    // canFrameArrayFeedback();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg)
{
  #ifndef __DEBUG_EMPTY_PUB_SUB__
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, data_msg.can_frames[LIGHT_CAN_MSG_INDEX].data[ENABLE_LIGHT_LEFT_MSG_INDEX]);
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, data_msg.can_frames[LIGHT_CAN_MSG_INDEX].data[ENABLE_LIGHT_RIGHT_MSG_INDEX]);

    xQueueSend(tx_task_queue, &data_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
  #else
    canFrameArrayPublisher.publish(&data_msg);
  #endif
}

void canFrameArrayFeedback(void)
{
  can_message_t rx_msg;
  feedback_msg.header.frame_id = "/jimmbot/hw/status";
  feedback_msg.header.stamp = nh.now();

  can_receive(&rx_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
  int index = rx_msg.identifier - CAN_MSG_COUNT;

  feedback_msg.can_frames[index].is_extended = rx_msg.extd;
  feedback_msg.can_frames[index].is_rtr = rx_msg.rtr;
  feedback_msg.can_frames[index].id = rx_msg.identifier;
  feedback_msg.can_frames[index].dlc = rx_msg.data_length_code;
  memcpy(feedback_msg.can_frames[index].data, rx_msg.data, rx_msg.data_length_code);

  //CAN with index 5 reserved for other status from HW. @todo

  canFrameArrayPublisher.publish(&feedback_msg);
}

esp_err_t can_init(void)
{
  esp_err_t err;

  tx_task_queue = xQueueCreate(1, sizeof(jimmbot_msgs::canFrameArray));

  err = can_driver_install(&g_config, &t_config, &f_config);
  err = can_start();

  return err;
}

esp_err_t can_destroy(void)
{
  esp_err_t err;

  if(txHandle != NULL)
  {
    vTaskDelete(txHandle);
  }
  vQueueDelete(tx_task_queue);

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

esp_err_t output_gpio_destroy(void)
{
  esp_err_t err;
  err = gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, false);
  err = gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, false);

  return err;
}

esp_err_t rosserial_setup(void)
{
  nh.initNode();
  nh.subscribe(canFrameArraySubscriber);
  nh.advertise(canFrameArrayPublisher);

  xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", STACK_SIZE, NULL, RX_TASK_PRIO, &rxHandle, tskNO_AFFINITY);
  configASSERT(rxHandle);
  xTaskCreatePinnedToCore(can_transmit_task, "CAN_tx", STACK_SIZE, NULL, TX_TASK_PRIO, &txHandle, tskNO_AFFINITY);
  configASSERT(txHandle);

  return ESP_OK;
}

esp_err_t rosserial_spinonce(void)
{
  nh.spinOnce();

  return ESP_OK;
}
