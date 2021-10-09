/**
 * @file esp32_ros_interface.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the implementation for initializations, callbacks and types used in this program
 * @version 0.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * 
 */
#include <esp32_ros_interface.h>

#include <ros.h>
#include "jimmbot_msgs/canFrame.h"
#include "jimmbot_msgs/canFrameArray.h"

#include "driver/twai.h"
#include "driver/gpio.h"

constexpr int STACK_SIZE = 4096;
constexpr int RX_TASK_PRIO = 8;
constexpr int TX_TASK_PRIO = 9;
constexpr int CAN_RX_TX_DELAY_MS = 10;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

static const twai_filter_config_t f_config = {
  .acceptance_code = 0x00000004,
  .acceptance_mask = 0xFFFFFFF7,
  .single_filter = true
};

static const twai_general_config_t g_config = {
  .mode = TWAI_MODE_NORMAL,
  .tx_io = (gpio_num_t)GPIO_CAN_TRANSMIT,
  .rx_io = (gpio_num_t)GPIO_CAN_RECEIVE,
  .clkout_io = (gpio_num_t)TWAI_IO_UNUSED,
  .bus_off_io = (gpio_num_t)TWAI_IO_UNUSED,
  .tx_queue_len = 100,
  .rx_queue_len = 100,
  .alerts_enabled = TWAI_ALERT_NONE,
  .clkout_divider = 0,
  .intr_flags = ESP_INTR_FLAG_LEVEL1
};

jimmbot_msgs::canFrameArray feedback_msg;

constexpr int ENABLE_LIGHT_LEFT_MSG_INDEX = 3;
constexpr int ENABLE_LIGHT_RIGHT_MSG_INDEX = 7;
constexpr int LIGHT_CAN_MSG_INDEX = 4;

ros::NodeHandle nh;
void canFrameArrayFeedback(void);
ros::Publisher canFrameArrayPublisher(PUBLISHER_FEEDBACK_TOPIC_CAN_MSG_ARRAY,
                                      &feedback_msg);
void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg);
ros::Subscriber<jimmbot_msgs::canFrameArray>
    canFrameArraySubscriber(SUBSCRIBER_COMMAND_TOPIC_CAN_MSG_ARRAY,
                            &canFrameArrayCallback);
static QueueHandle_t tx_task_queue;
TaskHandle_t rxHandle = NULL;
TaskHandle_t txHandle = NULL;

const twai_message_t
  toTwaiMessage(const jimmbot_msgs::canFrame& jimmBotCanMessage) {
  twai_message_t twai_msg;

  twai_msg.extd = jimmBotCanMessage.is_extended;
  twai_msg.rtr = jimmBotCanMessage.is_rtr;
  twai_msg.identifier = jimmBotCanMessage.id;
  twai_msg.data_length_code = jimmBotCanMessage.dlc;
  twai_msg.ss = 1;
  memcpy(twai_msg.data, jimmBotCanMessage.data, jimmBotCanMessage.dlc);

  return twai_msg;
}

const jimmbot_msgs::canFrame
  toJimmBotCanMessage(const twai_message_t& twai_msg) {
  jimmbot_msgs::canFrame jimmBotCanMessage;

  jimmBotCanMessage.is_extended = twai_msg.extd;
  jimmBotCanMessage.is_rtr = twai_msg.rtr;
  jimmBotCanMessage.id = twai_msg.identifier;
  jimmBotCanMessage.dlc = twai_msg.data_length_code;
  memcpy(jimmBotCanMessage.data, twai_msg.data, twai_msg.data_length_code);

  return jimmBotCanMessage;
}

static void can_transmit_task(void *arg) {
  while (true) {
    jimmbot_msgs::canFrameArray data_msg;

    if (xQueueReceive(tx_task_queue,
                      &data_msg,
                      pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS)) == pdPASS) {
      for (auto& frame : data_msg.can_frames) {
        if (frame.id == LIGHT_MSG_ID) { continue; }
        twai_message_t twai_msg = toTwaiMessage(frame);
        twai_transmit(&twai_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
      }
    }
  }
}

static void can_receive_task(void *arg) {
  while (true) {
    // canFrameArrayFeedback();
    //Todo Add a logic to fill a message with 4 wheel information
    //And publish for 4 wheels.
    vTaskDelay(pdMS_TO_TICKS(100)); // Remove when reactivated feedback array
  }
}

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg) {
  gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT,
    data_msg.can_frames[LIGHT_CAN_MSG_INDEX].data[ENABLE_LIGHT_LEFT_MSG_INDEX]);
  gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT,
                data_msg.can_frames[LIGHT_CAN_MSG_INDEX]
                                    .data[ENABLE_LIGHT_RIGHT_MSG_INDEX]);

  xQueueSend(tx_task_queue, &data_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
}

void canFrameArrayFeedback(void) {
  twai_message_t rx_msg;
  twai_receive(&rx_msg, pdMS_TO_TICKS(CAN_RX_TX_DELAY_MS));
  int index = rx_msg.identifier - CAN_MSG_COUNT;

  feedback_msg.header.frame_id = "/jimmbot/hw/status";
  feedback_msg.header.stamp = nh.now();
  feedback_msg.can_frames[index] = toJimmBotCanMessage(rx_msg);

  // CAN with index 5 reserved for other status from HW. @todo
  //feedback_msg.can_frames[LIGHT_CAN_MSG_INDEX] = Add voltage and/or other information
  canFrameArrayPublisher.publish(&feedback_msg);
}

esp_err_t can_init(void) {
  esp_err_t err;

  tx_task_queue = xQueueCreate(1, sizeof(jimmbot_msgs::canFrameArray));

  err = twai_driver_install(&g_config, &t_config, &f_config);
  err = twai_start();

  return err;
}

esp_err_t can_destroy(void) {
  esp_err_t err;

  if (txHandle != NULL) {
    vTaskDelete(txHandle);
  }
  vQueueDelete(tx_task_queue);

  err = twai_stop();
  err = twai_driver_uninstall();

  return err;
}

esp_err_t output_gpio_init(void) {
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

esp_err_t output_gpio_destroy(void) {
  esp_err_t err;
  err = gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_LEFT, false);
  err = gpio_set_level((gpio_num_t)GPIO_OUTPUT_LIGHT_RIGHT, false);

  return err;
}

esp_err_t rosserial_setup(void) {
  nh.initNode();
  nh.subscribe(canFrameArraySubscriber);
  nh.advertise(canFrameArrayPublisher);

  xTaskCreatePinnedToCore(can_receive_task,
                          "CAN_rx",
                          STACK_SIZE,
                          NULL,
                          RX_TASK_PRIO,
                          &rxHandle,
                          tskNO_AFFINITY);
  configASSERT(rxHandle);

  xTaskCreatePinnedToCore(can_transmit_task,
                          "CAN_tx",
                          STACK_SIZE,
                          NULL,
                          TX_TASK_PRIO,
                          &txHandle,
                          tskNO_AFFINITY);
  configASSERT(txHandle);

  return ESP_OK;
}

esp_err_t rosserial_spinonce(void) {
  if (nh.spinOnce() == 0) {
    return ESP_OK;
  }

  return ESP_FAIL;
}

esp_err_t rosserial_spin(void) {
  while (true) {
    rosserial_spinonce();

    // @todo Find a way to break this when not connected with ROS PC
    // if (!nh.connected()) { break; }
  }

  return ESP_FAIL;
}
