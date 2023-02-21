/**
 * @file esp32_ros_interface.cpp
 * @author Mergim Halimi (m.halimi123@gmail.com)
 * @brief This file holds the implementation for initializations, callbacks and
 * types used in this program
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021, mhRobotics, Inc., All rights reserved.
 * @license This project is released under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include <esp32_ros_interface.h>
#include <ros.h>  // for ros::*

#include "driver/gpio.h"
#include "driver/twai.h"
#include "jimmbot_msgs/CanFrame.h"         // for jimmbot_msgs::CanFrame
#include "jimmbot_msgs/CanFrameStamped.h"  // for jimmbot_msgs::CanFrameStamped

constexpr auto kStackSize = 4096;
constexpr auto kRxTaskPrio = 8;
constexpr auto kTxTaskPrio = 9;
constexpr auto kCanRxTxDelayMs = 100;
constexpr auto kCanRxTxQueueLen = 10;

constexpr auto kLightCanMsgId{0x31};

constexpr auto kGpioCanTransmit{GPIO_NUM_21};
constexpr auto kGpioCanReceive{GPIO_NUM_22};
constexpr auto kGpioOutputLightLeft{GPIO_NUM_32};
constexpr auto kGpioOutputLightRight{GPIO_NUM_33};
constexpr auto kGpioOutputPinSel{(1ULL << kGpioOutputLightLeft) |
                                 (1ULL << kGpioOutputLightRight)};

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

static const twai_filter_config_t f_config = {.acceptance_code = 0x00000004,
                                              .acceptance_mask = 0xFFFFFFF7,
                                              .single_filter = true};

static const twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = kGpioCanTransmit,
    .rx_io = kGpioCanReceive,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = kCanRxTxQueueLen,
    .rx_queue_len = kCanRxTxQueueLen,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1};

jimmbot_msgs::CanFrameStamped feedback_msg;

constexpr int kEnableLightLeftMsgIndex = 3;
constexpr int kEnableLightRightMsgIndex = 7;

ros::NodeHandle nh;
void canFrameFeedbackCallback(void);
ros::Publisher canFramePublisher(kFeedbackTopicCanMsg, &feedback_msg);
void canFrameCallback(const jimmbot_msgs::CanFrameStamped &data_msg);
ros::Subscriber<jimmbot_msgs::CanFrameStamped> canFrameSubscriber(
    kCommandTopicCanMsg, &canFrameCallback);

static QueueHandle_t tx_task_queue;
TaskHandle_t rxHandle = NULL;
TaskHandle_t txHandle = NULL;

const twai_message_t toTwaiMessage(
    const jimmbot_msgs::CanFrameStamped &jimmBotCanMessage) {
  twai_message_t twai_msg;

  twai_msg.extd = jimmBotCanMessage.can_frame.is_extended;
  twai_msg.rtr = jimmBotCanMessage.can_frame.is_rtr;
  twai_msg.identifier = jimmBotCanMessage.can_frame.id;
  twai_msg.data_length_code = jimmBotCanMessage.can_frame.dlc;
  twai_msg.ss = 1;
  memcpy(twai_msg.data, jimmBotCanMessage.can_frame.data,
         jimmBotCanMessage.can_frame.dlc);

  return twai_msg;
}

const jimmbot_msgs::CanFrame toJimmBotCanMessage(
    const twai_message_t &twai_msg) {
  jimmbot_msgs::CanFrame jimmBotCanMessage;

  jimmBotCanMessage.is_extended = twai_msg.extd;
  jimmBotCanMessage.is_rtr = twai_msg.rtr;
  jimmBotCanMessage.id = twai_msg.identifier;
  jimmBotCanMessage.dlc = twai_msg.data_length_code;
  memcpy(jimmBotCanMessage.data, twai_msg.data, twai_msg.data_length_code);

  return jimmBotCanMessage;
}

static void can_transmit_task(void *arg) {
  while (true) {
    jimmbot_msgs::CanFrameStamped data_msg;
    if (xQueueReceive(tx_task_queue, &data_msg,
                      pdMS_TO_TICKS(kCanRxTxDelayMs)) == pdPASS) {
      twai_message_t twai_msg = toTwaiMessage(data_msg);
      twai_transmit(&twai_msg, pdMS_TO_TICKS(kCanRxTxDelayMs));
    }
  }
}

static void can_receive_task(void *arg) {
  while (true) {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(kCanRxTxDelayMs)) == ESP_OK) {
      feedback_msg.header.frame_id = kFeedbackFrameId;
      feedback_msg.header.stamp = nh.now();
      feedback_msg.can_frame = toJimmBotCanMessage(rx_msg);

      canFramePublisher.publish(&feedback_msg);
    }
  }
}

void canFrameCallback(const jimmbot_msgs::CanFrameStamped &data_msg) {
  if (data_msg.can_frame.id == kLightCanMsgId) {
    gpio_set_level(kGpioOutputLightLeft,
                   data_msg.can_frame.data[kEnableLightLeftMsgIndex]);
    gpio_set_level(kGpioOutputLightRight,
                   data_msg.can_frame.data[kEnableLightRightMsgIndex]);
    return;
  }

  xQueueSend(tx_task_queue, &data_msg, pdMS_TO_TICKS(kCanRxTxDelayMs));
}

esp_err_t can_init(void) {
  esp_err_t err;

  tx_task_queue = xQueueCreate(1, sizeof(jimmbot_msgs::CanFrameStamped));

  err = twai_driver_install(&g_config, &t_config, &f_config);
  err = twai_start();

  return err;
}

esp_err_t can_destroy(void) {
  esp_err_t err;

  if (txHandle != NULL) {
    vTaskDelete(txHandle);
  }
  if (rxHandle != NULL) {
    vTaskDelete(rxHandle);
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
  io_conf.pin_bit_mask = kGpioOutputPinSel;
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  io_conf.pull_up_en = (gpio_pullup_t)0;

  err = gpio_config(&io_conf);

  return err;
}

esp_err_t output_gpio_destroy(void) {
  esp_err_t err;
  err = gpio_set_level(kGpioOutputLightLeft, false);
  err = gpio_set_level(kGpioOutputLightRight, false);

  return err;
}

esp_err_t rosserial_setup(void) {
  nh.initNode();
  nh.subscribe(canFrameSubscriber);
  nh.advertise(canFramePublisher);

  xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", kStackSize, NULL,
                          kRxTaskPrio, &rxHandle, tskNO_AFFINITY);
  configASSERT(rxHandle);

  xTaskCreatePinnedToCore(can_transmit_task, "CAN_tx", kStackSize, NULL,
                          kTxTaskPrio, &txHandle, tskNO_AFFINITY);
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
