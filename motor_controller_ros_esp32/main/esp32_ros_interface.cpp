#include "ros.h"
#include "jimmbot_msgs/canFrame.h"
#include "jimmbot_msgs/canFrameArray.h"

#include "esp32_ros_interface.h"

ros::NodeHandle nh;

jimmbot_msgs::canFrame frame;
jimmbot_msgs::canFrameArray feedback_msg;

ros::Publisher canFrameArrayPublisher("/esp32/feedback/can_msg_array", &feedback_msg);

void canFrameArrayCallback(const jimmbot_msgs::canFrameArray& data_msg)
{
  can_message_t tx_msg;

  for(int i = 0; i < 4; i++)
  {
    tx_msg.data_length_code = 8;
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

//   gpio_set_level((gpio_num_t)GPIO_OUTPUT_IO_0, data_msg.lights[0]);
//   gpio_set_level((gpio_num_t)GPIO_OUTPUT_IO_1, data_msg.lights[1]);
}

ros::Subscriber<jimmbot_msgs::canFrameArray> canFrameArraySubscriber("/esp32/command/can_msg_array", &canFrameArrayCallback);

esp_err_t rosserial_setup()
{
  // Initialize ROS
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
