

#include "limon_imu_driver/mpu6050_imu.h"
#include <stdio.h>

using namespace agx;
#define MAX_BUFF_SIZE 10240

SerialRead::SerialRead() {
  bytestream_ = new char[MAX_BUFF_SIZE];
  memset(bytestream_, 0, MAX_BUFF_SIZE);

  binarystream_.resize(MAX_BUFF_SIZE);
}

SerialRead::~SerialRead() {
  if (bytestream_) {
    delete[] bytestream_;
  }
  Disconnect();
}

void SerialRead::Disconnect() {
  if (serial_connected_) {
    try {
      serial_->StopService();
    } catch (std::exception& e) {
      printf("serial stop error: %s\n", e.what());
    }
  }
}

void SerialRead::Connect(std::string dev_name, uint32_t bouad_rate,
                         SerialFrameCallback cb) {
  assert(not dev_name.empty());
  assert(bouad_rate > 0);

  serial_ = std::make_shared<AsyncSerial>(dev_name, bouad_rate);
  serial_->SetReceiveCallback(cb);
  serial_->StartListening();
  serial_connected_ = true;
}

void SerialRead::ReadRawSerialData(uint8_t* data, const size_t bufsize,
                                   size_t len) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (len == 0) {
    return;
  } else {
    // copy the data from callback stack
    memcpy(&bytestream_[0], (char*)data, len);
  }

  printf("get the data: %d\n", len);
}

// main
#include <ros/ros.h>

// using namespace std::placeholders;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mpu6050_imu");
  ROS_INFO("mpu6050 imu read driver start...");

  SerialRead sr;
  sr.Connect(
      "/dev/ttyUSB1", 115200,
      std::bind(&SerialRead::ReadRawSerialData, &sr, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  while (ros::ok()) {
    ros::spin();
  }

  ROS_INFO("mpu6050 main thread exist!");
  return 0;
}