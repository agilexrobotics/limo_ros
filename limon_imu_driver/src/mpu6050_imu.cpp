

#include "limon_imu_driver/mpu6050_imu.h"
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tf/tf.h>

using namespace agx;
#define MAX_BUFF_SIZE 10240

SerialRead::SerialRead() {
  bytestream_ = new char[MAX_BUFF_SIZE];
  memset(bytestream_, 0, MAX_BUFF_SIZE);
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

  // max read data number per loop
  if (len > MAX_BUFF_SIZE) {
    len = MAX_BUFF_SIZE;
  } else if (len == 0) {
    return;
  }

  // clear the older data
  size_t curr_len = binarystream_.size();
  if (curr_len >= MAX_BUFF_SIZE - len) {
    binarystream_.clear();
    curr_len = 0;
  }

  // save the raw data to a vector
  for (int i = 0; i < len; i++) {
    binarystream_.push_back(*(&data[0] + i));
  }
  curr_len = binarystream_.size();

  // parse the data from buff vector
  int use_len = ParseRawData(binarystream_);

  // if raw data have bean used with `use_len`, remove use_len datas at the
  // vector front
  if (use_len > 0) {
    binarystream_.erase(binarystream_.begin(), binarystream_.begin() + use_len);
  }
}

int SerialRead::ParseRawData(const std::vector<uint8_t>& data) {
  int use_len = 0;
  if (data.size() < MPU6050_FRAME_LEN) {
    return 0;
  }
  if (data[0] == TTL_FRAME_START) {
    ParseSingleFrame(data, static_cast<TTLFrameType>(data[1]));
    return MPU6050_FRAME_LEN;
  }
  return 1;
}

void SerialRead::ParseSingleFrame(const std::vector<uint8_t>& data,
                                  TTLFrameType type) {
  switch (type) {
    case TTLFrameType::TIME:
      break;
    case TTLFrameType::ACCELERATION:
      ParseAcceleration(data);
      break;
    case TTLFrameType::ANGULAR_VELOCITY:
      ParseAngularVelocity(data);
      break;
    case TTLFrameType::ANGLE:
      ParseAngle(data);
      break;
    default:
      printf("ttl frame type not supported: %d\n", type);
      break;
  }
}
void SerialRead::ParseAcceleration(const std::vector<uint8_t>& data) {
  uint8_t sum = 0;
  for (size_t i = 0; i < MPU6050_FRAME_LEN - 1; i++) {
    // printf("%02x\t", data[i]);
    sum += data[i];
  }

  if (sum != data[10]) {
    return;
  }

  // clang-format off
  Ax_ = ((int16_t)((int16_t)data[3] << 8 | data[2])) / 32768.0 * 16.0 * 9.8;  // m/s^2
  Ay_ = ((int16_t)((int16_t)data[5] << 8 | data[4])) / 32768.0 * 16.0 * 9.8;  // m/s^2
  Az_ = ((int16_t)((int16_t)data[7] << 8 | data[6])) / 32768.0 * 16.0 * 9.8;  // m/s^2
  temperature_ = ((int16_t)((int16_t)data[9] << 8 | data[8])) / 100.0;  // degree
  // clang-format on

  //   printf("sum: %02x, %02x\n", sum, data[10]);
  //   printf("Ax: %f, Ay: %f, Az: %f, Temp: %f\n", Ax_, Ay_, Az_,
  //   temperature_);
}
void SerialRead::ParseAngularVelocity(const std::vector<uint8_t>& data) {
  uint8_t sum = 0;
  for (size_t i = 0; i < MPU6050_FRAME_LEN - 1; i++) {
    sum += data[i];
  }

  if (sum != data[10]) {
    return;
  }

  // clang-format off
  Wx_ = ((int16_t)((int16_t)data[3] << 8 | data[2])) / 32768.0 * 2000.0;  // degree/s
  Wy_ = ((int16_t)((int16_t)data[5] << 8 | data[4])) / 32768.0 * 2000.0;  // degree/s
  Wz_ = ((int16_t)((int16_t)data[7] << 8 | data[6])) / 32768.0 * 2000.0;  // degree/s
  float temperature = ((int16_t)((int16_t)data[9] << 8 | data[8])) / 100.0;  // degree
  // clang-format on

  Wx_ *= DEG_TO_RAD;  // rad/s
  Wy_ *= DEG_TO_RAD;  // rad/s
  Wz_ *= DEG_TO_RAD;  // rad/s

  printf("Wx: %f, Wy: %f, Wz: %f, Temp: %f\n", Wx_, Wy_, Wz_, temperature);
}
void SerialRead::ParseAngle(const std::vector<uint8_t>& data) {
  uint8_t sum = 0;
  for (size_t i = 0; i < MPU6050_FRAME_LEN - 1; i++) {
    sum += data[i];
  }

  if (sum != data[10]) {
    return;
  }

  // clang-format off
  roll_ =  ((int16_t)((int16_t)data[3] << 8 | data[2])) / 32768.0 * 180.0;   // degree
  pitch_ = ((int16_t)((int16_t)data[5] << 8 | data[4])) / 32768.0 * 180.0;  // degree
  yaw_ =   ((int16_t)((int16_t)data[7] << 8 | data[6])) / 32768.0 * 180.0;    // degree
  float temperature = ((int16_t)((int16_t)data[9] << 8 | data[8])) / 100.0;  // degree
  // clang-format on

  //   printf("sum: %02x, %02x\n", sum, data[10]);
  //   printf("roll: %f, pitch: %f, yaw: %f, Temp: %f\n", roll_, pitch_, yaw_,
  //  temperature);

  roll_ = roll_ * DEG_TO_RAD;    // rad
  pitch_ = pitch_ * DEG_TO_RAD;  // rad
  yaw_ = yaw_ * DEG_TO_RAD;      // rad
}

sensor_msgs::Imu SerialRead::GetImuData() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.frame_id = "imu_link";

  imu_data_.linear_acceleration.x = Ax_;
  imu_data_.linear_acceleration.y = Ay_;
  imu_data_.linear_acceleration.z = Az_;

  imu_data_.angular_velocity.x = Wx_;
  imu_data_.angular_velocity.y = Wy_;
  imu_data_.angular_velocity.z = Wz_;

  tf::Quaternion q;
  q.setRPY(roll_, pitch_, yaw_);
  imu_data_.orientation.x = q.x();
  imu_data_.orientation.y = q.y();
  imu_data_.orientation.z = q.z();
  imu_data_.orientation.w = q.w();

  imu_data_.linear_acceleration_covariance[0] = 1.0f;
  imu_data_.linear_acceleration_covariance[4] = 1.0f;
  imu_data_.linear_acceleration_covariance[8] = 1.0f;

  imu_data_.angular_velocity_covariance[0] = 1e-6;
  imu_data_.angular_velocity_covariance[4] = 1e-6;
  imu_data_.angular_velocity_covariance[8] = 1e-6;

  imu_data_.orientation_covariance[0] = 1e-6;
  imu_data_.orientation_covariance[4] = 1e-6;
  imu_data_.orientation_covariance[8] = 1e-6;

  return imu_data_;
}

// main
#include <ros/ros.h>

// using namespace std::placeholders;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mpu6050_imu");
  ROS_INFO("mpu6050 imu read driver start...");

  ros::NodeHandle nh;
  SerialRead sr;
  sr.Connect(
      "/dev/ttyUSB1", 115200,
      std::bind(&SerialRead::ReadRawSerialData, &sr, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10, true);

  ros::Rate r(50);
  while (ros::ok()) {
    auto imu_msg = sr.GetImuData();
    imu_pub.publish(imu_msg);
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("mpu6050 main thread exist!");
  return 0;
}