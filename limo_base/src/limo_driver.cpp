#include "limo_driver.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "limo_message.h"
#include "limo_msg_parser.h"

#define DEG_TO_RAD (0.01745329)

LimoDriver::LimoDriver() { send_buf_ = new uint8_t[1024]; }
LimoDriver::~LimoDriver() {
  if (send_buf_) delete[] send_buf_;
}

void LimoDriver::readData() {
  uint8_t rx_data = 0;
  while (ros::ok()) {
    auto len = port_->readByte(&rx_data);
    if (len < 1) continue;
    decodeMessage(rx_data);
  }
}

void LimoDriver::decodeMessage(uint8_t data) {
  static LIMO_t_RAW_t limoRaw;
  static int data_num = 0;
  static uint8_t checksum = 0;

  switch (limoRaw.state) {
    case LIMO_WAIT_SYNC1: {
      if (data == LIMO_SYNC1) {
        limoRaw.state = LIMO_WAIT_SYNC2;
      }
    } break;
    case LIMO_WAIT_SYNC2: {
      if (data == LIMO_SYNC2) {
        limoRaw.state = LIMO_WAIT_ID_HIGH;
      } else {
        limoRaw.state = LIMO_WAIT_SYNC1;
      }
    } break;
    case LIMO_WAIT_ID_HIGH: {
      limoRaw.can_id = ((uint16_t)data) << 8;
      limoRaw.state = LIMO_WAIT_ID_LOW;
      limoRaw.hight = data;
    } break;
    case LIMO_WAIT_ID_LOW: {
      limoRaw.can_id |= (uint16_t)data;
      limoRaw.state = LIMO_WAIT_DATA;
      data_num = 0;
      limoRaw.low = data;
    } break;
    case LIMO_WAIT_DATA: {
      if (data_num < 8) {
        limoRaw.data[data_num] = data;
        data_num++;
        checksum += data;
      } else {
        limoRaw.count = data;
        limoRaw.state = LIMO_CHECK;
        data_num = 0;
      }
    } break;
    case LIMO_CHECK: {
      limoRaw.RxCK = data;
      limoRaw.state = LIMO_WAIT_SYNC1;
      parseFrame(limoRaw);
      checksum = 0;
      memset(&limoRaw.data[0], 0, 8);
    } break;
    default:
      break;
  }
}

void LimoDriver::parseFrame(LIMO_t_RAW_t &frame) {
    AgxMessage status_msg;
    can_frame rx_frame;
    rx_frame.can_id = frame.can_id;
    memcpy(&rx_frame.data[0], &frame.data[0], 8);
    DecodeCanFrame(&rx_frame, &status_msg);
    UpdateLimoState(status_msg, limo_state_);
}

void LimoDriver::UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state) {
  std::lock_guard<std::mutex> lg(state_mutex_);

  switch (status_msg.type) {
    case AgxMsgSystemState: {
      state.system_state = status_msg.body.system_state_msg;
      break;
    }
    case AgxMsgMotionState: {
      state.motion_state = status_msg.body.motion_state_msg;
      // std::cout << "motion state in system state: " <<
      // state.system_state.motion_mode << std::endl;
      break;
    }
    case AgxMsgLightState: {
      state.light_state = status_msg.body.light_state_msg;
      break;
    }
    case AgxMsgRcState: {
      state.rc_state = status_msg.body.rc_state_msg;
      break;
    }
    case AgxMsgActuatorHSState: {
      state.actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
          status_msg.body.actuator_hs_state_msg;
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      state.actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
          status_msg.body.actuator_ls_state_msg;
      break;
    }
    case AgxMsgMotionModeState: {
      state.current_motion_mode = status_msg.body.motion_mode_feedback_msg;
      // std::cout << "mode feedback : " <<
      // state.current_motion_mode.motion_mode << std::endl; std::cout <<
      // "changing feedback : " << state.current_motion_mode.mode_changing <<
      // std::endl;
      break;
    }
    /* sensor feedback */
    case AgxMsgOdometry: {
      // std::cout << "Odometer msg feedback received: " <<
      // state.odometry.left_wheel <<" , " << state.odometry.right_wheel <<
      // std::endl;
      state.odometry = status_msg.body.odometry_msg;
      break;
    }
    case AgxMsgImuAccel: {
      state.imu_accel_ = status_msg.body.imu_accel_msg;
      break;
    }
    case AgxMsgImuGyro: {
      state.imu_gyro_ = status_msg.body.imu_gyro_msg;
      break;
    }
    case AgxMsgImuEuler: {
      state.imu_euler_ = status_msg.body.imu_euler_msg;
      break;
    }
    default:
      break;
  }
}

//----------------------------------------------------------------
void LimoDriver::Connect(std::string dev_name, uint32_t bouadrate) {
  port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_name, bouadrate));
  if (port_->openPort() == 0) {
    read_data_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&LimoDriver::readData, this)));
  } else {
    ROS_ERROR("Failed to open %s", port_->getDevPath().c_str());
    port_->closePort();
    exit(-1);
  }
}
void LimoDriver::SetMotionMode(uint8_t mode) {
  AgxMessage msg;
  msg.type = AgxMsgSetMotionMode;
  msg.body.motion_mode_msg.motion_mode = mode;

  // send to can bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  SendFrame(frame);
}
void LimoDriver::EnableCommandedMode() {
  // construct message
  AgxMessage msg;
  msg.type = AgxMsgControlModeConfig;
  msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

  // encode msg to can frame and send to bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  SendFrame(frame);
}

LimoState LimoDriver::GetLimoState() {
  std::lock_guard<std::mutex> lg(state_mutex_);
  return limo_state_;
}

void LimoDriver::SetMotionCommand(double linear_vel, double angular_vel,
                                  double lateral_velocity,
                                  double steering_angle) {
  current_motion_cmd_.linear_velocity = linear_vel;
  current_motion_cmd_.angular_velocity = angular_vel;
  current_motion_cmd_.lateral_velocity = lateral_velocity;
  current_motion_cmd_.steering_angle = steering_angle;

  AgxMessage msg;
  msg.type = AgxMsgMotionCommand;

  msg.body.motion_command_msg = current_motion_cmd_;

  // send to can bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  SendFrame(frame);
}
void LimoDriver::SendFrame(const struct can_frame &frame) {
  size_t len = frame.can_dlc;
  if(len>8){
    return;
  }
  uint32_t checksum = 0;
  uint8_t frame_len = 0x0e;
  uint8_t data[frame_len] = {0x55, frame_len};
  data[2] = (uint8_t)(frame.can_id >> 8);
  data[3] = (uint8_t)(frame.can_id & 0xff);
  for(size_t i=0; i<len; i++){
    data[i+4] = frame.data[i];
    checksum += frame.data[i];
  }
  data[frame_len - 1] = (uint8_t)(checksum &0xff);

  port_->writeData(data, frame_len);
}