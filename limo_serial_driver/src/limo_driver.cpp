#include "limo_driver.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "limo_data.h"
#include "limo_msg_parser.h"
#include "limo_message.h"


#define DEG_TO_RAD (0.01745329)

LimoDriver::LimoDriver() {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string dev_path;
  private_nh.param("depth_path", dev_path, std::string("/dev/ttyTHS1"));

  port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_path, B460800));

  imu_data_pub_ = nh.advertise<sensor_msgs::Imu>("/imu", 10, true);
  initialize();
}

void LimoDriver::initialize() {
  if (port_->openPort() == 0) {
    read_data_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&LimoDriver::readData, this)));
  } else {
    ROS_ERROR("Failed to open %s", port_->getDevPath().c_str());
    exit(-1);
  }
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

void LimoDriver::parseFrame(LIMO_t_RAW_t& frame) {
  if (0x321 <= frame.can_id <= 0x323) {
    AgxMessage status_msg;
    can_frame rx_frame;
    rx_frame.can_id = frame.can_id;
    memcpy(&rx_frame.data[0], &frame.data[0], 8);
    DecodeCanFrame(&rx_frame, &status_msg);
    UpdateLimoState(status_msg, limo_state_);

    GenerateImuMsg(limo_state_);
  }
}

void LimoDriver::UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state) {
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

void LimoDriver::GenerateImuMsg(LimoState& state) {
  sensor_msgs::Imu imu_data_;
  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.frame_id = "imu_link";

  imu_data_.linear_acceleration.x =  state.imu_accel_.accel_x;
  imu_data_.linear_acceleration.y = -state.imu_accel_.accel_y;
  imu_data_.linear_acceleration.z = -state.imu_accel_.accel_z;

  imu_data_.angular_velocity.x =  state.imu_gyro_.gyro_x * DEG_TO_RAD;
  imu_data_.angular_velocity.y = -state.imu_gyro_.gyro_y * DEG_TO_RAD;
  imu_data_.angular_velocity.z = -state.imu_gyro_.gyro_z * DEG_TO_RAD;

  auto y = state.imu_euler_.yaw;
  auto p = state.imu_euler_.pitch;
  auto r = state.imu_euler_.roll;

  tf::Quaternion q;
  q.setRPY(r * DEG_TO_RAD, p * DEG_TO_RAD, y * DEG_TO_RAD);
  tf::Quaternion q2(1, 0, 0, 0);
  tf::Quaternion q_trans = q2 * q;

  imu_data_.orientation.x = q_trans.x();
  imu_data_.orientation.y = q_trans.y();
  imu_data_.orientation.z = q_trans.z();
  imu_data_.orientation.w = q_trans.w();

  imu_data_.linear_acceleration_covariance[0] = 1.0f;
  imu_data_.linear_acceleration_covariance[4] = 1.0f;
  imu_data_.linear_acceleration_covariance[8] = 1.0f;

  imu_data_.angular_velocity_covariance[0] = 1e-6;
  imu_data_.angular_velocity_covariance[4] = 1e-6;
  imu_data_.angular_velocity_covariance[8] = 1e-6;

  imu_data_.orientation_covariance[0] = 1e-6;
  imu_data_.orientation_covariance[4] = 1e-6;
  imu_data_.orientation_covariance[8] = 1e-6;

  imu_data_pub_.publish(imu_data_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "limo_driver");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  LimoDriver driver;

  ros::spin();

  return 0;
}