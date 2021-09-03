#include "limo_driver.h"
#include "limo_data.h"
#include "limo_msg_parser.h"


LimoDriver::LimoDriver() {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string dev_path;
  private_nh.param("depth_path", dev_path, std::string("/dev/ttyTHS1"));

  port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_path, B460800));

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
  uint8_t checksum = 0;
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
      limoRaw.can_id = (uint16_t)data << 8;
      limoRaw.state = LIMO_WAIT_ID_LOW;
    } break;
    case LIMO_WAIT_ID_LOW: {
      limoRaw.can_id |= (uint16_t)data;
      limoRaw.state = LIMO_WAIT_DATA;
      data_num = 0;
    } break;
    case LIMO_WAIT_DATA: {
      if (data_num < 8) {
        limoRaw.data[data_num] = data;
        data_num++;
        checksum+=data;
      } else {
        limoRaw.count = data;
        limoRaw.state = LIMO_CHECK;
      }
    } break;
    case LIMO_CHECK: {
      limoRaw.RxCK = data;
      limoRaw.state = LIMO_WAIT_SYNC1;
      parseFrame(limoRaw);
      printf("c1 %d, c2 %d", checksum, data);
    } break;
    default:
      limoRaw.state = LIMO_WAIT_SYNC1;
      break;
  }
}

void LimoDriver::parseFrame(LIMO_t_RAW_t& frame)
{
  if(frame.can_id = 0x211){
    printf("limoRaw.can_id: %x\n", frame.can_id);

    AgxMessage status_msg;
    can_frame rx_frame;
    rx_frame.can_id = frame.can_id;
    memcpy(&rx_frame.data[0], &frame.data[0] , 8);
    DecodeCanFrame(&rx_frame, &status_msg);
    // std::lock_guard<std::mutex> guard(state_mutex_);
    // UpdateLimoState(status_msg, limo_state_);
    printf("status vehicle_state: %d\n", status_msg.body.system_state_msg.vehicle_state);
    printf("status control_mode: %d\n", status_msg.body.system_state_msg.control_mode);
    printf("status battery_voltage: %f\n", status_msg.body.system_state_msg.battery_voltage);
    printf("status error_code: %d\n", status_msg.body.system_state_msg.error_code);
    printf("status motion_mode: %d\n", status_msg.body.system_state_msg.motion_mode);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "limo_driver");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  LimoDriver driver;

  ros::spin();

  return 0;
}