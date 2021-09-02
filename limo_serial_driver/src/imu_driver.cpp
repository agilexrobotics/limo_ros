#include "imu_driver.h"

IMUDriver::IMUDriver() {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string dev_path;
  private_nh.param("depth_path", dev_path, std::string("/dev/ttyTHS1"));

  port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_path));

  initialize();
}

void IMUDriver::initialize() {
  if (port_->openPort() == 0) {
    read_data_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&IMUDriver::readData, this)));
  }else{
      ROS_ERROR("Failed to open %s", port_->getDevPath().c_str());
        exit(-1);
  }
}
void IMUDriver::readData() {
    uint8_t rx_data = 0;
    while(ros::ok()) {
      auto len = port_->readByte(&rx_data);
        if(len < 1)
            continue;

        // decodeMessage(rx_data);
        printf("read msg: %d \t %d\n", time(0), len);
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_driver");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  IMUDriver driver;

  ros::spin();

  return 0;
}