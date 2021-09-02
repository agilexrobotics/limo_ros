#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "serial_port.h"

class IMUDriver {
public:
    IMUDriver();
    ~IMUDriver() {}

    void initialize();
    void setPort(uint32_t baudrate);
    void setRate(uint16_t rate);
    void enableMessage(uint8_t type, uint8_t id, uint8_t rate);
    void decodeMessage(uint8_t data);
    // void decodePayload(IMU_t_RAW_t imuRawData);
    void readData();

private:
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;

    ros::Publisher imu_data_pub_;
};

#endif // IMU_DRIVER_H
