#ifndef LIMO_DRIVER_H
#define LIMO_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "serial_port.h"
#include "limo_data.h"
#include "limo_message.h"

#include <ros/ros.h>

class LimoDriver {
public:
    LimoDriver();
    ~LimoDriver() {}

    void initialize();
    void setPort(uint32_t baudrate);
    void setRate(uint16_t rate);
    void enableMessage(uint8_t type, uint8_t id, uint8_t rate);
    void decodeMessage(uint8_t data);
    void decodePayload(uint8_t rx_data);
    void readData();
    void parseFrame(LIMO_t_RAW_t& frame);

private:
    void GenerateImuMsg(LimoState& msg);
    void UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state);

    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;

    LimoState limo_state_;

    ros::Publisher imu_data_pub_;
};

#endif // LIMO_DRIVER_H
