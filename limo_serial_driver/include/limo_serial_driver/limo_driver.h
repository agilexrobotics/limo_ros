#ifndef LIMO_DRIVER_H
#define LIMO_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "serial_port.h"
#include "limo_data.h"
#include "limo_message.h"
#include "limo_msg_parser.h"

#include <ros/ros.h>

class LimoDriver {
public:
    LimoDriver();
    ~LimoDriver() {}

    void SetBaudRate(uint32_t baudrate);
    void setRate(uint16_t rate);
    void enableMessage(uint8_t type, uint8_t id, uint8_t rate);
    void decodeMessage(uint8_t data);
    void decodePayload(uint8_t rx_data);
    void readData();
    void parseFrame(LIMO_t_RAW_t& frame);

    // set up connection
    void Connect(std::string dev_name, uint32_t bouadrate);

    // robot control
    void SetMotionCommand(double linear_vel, double steer_angle,
                            double lateral_vel = 0.0, double angular_vel = 0.0);
    void SetMotionMode(uint8_t mode);
    void EnableCommandedMode();

    // get robot state
    LimoState GetLimoState();
    void SendFrame(can_frame frame);

private:
    void GenerateImuMsg(LimoState& msg);
    void UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state);

    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;
    MotionCommandMessage current_motion_cmd_;

    LimoState limo_state_;

    ros::Publisher imu_data_pub_;
};

#endif // LIMO_DRIVER_H
