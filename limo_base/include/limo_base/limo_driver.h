#ifndef LIMO_DRIVER_H
#define LIMO_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <mutex>
#include "serial_port.h"
#include "limo_message.h"
#include "limo_msg_parser.h"

class LimoDriver {
public:
    LimoDriver();
    ~LimoDriver();

    // set up connection
    void Connect(std::string dev_name, uint32_t bouadrate);

    // robot control
    void SetMotionCommand(double linear_vel, double steer_angle,
                            double lateral_vel = 0.0, double angular_vel = 0.0);
    void SetMotionMode(uint8_t mode);
    void EnableCommandedMode();

    // get robot state
    LimoState GetLimoState();

private:
    void readData();
    void decodeMessage(uint8_t data);
    void parseFrame(LIMO_t_RAW_t& frame);
    void GenerateImuMsg(LimoState& msg);
    void UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state);
    void SendFrame(const struct can_frame &frame);

    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;
    MotionCommandMessage current_motion_cmd_;
    std::mutex state_mutex_;
    LimoState limo_state_;
    uint8_t *send_buf_;
};

#endif // LIMO_DRIVER_H
