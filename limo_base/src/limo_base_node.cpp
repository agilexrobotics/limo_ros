/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-08-10  14:46:21
 * @FileName  : limo_base_node.cpp
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#include "limo_base/limo_messenger.h"
#include "limo_base/limo_params.h"

#include <ugv_sdk/limo_base.h>

using namespace agx;
using namespace westonrobot;

std::shared_ptr<LimoBase> robot;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "limo_node");
  ros::NodeHandle node(""), private_node("~");

  ROS_INFO("limo_node start...");

  robot = std::make_shared<LimoBase>();
  LimoROSMessenger messenger(robot.get(), &node);

  std::string port_name;
  private_node.param<std::string>("port_name", port_name,
                                  std::string("ttyTHS1"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));
  private_node.param<std::string>("motion_mode", messenger.motion_mode_string_, std::string("diff"));

  messenger.SetupSubscription();

  // connect to the serial port
  if(port_name.find("tty") != port_name.npos){
    port_name= "/dev/" + port_name;
    robot->Connect(port_name,460800);
    robot->SetBaudRate(460800);
    robot->EnableCommandedMode();
    ROS_INFO("open the serial port: %s", port_name.c_str());
  }

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    messenger.PublishStateToROS();
    // robot->SetMotionCommand(0.1,0.38);
    rate.sleep();
  }

  return 0;
}
