#include "limon_base/limon_messenger.h"
#include "limon_base/limon_params.h"

#include <ugv_sdk/limon_base.h>

using namespace agx;
using namespace westonrobot;

std::shared_ptr<LimonBase> robot;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "limon_node");
  ros::NodeHandle node(""), private_node("~");

  ROS_INFO("limon_node start...");

  robot = std::make_shared<LimonBase>();
  LimonROSMessenger messenger(robot.get(), &node);

  std::string port_name;
  private_node.param<std::string>("port_name", port_name,
                                  std::string("ttyTHS1"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));

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
    rate.sleep();
  }

  return 0;
}