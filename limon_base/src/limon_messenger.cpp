#include "limon_base/limon_messenger.h"
#include <limon_msgs/LimonStatus.h>
#include <math.h>

using namespace agx;

LimonROSMessenger::LimonROSMessenger(ros::NodeHandle *nh)
    : limon_(nullptr), nh_(nh) {}
LimonROSMessenger::LimonROSMessenger(LimonBase *limon, ros::NodeHandle *nh)
    : limon_(limon), nh_(nh) {}
void LimonROSMessenger::SetupSubscription() {
  odom_publisher_ =
      nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50, true);
  status_publisher_ =
      nh_->advertise<limon_msgs::LimonStatus>("/limon_status", 10, true);

  motion_cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &LimonROSMessenger::TwistCmdCallback, this);
  limon_setting_sub_ = nh_->subscribe<limon_msgs::LimonSetting>(
      "/limon_setting", 1, &LimonROSMessenger::LimonSettingCbk, this);
}

void LimonROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  ROS_INFO("get cmd %lf %lf", msg->linear.x, msg->angular.z);
  limon_->SetMotionCommand(msg->linear.x, msg->angular.z);
}
void LimonROSMessenger::LimonSettingCbk(
    const limon_msgs::LimonSetting::ConstPtr &msg) {
  // set motion mode
  ROS_INFO("got setting %d", msg->motion_mode);
}
void LimonROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  static bool init_run = true;

  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = limon_->GetLimonState();

  limon_msgs::LimonStatus status_msg;

  status_msg.header.stamp = current_time_;
  status_msg.vehicle_state = state.system_state.vehicle_state;
  status_msg.control_mode = state.system_state.control_mode;
  status_msg.error_code = state.system_state.error_code;
  status_msg.battery_voltage = state.system_state.battery_voltage;
  status_msg.current_motion_mode = state.system_state.motion_mode;

  double l_v = 0.0;
  double phi_i = state.motion_state.steering_angle / 180.0 * M_PI;
  status_msg.linear_velocity = l_v;
  status_msg.angular_velocity = 0.0;
  status_msg.lateral_velocity = 0.0;
  status_msg.steering_angle = phi_i;
  status_msg.x_linear_vel = 0.0;
  status_msg.y_linear_vel = 0.0;
  status_msg.motion_radius = 0.0;

  status_publisher_.publish(status_msg);

  last_time_ = current_time_;
}
