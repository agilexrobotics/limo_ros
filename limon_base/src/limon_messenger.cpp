#include "limon_base/limon_messenger.h"

using namespace agx;

LimonROSMessenger::LimonROSMessenger(ros::NodeHandle *nh)
    : limon_(nullptr), nh_(nh) {}
LimonROSMessenger::LimonROSMessenger(LimonBase *limon, ros::NodeHandle *nh)
    : limon_(limon), nh_(nh) {}
void LimonROSMessenger::SetupSubscription() {
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);

  motion_cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &LimonROSMessenger::TwistCmdCallback, this);
}

void LimonROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
        ROS_INFO("get cmd %lf %lf", msg->linear.x, msg->angular.z);
  limon_->SetMotionCommand(msg->linear.x, msg->angular.z);
}
void LimonROSMessenger::PublishOdomtryToROS(double linear, double angular,
                                            double dt) {
  linear_speed_ = linear;
  angular_speed_ = angular;

  double d_x = linear_speed_ * std::cos(theta_) * dt;
  double d_y = linear_speed_ * std::sin(theta_) * dt;
  double d_theta = angular_speed_ * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;
}
