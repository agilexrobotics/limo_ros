#include "limon_base/limon_messenger.h"
#include <limon_msgs/LimonStatus.h>

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
void LimonROSMessenger::PublishStateToROS() {
    current_time_ = ros::Time::now();
    double dt = (current_time_-last_time_).toSec();
    static bool init_run=true;

    if(init_run){
        last_time_=current_time_;
        init_run=false;
        return;
    }

    auto state = limon_->GetLimonState();

    limon_msgs::LimonStatus status_msg;
}
