#ifndef LIMON_MESSENGER_H
#define LIMON_MESSENGER_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <limon_msgs/LimonSetting.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <ugv_sdk/limon_base.h>
#include "limon_base/limon_params.h"

using namespace westonrobot;
namespace agx {

class LimonROSMessenger {
 public:
  explicit LimonROSMessenger(ros::NodeHandle *nh);
  LimonROSMessenger(LimonBase *limon, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  int sim_control_rate_ = 50;

  void SetupSubscription();
  void PublishStateToROS();
  void PublishOdometryToROS(double linear, double angle_vel,
                                              double x_linear_vel,
                                              double y_linear_vel, double dt);

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

 private:
  LimonBase *limon_;
  ros::NodeHandle *nh_;
  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;

  ros::Publisher odom_publisher_;      // robot odometry
  ros::Publisher status_publisher_;    // robot status
  ros::Subscriber motion_cmd_sub_;     // get motion control
  ros::Subscriber light_cmd_sub_;      // get light control
  ros::Subscriber limon_setting_sub_;  // system setting
  //   tf2_ros::TransformBroadcaster tf_broadcaster_;

  static constexpr double l = LimonParams::wheelbase;
  static constexpr double w = LimonParams::track;
  static constexpr double steer_angle_tolerance = 0.002; // +- 0.287 degrees

    // speed variables
  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double theta_{0.0};
  double x_linear_vel_ = 0.0;  // x direction linear velocity
  double y_linear_vel_ = 0.0;  // y direction linear velocity

  ros::Time last_time_;
  ros::Time current_time_;
  uint8_t motion_mode_; // current motion type

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void LimonSettingCbk(const limon_msgs::LimonSetting::ConstPtr &msg);
};
}  // namespace agx
#endif