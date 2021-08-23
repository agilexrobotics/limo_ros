/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-08-10  14:45:30
 * @FileName  : limo_messenger.h
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef LIMO_MESSENGER_H
#define LIMO_MESSENGER_H

#include <limo_msgs/LimoSetting.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/master.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <ugv_sdk/limo_base.h>
#include "limo_base/limo_params.h"


using namespace westonrobot;
namespace agx {

class LimoROSMessenger {
 public:
  explicit LimoROSMessenger(ros::NodeHandle *nh);
  LimoROSMessenger(LimoBase *limo, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  bool pub_odom_tf_{false};

  int sim_control_rate_ = 50;

  void GenerateImuMsg(const LimoState& state);
  void SetupSubscription();
  void PublishStateToROS();
  void PublishOdometryToROS(double linear, double angle_vel,
                            double x_linear_vel, double y_linear_vel,
                            double dt);

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

 private:
  LimoBase *limo_;
  ros::NodeHandle *nh_;
  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;
  sensor_msgs::Imu imu_data_;

  ros::Publisher odom_publisher_;      // robot odometry
  ros::Publisher status_publisher_;    // robot status
  ros::Publisher imu_publisher_;       // imu data
  ros::Subscriber motion_cmd_sub_;     // get motion control
  ros::Subscriber light_cmd_sub_;      // get light control
  ros::Subscriber limo_setting_sub_;  // system setting
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  static constexpr double l = LimoParams::wheelbase;
  static constexpr double w = LimoParams::track;
  static constexpr double steer_angle_tolerance = 0.002;  // +- 0.287 degrees

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
  uint8_t motion_mode_;  // current motion type
  std::string motion_mode_string_; // diff, ackermann, mcnamu

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void LimoSettingCbk(const limo_msgs::LimoSetting::ConstPtr &msg);
};
}  // namespace agx
#endif
