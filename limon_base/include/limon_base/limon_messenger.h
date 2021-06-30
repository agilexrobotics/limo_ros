#ifndef LIMON_MESSENGER_H
#define LIMON_MESSENGER_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <limon_msgs/LimonSetting.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <ugv_sdk/limon_base.h>

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

  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double theta_{0.0};

  ros::Time last_time_;
  ros::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void LimonSettingCbk(const limon_msgs::LimonSetting::ConstPtr &msg);
};
}  // namespace agx
#endif