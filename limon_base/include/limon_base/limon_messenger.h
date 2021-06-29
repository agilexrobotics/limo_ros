#ifndef LIMON_MESSENGER_H
#define LIMON_MESSENGER_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <ugv_sdk/scout_base.hpp>

using namespace westonrobot;
using LimonBase = ScoutBase;
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

  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber motion_cmd_sub_;
  ros::Subscriber light_cmd_sub_;
//   tf2_ros::TransformBroadcaster tf_broadcaster_;

  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double theta_{0.0};

  ros::Time last_time_;
  ros::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void PublishOdomtryToROS(double linear, double angular, double dt);
};
}  // namespace agx
#endif