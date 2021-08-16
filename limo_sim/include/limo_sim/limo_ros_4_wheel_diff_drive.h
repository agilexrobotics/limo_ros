#ifndef WHEEL_4_DIFF_DRIVE_PLUGIN_H
#define WHEEL_4_DIFF_DRIVE_PLUGIN
#include <assert.h>
#include <algorithm>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <map>
#include "gazebo_ros_utils.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo {
class Joint;
class Entity;

class GazeboRosFourWheelDiffDrive : public ModelPlugin {
  enum OdomSource {
    ENCODER = 0,
    WORLD = 1,
  };

 public:
  GazeboRosFourWheelDiffDrive();
  ~GazeboRosFourWheelDiffDrive();
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  void Reset();

 protected:
  virtual void UpdateChild();
  virtual void FiniChild();

 private:
  void publishOdometry(double step_time);
  void getWheelVelocities();
  void publishWheelTF();
  void publishWheelJointState();
  void UpdateOdometryEncoder();

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;

  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque_;
  double wheel_speed_[4];
  double wheel_accel_;
  double wheel_speed_instr_[4];

  std::vector<physics::JointPtr> joints_;

  ros::Publisher odometry_publisher_;
  ros::Subscriber cmd_vel_subscriber_;
  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
  sensor_msgs::JointState joint_state_;
  ros::Publisher joint_state_publisher_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock_;

  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;

  bool publish_tf_;

  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  double x_;
  double rot_;
  bool alive_;

  double update_rate_;
  double update_period_;
  common::Time last_update_time_;

  OdomSource odom_source_;
  geometry_msgs::Pose2D pose_encoder_;
  common::Time last_odom_update_;

  bool publishWheelTF_;
  bool publishOdomTF_;
  bool publishWheelJointState_;
};
}  // namespace gazebo

// reference:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.h

#endif  // WHEEL_4_DIFF_DRIVE_PLUG