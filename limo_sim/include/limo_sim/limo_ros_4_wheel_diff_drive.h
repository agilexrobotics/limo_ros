#ifndef WHEEL_4_DIFF_DRIVE_PLUGIN_H
#define WHEEL_4_DIFF_DRIVE_PLUGIN
#include <assert.h>
#include <algorithm>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_ros_utils.h"
#include <map>

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
  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;

  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque_;
  double wheel_speed_[2];
  double wheel_accel_;
  double wheel_speed_instr_[2];

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
  
};
}  // namespace gazebo

// reference:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.h

#endif  // WHEEL_4_DIFF_DRIVE_PLUG