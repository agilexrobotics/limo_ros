#include "limon_base/limon_messenger.h"
#include <limon_msgs/LimonSetting.h>
#include <limon_msgs/LimonStatus.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "limon_base/limon_params.h"

using namespace agx;
using namespace limon_msgs;

// for template use
double FilteVelocity(float data) {
  if (std::fabs(data) <= 0.02) {
    return 0.0;
  }
  return data;
}

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

  double steer_cmd = msg->angular.z;  // steer angle, in rad
  switch (motion_mode_) {
    case LimonSetting::MOTION_MODE_FOUR_WHEEL_DIFF: {
      limon_->SetMotionCommand(msg->linear.x, 0, 0, msg->angular.z);
    } break;
    case LimonSetting::MOTION_MODE_ACKERMANN: {
      if (steer_cmd > LimonParams::max_steer_angle_central) {
        steer_cmd = LimonParams::max_steer_angle_central;
      }
      if (steer_cmd < -LimonParams::max_steer_angle_central) {
        steer_cmd = -LimonParams::max_steer_angle_central;
      }
      double phi_i = ConvertCentralAngleToInner(steer_cmd);
      limon_->SetMotionCommand(msg->linear.x, phi_i);
    } break;
    default:
      ROS_INFO("motion mode not supported in receive cmd_vel");
      break;
  }
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

  // system state
  status_msg.header.stamp = current_time_;
  status_msg.vehicle_state = state.system_state.vehicle_state;
  status_msg.control_mode = state.system_state.control_mode;
  status_msg.error_code = state.system_state.error_code;
  status_msg.battery_voltage = state.system_state.battery_voltage;
  status_msg.current_motion_mode = state.system_state.motion_mode;
  motion_mode_ = status_msg.current_motion_mode;

  // TODO: 底层反馈的线速度，角速度和实际反了
  state.motion_state.linear_velocity = -state.motion_state.linear_velocity;
  state.motion_state.angular_velocity = -state.motion_state.angular_velocity;

  // calculate the motion state
  // linear velocity (m/s) , angular velocity (rad/s), central steering angle
  // (rad)
  double l_v = 0.0, a_v = 0.0, phi = 0.0;
  // x 、y direction linear velocity (m/s), motion radius (rad)
  double x_v = 0.0, y_v = 0.0, radius = 0.0;
  double phi_i = FilteVelocity(state.motion_state.steering_angle);  // rad

  switch (motion_mode_) {
    case LimonSetting::MOTION_MODE_FOUR_WHEEL_DIFF: {
      l_v = FilteVelocity(state.motion_state.linear_velocity);
      a_v = FilteVelocity(state.motion_state.angular_velocity);
      x_v = l_v;
      y_v = 0;
    } break;
    case LimonSetting::MOTION_MODE_ACKERMANN: {
      l_v = FilteVelocity(state.motion_state.linear_velocity);
      double r = l / std::tan(std::fabs(phi_i)) + w / 2.0;
      phi = ConvertInnerAngleToCentral(phi_i);
      if (phi > steer_angle_tolerance) {
        a_v = l_v / r;
      } else {
        a_v = -l_v / r;
      }
      x_v = l_v * std::cos(phi);
      if (l_v >= 0.0) {
        y_v = l_v * std::sin(phi);
      } else {
        y_v = l_v * std::sin(-phi);
      }
      radius = r;
    } break;
    default:
      ROS_INFO("motion mode not support: %d", motion_mode_);
      break;
  }

  status_msg.linear_velocity = l_v;
  status_msg.angular_velocity = a_v;
  status_msg.lateral_velocity = 0.0;
  status_msg.steering_angle = phi;
  status_msg.x_linear_vel = x_v;
  status_msg.y_linear_vel = y_v;
  status_msg.motion_radius = radius;

  status_publisher_.publish(status_msg);

  printf("l_v: %f, a_v: %f, x_v: %f, y_v: %f, dt: %f \n\n", l_v, a_v, x_v, y_v,
         dt);
  PublishOdometryToROS(l_v, a_v, x_v, y_v, dt);

  last_time_ = current_time_;
}
void LimonROSMessenger::PublishOdometryToROS(double linear, double angle_vel,
                                             double x_linear_vel,
                                             double y_linear_vel, double dt) {
  linear_speed_ = linear;
  angular_speed_ = angle_vel;
  x_linear_vel_ = x_linear_vel;
  y_linear_vel_ = y_linear_vel;

  position_x_ +=
      cos(theta_) * x_linear_vel_ * dt - sin(theta_) * y_linear_vel_ * dt;
  position_y_ +=
      sin(theta_) * x_linear_vel_ * dt + cos(theta_) * y_linear_vel_ * dt;
  theta_ += angular_speed_ * dt;

  if (theta_ > M_PI) {
    theta_ -= 2 * M_PI;
  } else if (theta_ < -M_PI) {
    theta_ += 2 * M_PI;
  }

  printf("angle: %f\n\n", theta_ / M_PI * 180.0);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  if (pub_odom_tf_) {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;
    tf_broadcaster_.sendTransform(tf_msg);
  }

  // odom message
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = x_linear_vel_;
  odom_msg.twist.twist.linear.y = y_linear_vel_;
  odom_msg.twist.twist.angular.z = angular_speed_;

  odom_msg.pose.covariance[0] = 0.1;
  odom_msg.pose.covariance[7] = 0.1;
  odom_msg.pose.covariance[14] = 0.1;
  odom_msg.pose.covariance[21] = 1.0;
  odom_msg.pose.covariance[28] = 1.0;
  odom_msg.pose.covariance[35] = 1.0;


  odom_publisher_.publish(odom_msg);
  // printf("x: %f, y: %f, lx: %f, ly %f, a_z: %f\n", position_x_, position_y_,
        //  x_linear_vel_, y_linear_vel_, angular_speed_);
}
double LimonROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = angle;
  if (phi_i > steer_angle_tolerance) {
    double r = l / std::tan(phi_i) + w / 2;
    phi = std::atan(l / r);
  } else if (phi_i < -steer_angle_tolerance) {
    double r = l / std::tan(-phi) + w / 2;
    phi = std::atan(l / r);
    phi = -phi;
  }
  return phi;
}
double LimonROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = angle;
  double phi_i = 0.0;
  if (phi > steer_angle_tolerance) {
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
  } else if (phi < -steer_angle_tolerance) {
    phi = -phi;
    phi_i =
        std::atan(l * std::sin(phi) / (l * std::cos(phi) - w * std::sin(phi)));
    phi_i = -phi_i;
  }
  return phi_i;
}