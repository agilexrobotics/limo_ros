
#include "limo_sim/limo_ros_4_wheel_diff_drive.h"

// reference:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <iostream>

namespace gazebo {
enum {
  FRONT_LEFT = 0,
  REAR_LEFT,
  REAR_RIGHT,
  FRONT_RIGHT,
};

GazeboRosFourWheelDiffDrive::GazeboRosFourWheelDiffDrive() {}
GazeboRosFourWheelDiffDrive::~GazeboRosFourWheelDiffDrive() { FiniChild(); }

// load config
void GazeboRosFourWheelDiffDrive::Load(physics::ModelPtr parent,
                                       sdf::ElementPtr sdf) {
  // clang-format off
    this->parent_ = parent;
    gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "FourWheelDiffDrive"));
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
    gazebo_ros_->getParameter<std::string>(odometry_topic_, "odometryTopic", "odom");
    gazebo_ros_->getParameter<std::string>(odometry_frame_, "odometryFrame", "odom");
    gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_footprint");
    gazebo_ros_->getParameterBoolean(publishWheelTF_, "publishWheelTF", false);
    gazebo_ros_->getParameterBoolean(publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", true);
    gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.34);
    gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.15);
    gazebo_ros_->getParameter<double>(wheel_accel_, "wheelAcceleration", 0.0);
    gazebo_ros_->getParameter<double>(wheel_torque_, "wheelTorque", 5.0);
    gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);


    ROS_INFO_NAMED("FourWheelDiffDrive", "params publishWheelTF_: %d, publishOdomTF_: %d", publishWheelTF_, publishOdomTF_);
    this->publish_tf_ = true;
    if (!sdf->HasElement("publishTf")) {
    ROS_INFO_NAMED("FourWheelDiffDrive", "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
        this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
    this->publish_tf_ = sdf->GetElement("publishTf")->Get<bool>();
    }

        // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent_->GetWorld()->SimTime();
    #else
        last_update_time_ = parent_->GetWorld()->GetSimTime();
    #endif

    // Initialize velocity stuff
    wheel_speed_[FRONT_LEFT] = 0;
    wheel_speed_[REAR_LEFT] = 0;
    wheel_speed_[REAR_RIGHT] = 0;
    wheel_speed_[FRONT_RIGHT] = 0;

    // Initialize velocity support stuff
    wheel_speed_instr_[FRONT_LEFT] = 0;
    wheel_speed_instr_[REAR_LEFT] = 0;
    wheel_speed_instr_[REAR_RIGHT] = 0;
    wheel_speed_instr_[FRONT_RIGHT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    if(this->publishWheelJointState_){
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("FourWheelDiffDrive", "publish wheel joint");
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_,
        1, boost::bind(&GazeboRosFourWheelDiffDrive::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("FourWheelDiffDrive", "subscribe to: %s", command_topic_.c_str());

    if(this->publish_tf_){
        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_,1);
        ROS_INFO_NAMED("FourWheelDiffDrive","publish odom ");
    }

    this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosFourWheelDiffDrive::QueueThread, this));
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosFourWheelDiffDrive::UpdateChild, this));

    joints_.resize(4);
    joints_[FRONT_LEFT] = gazebo_ros_->getJoint(parent, "frontLeftJoint", "wheel_front_left_joint");
    joints_[REAR_LEFT] = gazebo_ros_->getJoint(parent, "rearLeftJoint", "wheel_rear_left_joint");
    joints_[REAR_RIGHT] = gazebo_ros_->getJoint(parent, "rearRightJoint", "wheel_rear_right_joint");
    joints_[FRONT_RIGHT] = gazebo_ros_->getJoint(parent, "frontRightJoint", "wheel_front_right_joint");
    joints_[FRONT_LEFT]->SetParam("fmax",0,wheel_torque_);
    joints_[REAR_LEFT]->SetParam("fmax",0,wheel_torque_);
    joints_[REAR_RIGHT]->SetParam("fmax",0,wheel_torque_);
    joints_[FRONT_RIGHT]->SetParam("fmax",0,wheel_torque_);

  // clang-format on
}

void GazeboRosFourWheelDiffDrive::Reset() {
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent_->GetWorld()->SimTime();
#else
  last_update_time_ = parent_->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
  joints_[FRONT_LEFT]->SetParam("fmax", 0, wheel_torque_);
  joints_[REAR_LEFT]->SetParam("fmax", 0, wheel_torque_);
  joints_[REAR_RIGHT]->SetParam("fmax", 0, wheel_torque_);
  joints_[FRONT_LEFT]->SetParam("fmax", 0, wheel_torque_);
}

void GazeboRosFourWheelDiffDrive::publishWheelJointState() {
  ros::Time current_time_ = ros::Time::now();
  joint_state_.header.stamp = current_time_;
  joint_state_.name.resize(joints_.size());
  joint_state_.position.resize(joints_.size());

  for (int i = 0; i < 4; i++) {
    physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
    double position = joint->Position(0);
#else
    double position = joint->GetAngle(0).Radian();
#endif

    joint_state_.name[i] = joint->GetName();
    joint_state_.position[i] = position;
  }
  joint_state_publisher_.publish(joint_state_);
}

void GazeboRosFourWheelDiffDrive::publishWheelTF() {}

void GazeboRosFourWheelDiffDrive::UpdateChild() {
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosFourWheelDiffDrive::UpdateChild");
  IGN_PROFILE_BEGIN("update");
#endif

  for (int i = 0; i < 2; i++) {
    if (fabs(wheel_torque_ - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
      joints_[i]->SetParam("fmax", 0, wheel_torque_);
    }
  }

  if (odom_source_ == ENCODER) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time current_time = parent_->GetWorld()->SimTime();
#else
  common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
  double seconds_since_last_update =
      (current_time - last_update_time_).Double();

  if (seconds_since_last_update > update_period_) {
    if (this->publish_tf_) {
      publishOdometry(seconds_since_last_update);
    }
    if (publishWheelTF_) publishWheelTF();
    if (publishWheelJointState_) publishWheelJointState();

    // Update robot in case new velocities have been requested
    getWheelVelocities();

    double current_speed[4];
    // clang-format off
    current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
    current_speed[REAR_LEFT] = joints_[REAR_LEFT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
    current_speed[REAR_RIGHT] = joints_[REAR_RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
    current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

    if ( wheel_accel_ == 0 ||
            ( fabs ( wheel_speed_[FRONT_LEFT] - current_speed[FRONT_LEFT] ) < 0.01 ) ||
            ( fabs ( wheel_speed_[FRONT_RIGHT] - current_speed[FRONT_RIGHT] ) < 0.01 ) ) {
        //if max_accel == 0, or target speed is reached
        joints_[FRONT_LEFT]->SetParam ( "vel", 0, wheel_speed_[FRONT_LEFT]/ ( wheel_diameter_ / 2.0 ) );
        joints_[REAR_LEFT]->SetParam ( "vel", 0, wheel_speed_[REAR_LEFT]/ ( wheel_diameter_ / 2.0 ) );
        joints_[REAR_RIGHT]->SetParam ( "vel", 0, wheel_speed_[REAR_RIGHT]/ ( wheel_diameter_ / 2.0 ) );
        joints_[FRONT_RIGHT]->SetParam ( "vel", 0, wheel_speed_[FRONT_RIGHT]/ ( wheel_diameter_ / 2.0 ) );

        std::cout << "set vel 1 ..." << std::endl;
        std::cout << wheel_speed_[FRONT_LEFT]/ ( wheel_diameter_ / 2.0 ) << std::endl;
    } else {
        if ( wheel_speed_[FRONT_LEFT]>=current_speed[FRONT_LEFT] ){
            wheel_speed_instr_[FRONT_LEFT]+=fmin ( wheel_speed_[FRONT_LEFT]-current_speed[FRONT_LEFT],  wheel_accel_ * seconds_since_last_update );
            wheel_speed_instr_[REAR_LEFT] = wheel_speed_[FRONT_LEFT];
        }else{
            wheel_speed_instr_[FRONT_LEFT]+=fmax ( wheel_speed_[FRONT_LEFT]-current_speed[FRONT_LEFT], -wheel_accel_ * seconds_since_last_update );
            wheel_speed_instr_[REAR_LEFT] = wheel_speed_[FRONT_LEFT];
        }
        if ( wheel_speed_[FRONT_RIGHT]>current_speed[FRONT_RIGHT] ){
            wheel_speed_instr_[FRONT_RIGHT]+=fmin ( wheel_speed_[FRONT_RIGHT]-current_speed[FRONT_RIGHT], wheel_accel_ * seconds_since_last_update );
            wheel_speed_instr_[REAR_RIGHT] = wheel_speed_[FRONT_RIGHT];
        }else{
            wheel_speed_instr_[FRONT_RIGHT]+=fmax ( wheel_speed_[FRONT_RIGHT]-current_speed[FRONT_RIGHT], -wheel_accel_ * seconds_since_last_update );
            wheel_speed_instr_[REAR_RIGHT] = wheel_speed_[FRONT_RIGHT];
        }
        joints_[FRONT_LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[FRONT_LEFT] / ( wheel_diameter_ / 2.0 ) );
        joints_[REAR_LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[REAR_LEFT] / ( wheel_diameter_ / 2.0 ) );
        joints_[REAR_RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[REAR_RIGHT] / ( wheel_diameter_ / 2.0 ) );
        joints_[FRONT_RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[FRONT_RIGHT] / ( wheel_diameter_ / 2.0 ) );
        std::cout <<"set vel..." << std::endl;
    }
    last_update_time_+= common::Time ( update_period_ );
    // clang-format on

    std::cout << "update..." << std::endl;
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}

void GazeboRosFourWheelDiffDrive::FiniChild() {
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosFourWheelDiffDrive::getWheelVelocities() {
  boost::mutex::scoped_lock scoped_lock(lock_);

  double vr = x_;
  double va = rot_;

  wheel_speed_[FRONT_LEFT] = vr - va * wheel_separation_ / 2.0;
  wheel_speed_[REAR_LEFT] = wheel_speed_[FRONT_LEFT];
  wheel_speed_[FRONT_RIGHT] = vr + va * wheel_separation_ / 2.0;
  wheel_speed_[REAR_RIGHT] = wheel_speed_[FRONT_RIGHT];
}

void GazeboRosFourWheelDiffDrive::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  boost::mutex::scoped_lock scoped_lock(lock_);
  x_ = msg->linear.x;
  rot_ = msg->angular.z;
}
void GazeboRosFourWheelDiffDrive::QueueThread() {
  static const double timeout = 0.01;
  while (alive_ and gazebo_ros_->node()->ok()) {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}
void GazeboRosFourWheelDiffDrive::UpdateOdometryEncoder() {}
void GazeboRosFourWheelDiffDrive::publishOdometry(double step_time) {
  // clang-format off
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = gazebo_ros_->resolveTF(odometry_frame_);
  std::string base_footprint_frame = gazebo_ros_->resolveTF(robot_base_frame_);

  tf::Quaternion qt;
  tf::Vector3 vt;

  if(odom_source_ == ENCODER){
      qt = tf::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
      vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  }

  if (odom_source_ == WORLD) {
    // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif
    qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    // clang-format on

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
    linear = parent_->WorldLinearVel();
    odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();
#else
    linear = parent_->GetWorldLinearVel().Ign();
    odom_.twist.twist.angular.z = parent_->GetWorldAngularVel().Ign().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x =
        cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y =
        cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  }

  if (publishOdomTF_ == true) {
    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
                             base_footprint_frame));
  }

  // set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  // set header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  odometry_publisher_.publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosFourWheelDiffDrive)
}  // namespace gazebo