
#include "limo_sim/limo_ros_4_wheel_diff_drive.h"

// reference:
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo {
enum {
  FRONT_LEFT,
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
    gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", false);
    gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.34);
    gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.15);
    gazebo_ros_->getParameter<double>(wheel_accel_, "wheelAcceleration", 0.0);
    gazebo_ros_->getParameter<double>(wheel_torque_, "wheelTorque", 5.0);
    gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);

    joints_.resize(4);
    joints_[FRONT_LEFT] = gazebo_ros_->getJoint(parent, "frontLeftJoint", "front_left_joint");
    joints_[REAR_LEFT] = gazebo_ros_->getJoint(parent, "rearLeftJoint", "rear_left_joint");
    joints_[REAR_RIGHT] = gazebo_ros_->getJoint(parent, "rearRightJoint", "rear_right_joint");
    joints_[FRONT_RIGHT] = gazebo_ros_->getJoint(parent, "frontRightJoint", "front_right_joint");
    joints_[FRONT_LEFT]->SetParam("fmax",0,wheel_torque_);
    joints_[REAR_LEFT]->SetParam("fmax",0,wheel_torque_);
    joints_[REAR_RIGHT]->SetParam("fmax",0,wheel_torque_);
    joints_[FRONT_RIGHT]->SetParam("fmax",0,wheel_torque_);

    this->publish_tf_ = true;
    if (!sdf->HasElement("publishTf")) {
    ROS_WARN_NAMED("diff_drive", "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
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
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_,
        1, boost::bind(&GazeboRosFourWheelDiffDrive::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);

    if(this->publish_tf_){
        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_,1);
    }

    this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosFourWheelDiffDrive::QueueThread, this));
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosFourWheelDiffDrive::UpdateChild, this));
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
    if (this->publish_tf_) publishOdometry(seconds_since_last_update);
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
    }
    last_update_time_+= common::Time ( update_period_ );
    // clang-format on
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}
}  // namespace gazebo