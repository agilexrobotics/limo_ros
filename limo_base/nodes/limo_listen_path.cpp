#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "listen_path");
  ros::NodeHandle node;

  tf::TransformListener listener;
  rosbag::Bag bag;
  bag.open("arrived_path.bag", rosbag::bagmode::Write);

  ros::Rate rate(1.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    nav_msgs::Path path;
    geometry_msgs::PoseStamped p;

    ros::Publisher path_pub = node.advertise<nav_msgs::Path>("arrived_path", 10);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    p.pose.position.x = transform.getOrigin().x();
    p.pose.position.y = transform.getOrigin().y();
    p.pose.position.z = transform.getOrigin().z();

    p.pose.orientation.x = transform.getRotation().x();
    p.pose.orientation.y = transform.getRotation().y();
    p.pose.orientation.z = transform.getRotation().z();
    p.pose.orientation.w = transform.getRotation().w();

    path.poses.push_back(p);
    path_pub.publish(path);

    bag.write("arrived_path", ros::Time::now(), path);

    std::cout << path << std::endl;

    rate.sleep();
  }
  return 0;
}