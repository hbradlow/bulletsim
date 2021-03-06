#include "simulation/openravesupport.h"
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
struct RobotSync {
  RaveRobotObject::Ptr m_robot;
  ros::Subscriber m_jointSub;
  sensor_msgs::JointState m_lastMsg;
  RobotSync(ros::NodeHandle& nh, RaveRobotObject::Ptr robot);
  void jointCB(const sensor_msgs::JointState&);
  void updateRobot();
};
