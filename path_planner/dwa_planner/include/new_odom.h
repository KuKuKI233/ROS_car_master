#include "ros/ros.h"
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>

namespace new_odom{
ros::Publisher odom_pub_;
ros::Subscriber odom_sub_;
nav_msgs::Odometry::ConstPtr my_odmo_mgs_ptr_;

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};