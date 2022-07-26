#include "new_odom.h"

namespace new_odom
{

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    nav_msgs::Odometry vel;
    vel = *msg;
    my_odmo_mgs_ptr_ = msg;
    // ROS_INFO("vel %.2f, %.2f", vel.twist.twist.linear.x, vel.twist.twist.angular.z);
    odom_pub_.publish(vel);
}

} 
int main(int argc, char *argv[]){
    std::cout << "!!!!!!new odom" << std::endl;
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"new_odom_node");

    ros::NodeHandle my_odom_nh;
    // odom_pub_ = my_odom_nh.advertise<nav_msgs::Odometry>("my_odom", 50);
    new_odom::odom_pub_= my_odom_nh.advertise<nav_msgs::Odometry>("new_odom", 50);
    new_odom::odom_sub_ = my_odom_nh.subscribe<nav_msgs::Odometry>("odom", 1000, new_odom::OdomCallback);
    ros::spin();
}
