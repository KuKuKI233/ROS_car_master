#include <string>
#include <iostream>
#include "ros/ros.h"
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <tf2_ros/transform_listener.h>//与tf监听有些区别
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>
#include <cmath>
#include "new_odom.h"

nav_msgs::OdometryConstPtr odom_p_;

void subCallback(const nav_msgs::OdometryConstPtr &msg_p){
    odom_p_ = msg_p;
    ROS_INFO("lv %.2f , angv %.2f", msg_p->twist.twist.linear.x, msg_p->twist.twist.angular.z);
}

class Odom
{
private:
    /* data */
public:
 
    Odom();
    nav_msgs::OdometryConstPtr getOdomVel2();
};
Odom::Odom(){}

nav_msgs::OdometryConstPtr Odom::getOdomVel2(){
    nav_msgs::OdometryConstPtr odom_msgs = ros::topic::waitForMessage<nav_msgs::Odometry>("/my_odom", ros::Duration(5));
    return odom_msgs;
};

nav_msgs::OdometryConstPtr getOdomVel(){
    nav_msgs::OdometryConstPtr odom_msgs = ros::topic::waitForMessage<nav_msgs::Odometry>("/my_odom", ros::Duration(5));
    return odom_msgs;
};

int main(int argc, char *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"listener");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;
    // std::string a{"xia_"};
    // printf("test!\n");
    // std::string prefix_ = "j2n6s200";
    // std::string robot = prefix_ + "_robot";
    // // robot += 's';
    // // std::string robot_name = prefix_ + robot;
    // std::cout << a+robot << std::endl;

    // A x("");
    double d_inf = INFINITY;
    std::cout << d_inf << std::endl;

    ros::Rate r(20);
    ROS_INFO("start");
    ros::Subscriber my_odom_sub = nh.subscribe<nav_msgs::Odometry>("/my_odom", 1, subCallback);
    r.sleep();
    ros::spinOnce();

    // while(ros::ok()){
    //     ROS_INFO("start2");
    //     nav_msgs::OdometryConstPtr odom_msgs = ros::topic::waitForMessage<nav_msgs::Odometry>("/my_odom", ros::Duration(5));
    //     ROS_INFO("end2 %.2f", odom_msgs->twist.twist.angular.z);
    // }
    Odom od;


    while (ros::ok())
    {
        // ROS_INFO("new odom %.2f", new_odom::my_odmo_mgs_ptr_->twist.twist.angular.z);
        ROS_INFO("new odom ");
        // nav_msgs::OdometryConstPtr msg_p = getOdomVel();
        nav_msgs::OdometryConstPtr msg_p = od.getOdomVel2();

        ROS_INFO("new odom %.2f", msg_p->twist.twist.angular.z);
    }

    return 0;
}
