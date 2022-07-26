#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>


#include <nav_core/parameter_magic.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>

using namespace std;
namespace dwa_planner{

class DWAPlanner : public nav_core::BaseLocalPlanner{

public:
//必须实现的方法：
    DWAPlanner();

    DWAPlanner(std::string name, tf2_ros::Buffer* tf,
               costmap_2d::Costmap2DROS *costmap_ros);

    ~DWAPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

//自定义实现的方法：

    void getConfigure(base_local_planner::LocalPlannerLimits &limits_);

    bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void dwaLatchedStopRoatateController(geometry_msgs::Twist& cmd_vel, const double &e_theta);

    void dwaController(geometry_msgs::Twist &cmd_vel, nav_msgs::OdometryConstPtr &odom_msgs);

    vector<double> dwaCalculatedw(nav_msgs::OdometryConstPtr &odom_msgs);

    vector<double> dwaCalculateBestSpeed(nav_msgs::OdometryConstPtr &odom_msgs, const std::vector<double> &dw);

    void dwaPredictTrajectory(nav_msgs::OdometryConstPtr &odom_msgs, 
                                const double &linear_v, 
                                const double &angular_v, 
                                std::vector<nav_msgs::Odometry> &trajectoryTmp);

    nav_msgs::Odometry motionModel(const nav_msgs::Odometry &robot_state, const double &linear_v, const double &angular_v);

    int dwaCalculateObsCost(std::vector<nav_msgs::Odometry> &trajectoryTmp);

    double dwaCalculateGoalCost(std::vector<nav_msgs::Odometry> &trajectoryTmp);

    void getTheta(double &theta, const double &w, const double &z);

    

    void conversePI(double &theta);

    nav_msgs::Odometry::ConstPtr getCurrentVel();

    static void velSubCallback(nav_msgs::OdometryConstPtr msg_p);

    nav_msgs::OdometryConstPtr getOdomVel(std::string odom_topic);

    bool isGoalPositionReached();


private:

    //方法
    
    //必须声明的变量：
    // private 类型变量命名 结尾加下划线
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_ = false;
    // tf::TransformListener *tf_;
    tf2_ros::Buffer *tf_;

    ros::Publisher g_plan_pub_, l_plan_pub_;
    base_local_planner::LocalPlannerUtil planner_util_;
    std::shared_ptr<DWAPlanner> dp_;


    
    bool setup_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped global_goal_pose_;

    // base_local_planner::LatchedStopRotateController latchedStopRotateController_();
    // base_local_planner::OdometryHelperRos odom_helper_;
    // base_local_planner::LatchedStopRotateController latchedStopRotateController_;
    std::string odom_topic_ = "/my_odom"; // odom话题;
    // base_local_planner::OdometryHelperRos odom_helper_;

    geometry_msgs::PoseStamped window_current_pose_;
    geometry_msgs::PoseStamped window_goal_pose_;
    double window_goal_theta_;
    std::vector<geometry_msgs::PoseStamped> window_global_path_;
    base_local_planner::LocalPlannerLimits limits_;

    double linear_vel_resolution_ = 0.025; //线速度分辨率;
    double angular_vel_resolution_ = 0.05; //角速度分辨率
    double sim_dt_ = 0.1;                  // dwa 窗口计算时间 0.2s;
    double predict_time_ = 1.0;            //预测轨迹时长;
};
};

#endif