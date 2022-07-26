#include "dwa_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dwa_planner::DWAPlanner, nav_core::BaseLocalPlanner)

namespace dwa_planner{
    DWAPlanner::DWAPlanner(): costmap_ros_(NULL), tf_(NULL), initialized_(false){};

    DWAPlanner::DWAPlanner(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS *costmap_ros): 
                            costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {   
        // tf::TransformListener *tf
        initialize(name, tf, costmap_ros);
    };

    DWAPlanner::~DWAPlanner(){};

    void DWAPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
    {   
        if(!initialized_){
            ROS_INFO("do initialize");
            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

            
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            costmap_ros_->getRobotPose(current_pose_);
            ROS_INFO("robot current_pose: x %.2f, y %.2f, theta_z %.2f, theta_w %.2f", current_pose_.pose.position.x,
                                                        current_pose_.pose.position.y,
                                                        current_pose_.pose.orientation.z,
                                                        current_pose_.pose.orientation.w);
            // odom_helper_.setOdomTopic( odom_topic_ );

            //重要的初始化项结束
            // planner_util_.initialize(tf, costmap_, costmap_ros_->getGlobalFrameID());

            getConfigure(limits_);
            // bool config_restore_defaults = false;
            // planner_util_.reconfigureCB(limits_, config_restore_defaults);

            initialized_ = true;
        }
        else{
            ROS_INFO("It is initialized");
            return;
        }
    };

    bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        std::cout << "path size: " << orig_global_plan.size() << std::endl;
        ROS_INFO("path begin: x %.2f, y%.2f", (orig_global_plan.begin())->pose.position.x, (orig_global_plan.begin())->pose.position.y);
        ROS_INFO("path end: x %.2f, y%.2f", (orig_global_plan.end()-1)->pose.position.x, (orig_global_plan.end()-1)->pose.position.y);

        

        window_global_path_.clear();
        window_global_path_ = orig_global_plan;
        global_goal_pose_ = window_global_path_.back();
        std::cout << "window path size: " << window_global_path_.size() << std::endl;

        geometry_msgs::Twist window_vel;


        return true;
    };

    bool DWAPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        // ROS_INFO("!computeVelocityCommands");

        geometry_msgs::PoseStamped window_current_pose;
        costmap_ros_->getRobotPose(window_current_pose);
        double start_x = window_current_pose.pose.position.x;
        double start_y = window_current_pose.pose.position.y;
        

        double window_goal_x;
        double window_goal_y;
        double window_goal_dist;

        double min_window_goal_dist = INFINITY;
        geometry_msgs::PoseStamped nearest_path_point;//路径上距离robot最近点；
        bool find_window_goal = false;

        for (auto iter = window_global_path_.end() - 1; iter != window_global_path_.begin(); iter--)
        {
            window_goal_x = iter->pose.position.x;
            window_goal_y = iter->pose.position.y;
            double x2 = (window_goal_x - start_x) * (window_goal_x - start_x);
            double y2 = (window_goal_y - start_y) * (window_goal_y - start_y);
            double window_goal_dist = sqrt(x2 + y2);
            if(window_goal_dist < min_window_goal_dist){
                nearest_path_point = *iter;
            }
            if (window_goal_dist < 0.3)
            {
                window_goal_pose_ = *iter;
                find_window_goal = true;
                break;
            }
        }
        if(!find_window_goal){
            window_goal_pose_ = nearest_path_point;
        }
        ROS_INFO(" window_goal %.2f, %.2f", window_goal_pose_.pose.position.x, window_goal_pose_.pose.position.y);

        // double window_goal_theta_;// 局部/窗口目标点的航角
        getTheta(window_goal_theta_, window_goal_pose_.pose.orientation.w, window_goal_pose_.pose.orientation.z);
        double current_theta; //目前位置航角
        getTheta(current_theta, window_current_pose.pose.orientation.w, window_current_pose.pose.orientation.z);

        double diff_theta;// 局部/窗口目标点航角和目前位置航角差值
        diff_theta = window_goal_theta_ - current_theta;
        conversePI(diff_theta);

        double currentpoint_to_goalpoint_theta;//当前点位置和局部/窗口目标点位置的角度差
        double to_goal_diff_x, to_goal_diff_y;
        to_goal_diff_x = window_goal_pose_.pose.position.x - window_current_pose.pose.position.x;
        to_goal_diff_y = window_goal_pose_.pose.position.y - window_current_pose.pose.position.y;
        currentpoint_to_goalpoint_theta = atan2(to_goal_diff_y, to_goal_diff_x);
        ROS_INFO("!currentpoint_to_goalpoint_theta: %.2f", currentpoint_to_goalpoint_theta);

        double currentpointtheta_to_goalpoint_theta;//当前点航角和当前点位置到目标点位置的航向的角度差
        currentpointtheta_to_goalpoint_theta = currentpoint_to_goalpoint_theta - current_theta;
        conversePI(currentpointtheta_to_goalpoint_theta);
        ROS_INFO("!currentpointtheta_to_goalpoint_theta: %.2f", currentpointtheta_to_goalpoint_theta);

        // // 使用waitForMessage正常情况的延迟不高，但在此处延迟为200ms级别，无论是否重构了odom topic
        // // 若使用odom获取速度信息，控制频率只能实现4～5hz
        // 获取机器人当前odom信息 位置、速度
        ROS_INFO("start2");
        nav_msgs::OdometryConstPtr odom_msgs = ros::topic::waitForMessage<nav_msgs::Odometry>("/my_odom", ros::Duration(5));
        ROS_INFO("end2 %.2f", odom_msgs->twist.twist.angular.z);

        // ROS_INFO("getCurrentVel start");
        // nav_msgs::OdometryConstPtr odom_msg_p = getOdomVel(odom_topic_);
        // ROS_INFO("getCurrentVel end %.2f", odom_msg_p->twist.twist.angular.z);
        // geometry_msgs::PoseStamped goal_pose;
        // planner_util_.getGoal(goal_pose);
        // ROS_INFO("Gola %.2f, %.2f, %.2f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.orientation.z);

        // set vel
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        // dwaLatchedStopRoatateController(cmd_vel, currentpointtheta_to_goalpoint_theta);

        // isGoalPositionReached_flag 是否到达全局目标点位置
        // bool isGoalPositionReached_flag = isGoalPositionReached();
        // ROS_INFO("isGoalPositionReached_flag %d", isGoalPositionReached_flag);
        if(isGoalPositionReached()){
            //到达全局目标点位置，使用 dwaLatchedStopRoatateController 原地旋转，到达全局目标角度
            
            double g_goal_theta;
            getTheta(g_goal_theta, global_goal_pose_.pose.orientation.w, global_goal_pose_.pose.orientation.z);  
            double diff_globalgoal_theta = g_goal_theta - current_theta;
            conversePI(diff_globalgoal_theta);
            ROS_INFO("isGoalPositionReached_flag 1  %.2f", diff_globalgoal_theta);
            dwaLatchedStopRoatateController(cmd_vel, diff_globalgoal_theta);
        }
        else{
            //没有到达全局点位置
            if(abs(currentpointtheta_to_goalpoint_theta)>(M_PI/4)){
                // 如果和局部目标点角度差 大于 pi/4，使用 dwaLatchedStopRoatateController 原地旋转
                dwaLatchedStopRoatateController(cmd_vel, currentpointtheta_to_goalpoint_theta);
            }
            else{
                // 如果和局部目标点角度差 小于 pi/4，使用 dwa
                dwaController(cmd_vel, odom_msgs);
                // double a;
            }
        }

        return true;
    };
    
    nav_msgs::OdometryConstPtr DWAPlanner::getOdomVel(std::string odom_topic){
        nav_msgs::OdometryConstPtr odom_msgs = ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic, ros::Duration(5));
        return odom_msgs;
    };

    nav_msgs::Odometry::ConstPtr DWAPlanner::getCurrentVel(){
        ros::NodeHandle vel_nh("vel_nh");
        nav_msgs::Odometry::ConstPtr mp;
        std::cout << "getCurrentVel ing" << std::endl;
        ros::Subscriber odom_sub = vel_nh.subscribe<nav_msgs::Odometry>(odom_topic_, 1000, velSubCallback);
        ros::spinOnce();
        return mp;
    };

    void DWAPlanner::velSubCallback(nav_msgs::OdometryConstPtr msg_p){
        std::cout << "velSubCallback ing" << std::endl;
        std::cout << "l_v ang_v"<< msg_p->twist.twist.linear.x << "," << msg_p->twist.twist.angular.z << std::endl;
        ROS_INFO("odom msg: linear_v %.2f, angular_v %.2f",
                                msg_p->twist.twist.linear.x,
                                msg_p->twist.twist.angular.z);
        
    };

    void DWAPlanner::dwaController(geometry_msgs::Twist &cmd_vel,
                                    nav_msgs::OdometryConstPtr &odom_msgs)
    {
        std::vector<double> dw(4); //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
        //计算动态窗口
        dw = dwaCalculatedw(odom_msgs);
        //计算最佳（v, w）
        vector<double> v_w(2);
        v_w = dwaCalculateBestSpeed(odom_msgs, dw);
        ROS_INFO("best speed %.2f, %.2f", v_w[0], v_w[1]);
        cmd_vel.linear.x = v_w[0];
        cmd_vel.angular.z = v_w[1];
        return;
    };

    vector<double> DWAPlanner::dwaCalculatedw(nav_msgs::OdometryConstPtr &odom_msgs){

        // 机器人速度属性限制的动态窗口 dw_robot_state
        std::vector<double> dw_robot_state{limits_.min_vel_x, 
                                            limits_.max_vel_x, 
                                            limits_.min_vel_theta, 
                                            limits_.max_vel_theta};
        // 机器人模型(加速度)限制的动态窗口 dw_robot_model
        std::vector<double> dw_robot_model(4);
        dw_robot_model[0] = odom_msgs->twist.twist.linear.x - limits_.acc_lim_x * sim_dt_;
        dw_robot_model[1] = odom_msgs->twist.twist.linear.x + limits_.acc_lim_x * sim_dt_;
        dw_robot_model[2] = odom_msgs->twist.twist.angular.z - limits_.acc_lim_theta * sim_dt_;
        dw_robot_model[3] = odom_msgs->twist.twist.angular.z + limits_.acc_lim_theta * sim_dt_;
    
        // 机器人速度动态窗口 dw
        vector<double> dw{max(dw_robot_state[0], dw_robot_model[0]),
                          min(dw_robot_state[1], dw_robot_model[1]),
                          max(dw_robot_state[2], dw_robot_model[2]),
                          min(dw_robot_state[3], dw_robot_model[3])};
        return dw;
    };

    vector<double> DWAPlanner::dwaCalculateBestSpeed(nav_msgs::OdometryConstPtr &odom_msgs,
                                                    const std::vector<double> &dw){
        std::vector<double> best_speed{0, 0};
        std::vector<nav_msgs::Odometry> trajectoryTmp;
        double min_cost = INFINITY;
        double final_cost;
        double goal_cost;
        double speed_cost = 0;
        double obstacle_cost = 0;
        double distance_cost = 0;

        for (double i = dw[0]; i < dw[1];i += linear_vel_resolution_){
            for (double j = dw[2]; j < dw[3]; j += angular_vel_resolution_){
                //预测轨迹
                trajectoryTmp.clear();
                dwaPredictTrajectory(odom_msgs, i, j, trajectoryTmp);
                //计算cost
                goal_cost = 0.5 * dwaCalculateGoalCost(trajectoryTmp);
                obstacle_cost = 0.5 * dwaCalculateObsCost(trajectoryTmp);
                speed_cost = 0.5 * (limits_.max_vel_x - trajectoryTmp.back().twist.twist.linear.x);
                distance_cost = 0.5 * sqrt(pow(window_goal_pose_.pose.position.x - trajectoryTmp.back().pose.pose.position.x, 2) + pow(window_goal_pose_.pose.position.y - trajectoryTmp.back().pose.pose.position.y, 2));
                final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost;

                if(final_cost < min_cost){
                    min_cost = final_cost;
                    best_speed[0] = i;
                    best_speed[1] = j;
                }
                if(best_speed[0] < 0.001 && odom_msgs->twist.twist.linear.x < 0.001){
                    best_speed[1] = -limits_.max_vel_theta;
                }
            }
        }
        return best_speed;
    };

    // 在一段时间内预测轨迹
    void DWAPlanner::dwaPredictTrajectory(nav_msgs::OdometryConstPtr &odom_msgs,
                              const double &linear_v,
                              const double &angular_v,
                              std::vector<nav_msgs::Odometry> &trajectoryTmp){

        double d_time = 0;
        nav_msgs::Odometry next_state;
        // nav_msgs::Odometry next_state = *odom_msgs;
        next_state.twist.twist.linear.x = linear_v;
        next_state.twist.twist.angular.z = angular_v;
        while (d_time < predict_time_)
        {
            //nextState = motion_model(nextState, speed, angular_speed);
            next_state = motionModel(next_state, linear_v, angular_v);
            trajectoryTmp.push_back(next_state);
            d_time += sim_dt_;
        }
        return;
    };

    //根据动力学模型计算下一时刻状态
    nav_msgs::Odometry DWAPlanner::motionModel(const nav_msgs::Odometry &robot_state, 
                                    const double &linear_v, 
                                    const double &angular_v){
        nav_msgs::Odometry next_robot_state;
        double robot_state_theta;
        getTheta(robot_state_theta, robot_state.pose.pose.orientation.w, robot_state.pose.pose.orientation.z);
        next_robot_state.pose.pose.position.x = robot_state.pose.pose.position.x + linear_v * sim_dt_ * cos(robot_state_theta);
        next_robot_state.pose.pose.position.x = robot_state.pose.pose.position.y + linear_v * sim_dt_ * sin(robot_state_theta);
        next_robot_state.twist.twist.linear.x = robot_state.twist.twist.linear.x;
        next_robot_state.twist.twist.angular.z = robot_state.twist.twist.angular.z;

        return next_robot_state;
    };

    // 计算障碍代价
    int DWAPlanner::dwaCalculateObsCost(std::vector<nav_msgs::Odometry> &trajectoryTmp){
        // double obstacle_cost
        int total_map_cost = 0;
        for (auto iter = trajectoryTmp.begin(); iter != trajectoryTmp.end(); iter++)
        {
            //获取轨迹点(world)位置
            double wx = iter->pose.pose.position.x;
            double wy = iter->pose.pose.position.y;
            //(world)位置转换成map位置
            unsigned int mx, my;
            costmap_->worldToMap(wx, wy, mx, my);
            //获取轨迹点位置的mapcost
            int map_cost = costmap_->getCost(mx, my);
            total_map_cost += map_cost;
        }
        return total_map_cost;
    };

    double DWAPlanner::dwaCalculateGoalCost(std::vector<nav_msgs::Odometry> &trajectoryTmp){
        // double error_yaw = atan2(window_goal_pose_.pose.position.y - trajectory.back().,)
        double error_yaw = atan2(window_goal_pose_.pose.position.y - trajectoryTmp.back().pose.pose.position.y,
                                 window_goal_pose_.pose.position.x - trajectoryTmp.back().pose.pose.position.x);
        double end_predict_point_theta;
        getTheta(end_predict_point_theta, trajectoryTmp.back().pose.pose.orientation.w, trajectoryTmp.back().pose.pose.orientation.z);
        double goal_cost = error_yaw - end_predict_point_theta;
        goal_cost = atan2(sin(goal_cost), cos(goal_cost));

        return abs(goal_cost);
    };

    void DWAPlanner::dwaLatchedStopRoatateController(geometry_msgs::Twist& cmd_vel, const double &e_theta){
        if(e_theta > limits_.yaw_goal_tolerance)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.1;
        }
        else if (e_theta < -limits_.yaw_goal_tolerance)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -0.1;
        }
        return;
    };
    

    //调用一次computeVelocityCommands()后，会调用一次isGoalReached()
    bool DWAPlanner::isGoalReached(){
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        ROS_INFO("!isGoalReached");
        return false;
    };

    //检测是否到达目标位置，
    bool DWAPlanner::isGoalPositionReached(){
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        geometry_msgs::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);
        ROS_INFO("current %.2f, %.2f, %.2f", robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.orientation.z);
        // window_global_path_;
        // global_goal_pose_;
        ROS_INFO("goal %.2f, %.2f, %.2f", global_goal_pose_.pose.position.x, global_goal_pose_.pose.position.y, global_goal_pose_.pose.orientation.z);
        // double g_goal_theta;
        // getTheta(g_goal_theta, g_goal_pose.pose.orientation.w, g_goal_pose.pose.orientation.z);
        double dx = global_goal_pose_.pose.position.x - robot_pose.pose.position.x;
        double dy = global_goal_pose_.pose.position.y - robot_pose.pose.position.y;
        double d = sqrt(dx * dx + dy * dy);
        if (d < limits_.xy_goal_tolerance){
            return true;
        }
        else{
            return false;
        } 
    };

    void DWAPlanner::getTheta(double &theta, const double &w, const double &z){
        if(z >= 0){
            theta = 2*acos(w);
        }
        else{
            theta = 2 * M_PI - 2*acos(w);
        }
        return;
    };

    // 将theta转到 (-pi,pi)之间
    void DWAPlanner::conversePI(double &theta){
        if (theta > M_PI){
            theta -= 2*M_PI;
        }
        else if(theta < -M_PI){
            theta += 2 * M_PI;
        }
        return;
    };

    void DWAPlanner::getConfigure(base_local_planner::LocalPlannerLimits &limits_){
        if(!initialized_){
            std::string prefix_ = "/move_base/TrajectoryPlannerROS/";

            // base_local_planner::LocalPlannerLimits limits;

            ros::param::get(prefix_ + "max_vel_trans", limits_.max_vel_trans);
            ros::param::get(prefix_ + "min_vel_trans", limits_.min_vel_trans);
            ros::param::get(prefix_ + "max_vel_x", limits_.max_vel_x);
            ros::param::get(prefix_ + "min_vel_x", limits_.min_vel_x);
            ros::param::get(prefix_ + "max_vel_y", limits_.max_vel_y);
            ros::param::get(prefix_ + "min_vel_y", limits_.min_vel_y);
            ros::param::get(prefix_ + "max_vel_theta", limits_.max_vel_theta);
            ros::param::get(prefix_ + "min_vel_theta", limits_.min_vel_theta);
            ros::param::get(prefix_ + "acc_lim_x", limits_.acc_lim_x);
            ros::param::get(prefix_ + "acc_lim_y", limits_.acc_lim_y);
            ros::param::get(prefix_ + "acc_lim_theta", limits_.acc_lim_theta);
            ros::param::get(prefix_ + "acc_lim_trans", limits_.acc_lim_trans);
            ros::param::get(prefix_ + "xy_goal_tolerance", limits_.xy_goal_tolerance);
            ros::param::get(prefix_ + "yaw_goal_tolerance", limits_.yaw_goal_tolerance);
            limits_.prune_plan = true;
            limits_.trans_stopped_vel = 0.1;
            limits_.theta_stopped_vel = 0.1;
            
            std::cout << "max_vel_trans: " << limits_.max_vel_trans << std::endl;
            
        }
    };
}