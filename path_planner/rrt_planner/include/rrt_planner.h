/** include the libraries you need in your planner here */
 /** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <float.h>

#include <vector>
#include <random>
#include <iostream>


struct Node 
{
    double wx;
    double wy;
    int parent_index; // 在node_list 中的序列值
    Node *parent_ptr = nullptr; // 指向父节点的指针
    // Node(double x, double y): wx(x), wy(y), parent_index(NULL){} //节点的构造函数
};

using std::string;

#ifndef RRT_GLOBAL_PLANNER_CPP
#define RRT_GLOBAL_PLANNER_CPP

namespace rrt_planner {

class RRTPlanner : public nav_core::BaseGlobalPlanner {
public:

    

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
                );

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    // initialize makePlan 为必须重写的方法，若没重写，则RRTPlanner类则无法实例化；
    // publishPlan 为自定义的发布方法，若将path发布加入到makePlan中，则无需 publishPlan 方法

    // 以下方法均为自定义方法：



    std::pair<double, double> generate_sample_node(int &goal_index, double &goal_wx, double &goal_wy);

    Node generate_node_near_new(const std::pair<double, double> &node_rand, std::vector<Node> &node_list, double &goal_wx, double &goal_wy);

    Node find_node_near(const std::pair<double, double> &node_rand, std::vector<Node> &node_list);



    bool sample();

    Node generate_node_new( Node &node_near, const std::pair<double, double> &node_rand);

    // check the node whether is valid (cost_map value = 0);
    bool is_node_valid(const Node &node); 

private:

    ros::Publisher plan_pub_;
    std::string frame_id_;
    bool initialized_ = false;
    int initialized_times = 0;//初始化次数
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    int width;
    int height;
    int map_size;

    std::vector<int> sampleable_space;//存储可以采样的栅格index

    int sampleable_space_size = 0;//sampleable_space的大小

    float goal_sample_rate = 0.5;//向目标采样的概率

    

};
};
#endif
