#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm>
#include <thread>
#include<future>
#include <mutex>

#include "dubins_path.h"
#include "qp_solver.h"

struct Node //struct 
{
    double position_x; //位置x
    double position_y;  //位置y
    double direction_theta; //方向角theta
    float f_cost;
    float g_cost;
    int index;
    int parent_index;
    std::shared_ptr<Node> parent_node_ptr;
};

namespace hybrid_astar_planner{
    class HybridAstarPlanner : public nav_core::BaseGlobalPlanner{

        private:
            //变量声明；
            ros::Publisher plan_pub_;
            std::string frame_id_;
            bool initialized_ = false;
            int initialized_times = 0;//初始化次数
            costmap_2d::Costmap2DROS *costmap_ros_;
            costmap_2d::Costmap2D* costmap_;

            int width;
            int height;
            int map_size;
            std::vector<bool> OGM;
            std::vector<bool> HasNode;//检测该栅格是否已存在节点
            std::vector<int> Static_cost; //存储栅格cost值，cost < 128 为低风险区域

            float steer_radius = 0.5;//转弯半径
            //设置lattice节点 向前生长3个节点
            

        public:
            HybridAstarPlanner();
            HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            
            //必须覆盖的方法；
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


            //必须覆盖的方法；
            bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan);

            void publishPlan();

            

            //检测节点是否在地图内部： true 在内部；
            bool isInBounds(int x, int y);

            float getHeuristic(int cell_index, int goal_index);

            // float getMoveCost(int first_index, int second_indexs);

            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            //三个方向生长节点邻节点
            std::vector<std::vector<double>> steer_nodes(std::vector<double> &current_node_position);

            /* 检查生长的邻节点是否有效
                @param node_x 节点坐标x; 
                @param node_y 节点坐标y；
                @param steer_flag 生长方式， -1 左转， 0直行， 1右转；
                @param new_node_x 邻节点坐标x
                @param new_node_y 邻节点坐标y
                @return is_valid 是否有效
             */
            bool check_steer_node(int steer_flag,double node_x, double node_y,double new_node_x, double new_node_y);

            double getMoveCost(const std::vector<double> &node1_state, const std::vector<double> &node2_state);

            //四元数（w，z）转换为 theta角；
            double get_theta(double w, double z);

            // (0,2*pi) 转换至 (-pi,pi)
            double converse_pi(const double &theta);

            bool check_path_points_collied(const std::vector<std::vector<double>> &path);

            double Astar_searching(const std::vector<double> &vector_start, const std::vector<double> &vector_goal, std::vector<geometry_msgs::PoseStamped>& plan);

            std::vector<int> get_neighbors(int current_cell_index);

            float Astar_searching_getMoveCost(int firstIndex, int secondIndex);

            bool get_h1(Eigen::RowVector3d vector_start,
                        Eigen::RowVector3d vector_goal, 
                        double r,
                        double path_length);

            void get_h2(std::vector<double> vector_start, std::vector<double> vector_goal, std::vector<geometry_msgs::PoseStamped> plan);

            double add_a_and_b(const double a, const double b);
    };
};
