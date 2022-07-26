#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>

#include <fstream>

#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm>

struct Node //struct 占用内存较大，效率较低
{
    float f_cost;
    float g_cost;
    int index;
    int parent_index;
};


namespace new_astar_planner{
    class NewAstarPlanner : public nav_core::BaseGlobalPlanner{

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

        public:


            NewAstarPlanner();
            NewAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            
            //必须覆盖的方法；
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


            //必须覆盖的方法；
            bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan);

            void publishPlan();

            std::vector<int> get_neighbors(int current_cell_index);

            //检测节点是否在地图内部： true 在内部；
            bool isInBounds(int x, int y);

            float getHeuristic(int cell_index, int goal_index);

            float getMoveCost(int first_index, int second_indexs);

            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            std::vector<std::vector<double>> steer_nodes(std::vector<double> &current_node_state);
    };




};
