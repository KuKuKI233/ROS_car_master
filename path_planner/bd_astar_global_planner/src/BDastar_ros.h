#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fstream>

#include <vector>
#include <queue>
#define infinity 1.0e10
using namespace std;


struct Node{
  float cost;
  int index;
};

namespace BDastar_planner {
    class BDastarPlanner : public nav_core::BaseGlobalPlanner{
        public:
            BDastarPlanner();
            BDastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

            int width;
            int height;
            int map_size;
            vector<bool> OGM;
            vector<int> state_p;

            double getHeuristic(int cell_index, int goal_index);

            vector<int> get_neighbors(int current_cell);

            double getMoveCost(int firstIndex, int secondIndex);

            bool isInBounds(int x, int y);

            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            float f_value_front(int cell_index, int goal_index, float g_sfront);
            float f_value_back(int cell_index, int goal_index, float g_sback);

            ros::Publisher plan_pub_;
            std::string frame_id_;
            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;




    };
};
