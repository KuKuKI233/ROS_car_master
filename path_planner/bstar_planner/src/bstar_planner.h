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

namespace bstar_planner {
    class BstarPlanner : public nav_core::BaseGlobalPlanner{
        public:

            //变量声明；
            ros::Publisher plan_pub_;
            std::string frame_id_;
            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;

            int width;
            int height;
            int map_size;
            vector<bool> OGM;

            //函数声明；

            BstarPlanner();
            BstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            
            /**
             * @brief  Initialization function for the DijstraPlanner
             * @param  name The name of this planner
             * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
             */
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * @brief Given a goal pose in the world, compute a plan
             * @param start The start pose 
             * @param goal The goal pose 
             * @param plan The plan... filled by the planner
             * @return True if a valid plan was found, false otherwise
             */
            bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            
            

            double getHeuristic(int cell_index, int goal_index);
            
            geometry_msgs::PoseStamped modify_start_orientation(const geometry_msgs::PoseStamped& goal);
            
            /**
             * @brief 2D：转换四元数到XY方向向量；
             * @param orientation: The orientation which need to transform;
             * @return vector: The direction vector in XY aixs;
             */        
            vector<float> orientation2XY(const geometry_msgs::PoseStamped& orientation);
            
            /**
             * @brief 2D：使用四元数w计算XY方向向量；
             * @param w: The orientation which need to transform;
             * @return vector: The direction vector in XY aixs;
             */ 
            vector<float> W2XY(float w);

            bool is_line_Collision(int start_index, int end_index);

            vector<int> get_neighbors(int current_cell);

            vector<int> get_neighbors_by_direction(int current_cell, float node_w);
            vector<int> get_neighbors_by_dirXY(int current_cell, int &dir_x,int &dir_y);
            vector<int> caculate_start_direction(float Dx,float Dy);



            double getMoveCost(int firstIndex, int secondIndex);

            bool isInBounds(int x, int y);    
            /**
             * @brief  Publish a path for visualization purposes
             */
            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);           
        
    };
};
