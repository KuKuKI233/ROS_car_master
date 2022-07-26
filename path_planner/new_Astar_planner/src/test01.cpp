#include "new_astar_planner.h"

#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(new_astar_planner::NewAstarPlanner, nav_core::BaseGlobalPlanner)

namespace new_astar_planner{

    NewAstarPlanner::NewAstarPlanner(){}
    NewAstarPlanner::NewAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void NewAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        std::cout << "initialized_ ====== " << initialized_ << std::endl;

        if(!initialized_){
            //初始化
            ROS_INFO("Initializ first");
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);//查询地图 true cost=0; false cost>0;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                    //get_cost << cost << endl;
                    //cout << "i:, j:" << cost << endl;
                    
                    if (cost == 0)
                        OGM[i * width + j] = true;
                    else
                        OGM[i * width + j] = false;
                    
                }
            }


            //查询cost map上节点的cost
            //unsigned int cost = static_cast<int>(costmap_->getCost(j, i));

            frame_id_ = costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized_ = true;
            initialized_times++;
        }

        else
            ROS_INFO("Warnning: This planner has already been initialized... doing nothing");
    }

    bool NewAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(!initialized_){
            ROS_INFO("ERROR: The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                    goal.pose.position.x,goal.pose.position.y);
        double wx = start.pose.position.x;//起点坐标(世界坐标系)
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;//起点坐标(地图坐标系)
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);//start_index 序列号

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        //维护三个OPEN，一个COSLE
        std::multimap<float, int> OPEN_f_index;
        std::multimap<int, float> OPEN_index_g;
        std::multimap<int, int> OPEN_index_parentIndex;
        std::multimap<int, Node> CLOSE; // close 使用 index 排序

        Node currentNode;
        //设置起点node
        OPEN_f_index.insert(std::make_pair(getHeuristic(start_index, goal_index), start_index));
        OPEN_index_g.insert(std::make_pair(start_index, 0));
        OPEN_index_parentIndex.insert(std::make_pair(start_index, start_index));


        plan.clear();
        int currentNodeIndex;
        float currentNode_f;
        float currentNode_g;
        int currentNode_parentIndex;

        while (!OPEN_f_index.empty())
        {
            currentNodeIndex = (OPEN_f_index.begin())->second;

            if (currentNodeIndex== goal_index){
                //将 currentNode 加入CLOSE；并结束
                ROS_INFO("!!!!!!!!!!FIND GOAL!!!!!!!!!");
                Node currentNode;
                currentNode.f_cost = (OPEN_f_index.begin())->first;
                currentNode.g_cost = currentNode.f_cost;
                currentNode.index = currentNodeIndex;
                currentNode.parent_index = OPEN_index_parentIndex.find(currentNodeIndex)->second;
                CLOSE.insert(std::make_pair(currentNodeIndex, currentNode));
                break;
            }

            currentNode_f = (OPEN_f_index.begin())->first;
            OPEN_f_index.erase(OPEN_f_index.begin());
            //多个f值可对应一个index，若OPEN_index_g中没有该index，说明该node已经在close中了
            auto NotInCLOSE = OPEN_index_g.count(currentNodeIndex);
            if(!NotInCLOSE)
                continue;
            else{
                //将 currentNode 加入到 CLOSE；
                auto iter1 = OPEN_index_g.find(currentNodeIndex);
                auto iter2 = OPEN_index_parentIndex.find(currentNodeIndex);
                Node currentNode;
                currentNode.f_cost = currentNode_f;
                currentNode.g_cost = iter1->second;
                currentNode.index = currentNodeIndex;
                currentNode.parent_index = iter2->second;
                CLOSE.insert(std::make_pair(currentNodeIndex, currentNode));

                OPEN_index_g.erase(iter1);
                OPEN_index_parentIndex.erase(iter2);

            }
            // Get neighbors
            std::vector<int> neighborIndexes = get_neighbors(currentNode.index);
            for(int i = 0; i < neighborIndexes.size(); i++){

                //检测neighbor 是否在OPEN中
                auto inOPEN = OPEN_index_g.count(neighborIndexes[i]);
                if (!inOPEN){
                    currentNode_g = (OPEN_index_g.find(currentNodeIndex)->second);
                    // currentNode_parentIndex = (OPEN_index_parentIndex.find(currentNodeIndex)->second);
                    float neighborNode_g = currentNode_g + getMoveCost(currentNodeIndex, neighborIndexes[i]);
                    OPEN_index_g.insert(std::make_pair(neighborIndexes[i], neighborNode_g));
                    float neighborNode_f = neighborNode_g + getHeuristic(neighborIndexes[i], goal_index);
                    OPEN_f_index.insert(std::make_pair(neighborNode_f, neighborIndexes[i]));
                    std::cout << neighborIndexes[i] << std::endl;
                    OPEN_index_parentIndex.insert(std::make_pair(neighborIndexes[i], currentNodeIndex));
                }
                else{
                    float old_neighborNode_g_cost = (OPEN_index_g.find(neighborIndexes[i])->second);
                    currentNode_g = (OPEN_index_g.find(currentNodeIndex)->second);
                    float new_neighborNode_g_cost = currentNode_g + getMoveCost(currentNodeIndex, neighborIndexes[i]);
                    if(old_neighborNode_g_cost>new_neighborNode_g_cost){
                        OPEN_index_g.find(neighborIndexes[i])->second = new_neighborNode_g_cost;
                        OPEN_index_parentIndex.find(neighborIndexes[i])->second = currentNodeIndex;

                    }
                }
            }
        }
    
        //find path
        if(!CLOSE.count(goal_index)){
            std::cout << "Goal not reachable, failed making a global path." << std::endl;
            return false;
        }
        if(start_index == goal_index)
            return false;

        std::vector<int> bestPath;
        int current_index = goal_index;
        while (current_index!=start_index)
        {
            bestPath.push_back(current_index);
            current_index = CLOSE.find(current_index)->second.parent_index;
        }

        reverse(bestPath.begin(), bestPath.end());
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size(); i++){
            unsigned int tmp1, tmp2;
            costmap_->indexToCells(bestPath[i], tmp1, tmp2);
            double x, y;
            costmap_->mapToWorld(tmp1,tmp2, x, y);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            plan.push_back(pose);
        }

        plan.push_back(goal);
        publishPlan(plan);
        return true;

        };



    float NewAstarPlanner::getHeuristic(int cell_index, int goal_index){

        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
 
        //float h=abs(startX-goalX)+abs(startY-goalY)+(sqrt(2)-2)*(std::min(abs(startX-goalX),abs(startY-goalY)));
        float h=abs(startX-goalX)+abs(startY-goalY)+(1.4-2)*(std::min(abs(startX-goalX),abs(startY-goalY)));
        
        return h;
    }


    std::vector<int> NewAstarPlanner::get_neighbors(int current_cell_index){
        std::vector<int> neighborIndexes;
        
        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                if((i==0 && j==0))
                    continue;
                else{
                    unsigned tmp1, tmp2;
                    costmap_->indexToCells(current_cell_index, tmp1, tmp2);//
                    int nextX = tmp1 + i;
                    int nextY = tmp2 + j;                    
                    int nextIndex=costmap_->getIndex(tmp1+i, tmp2+j);
                    if((isInBounds(nextX, nextY)) && (OGM[nextIndex])){
                        neighborIndexes.push_back(nextIndex);
                    }
                    else{
                        continue;
                    }
                }
            }
        }
        return neighborIndexes;
    }

    float NewAstarPlanner::getMoveCost(int firstIndex, int secondIndex){

        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;
        
        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.4;
    }


    bool NewAstarPlanner::isInBounds(int x, int y){
        if(x<0 || y<0 || x>=height || y>=width){
            return false;
        }
        return true;
    }

    void NewAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

};


// Required for multiset sorting
bool operator <(const Node& x, const Node& y) {
  return x.f_cost < y.f_cost;
}