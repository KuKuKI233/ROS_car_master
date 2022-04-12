#include "BDastar_ros.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(BDastar_planner::BDastarPlanner, nav_core::BaseGlobalPlanner)

namespace BDastar_planner{
    BDastarPlanner::BDastarPlanner(){};
    BDastarPlanner::BDastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    };

    void BDastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        //初始化
        if (!initialized_)
        {
            //获取cosmap属性
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size); //OGM用于存储各像素点的可检查情况 true/falise；
            state_p.resize(map_size);//state_p存储个像素点的状态 0：未检查 1：在CLOSE_front队列 2：在CLOSE_fback队列

            //获取各像素点的可检查情况
            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {   
                    state_p[i * width + j]=0;
                    unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                    //get_cost << cost << endl;
                    //cout << "i:, j:" << cost << endl;
                    if (cost == 0)
                        OGM[i * width + j] = true;
                    else
                        OGM[i * width + j] = false;     

                }
            }
            frame_id_ = costmap_ros->getGlobalFrameID(); //获取GlobalFrameID
            
            ros::NodeHandle private_nh("~/" + name);//创建ros句柄；
            
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);//创建消息发布器，发布plan

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");

    };

    bool BDastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
        {
            if(!initialized_){
                ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
                return false;
            }
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                    goal.pose.position.x,goal.pose.position.y);
            double wx = start.pose.position.x;//wx,wy 世界坐标系下的坐标
            double wy = start.pose.position.y;
            unsigned int start_x, start_y;
            //将世界坐标系起点wx, wy 转换成地图坐标系起点 start_x, start_y
            costmap_->worldToMap(wx, wy, start_x, start_y);
            int start_index = costmap_->getIndex(start_x, start_y);

            wx = goal.pose.position.x;
            wy = goal.pose.position.y;
            unsigned int goal_x, goal_y;
            //将世界坐标系目标点wx, wy 转换成地图坐标系目标点 goal_x, goal_y
            costmap_->worldToMap(wx, wy, goal_x, goal_y);
            int goal_index = costmap_->getIndex(goal_x, goal_y);

            vector<float> gfornt_Costs(map_size, infinity);
            vector<float> gback_Costs(map_size, infinity);
            vector<int> cameFrom_front(map_size, -1);//存储front node的父节点（最小代价路径上的）；
            vector<int> cameFrom_back(map_size, -1);//存储back node的父节点（最小代价路径上的）；
            
            bool find_flag=false;//是否找到可行路径；
            //OPEN_front为向前的OPEN，即待检查的节点队列；
            //使用multiset保证 OPEN_front 是按照 Node.cost有序排列；
            multiset<Node> OPEN_front; 
            multiset<Node> OPEN_back;

            gfornt_Costs[start_index] = 0;
            gback_Costs[goal_index]=0;

            //初始化OPEN_front、OPEN_back
            Node currentNode;
            currentNode.index = start_index;
            currentNode.cost = gfornt_Costs[start_index] + 0;
            OPEN_front.insert(currentNode);

            currentNode.index = goal_index;
            currentNode.cost = gback_Costs[goal_index] + 0;
            OPEN_back.insert(currentNode);

            plan.clear();
            Node node_meet_back;
            Node node_meet_front;
            vector<int> neighborIndexes ;
            state_p[start_index]=1;
            state_p[goal_index]=2;
            int int_flag=0;
            while((!OPEN_front.empty())&&(!OPEN_front.empty())){
                //cout << "searching node." << endl;
                //前向，取出f值最小的节点；
                
                    Node node_front=*OPEN_front.begin();
                    OPEN_front.erase(OPEN_front.begin());
                    
                    if(state_p[node_front.index]==2){
                        // node_front 在 CLOSE_back中,路径联通；
                        find_flag=true;
                        int_flag=1;
                        node_meet_back=node_front;
                        node_meet_front=node_front;
                        break;
                    }
                    // node_meet_front = node_front;
                    //state_p[node_front.index]=1;//node_front加入到了CLOSE_front中；
                    neighborIndexes = get_neighbors(node_front.index);
                    for (int i = 0; i < neighborIndexes.size(); i++)
                    {   
                        state_p[neighborIndexes[i]]=1;
                        if(cameFrom_front[neighborIndexes[i]] == -1){
                            gfornt_Costs[neighborIndexes[i]] = gfornt_Costs[node_front.index] + getMoveCost(node_front.index, neighborIndexes[i]);
                            Node nextNode;
                            nextNode.index = neighborIndexes[i];
                            nextNode.cost = gfornt_Costs[neighborIndexes[i]]+ getHeuristic(neighborIndexes[i], goal_index);    //Dijkstra Algorithm
                            //   nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                            cameFrom_front[neighborIndexes[i]] = node_front.index;
                            OPEN_front.insert(nextNode);
                            
                        }
                        else{
                            float old_cost=gfornt_Costs[neighborIndexes[i]];
                            float new_cost= gfornt_Costs[node_front.index] + getMoveCost(node_front.index, neighborIndexes[i]); 
                            if(new_cost<old_cost){
                                Node change_node;
                                change_node.cost=new_cost + getHeuristic(neighborIndexes[i], goal_index);
                                change_node.index=neighborIndexes[i];
                                cameFrom_front[neighborIndexes[i]] = node_front.index;

                                gfornt_Costs[neighborIndexes[i]] = new_cost;                           
                                OPEN_front.insert(change_node);
                               
                            } 
                        }
                    }
                
                //反向，取出f值最小的节点；
                
                    Node node_back=*OPEN_back.begin();
                    OPEN_back.erase(OPEN_back.begin());
                    
                    if(state_p[node_back.index]==1){
                        // node_front 在 CLOSE_back中,路径联通；
                        find_flag=true;
                        int_flag=1;
                        node_meet_back=node_back;
                        node_meet_front=node_back;
                        break;
                    }
                    
                    neighborIndexes = get_neighbors(node_back.index);
                    // node_meet_back = node_back;
                    for (int i = 0; i < neighborIndexes.size(); i++)
                    {   state_p[neighborIndexes[i]]=2;
                        if(cameFrom_back[neighborIndexes[i]] == -1){
                            gback_Costs[neighborIndexes[i]] = gback_Costs[node_back.index] + getMoveCost(node_back.index, neighborIndexes[i]);
                            Node nextNode;
                            nextNode.index = neighborIndexes[i]; 
                            nextNode.cost = gback_Costs[neighborIndexes[i]]+ getHeuristic(neighborIndexes[i], start_index);    //Dijkstra Algorithm
                            //   nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                            cameFrom_back[neighborIndexes[i]] = node_back.index;
                            OPEN_back.insert(nextNode);
                            
                        }
                        else{
                            float old_cost=gback_Costs[neighborIndexes[i]];
                            float new_cost= gback_Costs[node_back.index] + getMoveCost(node_back.index, neighborIndexes[i]); 
                            if(new_cost<old_cost){
                                Node change_node;
                                change_node.cost=new_cost + getHeuristic(neighborIndexes[i], start_index);
                                change_node.index=neighborIndexes[i];
                                cameFrom_back[neighborIndexes[i]] = node_back.index;

                                gback_Costs[neighborIndexes[i]] = new_cost;                           
                                OPEN_back.insert(change_node);
                                
                            } 
                        }
                    } 
                          
            }

            if(!find_flag){
                cout << "Goal not reachable, failed making a global path." << endl;
                return false;
            }
            if(start_index == goal_index)
                return false;
            vector<int> Path_front;
            vector<int> Path_back;
            if(find_flag)
                cout << "find a path.//" << node_meet_front.index<< "//" << node_meet_back.index <<endl;
                cout << "int_flag = "<< int_flag <<endl;
                currentNode.index = node_meet_front.index;
                while (currentNode.index != start_index)
                {
                    Path_front.push_back(cameFrom_front[currentNode.index]);
                    currentNode.index = cameFrom_front[currentNode.index];
                }
                cout << "front//"<<endl;
                currentNode.index = node_meet_back.index;
                cout << "find a path.//" << node_meet_front.index<< "//" << node_meet_back.index <<endl;
                cout << goal_index <<"//goal"<< start_index <<"//start"<<endl;
                while (currentNode.index != goal_index&&currentNode.index!=-1)
                {
                    Path_back.push_back(cameFrom_back[currentNode.index]);
                    // cout << cameFrom_back[currentNode.index] <<"//1"<<endl;
                    currentNode.index = cameFrom_back[currentNode.index];
                    cout << currentNode.index <<"//";
                }

                while (currentNode.index != goal_index)
                {
                    Path_back.push_back(cameFrom_back[currentNode.index]);
                    // cout << cameFrom_back[currentNode.index] <<"//1"<<endl;
                    currentNode.index = cameFrom_back[currentNode.index];
                    // cout << currentNode.index <<"//2"<<endl;
                }
                cout << "back//"<<endl;
                reverse(Path_front.begin(),Path_front.end());
                cout << "reverse//"<<endl;
                //将Path_back加入到Path_front的结尾，合并成bestPath；
                Path_front.insert(Path_front.end(),Path_back.begin(),Path_back.end());
                cout << "insert//"<<endl;
                vector<int> bestPath=Path_front;
            cout << "Find bset path." << endl;
            //计算bestPath在世界坐标系中的坐标；
            ros::Time plan_time = ros::Time::now();
            plan.push_back(start);//加入start
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
            plan.push_back(goal);//加入goal
            publishPlan(plan);
            return true;

        };


    double BDastarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
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

    //Heuristic采用的欧几里德距离（直线距离）；
    double BDastarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        
        return abs(goalY - startY) + abs(goalX - startX);
    }

    bool BDastarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> BDastarPlanner::get_neighbors(int current_cell)
    {   
        vector<int> neighborIndexes;
        
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }
        return neighborIndexes;
    }


    void BDastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
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

    // float BDastarPlanner::f_value_front(int cell_index, int goal_index, float g_sfront){
    //     float f = g_sfront + getHeuristic(cell_index, goal_index);
    //     return f;
    // };

    // float BDastarPlanner::f_value_back(int cell_index, int goal_index, float g_sback){
    //     float f = g_sback + getHeuristic(cell_index, goal_index);
    //     return f;
    // };

};

bool operator <(const Node& x, const Node& y) {
  return x.cost < y.cost;
}