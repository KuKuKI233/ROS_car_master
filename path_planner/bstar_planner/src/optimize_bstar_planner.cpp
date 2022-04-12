#include "bstar_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(bstar_planner::BstarPlanner, nav_core::BaseGlobalPlanner)

namespace bstar_planner{
    BstarPlanner::BstarPlanner(){}
    
    BstarPlanner::BstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }
    
    void BstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);

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

            frame_id_ = costmap_ros->getGlobalFrameID();
            
            ros::NodeHandle private_nh("~/" + name);
            
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }
    
    bool BstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
          {
        
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
        
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                    goal.pose.position.x,goal.pose.position.y);
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);

        ROS_INFO("start point: %.2f, %.2f",start.pose.orientation.z,start.pose.orientation.w);
        geometry_msgs::PoseStamped ms=modify_start_orientation(start);
        // ROS_INFO("modified start : %.2f, %.2f",ms.pose.orientation.z,ms.pose.orientation.w);
        vector<float> XY_direction_start=orientation2XY(ms);
        ROS_INFO("start XY : %.2f, %.2f",XY_direction_start[0],XY_direction_start[1]);

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);
        
        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);
        vector<int> node_direction_x(map_size, -1);
        vector<int> node_direction_y(map_size, -1);

        multiset<Node> priority_costs;
        
        gCosts[start_index] = 0;
        
        Node currentNode;
        currentNode.index = start_index;
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);
        
        plan.clear();
        
        while(!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            //Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index){
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);
            // vector<int> neighborIndexes;
            // // cout<< "//"<<currentNode.index<<"//"<<start_index<<endl;
            // if(currentNode.index==start_index){
            //     cout<<"get_neighbors"<<endl;
            //     vector<int> neighborIndexes = get_neighbors(currentNode.index);
            // }            
            // else{
            //     cout<<"get_neighbors_by_dirXY"<<endl;
            //     int dx=node_direction_x[currentNode.index];
            //     int dy=node_direction_y[currentNode.index];
            //     vector<int> neighborIndexes = get_neighbors_by_dirXY(currentNode.index,dx,dy);
            // }
                
            
            for(int i = 0; i < neighborIndexes.size(); i++){
                //未加入过OPEN
                if(cameFrom[neighborIndexes[i]] == -1){
                    gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                    Node nextNode;
                    nextNode.index = neighborIndexes[i];
                    //nextNode.cost = gCosts[neighborIndexes[i]];    //Dijkstra Algorithm
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    
                    unsigned int x1, y1;
                    costmap_->indexToCells(cameFrom[neighborIndexes[i]], x1, y1);
                    unsigned int x2, y2;
                    costmap_->indexToCells(neighborIndexes[i], x2, y2);
                    node_direction_x[neighborIndexes[i]] = x2-x1;
                    node_direction_y[neighborIndexes[i]] = y2-y1;

                    priority_costs.insert(nextNode);
                }
                else{
                    // std::cout<< " begin change /" ;
                    // gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                    float old_cost= gCosts[neighborIndexes[i]];
                    float new_cost= gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);  
                    // 从priority_costs中找到index为neighborIndexes[i]的node
                    if(new_cost<old_cost){
                        Node change_node;
                        // change_node.cost=old_cost;
                        // change_node.index=neighborIndexes[i];
                        // priority_costs.erase(change_node);
                        change_node.cost=new_cost + getHeuristic(neighborIndexes[i], goal_index) ;
                        change_node.index=neighborIndexes[i];
                        cameFrom[neighborIndexes[i]] = currentNode.index;

                        unsigned int x1, y1;
                        costmap_->indexToCells(cameFrom[neighborIndexes[i]], x1, y1);
                        unsigned int x2, y2;
                        costmap_->indexToCells(neighborIndexes[i], x2, y2);
                        node_direction_x[neighborIndexes[i]] = x2-x1;
                        node_direction_y[neighborIndexes[i]] = y2-y1;
                        
                        gCosts[neighborIndexes[i]] = new_cost;
                           
                        priority_costs.insert(change_node);
                    }
                }

            }
        }
        
        if(cameFrom[goal_index] == -1){
            cout << "Goal not reachable, failed making a global path." << endl;
            return false;
        }
        
        if(start_index == goal_index)
            return false;
        //Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while(currentNode.index != start_index){
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
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
       
    }
    
    //修正start点姿态四元数不正确；
    geometry_msgs::PoseStamped BstarPlanner::modify_start_orientation(const geometry_msgs::PoseStamped& goal){
        geometry_msgs::PoseStamped modified_start_orientation;
        modified_start_orientation.pose.orientation.x=0.0;
        modified_start_orientation.pose.orientation.y=0.0;
        if((goal.pose.orientation.z>=(-sqrt(3)/2)) && (goal.pose.orientation.z<0) && (goal.pose.orientation.w>=0.5)){
            modified_start_orientation.pose.orientation.z=-goal.pose.orientation.z;
            modified_start_orientation.pose.orientation.w=-goal.pose.orientation.w;
        }
        else{
            modified_start_orientation.pose.orientation.z=goal.pose.orientation.z;
            modified_start_orientation.pose.orientation.w=goal.pose.orientation.w;
        }
        return modified_start_orientation;
    };

    vector<int> BstarPlanner::caculate_start_direction(float Dx,float Dy){

    };
    
    vector<float> BstarPlanner::orientation2XY(const geometry_msgs::PoseStamped& orientation){
        float theta = 2*acos(orientation.pose.orientation.w);
        vector<float> XY_direction{cos(theta),sin(theta)};
        //error: return vector<float> XY_direction{cos(theta),sin(theta)};
        //不能在return中定义；
        return XY_direction;
    };

    vector<float> BstarPlanner::W2XY(float w){
        float theta = 2*acos(w);
        vector<float> XY_direction{cos(theta),sin(theta)};
        return XY_direction;
    };

    double BstarPlanner::getMoveCost(int firstIndex, int secondIndex)
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
    
    bool BstarPlanner::is_line_Collision(int start_index, int end_index){
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(start_index, tmp1, tmp2);
        int start_x = tmp1, start_y = tmp2;
        costmap_->indexToCells(end_index, tmp1, tmp2);
        int end_x = tmp1, end_y = tmp2;
        
        //计算与横轴交点
        float intersection_x, intersection_y;
        float k = (end_y - start_y) / (end_x - start_x);
    
        for (int i = 0; i <(end_x-start_x); i++)
        {
            if (i == 0) {
                intersection_x = start_x + 0.5;
            }
            else
            {
                intersection_x = intersection_x + 1;
            }
            intersection_y = k * (intersection_x - start_x) + start_y;
            int x1 = floor(intersection_x);
            int x2 = x1+1;
            int y0= floor(intersection_y);
            int y1,y2;
            if ((intersection_y - y0) < 0.5) {
                //两点共享
                y1 = y0;
                if (!OGM[x1 * width + y1] || !OGM[x2 * width + y1])
                    return true;
            }
            else if ((intersection_y - y0) == 0.5)
            {   
                //四点共享
                y1 = y0; y2 = y0+1;  
                if (!OGM[x1 * width + y1] || !OGM[x2 * width + y1] || !OGM[x1 * width + y2] || !OGM[x2 * width + y2])
                    return true;              
            }           			
            else{ 
                //两点共享
                y1 = y0+1;
                if (!OGM[x1 * width + y1] || !OGM[x2 * width + y1])
                    return true;
            }         		            
        }
        return false;      
    }

    

    double BstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        
        return abs(goalY - startY) + abs(goalX - startX);
    }
    
    bool BstarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }
    
    vector<int> BstarPlanner::get_neighbors(int current_cell)
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

    vector<int> BstarPlanner::get_neighbors_by_dirXY(int current_cell, int &dir_x,int &dir_y){
        vector<int> neighborIndexes;
        vector<int> steer_x(3,0);
        vector<int> steer_y(3,0);
        if(dir_x!=0&&dir_y==0){
            steer_x[0]=dir_x;steer_y[0]=0;
            steer_x[1]=dir_x;steer_y[1]=1;
            steer_x[2]=dir_x;steer_y[2]=-1;
        }
        else if (dir_x==0&&dir_y!=0)
        {
            steer_x[0]=0;steer_y[0]=dir_y;
            steer_x[1]=1;steer_y[1]=dir_y;
            steer_x[2]=-1;steer_y[2]=dir_y;
        }
        else if (dir_x!=0&&dir_y!=0)
        {
            steer_x[0]=0;steer_y[0]=dir_y;
            steer_x[1]=dir_x;steer_y[1]=0;
            steer_x[2]=dir_x;steer_y[2]=dir_y;
        }
        printf("%d,%d//%d,%d//%d,%d",steer_x[0],steer_y[0],steer_x[1],steer_y[1],steer_x[2],steer_y[2]);

        for (int j = 0; j < 3; j++){
            unsigned tmp1, tmp2;
            costmap_->indexToCells(current_cell, tmp1, tmp2);
            int nextX = tmp1 + steer_x[j];
            int nextY = tmp2 + steer_y[j];
            int nextIndex = costmap_->getIndex(nextX, nextY);
            if(!( steer_x[j] == 0 && steer_y[j] == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
            {
                neighborIndexes.push_back(nextIndex);
            }
        }
        return neighborIndexes;
    };

    // vector<int> BstarPlanner::get_neighbors_by_direction(int current_cell, float node_w){
    //     vector<int> neighborIndexes;
    //     float theta = 2*acos(node_w);
    //     float node_dir_x=cos(theta);
    //     float node_dir_y=sin(theta);
    // };
    
    void BstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
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
  return x.cost < y.cost;
}
