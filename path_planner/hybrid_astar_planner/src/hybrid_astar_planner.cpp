/* 
    存在问题：
    1.当机器人到达cost>0区域，无法规划出路径，即cost>0区域导致路径无效

 */

#include "hybrid_astat_planner.h"


#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAstarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner{

    HybridAstarPlanner::HybridAstarPlanner(){};
    HybridAstarPlanner::HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    };

    void HybridAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        std::cout << "initialized_ ====== " << initialized_ << std::endl;

        if(!initialized_){
            //初始化
            ROS_INFO("!!!!Initializ first!!!!");
            //必要的初始化项 开始
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);//查询地图 true cost=0; false cost>0;
            HasNode.resize(map_size); //栅格是否有节点占据
            Static_cost.resize(map_size);
            ROS_INFO("OGM size::::%zu", OGM.size());

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                    Static_cost[i * width + j] = cost;
                    if (cost == 0)
                        OGM[i * width + j] = true;
                    else {
                        OGM[i * width + j] = false;
                    }
                    HasNode[i * width + j] = false;
                }
            }

            //查询cost map上节点的cost
            //unsigned int cost = static_cast<int>(costmap_->getCost(j, i));

            frame_id_ = costmap_ros->getGlobalFrameID();
            // geometry_msgs::PoseStamped current_pose_;
            // costmap_ros_->getRobotPose(current_pose_);
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized_ = true;
            initialized_times++;
            //必要的初始化项 结束

            // ！！！！！！
            // 测试用例请至少置于必要初始化项之后进行

            //test1： dubins path 
            //test1 start
            //计算一次 dubins path 时间花费 0.0005s-0.001s
            Dubins_path::test_include(); //测试包含Dubins_path头文件是否可以使用
            ROS_INFO("begin:"); // 测试计算一次 dubins path
            Eigen::RowVector3d vector_start{0, 0, 2*M_PI / 2};
            Eigen::RowVector3d vector_goal{4, 0, 0*M_PI/2};
            // std::cout << Dubins_path::get_dubins_L(vector_start, vector_goal, 1).first << std::endl;
            // std::cout << "minimun path index: " << Dubins_path::get_dubins_L(vector_start, vector_goal, 1).second << std::endl;
            // ROS_INFO("end1:");
            double path_length;
            std::vector<std::vector<double>> dubins_path = Dubins_path::compute_dubins_path_main(vector_start, vector_goal, 1, path_length);
            bool flag = check_path_points_collied(dubins_path);

            std::cout << "check" << flag << std::endl;
            ROS_INFO("end2:");
            //test1 end


            //test start
            // ROS_INFO("qp start:");
            // // qp_test();
            // ROS_INFO("qp end");
            //tset end
        }

        else{
            ROS_INFO("Warnning: This planner has already been initialized... doing nothing");
        }
            
    }

    bool HybridAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(!initialized_){
            ROS_INFO("ERROR: The planner has not been initialized, please call initialize() to use the planner");
            
            return false;
        }

        // ROS_INFO("Got a start x y theta: %.2f, %.2f,%.2f", start.pose.position.x, start.pose.position.y, start.pose.orientation.w);
        // ROS_INFO("Got a goal x y theta: %.2f, %.2f,%.2f", goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w);
        double wx = start.pose.position.x;//起点坐标(世界坐标系)
        double wy = start.pose.position.y;
        double w_theta = get_theta(start.pose.orientation.w, start.pose.position.z);
        
        unsigned int start_x, start_y; //起点坐标(地图坐标系)
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);//start_index 序列号
        double goal_theta = converse_pi(get_theta(goal.pose.orientation.w, goal.pose.position.z));
        std::vector<double> vector_start{wx, wy, converse_pi(w_theta)};
        std::vector<double> vector_goal{goal.pose.position.x, goal.pose.position.y, 
                                        goal_theta};
        
        Eigen::RowVector3d RowVector3d_start{wx, wy, w_theta};
        Eigen::RowVector3d RowVector3d_goal{goal.pose.position.x, goal.pose.position.y,
                                            get_theta(goal.pose.orientation.w, goal.pose.position.z)};


        // // test4 thread: 双线程计算 h1 h2 失败
        // ROS_INFO("test4 thread start");
        // double path_length = 0;
        // std::thread th1(get_h1, vector_start, vector_goal, steer_radius, path_length);
        // std::thread th2(get_h2, RowVector3d_start, RowVector3d_goal, plan);
        // th1.join();
        // th2.join();
        // ROS_INFO("test4 thread end");
        // // test4 end


        // test 5 异步线程处理 h1 h2
        // test5 start:
        ROS_INFO("test4 thread start");
        //设置异步线程
        double path_length = 0;
        double a = 1;
        double b = 2;
        std::future<bool> get_h1_flag = std::async(&HybridAstarPlanner::get_h1, this,
                                               RowVector3d_start,
                                               RowVector3d_goal,
                                               steer_radius,
                                               path_length);
        //主线程
        ROS_INFO("main start");
        double h2 = 0.05*Astar_searching(vector_start, vector_goal, plan);
        ROS_INFO("main end");
        //检查异步线程是否完成
        ROS_INFO("async start");
        std::future_status status;
        do {
		status = get_h1_flag.wait_for(std::chrono::seconds(1));
            switch (status)
            {
                case std::future_status::deferred:
                    std::cout << "std::future_status::deferred" << std::endl;
                    break;
                case std::future_status::timeout:
                    std::cout << "std::future_status::timeout" << std::endl;
                    break;
                case std::future_status::ready:
                    std::cout << "std::future_status::ready" << std::endl;
                    break;
                default:
                    std::cout << "none status" << std::endl;
                break;
            }
        } while (status != std::future_status::ready);
        std::cout << "get_h1_flag : " << get_h1_flag.get() << std::endl;
        ROS_INFO("async end");
        ROS_INFO("test4 thread end");

        // test3 dubins_searching: 计算无碰撞有约束启发值h1
        // test3 start:
        std::cout << "test3 dubins_searching"  << std::endl;
        ROS_INFO("test3 dubins_searching start:");
        // double path_length = 0;
        std::vector<std::vector<double>> dubins_path =
            Dubins_path::compute_dubins_path_main(RowVector3d_start, RowVector3d_goal, steer_radius, path_length);
        if(path_length>0){
            std::cout << "Find a dubins path, h1 = " << path_length << std::endl;
        }
        
        bool flag = check_path_points_collied(dubins_path);
        std::cout << "check" << flag << std::endl;
        ROS_INFO("test3 dubins_searching end;");

        // test3 end;

        // test2 Astar_searching
        // test2 start:
        // std::cout << "start  test Astar_searching "  << std::endl;        
        // ROS_INFO("Astar_searching start time");
        // double h2 = 0.05*Astar_searching(vector_start, vector_goal, plan);
        // ROS_INFO("Astar_searching end time");
        if (h2 > 0)
        {
            std::cout << "Find a astar path, h2= " << h2 << std::endl;
            plan.push_back(goal);
            publishPlan(plan);
            return true;
        }
        return false;
        // test2 end;
    }

    double HybridAstarPlanner::add_a_and_b(const double a, const double b){
        double c = a + b;
        return c;
    }

    bool HybridAstarPlanner::get_h1(Eigen::RowVector3d vector_start,
                        Eigen::RowVector3d vector_goal, 
                        double r,
                        double path_length)
    {

        ROS_INFO("get_h1 start:");
        std::vector<std::vector<double>> dubins_path =
            Dubins_path::compute_dubins_path_main(vector_start, vector_goal, r, path_length);
        if(path_length>0){
            ROS_INFO("Find a dubins path, h1 = %.2f ",path_length);        
        }
        bool flag = check_path_points_collied(dubins_path);
        std::cout << "check" << flag << std::endl;
        ROS_INFO("get_h1 end;");
        return true;
    }

    void HybridAstarPlanner::get_h2(std::vector<double> vector_start, std::vector<double> vector_goal, std::vector<geometry_msgs::PoseStamped> plan){
        ROS_INFO("get_h2 start");
        double h2 = 0.05*Astar_searching(vector_start, vector_goal, plan);
        std::cout << "get_h2 h2 " << h2 << std::endl;
        ROS_INFO("get_h2 end end");
    }


    /* @brief 传统A*搜索，用与计算有碰撞无约束启发式；时间花费随规划路径长度，复杂度（二次方）上升
        @param vector_start 起始点向量
        @param vector_goal 目标点向量
        @return 有碰撞无约束启发式值h2
     */
    double HybridAstarPlanner::Astar_searching(const std::vector<double> &vector_start, 
                                                const std::vector<double> &vector_goal, 
                                                std::vector<geometry_msgs::PoseStamped>& plan)
    {

        //初始化start
        double wx = vector_start[0];
        double wy = vector_start[1];
        unsigned int start_x, start_y;//起点坐标(地图坐标系)
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);//start_index 序列号
        // std::cout << "start index" << start_index << std::endl;

        //初始化goal
        wx = vector_goal[0];
        wy = vector_goal[1];
        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        //OPEN CLOSE
        //维护三个OPEN，一个COSLE
        std::multimap<float, std::shared_ptr<Node>> OPEN_f;        
        std::multimap<float, std::shared_ptr<Node>> OPEN_g;
        std::multimap<int, std::shared_ptr<Node>> OPEN_index;
        std::multimap<int, std::shared_ptr<Node>> OPEN_parent_index;
        std::multimap<int, std::shared_ptr<Node>> CLOSE; // close 使用 index 排序

        //设置 goal_node_ptr
        std::shared_ptr<Node> goal_node_ptr;
        plan.clear();

        //设置起点node
        if(OPEN_f.empty()){
            Node current_node;
            current_node.index = start_index;
            current_node.parent_index = start_index;
            current_node.parent_node_ptr = nullptr;
            current_node.g_cost = 0;
            current_node.f_cost = current_node.g_cost + getHeuristic(start_index, goal_index);
            std::shared_ptr<Node> current_node_ptr = std::make_shared<Node>(current_node);
            // std::cout << "current_node_ptr1" << current_node_ptr << std::endl;
            //将起点(shared_ptr)加入OPEN
            OPEN_f.insert(std::make_pair(current_node.f_cost, current_node_ptr));
            OPEN_index.insert(std::make_pair(current_node.index, current_node_ptr));
            OPEN_g.insert(std::make_pair(current_node.g_cost, current_node_ptr));
            OPEN_parent_index.insert(std::make_pair(current_node.parent_index, current_node_ptr));
        }
        // std::cout << "current_node_ptr2" << OPEN_f.begin()->second << std::endl;
        // std::cout << "open f" <<OPEN_f.begin()->second->index << std::endl;

        int currentNode_index;
        float currentNode_f;
        float currentNode_g;
        int currentNode_parentIndex;
        std::shared_ptr<Node> currentNode_ptr;
        std::shared_ptr<Node> currentNode_parent_ptr;

        //astar
        // std::cout << "start while" << std::endl;
        ROS_INFO("while start: ");
        int num_t = 0;
        while (!OPEN_f.empty())
        {   
            // std::cout << "OPEN_f.size " << OPEN_f.size()  << std::endl;
            //取出f值最小的node，并从OPEN_f移除 ， 加入CLOSE
            currentNode_ptr = OPEN_f.begin()->second;
            currentNode_index = OPEN_f.begin()->second->index;
            // currentNode_f =  OPEN_f.begin()->second->f_cost;
            // currentNode_g =  OPEN_f.begin()->second->g_cost;
            // currentNode_parentIndex = OPEN_f.begin()->second->parent_index;
            // currentNode_parent_ptr = OPEN_f.begin()->second->parent_node_ptr;

            // std::cout << "currentNode_ptr"<< currentNode_ptr << ","<< currentNode_ptr->index << std::endl;
            // std::cout << "goal_index " << goal_index << std::endl;

            // find a path to goal
            
            if(currentNode_ptr->index == goal_index){
                // ROS_INFO("!!!!!!!!!!FIND GOAL!!!!!!!!!");
                //将 currentNode 加入 CLOSE
                goal_node_ptr = currentNode_ptr;
                CLOSE.insert(std::make_pair(currentNode_ptr->index, currentNode_ptr));
                break;
            }

            OPEN_f.erase(OPEN_f.begin());

            //多个f值可对应一个index，若OPEN_index_g中没有该index，说明该node已经在close中了
            auto InCLOSE = CLOSE.count(currentNode_index);
            if(InCLOSE){
                continue;
            }
            else{
                //将 currentNode 加入到 CLOSE；
                CLOSE.insert(std::make_pair(currentNode_index, currentNode_ptr));

                //将 currentNode 从 OPEN_index OPEN_g OPEN_parent_index 移除
                // OPEN_g.erase(OPEN_g.find(currentNode_g));
                OPEN_index.erase(OPEN_index.find(currentNode_index));
                // OPEN_parent_index.erase(OPEN_parent_index.find(currentNode_parentIndex));
            }

            
            std::vector<int> neighborIndexes = get_neighbors(currentNode_ptr->index);
            if(num_t == 0){
                ROS_INFO("for");
            }
            
            for (int i = 0; i < neighborIndexes.size(); i++)// 时间开销：0.0001s级
            {
                //检测neighbor 是否在OPEN中
                auto inOPEN_index = OPEN_index.count(neighborIndexes[i]);
                if(!inOPEN_index){

                    Node neighborNode;
                    neighborNode.index = neighborIndexes[i];
                    neighborNode.g_cost = currentNode_ptr->g_cost + Astar_searching_getMoveCost(currentNode_ptr->index, neighborNode.index);
                    neighborNode.f_cost = neighborNode.g_cost + getHeuristic(neighborNode.index, goal_index);
                    neighborNode.parent_index = currentNode_ptr->index;
                    neighborNode.parent_node_ptr = currentNode_ptr;
                    std::shared_ptr<Node> neighborNode_prt = std::make_shared<Node>(neighborNode);

                    // 将 neighborNode 加入 OPEN
                    // OPEN_g.insert(std::make_pair(neighborNode.g_cost, neighborNode_prt));
                    OPEN_f.insert(std::make_pair(neighborNode.f_cost, neighborNode_prt));
                    OPEN_index.insert(std::make_pair(neighborNode.index, neighborNode_prt));
                    // OPEN_parent_index.insert(std::make_pair(neighborNode.parent_index, neighborNode_prt));
                }
                
                else{
                    std::shared_ptr<Node> neighborNode_prt = OPEN_index.find(neighborIndexes[i])->second;
                    Node neighborNode = *(OPEN_index.find(neighborIndexes[i])->second);
                    auto old_neighborNode_ptr = OPEN_index.find(neighborNode_prt->index)->second;
                    // neighborNode.index = neighborIndexes[i];
                    float old_neighborNode_g_cost = old_neighborNode_ptr->g_cost;
                    float new_neighborNode_g_cost = currentNode_ptr->g_cost 
                                                    + Astar_searching_getMoveCost(currentNode_ptr->index, neighborNode_prt->index);
                    float old_neighborNode_f_cost = old_neighborNode_ptr->f_cost;
                    // neighborNode.g_cost = currentNode_ptr->g_cost + Astar_searching_getMoveCost(currentNode_ptr->index, neighborNode.index);
                    // neighborNode.f_cost = neighborNode.g_cost + getHeuristic(neighborNode.index, goal_index);
                    if(old_neighborNode_g_cost>new_neighborNode_g_cost){
                        neighborNode_prt->g_cost = new_neighborNode_g_cost;
                        neighborNode_prt->f_cost = neighborNode_prt->f_cost - (old_neighborNode_g_cost - new_neighborNode_g_cost);
                        neighborNode_prt->parent_index = currentNode_ptr->index;
                        neighborNode_prt->parent_node_ptr = currentNode_ptr;
                    }
                    //将原来的(f,neighborNode_prt)从OPEN_f中删除，并重新 将(new f,neighborNode_prt)加入OPEN_
                    auto iter_range = OPEN_f.equal_range(old_neighborNode_f_cost);
                    for (auto iter = iter_range.first; iter != iter_range.second; iter++){
                        if(iter->second->index == old_neighborNode_ptr->index){
                            OPEN_f.erase(iter);
                            // std::cout << " earse1 " << std::endl;
                            break;
                        }
                    }
                    OPEN_f.insert(std::make_pair(neighborNode_prt->f_cost, neighborNode_prt));
                }
            }
            if(num_t == 0){
                ROS_INFO("for");
            }
            num_t++;
        }
        ROS_INFO("while end ");
        
        //find a path
        if(!CLOSE.count(goal_index)){
            // std::cout << "Goal not reachable, failed making a global path." << std::endl;
            // ROS_INFO("Goal not reachable, failed making a global path.");
            return -1;
        }
        // ROS_INFO("Got a path to gaol.");
        if(start_index == goal_index){
            return 0;
        }

        // 有碰撞无约束启发式值h2
        double h2 = goal_node_ptr->g_cost;
        // ROS_INFO("prepare plan start");
        std::vector<int> bestPath;
        std::shared_ptr<Node> current_node_ptr = goal_node_ptr;
        while (current_node_ptr->index != start_index)
        {
            bestPath.push_back(current_node_ptr->index);
            current_node_ptr = current_node_ptr->parent_node_ptr;
        }


        reverse(bestPath.begin(), bestPath.end());
        // std::cout << "goal index" << goal_index << std::endl;
        // std::cout << "bestPath" << *(bestPath.end() - 1) << std::endl;
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size()-1; i++){
            // 只到 bestPath.size()-1， 最后加入
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
        
        
        // ROS_INFO("prepare plan end");
        return h2;
    }

    float HybridAstarPlanner::Astar_searching_getMoveCost(int firstIndex, int secondIndex){

        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        float cost_punishment = (2 * Static_cost[secondIndex] / 128) + 1;
        // if (Static_cost[secondIndex] > 1)
        // {
        //     cost_punishment = 2;
        // }

        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            ROS_INFO("ERROR!!!!! Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0*cost_punishment;
        else
            return 1.4*cost_punishment;
    }


    std::vector<int> HybridAstarPlanner::get_neighbors(int current_cell_index){
        std::vector<int> neighborIndexes;
        // ROS_INFO("get neighbors1");
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++){

                if((i==0 && j==0))
                    continue;
                else{

                    unsigned tmp1, tmp2;
                    costmap_->indexToCells(current_cell_index, tmp1, tmp2);//

                    int nextX = tmp1 + i;
                    int nextY = tmp2 + j;                    
                    int nextIndex=costmap_->getIndex(tmp1+i, tmp2+j);

                    // if((isInBounds(nextX, nextY)) && (OGM[nextIndex])){
                    //     neighborIndexes.push_back(nextIndex);
                    // }
                    if((isInBounds(nextX, nextY)) && (Static_cost[nextIndex]<128)){
                        neighborIndexes.push_back(nextIndex);
                    }
                    else{
                        continue;
                    }
                }
            }
        }
        // ROS_INFO("get neighbors2");
        return neighborIndexes;
    }

    bool HybridAstarPlanner::isInBounds(int x, int y){
        if(x<0 || y<0 || x>=height || y>=width){
            return false;
        }
        return true;
    }


    float HybridAstarPlanner::getHeuristic(int cell_index, int goal_index){
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
 
        //float h=abs(startX-goalX)+abs(startY-goalY)+(sqrt(2)-2)*(std::min(abs(startX-goalX),abs(startY-goalY)));
        float h=abs(startX-goalX)+abs(startY-goalY)+(1.4-2)*(std::min(abs(startX-goalX),abs(startY-goalY)));
        
        return h;
    }

    std::vector<std::vector<double>> HybridAstarPlanner::steer_nodes(std::vector<double> &current_node_state){
        // current_node_state: <x, y, theta>
        // neighbor_nodes_state
        std::vector<std::vector<double>> neighbor_nodes_states;
        //存储格式 <左<x, y, theta>, 前<x, y, theta>, 右<x, y, theta>>
        double s_r = steer_radius;

        // delta_theta delta_x delta_y 为相对于机器人自身坐标系的 变化量；
        double delta_theta = M_PI / 18;
        double delta_x = s_r - s_r * (cos(delta_theta));
        double delta_y = s_r * (sin(delta_theta));
        double delta_p = sqrt(delta_x * delta_x + delta_y * delta_y);
        
        //向左生长
        double new_node_x = current_node_state[0] + delta_p * cos(current_node_state[2] + delta_theta / 2);
        double new_node_y = current_node_state[1] + delta_p * sin(current_node_state[2] + delta_theta / 2);
        double new_node_theta = current_node_state[2] + delta_theta;
        std::vector<double> neighbor_state = {new_node_x, new_node_y, new_node_theta};
        neighbor_nodes_states.push_back(neighbor_state);

        //向前生长
        new_node_x = current_node_state[0] + 0.1 * cos(current_node_state[2]);
        new_node_y = current_node_state[1] + 0.1 * sin(current_node_state[2]);
        new_node_theta = current_node_state[2];
        neighbor_state = {new_node_x, new_node_y, new_node_theta};
        neighbor_nodes_states.push_back(neighbor_state);

        //向右生长
        new_node_x = current_node_state[0] + delta_p * cos(current_node_state[2] - delta_theta / 2);
        new_node_y = current_node_state[1] + delta_p * sin(current_node_state[2] - delta_theta / 2);
        new_node_theta = current_node_state[2] - delta_theta;
        neighbor_state = {new_node_x, new_node_y, new_node_theta};
        neighbor_nodes_states.push_back(neighbor_state);

        return neighbor_nodes_states;
    };

    bool HybridAstarPlanner::check_steer_node(int steer_flag, double node_x, double node_y,double new_node_x, double new_node_y){
        double check_x, check_y;
        check_x = node_x + (new_node_x - node_x) / 2;
        check_y = node_y + (new_node_y - node_y) / 2;
        unsigned int mx, my;
        costmap_->worldToMap(check_x, check_y, mx, my);
        
        int index = costmap_->getIndex(mx, my);

        if(OGM[index]){
            return true;
        }
        return false;
    };

    double HybridAstarPlanner::get_theta(double w, double z){
        double theta;
        if(z >= 0){
            theta = 2*acos(w);
        }
        else{
            theta = 2 * M_PI - 2*acos(w);
        }
        return theta;
    }

    double HybridAstarPlanner::converse_pi(const double &theta){
        double new_theta = theta;
        if (theta > M_PI)
        {
            new_theta -= M_PI;
        }
        return new_theta;
    }

    double HybridAstarPlanner::getMoveCost(const std::vector<double> &node1_state, const std::vector<double> &node2_state){

        double steer_angular_changge_punishment = 1.2;
        if (node1_state[2] != node2_state[2])
        {
            return steer_angular_changge_punishment * sqrt((node1_state[0] - node2_state[0]) * (node1_state[0] - node2_state[0]) + (node1_state[1] - node2_state[1]) * (node1_state[1] - node2_state[1]));
        }
        else{
            return sqrt((node1_state[0] - node2_state[0]) * (node1_state[0] - node2_state[0]) + (node1_state[1] - node2_state[1]) * (node1_state[1] - node2_state[1]));
        }
        

    }

    /* @brief 检测dubins path 是否发生碰撞，即路径点上是否cost > 0
        @param path dubins path
    */
    bool HybridAstarPlanner::check_path_points_collied(const std::vector<std::vector<double>> &path){
        // std::cout << path.size() << std::endl;
        for (int i = 0; i < path.size(); i++)
        {
            // std::cout << "world" << path[i][0] << "," << path[i][1] << std::endl;
            double wx = path[i][0];
            double wy = path[i][1];
            unsigned int mx, my;
            costmap_->worldToMap(path[i][0], path[i][1], mx, my);
            // std::cout << mx << "," << my << "," << OGM[mx * width + my] << std::endl;
            int index = costmap_->getIndex(mx, my);
            if (Static_cost[index]>127)
            {
                // cost > 0 ,return true
                std::cout << "collison world(x,y): " << path[i][0] << "," << path[i][1] << std::endl;
                std::cout << "collison map(x,y): " << mx << "," << my << "," << OGM[mx * width + my] << std::endl;
                return true;
            }
        }
        return false;
    }

    void HybridAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
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
    
}
