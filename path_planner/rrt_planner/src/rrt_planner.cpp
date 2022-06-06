#include "rrt_planner.h"

#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)


namespace rrt_planner{
    RRTPlanner::RRTPlanner(){}
    RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        
        std::cout << "initialized_ ====== " << initialized_ << std::endl;
        
        if(!initialized_){

            //初始化
            ROS_INFO("Initializ first");
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            int max_no_information=0;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    // int state = costmap_2d::FREE_SPACE;
                    // int cost = static_cast<int>(costmap_->getCost(j, i));
                    //
                    
                    if (static_cast<int>(costmap_->getCost(j, i)) == costmap_2d::FREE_SPACE){
                        int index = costmap_->getIndex(j, i);
                        sampleable_space.push_back(index);
                        
                    }
                    else if(static_cast<int>(costmap_->getCost(j, i)) == costmap_2d::NO_INFORMATION){
                        max_no_information++;
                    }
                    // if(cost == -1){
                    //     std::cout << " -1 ";
                    // }
                }
            }
            sampleable_space_size = sampleable_space.size();
            std::cout << " !!! max_no_information " << max_no_information << std::endl;
            std::cout << "!!! size of sampleable_space: " << sampleable_space.size() << std::endl;

            frame_id_ = costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized_ = true;
            initialized_times++;     

        } 
        else{
            ROS_INFO("Warnning: This planner has already been initialized... doing nothing");
        }
    }

    

    bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(!initialized_){
            ROS_INFO("ERROR: The planner has not been initialized, please call initialize() to use the planner");
            
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                    goal.pose.position.x,goal.pose.position.y);
        plan.clear();

        double wx = start.pose.position.x;//起点坐标(世界坐标系)
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;//起点坐标(地图坐标系)
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);//start_index 序列号

        //创建根节点(起点)node
        Node start_node;
        start_node.wx = wx;
        start_node.wy = wy;
        Node *node_start_ptr = &start_node;
        start_node.parent_ptr = node_start_ptr;
        start_node.parent_index = -1;
        Node node_end; //预创建终点节点

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        double goal_wx = wx; //goal 的世界坐标
        double goal_wy = wy;

        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        // std::vector<int,std::pair<float, float>,int> node_list;
        std::vector<Node> node_list;
        //node_list中 只有 根节点(起点)的父节点为NULL,其他所有节点都有父节点
        node_list.emplace_back(std::move(start_node));

        

        bool find_path_flag = false;
        int i = 0;
        while (!find_path_flag && i<1000){
            i++;
            std::pair<double, double> random_point = generate_sample_node(goal_index, goal_wx, goal_wy);
            // std::cout << "node_rand:" << random_point.first << "," << random_point.second << std::endl;
            
            // Node node_near = find_node_near(random_point, node_list);
            // std::cout << " // &node_near:" << &node_near;
            
            // Node node_new = generate_node_new(node_near, random_point);
            // std::cout << " // &node_new:" << &node_new;
            // // std::cout << "//node_new:" << (node_new).wx << "," << (node_new).wy ;
            

            Node node_new = generate_node_near_new(random_point, node_list, goal_wx, goal_wy);
            // std::cout << "  new node_new ptr:  " << &node_new;
            // std::cout << "  new node_new.parent ptr:  " << node_new.parent_ptr;
            if(is_node_valid(node_new)){
                node_list.emplace_back(std::move(node_new));
            }

            double dist_node2goal = sqrt(pow(node_new.wx - goal_wx, 2) + pow(node_new.wy - goal_wy, 2));
            // std::cout << "     //dist_node2goal:" << dist_node2goal;
            // std::cout << " /x y:" << node_new.wx - goal_wx << "," << node_new.wy - goal_wy;
            if (dist_node2goal < 0.1)
            {

                find_path_flag= true;
                node_end = node_new;
                std::cout << "node_end "<< node_end.wx <<","<<node_end.wy<<","<<node_end.parent_index<< ","<< &node_end<< std::endl;
            }
            
            
        }

        if(start_index == goal_index){
            return false;
        }
            
        if(find_path_flag){
            // 找到可行路径

            // int num_nullptr = 0;
            // std::cout << " node_list size= "<< node_list.size() << std::endl;
            // for (int i = 0; i < node_list.size(); i++)
            // {
            //     if(node_list[i].parent_ptr==nullptr){
            //         num_nullptr++;
            //     }
            //     std::cout << "node "<< i<< ":" << node_list[i].wx <<","<<node_list[i].wy<<","<<node_list[i].parent_index<< ","<< &(node_list[i])<< std::endl;
            // }
            // std::cout << " num_nullptr = "<< num_nullptr << std::endl;

            std::vector<std::vector<double> > path_position;
            std::vector<double> xy = {node_end.wx, node_end.wy};
            std::cout << " node_end = " << node_end.wx << ","<<node_end.wy << std::endl;
            path_position.emplace_back(xy);
            
            int next_node_index = node_end.parent_index;
            // std::cout << " next_node_index = " << next_node_index << std::endl;
            while (next_node_index != -1)
            {   
                std::vector<double> xy = {node_list[next_node_index].wx, node_list[next_node_index].wy};

                path_position.emplace_back(xy);

                next_node_index = node_list[next_node_index].parent_index;
                
            }

            // std::cout << " path_position size() = "<< path_position.size() << std::endl;
            reverse(path_position.begin(), path_position.end());
            ros::Time plan_time = ros::Time::now();

            for (size_t i = 0; i < path_position.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                // double x, y;
                pose.pose.position.x = path_position[i][0];
                pose.pose.position.y = path_position[i][1];
                
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
            }
            plan.push_back(goal);

            // while ((next_node_ptr != node_start_ptr)&&(next_node_ptr != nullptr))
            // {
            //     xy = {next_node_ptr->wx, next_node_ptr->wy};
            //     std::cout << " next_node_ptr = "<< next_node_ptr<<","<< next_node_ptr->wx<< ","<<next_node_ptr->wy<< std::endl;
            //     path_position.emplace_back(xy);
            //     next_node_ptr = next_node_ptr->parent_ptr;
            //     std::cout << " next_node_ptr->parent_ptr = " << next_node_ptr->parent_ptr << std::endl;
            // }
            // xy = {node_start_ptr->wx, node_start_ptr->wy};
            // path_position.emplace_back(xy);

            publishPlan(plan);
            
            std::cout << "~~~~~~~~!!!!! Find a path!!!!!~~~~~~" << std::endl;

            return true;
        }
        else{
            std::cout << "~~~~~!!!!! Can't find a path!!!!!~~~~i= "<< i << std::endl;
            return false;
        }
    }

    bool RRTPlanner::is_node_valid(const Node &node){

        unsigned int mx, my;
        costmap_->worldToMap(node.wx, node.wy, mx, my);
        
        if (static_cast<int>(costmap_->getCost(mx, my)) == costmap_2d::FREE_SPACE){
            return true;
        }
        return false;
    };


    Node RRTPlanner::generate_node_near_new(const std::pair<double, double> &node_rand, std::vector<Node> &node_list, double &goal_wx, double &goal_wy){
        double min_dist = 10000;

        int node_near_index;
        double min_d_a, min_d_b, min_d_c;

        for (int i = 0; i < node_list.size(); i++){
            // std::cout << "$$node "<< i<< ":" << node_list[i].wx <<","<<node_list[i].wy<<","<<node_list[i].parent_index << ","<< &(node_list[i])<< std::endl;
            double d_a = node_rand.first - node_list[i].wx;
            double d_b = node_rand.second - node_list[i].wy;
            double d_c = sqrt(pow(d_a, 2) + pow(d_b, 2));

            if(d_c < min_dist){
                min_dist = d_c;
                node_near_index = i;
                min_d_c = d_c;
                min_d_a = d_a;
                min_d_b = d_b;
            }
        }
        double step = 0.08;
        double x = 0, y = 0;
        if(min_d_c){
            x = step * min_d_a / min_d_c;
            y = step * min_d_b / min_d_c;
        }
        Node node_new;
        node_new.wx = node_list[node_near_index].wx + x;
        node_new.wy = node_list[node_near_index].wy + y;
        node_new.parent_index = node_near_index;

        return node_new;
    }

    Node RRTPlanner::generate_node_new( Node &node_near, const std::pair<double, double> &node_rand){
        std::cout << "  new Node_near ptr:  " << &node_near;
        double step = 0.08;
        double x = 0, y = 0;
        double d_a = node_rand.first - node_near.wx;
        double d_b = node_rand.second - node_near.wy;
        double d_c = sqrt(pow(d_a, 2) + pow(d_b, 2));
        
        if(d_c){
            x = step * d_a / d_c;
            y = step * d_b / d_c;
        }
        Node node_new;
        node_new.wx = node_near.wx + x;
        node_new.wy = node_near.wy + y;
        node_new.parent_ptr = &node_near;
        std::cout << "  new node_new ptr:  " << &node_new;
        return node_new;
    };

    /*
       return：node_near_ptr 使用前需要检查是否为 nullptr;
    */
    Node RRTPlanner::find_node_near(const std::pair<double, double> &node_rand, std::vector<Node> &node_list){

        double min_dist = 10000;
        // double step = 0.08;
        auto node_near_ptr = node_list.begin();
        Node node_near;

        for (auto iter = node_list.begin(); iter < node_list.end(); iter++)
        {
            // std::cout << "// :" << (*iter).wx << "," << (*iter).wy ;
            double d_a = node_rand.first - (*iter).wx;
            double d_b = node_rand.second - (*iter).wy;
            double d_c = sqrt(pow(d_a, 2) + pow(d_b, 2));
            // Node *node_ptr = (*iter).parent_ptr;
            std::cout << "  1$$(*iter).parent_ptr:  " << (*iter).parent_ptr;
            if(d_c < min_dist){
                min_dist = d_c;
                // std::cout << "  1$$find Node_near ptr:  " << iter;
                
                node_near = (*iter);
                // std::cout << "  1$$find Node_near ptr:  " << (*iter);
                std::cout << "  2$$find Node_near ptr:  " << &node_near;
            }
            

        }
        // Node node_near = (*node_near_ptr);
        // std::cout << "  find Node_near ptr:  " << &node_near_ptr;
        // std::cout << "  find Node_near &:  " << &node_near;
        std::cout << "  3$$find Node_near ptr:  " << &node_near;
        return (node_near);
    };

    std::pair<double, double> RRTPlanner::generate_sample_node(int &goal_index, double &goal_wx, double &goal_wy){

        std::pair<double, double> random_point;
        int random_point_index = 0;
        double goal_sample_rate = 0.3;

        std::random_device rd;
        std::mt19937 gen(rd());

        //std::uniform_real_distribution<> 分布对象返回值的范围不包括上边界
        std::uniform_real_distribution<float> r_rate(0, 1);
        std::uniform_real_distribution<double> r_x(-8, 4);
        std::uniform_real_distribution<double> r_y(-4, 4);
        std::uniform_real_distribution<float> r_float(0, 0.05);
        double generate_rate = r_rate(gen);
        if(generate_rate>goal_sample_rate){
            // random sample
            
            double wx, wy;
            wx = r_x(gen);
            wy = r_y(gen);

            random_point.first = wx ;
            random_point.second = wy ;

            // std::cout << " /0/ "<< wx<<","<< wy; 
        }
        else{
            //向目标点采样
            random_point_index = goal_index;

            random_point.first = goal_wx;
            random_point.second = goal_wy;
            // std::cout << " /1/ ";
        }
        // std::cout << " node rand:: =" << random_point.first << "," << random_point.second << std::endl;
        return random_point;
    };

    void RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){

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
    };



}

