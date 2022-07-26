#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include <qpOASES.hpp>

using namespace qpOASES;
int main(int argc, char  *argv[]){
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    ROS_INFO("qptest");
    /* Setup data of first QP. */
	real_t H[2 * 2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1 * 2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

    QProblem example(2, 1);

    int_t nWSR = 10;
	example.init(H, g, A, lb, ub, lbA, ubA, nWSR);

	real_t xOpt[2];
	example.getPrimalSolution(xOpt);
    
	printf("\n xOpt = [ %e, %e ]; objVal = %e\n\n", xOpt[0], xOpt[1], example.getObjVal());

    return 0;
}