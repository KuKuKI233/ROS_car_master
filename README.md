# ROS_car_master
用于学习测试运动规划算法

启动 rviz 和 gazebo联合仿真环境：roslaunch nav_demo nav07_gazebo_rviz.launch
由于无人车ufdf模型问题，导致模型会输出标准错误，但不影响算法测试运行，测试算法时，可以屏蔽标准输出错误：roslaunch nav_demo nav07_gazebo_rviz.launch 2>/dev/null
