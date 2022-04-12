# ROS_car_master 用于学习测试运动规划算法
====

###########功能包介绍

├── nav_demo //配置无人车运动导航规划

├── path_planner //路径规划算法功能包

└── urdf_RosCar_gazebo //无人车模型



### 启动仿真环境
rviz 和 gazebo联合仿真环境：
```bash 
roslaunch nav_demo nav07_gazebo_rviz.launch
```
由于无人车ufdf模型问题，导致模型会输出标准错误，但不影响算法测试运行，测试算法时，可以屏蔽标准输出错误：
```bash 
roslaunch nav_demo nav07_gazebo_rviz.launch 2>/dev/null
```
