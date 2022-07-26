# ROS_car_master 用于学习测试运动规划算法
====

###########功能包介绍 \
├── nav_demo //配置无人车运动导航规划\
&emsp; ├── launch 启动项\
&emsp; &emsp;  ├── nav05_path.launch 修改路径规划器配置\
├── path_planner //路径规划算法功能包\
└── urdf_car_gazebo //无人车模型

### 依赖项

gazebo
rviz
Eiegn库

### 启动仿真环境
rviz 和 gazebo联合仿真环境：
```bash 
roslaunch nav_demo nav07_gazebo_rviz.launch
```
由于无人车ufdf模型问题，导致模型会输出标准错误，但不影响算法测试运行，测试算法时，可以屏蔽标准输出错误：
```bash 
roslaunch nav_demo nav07_gazebo_rviz.launch 2>/dev/null
```

### 关于自定义规划算法（global）插件的使用
首先新建一个功能包：包含一个cpp文件，为规划算法的实现，算法需要实现一个类，并继承nav_core::BaseGlobalPlanner。具体细节参考 ：http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
在cpp文件中注册插件：
```cpp 
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
```
新建global_planner_plugin.xml 并修改；

在package.xml中注册插件；

在nav05_path.launch中加入插件：
```xml 
<param name="base_global_planner" value="global_planner/GlobalPlanner"/>
```

#### 算法实现的注意
1.流程上按照官方教程编写注册算法插件且nav05_path.launch也添加插件后，在rviz中的global path中不显示自定义的planner话题，可能的原因是算法没有初始化，没有也plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1)， 也就没有发布话题。
修改.h中的 bool initialized_  为 bool initialized_ = false ，使用bool initialized_则initialized_没有初始化导致出现bug，bool initialized_ = false使每次启动都会进行初始化；

2.算法出现死循环，需要注意变量的作用域是否正确。

### 新增dwa局部规划器

