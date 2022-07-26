//1.优先位置
#include <ros/ros.h>
// #include <qpOASES.hpp>
//2.C系统文件

//3.C++系统文件
#include <iostream>

//4.其他库.h文件
#include <Eigen/Dense>
#include <Eigen/Core>

//5.项目内.h文件



namespace Dubins_path{

    std::pair<Eigen::MatrixXd, int> get_dubins_L(const Eigen::RowVector3d &vector_start, const Eigen::RowVector3d &vector_goal, const double &r);

    std::vector<std::vector<double>> get_dubins_path(const Eigen::RowVector3d &vector_start,
                                                     const Eigen::RowVector3d &vector_goal,
                                                     const double &r, const Eigen::MatrixXd &L,
                                                     const int &ind);

    std::vector<std::vector<double>> compute_dubins_path_main(const Eigen::RowVector3d &vector_start,
                                                              const Eigen::RowVector3d &vector_goal,
                                                              const double &r,
                                                              double &path_length);

    Eigen::RowVector3d dubins_segment(const double &seg_param, const Eigen::RowVector3d &seg_init, const char &seg_type);

    Eigen::RowVector4d LSL(const double &alpha, const double &beta, const double &d);

    Eigen::RowVector4d LRL(const double &alpha, const double &beta, const double &d);

    Eigen::RowVector4d LSR(const double &alpha, const double &beta, const double &d);

    Eigen::RowVector4d RSR(const double &alpha, const double &beta, const double &d);

    Eigen::RowVector4d RLR(const double &alpha, const double &beta, const double &d);

    Eigen::RowVector4d RSL(const double &alpha, const double &beta, const double &d);

    bool test_include();

    // (-pi,pi) 转换至 (0,2*pi)
    double converse_2pi(const double &theta);

    template <typename T>
    double compute_mod(const T &n1, const T &n2);

    void qp_test();
}
