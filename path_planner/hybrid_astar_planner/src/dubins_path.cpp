#include <vector>
#include <cmath>
#include <string>

#include "dubins_path.h"


namespace Dubins_path{

std::pair<Eigen::MatrixXd, int> get_dubins_L(const Eigen::RowVector3d &vector_start, const Eigen::RowVector3d &vector_goal, const double &r){
    double dx = vector_goal(0) - vector_start(0);
    double dy = vector_goal(1) - vector_start(1);
    double d = sqrt(dx * dx + dy * dy) / r;

    double theta = fmod(atan2(dy, dx), 2 * M_PI);
    double alpha = fmod(vector_start(2) - theta, 2 * M_PI);
    double beta = fmod(vector_goal(2) - theta, 2 * M_PI);

    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(6, 4);
    Eigen::RowVector4d lsl = LSL(alpha, beta, d);
    Eigen::RowVector4d lsr = LSR(alpha, beta, d);
    Eigen::RowVector4d rsl = RSL(alpha, beta, d);
    Eigen::RowVector4d rsr = RSR(alpha, beta, d);
    Eigen::RowVector4d rlr = RLR(alpha, beta, d);
    Eigen::RowVector4d lrl = LRL(alpha, beta, d);
    L << lsl, lsr, rsl, rsr, rlr, lrl;

    double min_path_length = L.col(0).minCoeff();
    int ind = 0;
    for (int i = 0; i < 6; i++)
    {
        if(L(i,0)==min_path_length){
            ind = i;
            break;
        }
    }
    std::pair<Eigen::MatrixXd, int> res = std::make_pair(L, ind);
    return res;
}


/*  @brief 计算得到dubins path 的路点
    @param vector_start 起始点向量(x, y, direction)
    @param vector_goal 同上
    @param r （最小）转弯半径
    @param L 六种行进方法计算得到的信息矩阵,行：[total path length, part1 length, part2 length, part3 length]
                                        ,列：[[path1] [path2] [path3] [path4] [path5] [path6]]
    @param ind 路径代价最小path的行序列号
    @return 
 */
std::vector<std::vector<double>> get_dubins_path(const Eigen::RowVector3d &vector_start, 
                                                const Eigen::RowVector3d &vector_goal, 
                                                const double &r, const Eigen::MatrixXd &L, 
                                                const int &ind){
    //路经类型
    std::vector<std::string> types{"LSL", "LSR", "RSL", "RSR", "RLR", "LRL"};
    //坐标变换后的起点
    Eigen::RowVector3d p_start{0, 0, vector_start(2)};
    // char seg_type = types[ind][0];
    // 计算 路径中间点 mid1 mid2
    Eigen::RowVector3d mid1 = dubins_segment(L(ind, 1), p_start, types[ind][0]);
    Eigen::RowVector3d mid2 = dubins_segment(L(ind, 2), mid1, types[ind][1]);

    //对dubins路径进行离散
    std::vector<std::vector<double>> path;
    double interval_step = 0.05;//离散路径步长（原坐标系）
    int num_step = floor((L(ind, 0) * r) / interval_step);
    // std::cout << num_step << std::endl;

    for (int i = 0; i < num_step + 2;i++){
        double step = i * interval_step;
        double t = step / r;
        Eigen::RowVector3d end_pt;
        if (t < L(ind, 1))
        {
            end_pt = dubins_segment(t, p_start, types[ind][0]);
        }
        else if (t < L(ind,1) + L(ind,2)){
            end_pt = dubins_segment(t - L(ind,1), mid1, types[ind][1]);
        }
        else{
            end_pt = dubins_segment(t - L(ind,1) - L(ind,2), mid2, types[ind][2]);
        }
        std::vector<double> path_point(3);
        path_point[0] = end_pt(0) * r + vector_start(0);
        path_point[1] = end_pt(1) * r + vector_start(1);
        path_point[2] = fmod(end_pt(2), 2*M_PI);
        path.push_back(path_point);
    }
    // 显示路点
    // std::cout << path.size() << std::endl;
    // Eigen::MatrixXd path_matrix(path.size(), 3);
    // for (int i = 0; i < path.size();i++){
    //     path_matrix(i, 0) = path[i][0];
    //     path_matrix(i, 1) = path[i][1];
    //     path_matrix(i, 2) = path[i][2];
    // }
    // std::cout << path_matrix << std::endl;

    return path;
}

std::vector<std::vector<double>> compute_dubins_path_main(const Eigen::RowVector3d &vector_start,
                                                          const Eigen::RowVector3d &vector_goal, 
                                                          const double &r,
                                                          double &path_length){
    std::pair<Eigen::MatrixXd, int> L_and_ind = get_dubins_L(vector_start, vector_goal, r);
    // std::cout << " L: " << L_and_ind.first << std::endl;
    path_length = L_and_ind.first(L_and_ind.second, 0);
    // std::cout << " path_length : " << path_length << std::endl;
    return get_dubins_path(vector_start, vector_goal, r, L_and_ind.first, L_and_ind.second);
}

/*  @brief 计算dubins曲线各部分
    @param seg_param 线段参数，即路径归一化（缩放路径至转弯半径为1）后，对于LR型来说，为路径长度以及转角弧度，对S型来说为路径长度
    @param seg_init 线段起始点向量 (x, y, direction)
    @param seg_type 线段类型（R、S、L中的一种）
    @return seg_end 线段结束点坐标和方向
 */
Eigen::RowVector3d dubins_segment(const double &seg_param, const Eigen::RowVector3d &seg_init, const char &seg_type){
    Eigen::RowVector3d seg_end{0, 0, 0};
    if(seg_type == 'L'){
        seg_end(0) = seg_init(0) + sin(seg_init(2) + seg_param) - sin(seg_init(2));
        seg_end(1) = seg_init(1) - cos(seg_init(2) + seg_param) + cos(seg_init(2));
        seg_end(2) = seg_init(2) + seg_param;
    }
    else if (seg_type == 'R')
    {
        seg_end(0) = seg_init(0) - sin(seg_init(2) - seg_param) + sin(seg_init(2));
        seg_end(1) = seg_init(1) + cos(seg_init(2) - seg_param) - cos(seg_init(2));
        seg_end(2) = seg_init(2) - seg_param;
    }
    else if (seg_type == 'S')
    {
        seg_end(0) = seg_init(0) + cos(seg_init(2)) * seg_param;
        seg_end(1) = seg_init(1) + sin(seg_init(2)) * seg_param;
        seg_end(2) = seg_init(2);
    }
    return seg_end;
}

/*   @brief 计算LSL 
    @param alpha 起点方向角度;
    @param beta 终点方向角度
    @param d 起点和终点距离
    @return dubins path     */
Eigen::RowVector4d LSL(const double &alpha, const double &beta, const double &d){
    Eigen::RowVector4d L;
    double tmp0 = d + sin(alpha) - sin(beta);
    double p_squared = 2 + (d * d) - (2 * cos(alpha - beta)) + (2 * d * (sin(alpha) - sin(beta)));
    if(p_squared < 0){
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double tmp1 = atan2((cos(beta) - cos(alpha)), tmp0);
        double t = converse_2pi(fmod((-alpha + tmp1), 2 * M_PI));
        double p = sqrt(p_squared);
        double q = converse_2pi(fmod((beta - tmp1), 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}


Eigen::RowVector4d LRL(const double &alpha, const double &beta, const double &d){
    Eigen::RowVector4d L;
    double tmp_lrl = (6. - d * d + 2 * cos(alpha - beta) + 2 * d * (-sin(alpha) + sin(beta))) / 8;
    if(abs(tmp_lrl)>1){
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double p = converse_2pi(fmod((2 *M_PI - acos(tmp_lrl)), 2 * M_PI));
        double t = (-alpha + atan2(-cos(alpha) + cos(alpha), d + sin(alpha) - sin(beta)) + p / 2);
        t = fmod(t, 2 * M_PI);
        double q = converse_2pi(fmod(beta, 2 * M_PI) - alpha + fmod(2 * p, 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}


Eigen::RowVector4d LSR(const double &alpha, const double &beta, const double &d){
    Eigen::RowVector4d L;
    double p_squared = -2 + (d * d) + (2 * cos(alpha - beta)) + (2 * d * (sin(alpha) + sin(beta)));
    if(p_squared < 0){
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double p = sqrt(p_squared);
        double tmp2 = atan2((-cos(alpha) - cos(beta)), (d + sin(alpha) + sin(beta))) - atan2(-2.0, p);
        double t = converse_2pi(fmod((-alpha + tmp2), 2 * M_PI));
        double q = converse_2pi(fmod((-fmod(beta, 2 * M_PI) + tmp2), 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}


Eigen::RowVector4d RSR(const double &alpha, const double &beta, const double &d){
    Eigen::RowVector4d L;
    double tmp0 = d - sin(alpha) + sin(beta);
    double p_squared = 2 + (d * d) - (2 * cos(alpha - beta)) + (2 * d * (sin(beta) - sin(alpha)));
    if (p_squared < 0)
    {
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double tmp1 = atan2((cos(alpha) - cos(beta)), tmp0);
        double t = converse_2pi(fmod(alpha - tmp1, 2 * M_PI));
        double p = sqrt(p_squared);
        double q = converse_2pi(fmod(-beta + tmp1, 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}


/*
RLR
@param alpha 起点方向角度;
@param beta 终点方向角度
@param d 起点和终点距离
@return dubins path
 */
Eigen::RowVector4d RLR(const double &alpha, const double &beta, const double &d){
    double tmp_rlr = (6 - d * d + 2 * cos(alpha - beta) + 2 * d * (sin(alpha) - sin(beta))) / 8;
    Eigen::RowVector4d L;
    if (abs(tmp_rlr) > 1)
    {
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double p = converse_2pi(fmod(2 * M_PI - acos(tmp_rlr), 2 * M_PI));
        double t = converse_2pi(fmod((alpha - atan2(cos(alpha) - cos(beta), d - sin(alpha) + sin(beta)) + fmod(M_PI / 2, 2 * M_PI)), 2 * M_PI));
        double q = converse_2pi(fmod((alpha - beta - t + fmod(p, 2 * M_PI)), 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}

Eigen::RowVector4d RSL(const double &alpha, const double &beta, const double &d){

    Eigen::RowVector4d L;
    double p_squared = (d * d) - 2 + (2 * cos(alpha - beta)) - (2 * d * (sin(alpha) + sin(beta)));

    if (abs(p_squared) < 0)
    {
        L << INFINITY, INFINITY, INFINITY, INFINITY;
    }
    else{
        double p = sqrt(p_squared);
        double tmp2 = atan2((cos(alpha) + cos(beta)), (d - sin(alpha) - sin(beta))) - atan2(2.0, p);
        double t = converse_2pi(fmod(alpha - tmp2, 2 * M_PI));
        double q = converse_2pi(fmod(beta - tmp2, 2 * M_PI));
        // t = abs(t);
        // p = abs(p);
        // q = abs(q);
        L << (abs(t) + abs(p) + abs(q)), t, p, q;
    }
    return L;
}

double converse_2pi(const double &theta){
    double new_theta = theta;
    if(theta < 0){
        new_theta = 2 * M_PI + theta;
    }
    return new_theta;
}

bool test_include(){
    std::cout << " !include dubins path " << std::endl;
    return true;
}


void qp_test(){
    
}


template <typename T>
double compute_mod(const T &n1, const T &n2){
    return fmod(n1, n2);
}

}
