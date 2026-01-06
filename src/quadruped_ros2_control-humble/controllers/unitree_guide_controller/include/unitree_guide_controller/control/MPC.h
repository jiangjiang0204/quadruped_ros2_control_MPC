// unitree_guide_controller/control/MPC.h
#ifndef MPC_H
#define MPC_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "unitree_guide_controller/common/mathTypes.h"

class QuadrupedRobot;

class MPC { //定义MPC类
public:
    /**
     * @brief 构造函数
     * @param robot 机器人模型指针
     * @param dt MPC预测的时间步长 (s)
     * @param horizon 预测步数 (N)
     */
    MPC(const std::shared_ptr<QuadrupedRobot>& robot, double dt, int horizon);
    
    ~MPC() = default;

    /**
     * @brief 计算最优足端反力 (World Frame)
     * @param current_pos 当前机身位置 (World Frame)
     * @param current_vel 当前机身速度 (World Frame)
     * @param current_rpy 当前机身欧拉角 (Roll, Pitch, Yaw)
     * @param current_omega 当前机身角速度 (Body Frame对应的World投影, 或直接传入Gyro)
     * @param traj_ref 参考轨迹矩阵 [13 x horizon]
     * 每一列包含: [rpy(3), pos(3), omega(3), vel(3), g(1)]^T
     * @param feet_pos_body 足端相对于机身的位置 (Body Frame)
     * @param gait_table 步态表 [4 x horizon], 1表示支撑相, 0表示摆动相
     * @return Vec12 计算出的4条腿的力 [fx0, fy0, fz0, ..., fx3, fy3, fz3]
     */
    Vec12 calF(const Vec3 &current_pos, const Vec3 &current_vel, 
               const Vec3 &current_rpy, const Vec3 &current_omega,
               const Eigen::MatrixXd &traj_ref, 
               const Vec34 &feet_pos_body, 
               const Eigen::MatrixXi &gait_table);

private:
    /**
     * @brief 更新离散时间状态空间矩阵 Ad, Bd
     */
    void calcStateSpace(const Vec3 &rpy, const Vec34 &feet_pos_body);

    std::shared_ptr<QuadrupedRobot> robot_model_;

    double dt_;
    int horizon_;
    double mass_;
    Mat3 inertia_;
    
    // 权重矩阵对角线
    Eigen::Matrix<double, 13, 1> Q_diag_;
    Eigen::Matrix<double, 12, 1> R_diag_;

    // 离散状态空间方程: x(k+1) = Ad*x(k) + Bd*u(k)
    Eigen::Matrix<double, 13, 13> Ad_;
    Eigen::Matrix<double, 13, 12> Bd_;
    
    // QP 密集预测矩阵
    Eigen::MatrixXd A_qp_; // [13*N, 13]
    Eigen::MatrixXd B_qp_; // [13*N, 12*N]
    
    // qpOASES 求解器
    // 变量数: 12 * horizon (每步4条腿*3个力)
    // 约束数: 5 * 4 * horizon (摩擦锥4个 + Z轴力1个，每条腿5个约束)
    std::unique_ptr<qpOASES::SQProblem> solver_;
    bool solver_initialized_ = false;

    // 摩擦锥与力限制参数
    double mu_;       // 摩擦系数
    double f_min_;    // 最小支撑力 (N)
    double f_max_;  // 最大支撑力 (N)
};

#endif //MPC_H