#include "unitree_guide_controller/control/MPC.h"
#include <iostream>
#include <cmath>

using namespace qpOASES;

MPC::MPC(const std::shared_ptr<QuadrupedRobot>& robot, double dt, int horizon)
    : robot_model_(robot), dt_(dt), horizon_(horizon)
{
    if (!robot_model_) throw std::runtime_error("MPC received null robot model");

    mass_ = 15.097; 
    inertia_ << 0.14, 0,    0,
                0,    0.35, 0,
                0,    0,    0.37;

    Q_diag_.resize(13); // 确保 resize
    Q_diag_ << 20, 20, 20,       // RPY (角度误差权重)
               10, 10, 80,      // Pos (位置误差权重，Z轴给大点)
               0.5, 0.5, 0.5,    // Omega (角速度权重)
               0.5, 0.5, 0.5,    // Vel (线速度权重)
               0;                // Gravity               
               
    R_diag_.resize(12);
    R_diag_.setConstant(2e-3);

    // --- 内存预分配 (关键修复) ---
    A_qp_.resize(13 * horizon, 13);
    B_qp_.resize(13 * horizon, 12 * horizon);
    
    // 预分配成员变量 (需在 .h 中声明这些为 private 成员)
    // 如果你不想改 .h，暂时在这里保留局部变量也可以，但为了性能建议改 .h
    // 下面为了保证你的代码能跑，我暂时还是用局部变量，但处理了 traj_vec 的 bug
    
    f_min_ = 5.0; 
    f_max_ = 450.0; 
    mu_ = 0.6;      

    int n_vars = 12 * horizon; 
    int n_constrs = 5 * 4 * horizon; 
    
    try {
        solver_ = std::make_unique<SQProblem>(n_vars, n_constrs);
    } catch (const std::bad_alloc& e) {
        throw;
    }
    
    Options options;
    options.setToMPC();
    options.printLevel = PL_NONE;
    solver_->setOptions(options);
}

void MPC::calcStateSpace(const Vec3 &rpy, const Vec34 &feet_pos_body) {
    double yaw = rpy(2);
    Mat3 R_z;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;
           
    Mat3 I_world = R_z * inertia_ * R_z.transpose();
    Mat3 I_inv = I_world.inverse();

    Eigen::Matrix<double, 13, 13> Ac = Eigen::Matrix<double, 13, 13>::Zero();
    Eigen::Matrix<double, 13, 12> Bc = Eigen::Matrix<double, 13, 12>::Zero();

    Ac.block<3, 3>(0, 6) = R_z.transpose(); 
    Ac.block<3, 3>(3, 9) = Mat3::Identity(); 
    Ac(11, 12) = 1.0; 

    for (int i = 0; i < 4; ++i) {
        Vec3 r_body = feet_pos_body.col(i);
        Vec3 r_world = R_z * r_body; // 这里的变换是正确的

        Mat3 r_skew;
        r_skew << 0, -r_world(2), r_world(1),
                  r_world(2), 0, -r_world(0),
                  -r_world(1), r_world(0), 0;
        
        Bc.block<3, 3>(6, i * 3) = I_inv * r_skew; 
        Bc.block<3, 3>(9, i * 3) = Mat3::Identity() / mass_; 
    }

    Ad_ = Eigen::Matrix<double, 13, 13>::Identity() + Ac * dt_;
    Bd_ = Bc * dt_;
}

Vec12 MPC::calF(const Vec3 &current_pos, const Vec3 &current_vel, 
                const Vec3 &current_rpy, const Vec3 &current_omega,
                const Eigen::MatrixXd &traj_ref, 
                const Vec34 &feet_pos_body, 
                const Eigen::MatrixXi &gait_table)
{
    // 1. 更新动力学
    calcStateSpace(current_rpy, feet_pos_body);

    // 2. 构建预测矩阵
    Eigen::Matrix<double, 13, 13> A_pow = Ad_;
    for(int i = 0; i < horizon_; ++i) {
        A_qp_.block<13, 13>(i*13, 0) = A_pow;
        A_pow = Ad_ * A_pow;
    }

    // 这里其实可以优化，不用每次 setZero，但为了逻辑清晰先保留
    B_qp_.setZero();
    for(int r = 0; r < horizon_; ++r) {
        Eigen::Matrix<double, 13, 12> tmp_ABd = Bd_;
        for(int c = r; c < horizon_; ++c) {
            B_qp_.block<13, 12>(c*13, r*12) = tmp_ABd;
            tmp_ABd = Ad_ * tmp_ABd;
        }
    }

    // 3. 构建 Hessian 和 Gradient
    Eigen::VectorXd L_diag(13 * horizon_); 
    Eigen::VectorXd K_diag(12 * horizon_); 
    for(int i=0; i<horizon_; ++i) {
        L_diag.segment(i*13, 13) = Q_diag_;
        K_diag.segment(i*12, 12) = R_diag_;
    }

    Eigen::MatrixXd B_T_L = B_qp_.transpose();
    for(int i=0; i<13*horizon_; ++i) {
        B_T_L.col(i) *= L_diag(i); 
    }
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = 2 * (B_T_L * B_qp_);
    for(int i=0; i<12*horizon_; ++i) {
        H(i, i) += 2 * K_diag(i); 
    }

    // --- [核心修复] 构建参考轨迹向量 ---
    Eigen::VectorXd traj_vec(13 * horizon_);
    for(int i=0; i<horizon_; ++i) {
        // 安全检查：只读取 traj_ref 的前 12 行
        // traj_ref 通常是 [12 x horizon]
        traj_vec.segment(i*13, 12) = traj_ref.col(i).head(12);
        
        // 手动把第 13 个状态 (重力状态) 设为 -9.81
        // 注意：这个值必须和你的 Ac(11, 12)=1.0 逻辑自洽
        // 如果你的状态定义是 [..., g]，且 Ac 中 g 的系数是 1，则此处期望值应为 -9.81 或 0
        // 通常在 condensed MPC 中，我们将 g 视为常量状态，其期望值就是其本身
        traj_vec(i*13 + 12) = -9.81; 
    }

    Eigen::Matrix<double, 13, 1> x0;
    // 确保 x0 的第 13 项也是 -9.81，这样初始误差为 0
    x0 << current_rpy, current_pos, current_omega, current_vel, -9.81; 
    
    Eigen::VectorXd error_term = A_qp_ * x0 - traj_vec;
    Eigen::VectorXd g_vec = 2 * B_T_L * error_term;

    // 4. 构建约束
    int n_constrs = 5 * 4 * horizon_;
    int n_vars = 12 * horizon_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_constr = Eigen::MatrixXd::Zero(n_constrs, n_vars);
    std::vector<double> lbA(n_constrs), ubA(n_constrs);
    
    int cnt = 0;
    for(int i=0; i<horizon_; ++i) {
        for(int leg=0; leg<4; ++leg) {
            int idx_u = i*12 + leg*3; 
            bool contact = gait_table(leg, i) == 1;
            
            // Friction Cone
            A_constr(cnt, idx_u) = 1.0; A_constr(cnt, idx_u+2) = -mu_;
            lbA[cnt] = -1e10; ubA[cnt] = 0.0; cnt++;
            
            A_constr(cnt, idx_u) = -1.0; A_constr(cnt, idx_u+2) = -mu_;
            lbA[cnt] = -1e10; ubA[cnt] = 0.0; cnt++;
            
            A_constr(cnt, idx_u+1) = 1.0; A_constr(cnt, idx_u+2) = -mu_;
            lbA[cnt] = -1e10; ubA[cnt] = 0.0; cnt++;

            A_constr(cnt, idx_u+1) = -1.0; A_constr(cnt, idx_u+2) = -mu_;
            lbA[cnt] = -1e10; ubA[cnt] = 0.0; cnt++;

            // Fz Limit
            A_constr(cnt, idx_u+2) = 1.0;
            if(contact) {
                lbA[cnt] = f_min_; ubA[cnt] = f_max_;
            } else {
                // 摆动相强制为 0
                lbA[cnt] = 0.0; ubA[cnt] = 0.0;
            }
            cnt++;
        }
    }

    // 5. 求解
    int nWSR = 100;
    if(!solver_initialized_) {
        solver_->init(H.data(), g_vec.data(), A_constr.data(), 
                      nullptr, nullptr, lbA.data(), ubA.data(), nWSR);
        solver_initialized_ = true;
    } else {
        solver_->hotstart(H.data(), g_vec.data(), A_constr.data(), 
                          nullptr, nullptr, lbA.data(), ubA.data(), nWSR);
    }

    std::vector<double> q_sol(n_vars);
    solver_->getPrimalSolution(q_sol.data());

    Vec12 f_out;
    for(int i=0; i<12; ++i) f_out(i) = q_sol[i];
    
    return f_out;
}