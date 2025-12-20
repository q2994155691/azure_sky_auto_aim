// src/ballistic_solver.cpp
#include "rm_serial_driver/ballistic_solver.hpp"
#include <cmath>
#include <limits>

namespace rm_serial_driver {

bool BallisticSolver::solve(const geometry_msgs::msg::Point& target_pos,
                            double& pitch_out, double& yaw_out) {
    // 1. 計算yaw角（不需要補償）
    yaw_out = std::atan2(target_pos.y, target_pos.x);
    
    // 2. 計算水平距離
    double rho = calculateRho(target_pos);
    
    // 檢查距離有效性
    if (rho < 0.01) {  // 目標太近（< 1cm）
        return false;
    }
    
    // 3. 迭代計算pitch補償
    double temp_z = target_pos.z;
    double error_z = 999.0;
    const int max_iterations = 10;
    const double tolerance = 0.001;  // 1mm
    
    for (int i = 0; i < max_iterations; i++) {
        // 計算當前pitch角
        pitch_out = std::atan2(temp_z, rho);
        
        // 計算水平速度分量
        double v_horizontal = bullet_speed_ * std::cos(pitch_out);
        
        // 防止除零
        if (std::abs(v_horizontal) < 0.01) {
            return false;
        }
        
        // 計算飛行時間
        fly_time_ = rho / v_horizontal;
        
        // 計算實際彈道落點的z坐標
        // 使用簡化彈道方程：z = v0*sin(θ)*t - 0.5*g*t²
        double real_z = bullet_speed_ * std::sin(pitch_out) * fly_time_ 
                       - 0.5 * gravity_ * fly_time_ * fly_time_;
        
        // 計算誤差
        error_z = target_pos.z - real_z;
        temp_z += error_z;
        
        // 收斂判斷
        if (std::abs(error_z) < tolerance) {
            break;
        }
    }
    
    // 4. 檢查結果有效性
    if (std::isnan(pitch_out) || std::isnan(yaw_out) || 
        std::isinf(pitch_out) || std::isinf(yaw_out)) {
        return false;
    }
    
    // 檢查角度合理性（pitch: -90° ~ 90°）
    if (std::abs(pitch_out) > M_PI / 2.0) {
        return false;
    }
    
    // 檢查飛行時間合理性（< 5秒）
    if (fly_time_ < 0 || fly_time_ > 5.0) {
        return false;
    }
    
    return true;
}

double BallisticSolver::calculateRho(const geometry_msgs::msg::Point& point) const {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

} // namespace rm_serial_driver

