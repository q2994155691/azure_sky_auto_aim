// include/rm_serial_driver/ballistic_solver.hpp
#pragma once

#include <cmath>
#include "geometry_msgs/msg/point.hpp"

namespace rm_serial_driver {

/**
 * @brief 簡化的彈道解算器
 * 
 * 使用迭代法計算重力補償，忽略空氣阻力。
 * 彈道方程：z = v0*sin(θ)*t - 0.5*g*t²
 */
class BallisticSolver {
public:
    /**
     * @brief 構造函數
     * @param bullet_speed 彈速 (m/s)
     * @param gravity 重力加速度 (m/s²), 默認9.8
     */
    explicit BallisticSolver(double bullet_speed = 28.0, double gravity = 9.8)
        : bullet_speed_(bullet_speed), gravity_(gravity), fly_time_(0.0) {}
    
    /**
     * @brief 計算補償後的云台角度
     * @param target_pos 目標在云台坐標系中的位置 (米)
     * @param pitch_out 輸出pitch角度（弧度）
     * @param yaw_out 輸出yaw角度（弧度）
     * @return 是否解算成功
     */
    bool solve(const geometry_msgs::msg::Point& target_pos, 
               double& pitch_out, double& yaw_out);
    
    /**
     * @brief 設置彈速
     * @param speed 彈速 (m/s)
     */
    void setBulletSpeed(double speed) { 
        bullet_speed_ = speed; 
    }
    
    /**
     * @brief 獲取當前彈速
     * @return 彈速 (m/s)
     */
    double getBulletSpeed() const { 
        return bullet_speed_; 
    }
    
    /**
     * @brief 設置重力加速度
     * @param g 重力加速度 (m/s²)
     */
    void setGravity(double g) { 
        gravity_ = g; 
    }
    
    /**
     * @brief 獲取最後一次計算的飛行時間
     * @return 飛行時間 (秒)
     */
    double getFlyTime() const { 
        return fly_time_; 
    }

private:
    /**
     * @brief 計算目標的水平距離
     * @param point 目標點
     * @return 水平距離 (米)
     */
    double calculateRho(const geometry_msgs::msg::Point& point) const;
    
    double bullet_speed_;  ///< 彈速 (m/s)
    double gravity_;       ///< 重力加速度 (m/s²)
    double fly_time_;      ///< 最後一次計算的飛行時間 (秒)
};

} // namespace rm_serial_driver

