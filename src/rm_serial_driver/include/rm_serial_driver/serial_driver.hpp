// include/rm_serial_driver/serial_driver.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>

#include "auto_aim_interfaces/msg/target.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/ballistic_solver.hpp"

namespace rm_serial_driver {

/**
 * @brief RoboMaster 串口驅動節點
 * 
 * 功能：
 * 1. 訂閱視覺追蹤目標 (/tracker/target)
 * 2. 進行彈道補償計算
 * 3. 通過串口發送控制命令給C板
 * 4. 接收C板姿態數據
 * 5. 發布機器人狀態和關節狀態
 * 6. 廣播TF變換 (odom -> gimbal_link)
 */
class SerialDriverNode : public rclcpp::Node {
public:
    /**
     * @brief 構造函數
     * @param options 節點選項
     */
    explicit SerialDriverNode(const rclcpp::NodeOptions& options);
    
    /**
     * @brief 析構函數
     */
    ~SerialDriverNode();

private:
    // ========================================
    // 回調函數
    // ========================================
    
    /**
     * @brief 視覺目標回調函數
     * @param msg 視覺追蹤目標消息
     */
    void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
    
    /**
     * @brief 串口接收線程函數
     */
    void receiveLoop();
    
    /**
     * @brief 串口發送線程函數
     */
    void sendLoop();
    
    /**
     * @brief TF廣播定時器回調
     */
    void broadcastTransform();
    
    /**
    * @brief 底盘速度指令回调函数
    * @param msg 速度指令消息
    */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    
     // ========================================
    // 数据结构
    // ========================================
    struct GimbalCmd {
        float pitch;
        float yaw;
        int8_t fire;
    };
    
    struct ChassisCmd {
        float vx;
        float vy;
        float wz;
    };
    
    // ========================================
    // 串口相關
    // ========================================
    std::unique_ptr<serial::Serial> serial_port_;  ///< 串口對象
    std::thread receive_thread_;                   ///< 接收線程
    std::thread send_thread_;                      ///< 发送线程
    std::atomic<bool> running_{true};              ///< 運行標誌
    
    // ========================================
   // 发送线程相关
   // ========================================
    std::atomic<bool> has_gimbal_cmd_{false};
    std::atomic<bool> has_chassis_cmd_{false};
    GimbalCmd latest_gimbal_cmd_;
    ChassisCmd latest_chassis_cmd_;
    
    // ========================================
    // 彈道解算器
    // ========================================
    std::unique_ptr<BallisticSolver> ballistic_solver_;
    
    // ========================================
    // TF相關
    // ========================================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    
    // ========================================
    // ROS2訂閱者
    // ========================================
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // ========================================
    // ROS2發布者
    // ========================================
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // ========================================
    // 當前姿態數據（需線程安全）
    // ========================================
    std::mutex data_mutex_;
    double current_pitch_{0.0};      ///< 當前俯仰角（弧度）
    double current_yaw_{0.0};        ///< 當前偏航角（弧度）
    rclcpp::Time last_data_time_;    ///< 最後接收數據時間
    bool has_data_{false};           ///< 是否已接收到數據
    
    // ========================================
    // 參數
    // ========================================
    std::string serial_port_name_;           ///< 串口設備名
    int baudrate_;                           ///< 波特率
    double bullet_speed_;                    ///< 彈速 (m/s)
    bool enable_ballistic_correction_;       ///< 是否啟用彈道補償
    bool debug_mode_;                        ///< 調試模式
};

} // namespace rm_serial_driver

