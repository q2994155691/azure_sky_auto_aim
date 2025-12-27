// src/serial_driver_node.cpp
#include "rm_serial_driver/serial_driver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace rm_serial_driver {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
    : Node("rm_serial_driver", options) {
    
    RCLCPP_INFO(this->get_logger(), "初始化 RM Serial Driver...");
    
    // ========================================
    // 參數聲明
    // ========================================
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("bullet_speed", 28.0);
    this->declare_parameter("gravity", 9.8);
    this->declare_parameter("enable_ballistic", false);
    this->declare_parameter("debug_mode", false);
    
    // ========================================
    // 讀取參數
    // ========================================
    serial_port_name_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    bullet_speed_ = this->get_parameter("bullet_speed").as_double();
    double gravity = this->get_parameter("gravity").as_double();
    enable_ballistic_correction_ = this->get_parameter("enable_ballistic").as_bool();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "參數配置:");
    RCLCPP_INFO(this->get_logger(), "  串口: %s", serial_port_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  波特率: %d", baudrate_);
    RCLCPP_INFO(this->get_logger(), "  彈速: %.1f m/s", bullet_speed_);
    RCLCPP_INFO(this->get_logger(), "  彈道補償: %s", 
                enable_ballistic_correction_ ? "啟用" : "禁用");
    
    // ========================================
    // 初始化彈道解算器
    // ========================================
    ballistic_solver_ = std::make_unique<BallisticSolver>(bullet_speed_, gravity);
    
    // ========================================
    // 初始化TF
    // ========================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // ========================================
    // 創建訂閱者
    // ========================================
	target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
	    "/tracker/target", 
	    rclcpp::SensorDataQoS(),
	    std::bind(&SerialDriverNode::targetCallback, this, std::placeholders::_1));
    	cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
	    "/cmd_vel", 
	     10,
	     std::bind(&SerialDriverNode::cmdVelCallback, this, std::placeholders::_1));
    // ========================================
    // 創建發布者
    // ========================================
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
    
    // ========================================
    // 打開串口
    // ========================================
    try {
        serial_port_ = std::make_unique<serial::Serial>(
            serial_port_name_, 
            baudrate_, 
            serial::Timeout::simpleTimeout(100));
        
        if (!serial_port_->isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 無法打開串口: %s", 
                        serial_port_name_.c_str());
            RCLCPP_ERROR(this->get_logger(), "請執行: sudo chmod 666 %s", 
                        serial_port_name_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ 串口已打開: %s @ %d", 
                    serial_port_name_.c_str(), baudrate_);
        
        // 啟動接收線程
        receive_thread_ = std::thread(&SerialDriverNode::receiveLoop, this);
        
        // 启动发送线程 
        send_thread_ = std::thread(&SerialDriverNode::sendLoop, this);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ 串口初始化失敗: %s", e.what());
        return;
    }
    
    // ========================================
    // 啟動TF廣播定時器 (100Hz)
    // ========================================
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SerialDriverNode::broadcastTransform, this));
    
    RCLCPP_INFO(this->get_logger(), "✅ RM Serial Driver 初始化完成！");
}

SerialDriverNode::~SerialDriverNode() {
    RCLCPP_INFO(this->get_logger(), "正在關閉 RM Serial Driver...");
    
    running_ = false;
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    if (send_thread_.joinable()) {
    send_thread_.join();
    }
    
    if (serial_port_ && serial_port_->isOpen()) {
        serial_port_->close();
        RCLCPP_INFO(this->get_logger(), "串口已關閉");
    }
}

void SerialDriverNode::targetCallback(
    const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    
    // 檢查是否正在跟蹤
    if (!msg->tracking) {
        if (debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "未跟蹤目標，跳過");
        }
        return;
    }
    
    // ========================================
    // 1. TF坐標轉換
    // ========================================
    geometry_msgs::msg::PointStamped target_in_camera;
    target_in_camera.header = msg->header;
    target_in_camera.point = msg->position;
    
    geometry_msgs::msg::PointStamped target_in_gimbal;
    
    try {
        // 查詢TF變換（設置短超時，非阻塞）
        auto transform = tf_buffer_->lookupTransform(
            "gimbal_link", 
            msg->header.frame_id,
            tf2::TimePointZero,  // 獲取最新可用的
            tf2::durationFromSec(0.01));  // 只等10ms
        
        // 手動執行坐標變換
        tf2::doTransform(target_in_camera, target_in_gimbal, transform);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF轉換失敗 (%s -> gimbal_link): %s", 
            msg->header.frame_id.c_str(), ex.what());
        return;
    }
    
    // ========================================
    // 2. 彈道解算
    // ========================================
    double target_pitch, target_yaw;
    bool solve_success = false;
    
    if (enable_ballistic_correction_) {
        solve_success = ballistic_solver_->solve(
            target_in_gimbal.point, target_pitch, target_yaw);
        
        if (solve_success && debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), 
                "彈道補償: pitch=%.2f° yaw=%.2f° 飛行時間=%.3fs",
                target_pitch * 180.0 / M_PI,
                target_yaw * 180.0 / M_PI,
                ballistic_solver_->getFlyTime());
        }
    } else {
        // 不進行彈道補償，直接計算角度
        double rho = std::sqrt(
            target_in_gimbal.point.x * target_in_gimbal.point.x +
            target_in_gimbal.point.y * target_in_gimbal.point.y);
        
        if (rho > 0.01) {
            target_yaw = std::atan2(target_in_gimbal.point.y, target_in_gimbal.point.x);
            target_pitch = std::atan2(target_in_gimbal.point.z, rho);
            solve_success = true;
        }
    }
    
    if (!solve_success) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "彈道解算失敗");
        return;
    }



    
    // ========================================
    // 3. 轉換為度（C板使用度）
    // ========================================
    float pitch_deg = static_cast<float>(target_pitch * 180.0 / M_PI);
    float yaw_deg = static_cast<float>(target_yaw * 180.0 / M_PI);
    
    //remove before fly
    RCLCPP_INFO(this->get_logger(),
    "目標: (%.2f, %.2f, %.2f)m 距離%.2fm | 相對角度: pitch=%.2f° yaw=%.2f°",
    target_in_gimbal.point.x, target_in_gimbal.point.y, target_in_gimbal.point.z,
    std::sqrt(target_in_gimbal.point.x * target_in_gimbal.point.x +
              target_in_gimbal.point.y * target_in_gimbal.point.y +
              target_in_gimbal.point.z * target_in_gimbal.point.z),
    pitch_deg, yaw_deg);
    
    {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!has_data_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "尚未收到C板反饋");
        return;
    }
    
    // 累積角度 = 相對角度 + 當前累積角度（可能超過±360°）
    pitch_deg += static_cast<float>(current_pitch_ * 180.0 / M_PI);
    yaw_deg += static_cast<float>(current_yaw_ * 180.0 / M_PI);
    }

    // ========================================
    // 4. 打包並發送
    // ========================================
    int8_t fire_cmd = msg->tracking ? 1 : 0;
    
    RCLCPP_INFO(this->get_logger(),
    " pitch=%.2f° yaw=%.2f° fire=%d",
    pitch_deg, yaw_deg, fire_cmd);
    
    latest_gimbal_cmd_.pitch = pitch_deg;
    latest_gimbal_cmd_.yaw = yaw_deg;
    latest_gimbal_cmd_.fire = fire_cmd;
    has_gimbal_cmd_.store(true, std::memory_order_release);
}

void SerialDriverNode::receiveLoop() {
    RCLCPP_INFO(this->get_logger(), "接收線程已啟動");
    
    std::array<uint8_t, 16> buffer;
    size_t buffer_pos = 0;
    
    while (running_ && rclcpp::ok()) {
        try {
            // 逐字節讀取，尋找幀頭
            if (serial_port_->available() > 0) {
                uint8_t byte;
                serial_port_->read(&byte, 1);
                
                // 尋找幀頭 0x50
                if (buffer_pos == 0) {
                    if (byte == 0x50) {
                        buffer[buffer_pos++] = byte;
                    }
                } else {
                    buffer[buffer_pos++] = byte;
                    
                    // 收滿16字節後解析
                    if (buffer_pos == 16) {
                        PresentData data;
                        if (Protocol::parsePresentData(buffer, data)) {
                            // 轉換為弧度存儲
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            current_pitch_ = data.present_pitch * M_PI / 180.0;
                            current_yaw_ = data.present_yaw * M_PI / 180.0;
                            last_data_time_ = this->now();
                            has_data_ = true;
                                                        
                            // 發布JointState
                            auto joint_msg = sensor_msgs::msg::JointState();
                            joint_msg.header.stamp = this->now();
                            joint_msg.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
                            joint_msg.position = {current_yaw_, current_pitch_};
                            joint_state_pub_->publish(joint_msg);
                            
                            if (debug_mode_) {
                                RCLCPP_DEBUG(this->get_logger(), 
                                    "接收姿態: pitch=%.2f° yaw=%.2f°",
                                    data.present_pitch, data.present_yaw);
                            }
                        } else {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "CRC校驗失敗");
                        }
                        
                        buffer_pos = 0;  // 重置緩衝區
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "串口接收錯誤: %s", e.what());
            buffer_pos = 0;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "接收線程已停止");
}

void SerialDriverNode::broadcastTransform() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!has_data_) {
        return;
    }
    
    // 檢查數據超時 (1秒)
    if ((this->now() - last_data_time_).seconds() > 1.0) {
        return;
    }
    
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "gimbal_link";
    
    // 設置旋轉（注意：根據實際安裝情況調整軸順序）
    tf2::Quaternion q;
    q.setRPY(0, -current_pitch_, current_yaw_);
    transform.transform.rotation = tf2::toMsg(q);
    
    // 設置平移
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    tf_broadcaster_->sendTransform(transform);
}

void SerialDriverNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    latest_chassis_cmd_.vx = static_cast<float>(msg->linear.x);
    latest_chassis_cmd_.vy = static_cast<float>(msg->linear.y);
    latest_chassis_cmd_.wz = static_cast<float>(msg->angular.z);
    has_chassis_cmd_.store(true, std::memory_order_release);
}

void SerialDriverNode::sendLoop() {
    RCLCPP_INFO(this->get_logger(), "发送线程已启动");
    
    using namespace std::chrono;
    auto period = milliseconds(5);  // 200Hz
    auto next_time = steady_clock::now() + period;
    
    while (running_ && rclcpp::ok()) {
        // 发送云台指令（如果有）
        if (has_gimbal_cmd_.load(std::memory_order_acquire)) {
            auto packet = Protocol::packActionData(
                latest_gimbal_cmd_.pitch,
                latest_gimbal_cmd_.yaw,
                latest_gimbal_cmd_.fire
            );
            
            try {
                serial_port_->write(packet.data(), packet.size());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "云台指令发送失败: %s", e.what());
            }
        }
        
        // 发送底盘指令（如果有）
        if (has_chassis_cmd_.load(std::memory_order_acquire)) {
            auto packet = Protocol::packNavData(
                latest_chassis_cmd_.vx,
                latest_chassis_cmd_.vy,
                latest_chassis_cmd_.wz
            );
            
            try {
                serial_port_->write(packet.data(), packet.size());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "底盘指令发送失败: %s", e.what());
            }
        }
        
        // 精确定时
        std::this_thread::sleep_until(next_time);
        next_time += period;
    }
    
    RCLCPP_INFO(this->get_logger(), "发送线程已停止");
}

} // namespace rm_serial_driver

// ========================================
// ROS2組件註冊
// ========================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode)

