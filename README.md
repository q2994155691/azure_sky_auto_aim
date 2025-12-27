哨兵自瞄＋電控接口
ros2_auto_aim_ws/src/
├── rm_vision/              # 视觉系统总入口
│   └── rm_vision_bringup/  # 一键启动包
│       └── launch/
│           └── vision_bringup.launch.py  # ← 启动自瞄 + 串口
│
├── rm_auto_aim/            # 装甲板自瞄
│   ├── armor_detector/     # 装甲板检测（深度学习）
│   └── armor_tracker/      # 目标跟踪（EKF）
│
├── rm_buff/                # 能量机关打击
│   ├── buff_detector/      # 大符检测
│   └── buff_tracker/       # 大符预测
│
├── rm_serial_driver/       # 串口驱动（与电控通信）← 您刚完成的
│   ├── src/
│   │   ├── serial_driver_node.cpp  # 主节点
│   │   ├── protocol.cpp            # 协议解析
│   │   └── ballistic_solver.cpp    # 弹道解算
│   ├── include/rm_serial_driver/
│   │   ├── serial_driver.hpp
│   │   ├── protocol.hpp
│   │   └── ballistic_solver.hpp
│   └── launch/
│       └── serial_driver.launch.py
│
├── ros2_hik_camera/        # 海康相机驱动
├── rm_gimbal_description/  # 云台 URDF 模型
└── serial/                 # 串口通信库（第三方）


<img width="1148" height="569" alt="image" src="https://github.com/user-attachments/assets/924dbc3e-fa17-4972-97c1-a2ed8a73cd41" />


发送到电控板：

    0x41 协议：云台绝对角度 + 射击指令（pitch, yaw, fire）

    0x03 协议：底盘速度指令（vx, vy, wz）

从电控板接收：

    0x50 协议：云台当前角度反馈（present_pitch, present_yaw）


单独启动串口驱动：

ros2 run rm_serial_driver rm_serial_driver_node

<img width="1080" height="421" alt="image" src="https://github.com/user-attachments/assets/59c2ad2c-3505-477e-bbc4-23c44cfbca9c" />


