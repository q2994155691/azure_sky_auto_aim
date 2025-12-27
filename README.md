哨兵自瞄＋電控接口

<img width="1148" height="569" alt="image" src="https://github.com/user-attachments/assets/924dbc3e-fa17-4972-97c1-a2ed8a73cd41" />


发送到电控板：

    0x41 协议：云台绝对角度 + 射击指令（pitch, yaw, fire）

    0x03 协议：底盘速度指令（vx, vy, wz）

从电控板接收：

    0x50 协议：云台当前角度反馈（present_pitch, present_yaw）


单独启动串口驱动：

ros2 run rm_serial_driver rm_serial_driver_node

<img width="1080" height="421" alt="image" src="https://github.com/user-attachments/assets/59c2ad2c-3505-477e-bbc4-23c44cfbca9c" />


