# rm_serial_driver

RoboMaster C板串口通信驅動，實現姿態接收和目標控制發送。

## 功能特性

- ✅ USB虛擬串口通信（/dev/ttyACM0）
- ✅ 接收C板姿態數據（Pitch/Yaw）
- ✅ 發送視覺控制命令
- ✅ 彈道解算與補償
- ✅ TF坐標系廣播
- ✅ CRC32校驗

## 通信協議

### C板 → PC (PresentData, 16字節)

