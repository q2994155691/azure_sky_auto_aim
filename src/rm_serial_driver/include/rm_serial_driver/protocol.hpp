// include/rm_serial_driver/protocol.hpp
#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <cstring>


namespace rm_serial_driver {

// ========================================
// C板發送的姿態數據包 (16字節)
// ========================================
struct __attribute__((packed)) PresentData {
    uint8_t sof;              // 0x50 'P'
    int8_t fire_times;        // 射擊次數
    float present_pitch;      // 當前俯仰角（度）
    float present_yaw;        // 當前偏航角（度）
    int16_t reserved_slot;    // 保留字段（實際存血量）
    uint32_t crc32;           // CRC32校驗
};

// ========================================
// 發送給C板的控制命令包 (16字節)
// ========================================
struct __attribute__((packed)) ActionData {
    uint8_t sof;              // 0x41 'A'
    int8_t fire_times;        // 射擊命令
    float abs_pitch;          // 目標俯仰角（度）
    float abs_yaw;            // 目標偏航角（度）
    int16_t reserved_slot;    // 保留
    uint32_t crc32;           // CRC32校驗
};

// ========================================
// 協議處理類
// ========================================
class Protocol {
public:
    /**
     * @brief STM32 硬件 CRC32 算法（小端序）
     * @param data 數據指針
     * @param len 數據長度（必須是4的倍數）
     * @return CRC32值
     */
    static uint32_t stm32_crc32(const uint8_t* data, size_t len);
    
    /**
     * @brief 打包發送命令
     * @param pitch 目標俯仰角（度）
     * @param yaw 目標偏航角（度）
     * @param fire_times 射擊命令（0或1）
     * @return 16字節數據包
     */
    static std::array<uint8_t, 16> packActionData(
        float pitch, float yaw, int8_t fire_times = 0);
    
    /**
     * @brief 解析接收數據
     * @param buffer 16字節接收緩衝區
     * @param output 解析後的數據結構
     * @return 解析是否成功（幀頭正確且CRC校驗通過）
     */
    static bool parsePresentData(
        const std::array<uint8_t, 16>& buffer, PresentData& output);
    
    /**
     * @brief 驗證CRC校驗碼
     * @param data 數據指針（至少16字節）
     * @param len 數據長度
     * @return CRC是否正確
     */
    static bool verifyCRC(const uint8_t* data, size_t len);
};

} // namespace rm_serial_driver

