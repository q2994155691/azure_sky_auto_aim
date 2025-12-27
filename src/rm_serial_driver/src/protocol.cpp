// src/protocol.cpp
#include "rm_serial_driver/protocol.hpp"
#include <string>
#include <cstring>

namespace rm_serial_driver {

uint32_t Protocol::stm32_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    
    // 按4字節為單位處理（小端序）
    for (size_t i = 0; i < len; i += 4) {
        // 小端序讀取32位字
        uint32_t word = 0;
        for (size_t j = 0; j < 4 && (i + j) < len; j++) {
            word |= static_cast<uint32_t>(data[i + j]) << (j * 8);
        }
        
        crc ^= word;
        
        // 32次移位和異或
        for (int bit = 0; bit < 32; bit++) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ 0x04C11DB7;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

std::array<uint8_t, 16> Protocol::packActionData(
    float pitch, float yaw, int8_t fire_times) {
    
    std::array<uint8_t, 16> packet;
    
    // 填充數據結構
    packet[0] = 0x41;  // SOF 'A'
    packet[1] = static_cast<uint8_t>(fire_times);
    
    // 寫入pitch（小端序float）
    std::memcpy(&packet[2], &pitch, sizeof(float));
    
    // 寫入yaw（小端序float）
    std::memcpy(&packet[6], &yaw, sizeof(float));
    
    // reserved（小端序int16_t）
    int16_t reserved = 0;
    std::memcpy(&packet[10], &reserved, sizeof(int16_t));
    
    // 計算前12字節的CRC32
    uint32_t crc = stm32_crc32(packet.data(), 12);
    
    // 寫入CRC（小端序）
    std::memcpy(&packet[12], &crc, sizeof(uint32_t));
    
    return packet;
}

bool Protocol::parsePresentData(
    const std::array<uint8_t, 16>& buffer, PresentData& output) {
    
    // 檢查幀頭
    if (buffer[0] != 0x50) {
        return false;
    }
    
    // 驗證CRC
    if (!verifyCRC(buffer.data(), 16)) {
        return false;
    }
    
    // 解析數據（結構體直接拷貝）
    std::memcpy(&output, buffer.data(), sizeof(PresentData));
    
    return true;
}

bool Protocol::verifyCRC(const uint8_t* data, size_t len) {
    if (len < 16) {
        return false;
    }
    
    // 計算前12字節的CRC
    uint32_t calc_crc = stm32_crc32(data, 12);
    
    // 讀取接收到的CRC（小端序）
    uint32_t recv_crc;
    std::memcpy(&recv_crc, &data[12], sizeof(uint32_t));
    
    return calc_crc == recv_crc;
}

uint8_t Protocol::calculate_xor_checksum(const uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

std::vector<uint8_t> Protocol::packNavData(float vx, float vy, float wz) {
    std::vector<uint8_t> packet(18);
    
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x03;
    packet[3] = 12;
    
    std::memcpy(&packet[4], &vx, sizeof(float));
    std::memcpy(&packet[8], &vy, sizeof(float));
    std::memcpy(&packet[12], &wz, sizeof(float));
    
    uint8_t checksum = calculate_xor_checksum(&packet[2], 14);
    packet[16] = checksum;
    packet[17] = 0xFE;
    
    return packet;
}

} // namespace rm_serial_driver

