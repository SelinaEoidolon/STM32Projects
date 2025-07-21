#include "hwt605.h"

HWT605_Data imu_data;

// 解析角度数据帧（0x55 0x53 开头，共11字节）
void HWT605_Parse(uint8_t *buf) {
    if (buf[0] != 0x55 || buf[1] != 0x53) return;

    // 将原始数据转换为角度值（-180° ~ +180°）
    imu_data.roll  = (short)(buf[3] << 8 | buf[2]) / 32768.0f * 180.0f;
    imu_data.pitch = (short)(buf[5] << 8 | buf[4]) / 32768.0f * 180.0f;
    imu_data.yaw   = (short)(buf[7] << 8 | buf[6]) / 32768.0f * 180.0f;
}

// 获取当前航向角（Yaw）
float HWT605_GetYaw(void) {
    return imu_data.yaw;
}
