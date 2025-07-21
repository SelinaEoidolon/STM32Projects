#include "hwt605_driver.h"
#include <string.h>

// 串口句柄指针
static UART_HandleTypeDef *hwt605_huart;

// 单字节接收缓存
static uint8_t rx_byte;

// 接收帧缓存区
static uint8_t rx_buffer[HWT605_FRAME_LENGTH];
static uint8_t rx_index = 0;

// 存储最新的姿态角数据
static HWT605_Angle_t angle_data;

// 初始化函数，传入串口句柄，启动接收中断
void HWT605_Init(UART_HandleTypeDef *huart) {
    hwt605_huart = huart;
    HAL_UART_Receive_IT(hwt605_huart, &rx_byte, 1);  // 启动第一次中断接收
}

// HAL 回调中的统一中断接收处理接口
void HWT605_UART_IRQHandler(UART_HandleTypeDef *huart) {
    if (huart != hwt605_huart) return;  // 判断是目标串口

    // 帧起始判断，防止丢帧
    if (rx_index == 0 && rx_byte != HWT605_FRAME_HEADER) return;

    // 逐字节填入缓存
    rx_buffer[rx_index++] = rx_byte;

    // 完成一帧后处理数据
    if (rx_index == HWT605_FRAME_LENGTH) {
        rx_index = 0;  // 重置下标

        // 判断帧ID是否是角度帧
        if (rx_buffer[1] == HWT605_ANGLE_FRAME_ID) {
            // 数据低位在前，高位在后，小端格式
            int16_t raw_roll  = (rx_buffer[3] << 8) | rx_buffer[2];
            int16_t raw_pitch = (rx_buffer[5] << 8) | rx_buffer[4];
            int16_t raw_yaw   = (rx_buffer[7] << 8) | rx_buffer[6];

            // 转换为角度值，范围 ±180 度
            angle_data.roll  = raw_roll / 32768.0f * 180.0f;
            angle_data.pitch = raw_pitch / 32768.0f * 180.0f;
            angle_data.yaw   = raw_yaw / 32768.0f * 180.0f;
        }
    }

    // 启动下一次中断接收
    HAL_UART_Receive_IT(hwt605_huart, &rx_byte, 1);
}

// 获取角度数据（结构体形式返回）
HWT605_Angle_t HWT605_GetAngle(void) {
    return angle_data;
}
