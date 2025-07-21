#ifndef __HWT605_DRIVER_H
#define __HWT605_DRIVER_H

#include "stdint.h"
#include "usart.h"  // 包含 UART_HandleTypeDef 类型

// === 宏定义部分 ===

// 数据帧固定起始头
#define HWT605_FRAME_HEADER     0x55

// 姿态角数据帧标识
#define HWT605_ANGLE_FRAME_ID   0x53

// 完整数据帧长度
#define HWT605_FRAME_LENGTH     11

// === 姿态角结构体 ===
typedef struct {
    float roll;    // 横滚角（度）
    float pitch;   // 俯仰角（度）
    float yaw;     // 航向角（度）
} HWT605_Angle_t;

// === 接口声明 ===

// 初始化模块，传入串口句柄
void HWT605_Init(UART_HandleTypeDef *huart);

// 串口中断接收处理（在 HAL 回调中调用）
void HWT605_UART_IRQHandler(UART_HandleTypeDef *huart);

// 获取最新角度数据
HWT605_Angle_t HWT605_GetAngle(void);

#endif
