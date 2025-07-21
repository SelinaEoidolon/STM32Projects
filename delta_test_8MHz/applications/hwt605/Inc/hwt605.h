#ifndef __HWT605_H
#define __HWT605_H

#include "main.h"

// 姿态角结构体，存储 Roll/Pitch/Yaw
typedef struct {
    float roll;
    float pitch;
    float yaw;
} HWT605_Data;

extern HWT605_Data imu_data;

// 解析数据帧函数
void HWT605_Parse(uint8_t *buf);

// 获取 Yaw 值
float HWT605_GetYaw(void);

#endif
