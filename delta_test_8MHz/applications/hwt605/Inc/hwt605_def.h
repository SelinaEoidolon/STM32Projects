#ifndef __HWT605_DEF_H
#define __HWT605_DEF_H

#include "stdint.h"

// 数据帧起始字节（固定为0x55）
#define HWT605_FRAME_HEADER     0x55

// 姿态角数据帧 ID（固定为0x53）
#define HWT605_ANGLE_FRAME_ID   0x53

// HWT605 一帧数据总长度为 11 字节
#define HWT605_FRAME_LENGTH     11

// 姿态角结构体，单位为角度（度）
typedef struct {
    float roll;    // 横滚角
    float pitch;   // 俯仰角
    float yaw;     // 航向角
} HWT605_Angle_t;

#endif
