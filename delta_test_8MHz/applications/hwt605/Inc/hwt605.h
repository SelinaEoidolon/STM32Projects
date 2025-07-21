#ifndef __HWT605_H
#define __HWT605_H

#include "main.h"

// ��̬�ǽṹ�壬�洢 Roll/Pitch/Yaw
typedef struct {
    float roll;
    float pitch;
    float yaw;
} HWT605_Data;

extern HWT605_Data imu_data;

// ��������֡����
void HWT605_Parse(uint8_t *buf);

// ��ȡ Yaw ֵ
float HWT605_GetYaw(void);

#endif
