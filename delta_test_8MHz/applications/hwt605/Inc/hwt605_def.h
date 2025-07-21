#ifndef __HWT605_DEF_H
#define __HWT605_DEF_H

#include "stdint.h"

// ����֡��ʼ�ֽڣ��̶�Ϊ0x55��
#define HWT605_FRAME_HEADER     0x55

// ��̬������֡ ID���̶�Ϊ0x53��
#define HWT605_ANGLE_FRAME_ID   0x53

// HWT605 һ֡�����ܳ���Ϊ 11 �ֽ�
#define HWT605_FRAME_LENGTH     11

// ��̬�ǽṹ�壬��λΪ�Ƕȣ��ȣ�
typedef struct {
    float roll;    // �����
    float pitch;   // ������
    float yaw;     // �����
} HWT605_Angle_t;

#endif
