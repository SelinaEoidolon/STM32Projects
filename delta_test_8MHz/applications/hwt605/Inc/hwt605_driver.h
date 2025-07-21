#ifndef __HWT605_DRIVER_H
#define __HWT605_DRIVER_H

#include "stdint.h"
#include "usart.h"  // ���� UART_HandleTypeDef ����

// === �궨�岿�� ===

// ����֡�̶���ʼͷ
#define HWT605_FRAME_HEADER     0x55

// ��̬������֡��ʶ
#define HWT605_ANGLE_FRAME_ID   0x53

// ��������֡����
#define HWT605_FRAME_LENGTH     11

// === ��̬�ǽṹ�� ===
typedef struct {
    float roll;    // ����ǣ��ȣ�
    float pitch;   // �����ǣ��ȣ�
    float yaw;     // ����ǣ��ȣ�
} HWT605_Angle_t;

// === �ӿ����� ===

// ��ʼ��ģ�飬���봮�ھ��
void HWT605_Init(UART_HandleTypeDef *huart);

// �����жϽ��մ����� HAL �ص��е��ã�
void HWT605_UART_IRQHandler(UART_HandleTypeDef *huart);

// ��ȡ���½Ƕ�����
HWT605_Angle_t HWT605_GetAngle(void);

#endif
