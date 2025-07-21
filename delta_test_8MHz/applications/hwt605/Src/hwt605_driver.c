#include "hwt605_driver.h"
#include <string.h>

// ���ھ��ָ��
static UART_HandleTypeDef *hwt605_huart;

// ���ֽڽ��ջ���
static uint8_t rx_byte;

// ����֡������
static uint8_t rx_buffer[HWT605_FRAME_LENGTH];
static uint8_t rx_index = 0;

// �洢���µ���̬������
static HWT605_Angle_t angle_data;

// ��ʼ�����������봮�ھ�������������ж�
void HWT605_Init(UART_HandleTypeDef *huart) {
    hwt605_huart = huart;
    HAL_UART_Receive_IT(hwt605_huart, &rx_byte, 1);  // ������һ���жϽ���
}

// HAL �ص��е�ͳһ�жϽ��մ���ӿ�
void HWT605_UART_IRQHandler(UART_HandleTypeDef *huart) {
    if (huart != hwt605_huart) return;  // �ж���Ŀ�괮��

    // ֡��ʼ�жϣ���ֹ��֡
    if (rx_index == 0 && rx_byte != HWT605_FRAME_HEADER) return;

    // ���ֽ����뻺��
    rx_buffer[rx_index++] = rx_byte;

    // ���һ֡��������
    if (rx_index == HWT605_FRAME_LENGTH) {
        rx_index = 0;  // �����±�

        // �ж�֡ID�Ƿ��ǽǶ�֡
        if (rx_buffer[1] == HWT605_ANGLE_FRAME_ID) {
            // ���ݵ�λ��ǰ����λ�ں�С�˸�ʽ
            int16_t raw_roll  = (rx_buffer[3] << 8) | rx_buffer[2];
            int16_t raw_pitch = (rx_buffer[5] << 8) | rx_buffer[4];
            int16_t raw_yaw   = (rx_buffer[7] << 8) | rx_buffer[6];

            // ת��Ϊ�Ƕ�ֵ����Χ ��180 ��
            angle_data.roll  = raw_roll / 32768.0f * 180.0f;
            angle_data.pitch = raw_pitch / 32768.0f * 180.0f;
            angle_data.yaw   = raw_yaw / 32768.0f * 180.0f;
        }
    }

    // ������һ���жϽ���
    HAL_UART_Receive_IT(hwt605_huart, &rx_byte, 1);
}

// ��ȡ�Ƕ����ݣ��ṹ����ʽ���أ�
HWT605_Angle_t HWT605_GetAngle(void) {
    return angle_data;
}
