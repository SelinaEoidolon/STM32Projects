#include "hwt605.h"

HWT605_Data imu_data;

// �����Ƕ�����֡��0x55 0x53 ��ͷ����11�ֽڣ�
void HWT605_Parse(uint8_t *buf) {
    if (buf[0] != 0x55 || buf[1] != 0x53) return;

    // ��ԭʼ����ת��Ϊ�Ƕ�ֵ��-180�� ~ +180�㣩
    imu_data.roll  = (short)(buf[3] << 8 | buf[2]) / 32768.0f * 180.0f;
    imu_data.pitch = (short)(buf[5] << 8 | buf[4]) / 32768.0f * 180.0f;
    imu_data.yaw   = (short)(buf[7] << 8 | buf[6]) / 32768.0f * 180.0f;
}

// ��ȡ��ǰ����ǣ�Yaw��
float HWT605_GetYaw(void) {
    return imu_data.yaw;
}
