#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "include.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;
	float Kf;//ǰ��ϵ��

    float max_out;  //������
    float max_iout; //���������
	float max_fout; //���ǰ�����

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
	float Fout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

}pid_type_def;

void PID_init(pid_type_def *pid, uint8_t mode, const float PID[4], float max_out, float max_iout,float max_fout);
extern float PID_calc(pid_type_def *pid, float ref, float set);
float vel_PID_calc(pid_type_def *pid,float ff, float ref, float set);
extern void PID_clear(pid_type_def *pid);

#endif



