//include
#include "include.h"



#define tran_coef 2*PI*60 //ת��ϵ����λ�Ƶ�Ȧ������60�ĵ�λ�Ǻ���

//����ϵ����
struct f_rob{
	float x;
	float y;
}
struct f_world{
	float x;
	float y;
}

f_world r1_world;
f_rob r1_rob;
float displacementWheel[3];//����λ������

//*********************************************
//����λ��
void CalculateWheelDisplacement(float *dpmarr,motor_measure_t *motor, float radius)
{
    dpmarr[0] = 2.0f * M_PI * radius * motor[0];
	  dpmarr[1] = 2.0f * M_PI * radius * motor[1];
	  dpmarr[2] = 2.0f * M_PI * radius * motor[2];
}
//�������������λ��
void rob_res_dpm(f_rob r1_rob,float *dpmarr){
  r1_rob.x = dpmarr[0]-dpmarr[1]*cos(2/3*PI)-dpmarr[3]*cos(2/3*PI);
  r1_rob.y = dpmarr[0]-dpmarr[1]*sin(2/3*PI)+dpmarr[3]*sin(2/3*PI);
}
//��������ϵλ��
void rob2world(f_world r1_world,f_rob r1_rob,float theta){
	r1_world.x=r1_rob.x*cos(theta);
	r1_world.y=r1_rob.y*sin(theta);
}
//**********************************************
void clac_dpm(f_rob f,