//include
#include "include.h"



#define tran_coef 2*PI*60 //转换系数（位移到圈数），60的单位是毫米

//坐标系定义
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
float displacementWheel[3];//轮子位移数组

//*********************************************
//计算位移
void CalculateWheelDisplacement(float *dpmarr,motor_measure_t *motor, float radius)
{
    dpmarr[0] = 2.0f * M_PI * radius * motor[0];
	  dpmarr[1] = 2.0f * M_PI * radius * motor[1];
	  dpmarr[2] = 2.0f * M_PI * radius * motor[2];
}
//计算机器人坐标位移
void rob_res_dpm(f_rob r1_rob,float *dpmarr){
  r1_rob.x = dpmarr[0]-dpmarr[1]*cos(2/3*PI)-dpmarr[3]*cos(2/3*PI);
  r1_rob.y = dpmarr[0]-dpmarr[1]*sin(2/3*PI)+dpmarr[3]*sin(2/3*PI);
}
//世界坐标系位移
void rob2world(f_world r1_world,f_rob r1_rob,float theta){
	r1_world.x=r1_rob.x*cos(theta);
	r1_world.y=r1_rob.y*sin(theta);
}
//**********************************************
void clac_dpm(f_rob f,