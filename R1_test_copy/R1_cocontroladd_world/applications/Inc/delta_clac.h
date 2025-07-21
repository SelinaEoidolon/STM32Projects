#include "math.h"
#include "include.h"
#define delta_PI 3.415926f


// 定义常数
extern const float under_R; // 电机半径（底座到电机中心的距离，单位：m）
extern const float move_r ; // 动平台半径，单位：m
extern const float u_L ; // 主动臂的长度，单位：m
extern const float a_l ; // 连杆的长度，单位：m

//丝杆
extern const float s_S ;//螺距
extern float delta_position[3];
extern float D_theta[3];//电机角度数组
extern float zero_theta;//归零角度

void get_delta_position(void);
float deltaLimitMax(float input);
extern void DeltaInversekinematic(float x, float y, float z, float *theta) ;
extern void move_delta(float theta , uint8_t theta_flag);











