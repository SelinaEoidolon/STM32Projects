#include "include.h"
#include "delta_clac.h"

float delta_position[3]={0};
void get_delta_position(void)
{
	for(uint8_t i=0;i<3;++i)
	{
		
		delta_position[i]=receive_data[3+i];
		
	}
	
}

// 定义delta常数
const float under_R = 0.09857; // 电机半径（底座到电机中心的距离，单位：m）
const float move_r = 0.11;  // 动平台半径，单位：m
const float u_L = 0.192; // 主动臂的长度，单位：m
const float u_l = 0.242; // 连杆的长度，单位：m

//丝杆
const float s_S = 4.0f;//螺距


float D_theta[3];
float zero_theta = 0.0f;

float deltaLimitMax(float input){                          
        if (input > 90.0)     
        {                      
            return 90.0;      
        }                      
        else if (input < -15)  
        {                      
            return -15;       
        } 
        else {
	          return input;
        }  	
}
		
/**
 * Delta 机器人逆解算函数
 * 给定动平台的目标位置 (x, y, z)，计算出每个电机的角度 theta1, theta2, theta3
 *
 * @param x 目标位置的 x 坐标
 * @param y 目标位置的 y 坐标
 * @param z 目标位置的 z 坐标
 * @param theta1 电机1的角度（返回值）
 * @param theta2 电机2的角度（返回值）
 * @param theta3 电机3的角度（返回值）
 * @return true 如果计算成功，false 如果没有可行解
 */
 
void DeltaInversekinematic(float x, float y, float z, float *theta) {
    // 用于计算三个电机角度的方程中，电机之间的相位差
    float phi_list[3] = { 0, 2.0 / 3.0 * delta_PI, 4.0 / 3.0 * delta_PI };  // 三个电机安装位置的相位差（电机分别位于等边三角形的三个角）,下方开始逆时针排序

    // 用于存储每个电机计算出的角度
    float theta_list[3];

    // 遍历三个电机
    for (int i = 0; i < 3; i++) {
        float phi = phi_list[i]; // 当前电机的相位角度

        // 根据几何公式计算 a, b, c，作为计算角度的参数
        float a = 2 * u_L * (under_R - move_r - x * cosf(phi) - y * sinf(phi));
        float b = -2 * u_L * z;
        float c = x * x + y * y + z * z + u_L * u_L + (under_R - move_r) * (under_R - move_r) - u_l * u_l - 2 * (under_R - move_r) * (x * cosf(phi) + y * sinf(phi));

        // 根据逆解公式计算电机的角度
        theta_list[i] = (-b - sqrt(a * a + b * b - c * c)) / (c - a);
    }

    // 分别获取三个电机的角度
    theta[0] = deltaLimitMax(2 * atanf(theta_list[0]) * 180.0 / delta_PI);
    theta[1] = deltaLimitMax(2 * atanf(theta_list[1]) * 180.0 / delta_PI);
    theta[2] = deltaLimitMax(2 * atanf(theta_list[2]) * 180.0 / delta_PI);

    if (isnan(theta[0]) || isnan(theta[1]) || isnan(theta[2])) {
       theta[0] = zero_theta * 180.0 / delta_PI;
       theta[1] = zero_theta * 180.0 / delta_PI;
       theta[2] = zero_theta * 180.0 / delta_PI;  // 如果解无效，角度归零  
    }


}

//*********************************************************
/*执行代码

接受目标位置
计算电机角度
delta击球
DeltaInversekinematic(0.0f,0.0f,0.00f,D_theta);
CAN1_CMD_1(pid_call_1(-(D_theta[0]+78)/360*8191*3591/187,1),
           pid_call_1(-(D_theta[1]+78)/360*8191*3591/187,2),
           pid_call_1(-(D_theta[2]+78)/360*8191*3591/187,3),0);
设定延时0.1s
delta调零

*/

//*************************************************************************
//云台执行代码
/*
//
//接收适合击球角度
//击球
//
//
//
*/
void move_delta(float theta , uint8_t theta_flag){
	if(theta_flag == 1){
		//if可能需要多个if选择执行角度
		CAN2_CMD_1(0,pid_call_2(theta*8191*36,2),0,0);
	}
	else{
		//可能需要延时等待或者重新接收标志位
		CAN2_CMD_1(0,0,0,0);
	}
	
}//需要在执行后进行延时进行pid计算










