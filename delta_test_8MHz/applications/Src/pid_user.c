#include "pid_user.h"

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_speed_classic0_pid[3] = {20, 0.1, 0};//3508底盘0号电机参数
float motor_position_classic0_pid[3] = {0.2, 0.1, 0};
float motor_speed_classic12_pid[3] = {10, 0.05, 0};//3508底盘12号电机参数
float motor_position_classic12_pid[3] = {0.2, 0.1, 0};

float motor_speed_delta5_pid[3] = {10, 0.0, 0.05};//3508delta5参数
float motor_position_delta5_pid[3] = {0.9 , 0.0, 0.05};
float motor_speed_delta67_pid[3] = {10, 0.0, 0.05};//3508delta67参数
float motor_position_delta67_pid[3] = {0.9 , 0.0, 0.05};

//float motor_speed_delta5_pid[3] = {20, 0.2, 0.5};//3508delta5参数
//float motor_position_delta5_pid[3] = {0.9 , 0.0002, 0};
//float motor_speed_delta67_pid[3] = {20, 0.1, 0.5};//3508delta67参数
//float motor_position_delta67_pid[3] = {0.9 , 0.0001, 0};



float motor_speed_2006_pid[3] = {20, 0.3, 0.8};//2006参数
float motor_position_2006_pid[3] = {0.9, 0, 0.05};


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//PID初始化
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{
		if(i==0){//零号电机负载较大，需要单独调pid
			PID_init(&pid_v_1[i], PID_POSITION, motor_speed_classic0_pid, 16000, 6000);
		  PID_init(&pid_pos_1[i], PID_POSITION, motor_position_classic0_pid, 16000, 5000);
		
		  PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 8000);
		  PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 10000, 3000);
		}
		else{
      PID_init(&pid_v_1[i], PID_POSITION, motor_speed_classic12_pid, 10000, 6000);
		  PID_init(&pid_pos_1[i], PID_POSITION, motor_position_classic12_pid, 10000, 5000);
		
		  PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 15000, 8000);
		  PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 12000, 3000);
		}
	}
	
	for(int i=4;i<8;i++)
	{		
    if(i==4){
			PID_init(&pid_v_1[i], PID_POSITION, motor_speed_delta5_pid, 10000, 3000);
		  PID_init(&pid_pos_1[i], PID_POSITION, motor_position_delta5_pid, 6000, 1000);
		
		  PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 6000);
		  PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 400, 300);
		}
		else{
			PID_init(&pid_v_1[i], PID_POSITION, motor_speed_delta67_pid, 10000, 3000);//原输出限幅6000,8000
		  PID_init(&pid_pos_1[i], PID_POSITION, motor_position_delta67_pid, 6000, 1000);
		
		  PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 6000);
		  PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 400, 300);
		}
	}
}


float PID_velocity_realize_1(float set_speed,int i)//spped
{
		PID_calc(&pid_v_1[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{

		PID_calc(&pid_pos_1[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos_1[i-1].out;

}

float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}






float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_can2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}


