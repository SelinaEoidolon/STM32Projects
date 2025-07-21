#include "pid_user.h"

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_speed_classic0_pid[4] = {20, 0.1, 0,0.8};//3508����0�ŵ������
float motor_position_classic0_pid[4] = {0.2, 0.1, 0,0};
float motor_speed_classic12_pid[4] = {20, 0.1, 0,2.5};//3508����12�ŵ������10, 0.05, 0
float motor_position_classic12_pid[4] = {0.2, 0.1, 0};

float motor_speed_delta_pid[4] = {20, 0.2, 0.5,0};//3508delta����
float motor_position_delta_pid[4] = {0.9 , 0.0002, 0,0};


float motor_speed_2006_pid[4] = {20, 0.3, 0.8};//2006����
float motor_position_2006_pid[4] = {0.9, 0, 0.05};

//********************************************************
//delta   up or down
pid_type_def pid_v_up[8],pid_pos_up[8];
pid_type_def pid_v_down[8],pid_pos_down[8];

float delta_speed_up_pid[4] = {20, 0.2, 0.5,0};//up
float delta_position_up_pid[4] = {0.9 , 0.0002, 0,0};
float delta_speed_down_pid[4] = {10,0.1,0.2,0};//down
float delta_position_down_pid[4] = {0.45,0.0001,0,0};

void PID_delta_init(void){
    for(int i=0;i<4;i++){
        PID_init(&pid_v_up[i], PID_POSITION, delta_speed_up_pid, 16000, 6000,0);
        PID_init(&pid_pos_up[i], PID_POSITION, delta_position_up_pid, 16000, 5000,0);
        
        PID_init(&pid_v_down[i], PID_POSITION, delta_speed_down_pid, 10000, 6000,0);
        PID_init(&pid_pos_down[i], PID_POSITION, delta_position_down_pid, 6000, 2000,0);
    }
    for(int i=4;i<8;i++){
        if(i==4){
            PID_init(&pid_v_up[i], PID_POSITION, delta_speed_up_pid, 16000, 8000,0);
            PID_init(&pid_pos_up[i], PID_POSITION, delta_position_up_pid, 16000, 7000,0);
        
            PID_init(&pid_v_down[i], PID_POSITION, delta_speed_down_pid, 10000, 6000,0);
            PID_init(&pid_pos_down[i], PID_POSITION, delta_position_down_pid, 6000, 2000,0);
        }
        else{
            PID_init(&pid_v_up[i], PID_POSITION, delta_speed_up_pid, 16000, 7000,0);
            PID_init(&pid_pos_up[i], PID_POSITION, delta_position_up_pid, 16000, 6000,0);
        
            PID_init(&pid_v_down[i], PID_POSITION, delta_speed_down_pid, 10000, 6000,0);
            PID_init(&pid_pos_down[i], PID_POSITION, delta_position_down_pid, 6000, 2000,0);
        }
    
    }
}


float PID_delta_v_up(float set_speed,int i)
{
	PID_calc(&pid_v_up[i-1],motor_can1[i-1].speed_rpm , set_speed);
	return pid_v_up[i-1].out;
}

float PID_delta_pos_up(float set_pos,int i)
{
	PID_calc(&pid_pos_up[i-1],motor_can1[i-1].total_angle , set_pos);
	return pid_pos_up[i-1].out;
}

float PID_delta_call_up(float position,int i)
{
	return PID_delta_v_up(PID_delta_pos_up(position,i),i);
}

float PID_delta_v_down(float set_speed,int i)
{
	PID_calc(&pid_v_down[i-1],motor_can1[i-1].speed_rpm , set_speed);
	return pid_v_down[i-1].out;
}

float PID_delta_pos_down(float set_pos,int i)
{
	PID_calc(&pid_pos_down[i-1],motor_can1[i-1].total_angle , set_pos);
	return pid_pos_down[i-1].out;
}

float PID_delta_call_down(float position,int i)
{
	return PID_delta_v_down(PID_delta_pos_down(position,i),i);
}

//*********************************************************


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


//PID��ʼ��
void PID_devices_Init(void)
{
    for(int i=0;i<4;i++)
    {
        if(i==0){//��ŵ�����ؽϴ���Ҫ������pid
            PID_init(&pid_v_1[i], PID_POSITION, motor_speed_classic0_pid, 15000, 4000,0);
            PID_init(&pid_pos_1[i], PID_POSITION, motor_position_classic0_pid, 10000, 5000,0);
        
            PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 8000,500);
            PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 10000, 3000,500);
        }
        else{
			PID_init(&pid_v_1[i], PID_POSITION, motor_speed_classic12_pid, 10000, 6000,500);
			PID_init(&pid_pos_1[i], PID_POSITION, motor_position_classic12_pid, 10000, 5000,500);
        
			PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 15000, 8000,500);
			PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 12000, 3000,500);
        }
    }
    
    for(int i=4;i<8;i++)
    {        
		PID_init(&pid_v_1[i], PID_POSITION, motor_speed_delta_pid, 10000, 6000,500);
        PID_init(&pid_pos_1[i], PID_POSITION, motor_position_delta_pid, 10000, 5000,500);
        
        PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 6000,0);
        PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 400, 300,0);
    }
}

float PID_velocity_realize_1(float set_speed,int i)//spped
{
        vel_PID_calc(&pid_v_1[i-1],body_accs[i-1],motor_can1[i-1].speed_rpm , set_speed);
        return pid_v_1[i-1].out;
}

float PID_velocity_realize_1_nonfilter(float set_speed,int i)//spped
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
        return PID_velocity_realize_1_nonfilter(PID_position_realize_1(position,i),i);
}






float PID_velocity_realize_2(float set_speed,int i)
{
        PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
        return pid_v_2[i-1].out;
}

float PID_velocity_realize_2_nonfilter(float set_speed,int i)//spped
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
        return PID_velocity_realize_2_nonfilter(PID_position_realize_2(position,i),i);
}
