#include "include.h"

uint8_t hwt_tx_buffer[5]={0};
angle_PID_config angular_displacement_PID_config={50,0,0,200,500};//kp,ki,kd,max_iout,max_out//184输出对应1rad/s的角速度
angle_PID_config angular_velocity_PID_config={40,0,0,400,1200};
angle_PID_struct angular_displacement_PID_struct={0};
angle_PID_struct angular_velocity_PID_struct={0};

void hwt605_init(void)
{
	
	hwt_tx_buffer[0] = 0xff;
	hwt_tx_buffer[1] = 0xaa;
	hwt_tx_buffer[2] = 0x01;
	hwt_tx_buffer[3] = 0x08;
	hwt_tx_buffer[4] = 0x00;
	HAL_UART_Transmit_IT(&huart2,hwt_tx_buffer,5);//陀螺仪调零
	
	hwt_tx_buffer[0] = 0xff;
	hwt_tx_buffer[1] = 0xaa;
	hwt_tx_buffer[2] = 0x03;
	hwt_tx_buffer[3] = 0x02;
	hwt_tx_buffer[4] = 0x00;
	HAL_UART_Transmit_IT(&huart2,hwt_tx_buffer,5);//回传速率改为100hz
	
}

void hwt605_angle_reset()
{
	 uint8_t clear_angle_cmd[3] = {0xFF, 0xAA, 0x52}; // 角度清零
    HAL_UART_Transmit_IT(&huart2, clear_angle_cmd, 3);
	
}

float angular_displacement_PID(float angular_displacement)
{
	angular_displacement_PID_struct.error[1]=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.error[0]=angular_displacement-my_angle_measure.filtered_yaw;
	
	angular_displacement_PID_struct.se+=angular_displacement_PID_struct.error[0];
	angular_displacement_PID_struct.de=angular_displacement_PID_struct.error[0]-angular_displacement_PID_struct.error[1];
	
	angular_displacement_PID_struct.dout=angular_displacement_PID_config.kd*angular_displacement_PID_struct.de;
	angular_displacement_PID_struct.iout=angular_displacement_PID_config.ki*angular_displacement_PID_struct.se;
	angular_displacement_PID_struct.pout=angular_displacement_PID_config.kp*angular_displacement_PID_struct.error[0];
	
	
	if(angular_displacement_PID_struct.iout>angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=angular_displacement_PID_config.max_iout;
		
	}
	
	else if(angular_displacement_PID_struct.iout<-angular_displacement_PID_config.max_iout)
	{
		
		angular_displacement_PID_struct.iout=-angular_displacement_PID_config.max_iout;
		
	}
	
	angular_displacement_PID_struct.out=(angular_displacement_PID_struct.pout+angular_displacement_PID_struct.dout+angular_displacement_PID_struct.iout);
	
	if(angular_displacement_PID_struct.out>angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=angular_displacement_PID_config.max_out;
		
	}
	
	else if(angular_displacement_PID_struct.out<-angular_displacement_PID_config.max_out)
	{
		
		angular_displacement_PID_struct.out=-angular_displacement_PID_config.max_out;
		
	}
	
	return angular_displacement_PID_struct.out;
	
}

float angular_velocity_PID(float angular_velocity)
{
	
	angular_velocity_PID_struct.error[1]=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.error[0]=angular_velocity-my_angle_measure.filtered_yaw_omega;
	
	angular_velocity_PID_struct.se+=angular_velocity_PID_struct.error[0];
	angular_velocity_PID_struct.de=angular_velocity_PID_struct.error[0]-angular_velocity_PID_struct.error[1];
	
	angular_velocity_PID_struct.dout=angular_velocity_PID_config.kd*angular_velocity_PID_struct.de;
	angular_velocity_PID_struct.iout=angular_velocity_PID_config.ki*angular_velocity_PID_struct.se;
	angular_velocity_PID_struct.pout=angular_velocity_PID_config.kp*angular_velocity_PID_struct.error[0];
	
	if(angular_velocity_PID_struct.iout>angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=angular_velocity_PID_config.max_iout;
		
	}
	
	else if(angular_velocity_PID_struct.iout<-angular_velocity_PID_config.max_iout)
	{
		
		angular_velocity_PID_struct.iout=-angular_velocity_PID_config.max_iout;
		
	}
	
	angular_velocity_PID_struct.out=(angular_velocity_PID_struct.pout+angular_velocity_PID_struct.dout+angular_velocity_PID_struct.iout);
	
	if(angular_velocity_PID_struct.out>angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=angular_velocity_PID_config.max_out;
		
	}
	
	else if(angular_velocity_PID_struct.out<-angular_velocity_PID_config.max_out)
	{
		
		angular_velocity_PID_struct.out=-angular_velocity_PID_config.max_out;
		
	}
	
	return angular_velocity_PID_struct.out;
	
}

float hwt_angle_to_radian(float angle)
{
	
	return((angle*2*3.1415926535)/180);
	
}
