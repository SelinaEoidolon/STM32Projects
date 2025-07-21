#include "include.h"
#include "displacement_PID.h"

float motors_rnd[3]={0};
displacement_PID my_displacement_pid;
float displace_buffer[3];

void get_rnd_count_and_diaplacement(void)
{
	
	motors_rnd[0]=motor_can1[0].total_angle-motor_can1[0].offset_angle+motor_can1[0].angle;
	motors_rnd[1]=motor_can1[1].total_angle-motor_can1[1].offset_angle+motor_can1[1].angle;
	motors_rnd[2]=motor_can1[2].total_angle-motor_can1[2].offset_angle+motor_can1[2].angle;
	
	motors_rnd[0]=motors_rnd[0]*((2*PI/8192.0)*(187.0/3591.0)*76);
	motors_rnd[1]=motors_rnd[1]*((2*PI/8192.0)*(187.0/3591.0)*76);
	motors_rnd[2]=motors_rnd[2]*((2*PI/8192.0)*(187.0/3591.0)*76);
	
	matrix_multiply(solution_matrix,motors_rnd,displace_buffer);
	
}

void displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out)
{
	
	my_displacement_pid.kp=kp;
	my_displacement_pid.kd=kd;
	my_displacement_pid.ki=ki;
	my_displacement_pid.max_iout=max_iout;
	my_displacement_pid.max_out=max_out;
	
}

void displacement_control(float x,float y)
{
	
	my_displacement_pid.x_error[1]=my_displacement_pid.x_error[0];
	my_displacement_pid.x_error[0]=x-displace_buffer[0];
	
	my_displacement_pid.y_error[1]=my_displacement_pid.y_error[0];
	my_displacement_pid.y_error[0]=y-displace_buffer[1];
	
	my_displacement_pid.s_x_error+=my_displacement_pid.x_error[0];
	my_displacement_pid.iout[0]=my_displacement_pid.s_x_error*my_displacement_pid.ki;
	my_displacement_pid.s_y_error+=my_displacement_pid.y_error[0];
	my_displacement_pid.iout[1]=my_displacement_pid.s_y_error*my_displacement_pid.ki;
	
	for(uint8_t i=0;i<2;++i)
	{
		if(my_displacement_pid.iout[i]>my_displacement_pid.max_iout)
		{
			
			my_displacement_pid.iout[i]=my_displacement_pid.max_iout;
			
		}
		else if(my_displacement_pid.iout[i]<-my_displacement_pid.max_iout)
		{
			
			my_displacement_pid.iout[i]=-my_displacement_pid.max_iout;
			
		}
	}
	
	
	my_displacement_pid.out[0]=my_displacement_pid.x_error[0]*my_displacement_pid.kp + my_displacement_pid.kd*(my_displacement_pid.x_error[0]-my_displacement_pid.x_error[1]);
	my_displacement_pid.out[1]=my_displacement_pid.y_error[0]*my_displacement_pid.kp + my_displacement_pid.kd*(my_displacement_pid.y_error[0]-my_displacement_pid.y_error[1]);

	for(uint8_t i=0;i<2;++i)
	{
		if(my_displacement_pid.out[i]>my_displacement_pid.max_out)
		{
			
			my_displacement_pid.out[i]=my_displacement_pid.max_out;
			
		}
		
		else if(my_displacement_pid.out[i]<-my_displacement_pid.max_out)
		{
			
			my_displacement_pid.out[i]=-my_displacement_pid.max_out;
			
		}
	}
}

void displacement_control_with_omega(float x,float y,float omega)
{
	
	matrix_multiply(solution_matrix,motors_rnd,displace_buffer);
	
	my_displacement_pid.x_error[1]=my_displacement_pid.x_error[0];
	my_displacement_pid.x_error[0]=x-displace_buffer[0];
	
	my_displacement_pid.y_error[1]=my_displacement_pid.y_error[0];
	my_displacement_pid.y_error[0]=y-displace_buffer[1];
	
	my_displacement_pid.omega_error[1]=my_displacement_pid.omega_error[0];
	my_displacement_pid.omega_error[0]=fmodf(omega-displace_buffer[2],180);
	
	my_displacement_pid.s_x_error+=my_displacement_pid.x_error[0];
	my_displacement_pid.iout[0]=my_displacement_pid.s_x_error*my_displacement_pid.ki;
	my_displacement_pid.s_y_error+=my_displacement_pid.y_error[0];
	my_displacement_pid.iout[1]=my_displacement_pid.s_y_error*my_displacement_pid.ki;
	my_displacement_pid.s_omega_error+=my_displacement_pid.omega_error[0];
	my_displacement_pid.iout[2]=my_displacement_pid.s_omega_error*my_displacement_pid.ki;
	
	for(uint8_t i=0;i<3;++i)
	{
		if(my_displacement_pid.iout[i]>my_displacement_pid.max_iout)
		{
			
			my_displacement_pid.iout[i]=my_displacement_pid.max_iout;
			
		}
		else if(my_displacement_pid.iout[i]<-my_displacement_pid.max_iout)
		{
			
			my_displacement_pid.iout[i]=-my_displacement_pid.max_iout;
			
		}
	}
	my_displacement_pid.pout[0]=my_displacement_pid.x_error[0]*my_displacement_pid.kp;
	my_displacement_pid.pout[1]=my_displacement_pid.y_error[0]*my_displacement_pid.kp;
	my_displacement_pid.pout[2]=my_displacement_pid.omega_error[0]*my_displacement_pid.kp;
	
	my_displacement_pid.dout[0]=my_displacement_pid.kd*(my_displacement_pid.x_error[0]-my_displacement_pid.x_error[1]);
	my_displacement_pid.dout[1]=my_displacement_pid.kd*(my_displacement_pid.y_error[0]-my_displacement_pid.y_error[1]);
	my_displacement_pid.dout[2]=my_displacement_pid.kd*(my_displacement_pid.omega_error[0]-my_displacement_pid.omega_error[1]);
	
	my_displacement_pid.out[0]= my_displacement_pid.pout[0]+my_displacement_pid.dout[0]+my_displacement_pid.iout[0];
	my_displacement_pid.out[1]= my_displacement_pid.pout[1]+my_displacement_pid.dout[1]+my_displacement_pid.iout[1];
	my_displacement_pid.out[2]= my_displacement_pid.pout[2]+my_displacement_pid.dout[2]+my_displacement_pid.iout[2];

	for(uint8_t i=0;i<3;++i)
	{
		if(my_displacement_pid.out[i]>my_displacement_pid.max_out)
		{
			
			my_displacement_pid.out[i]=my_displacement_pid.max_out;
			
		}
		
		else if(my_displacement_pid.out[i]<-my_displacement_pid.max_out)
		{
			
			my_displacement_pid.out[i]=-my_displacement_pid.max_out;
			
		}
	}
}

