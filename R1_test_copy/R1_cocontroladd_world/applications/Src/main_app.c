#include "include.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void user_init(void)
{
	
	 HAL_Delay(3000);
	PID_devices_Init();
	PID_delta_init();
	
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	
	CAN_Start(&hcan1);
	CAN_Start(&hcan2);
	
	HAL_UART_Receive_IT(&huart1,buffer,1);
	HAL_UART_Receive_IT(&huart2,angle_buffer,1);
	HAL_UART_Receive_IT(&huart4,rx_buffs_ykq,1);
	DWT_Init();
	hwt605_init();
	
	inverse_solution_matrix_init();
	solution_matrix_init();
	HAL_TIM_Base_Start_IT(&htim2);
	displacement_pid_config(10,0,0,184,2000);//kp,kd,ki,max_iout,max_out
	vel_designer_init(body_acc_design,1500);
	triple_spatium_designer_init(xyw_spatium_designer,5000,100);//单位是电机转子的转每分
	spatium_designer_init(&delta_designer,0.3,0.1);
	MI_motor_Init(&MI_Motor[2],&MI_CAN_2,2);
	MI_motor_Enable(&MI_Motor[2]);
	MI_motor_SetMechPositionToZero(&MI_Motor[2]);
	MI_motor_ModeSwitch(&MI_Motor[2], CONTROL_MODE);
	
}

void local_velocity_mode_run(void)
{
	calc_buffer[0]=receive_data[0]+ykq_xyw[0];
	calc_buffer[1]=receive_data[1]+ykq_xyw[1];
	calc_buffer[2]=angular_displacement_PID(receive_data[2]+ykq_xyw[2]);
	feedforword_control(body_acc_design,calc_buffer);
	matrix_multiply(inverse_solution_matrix,calc_buffer,calculatied_velocity);
	CAN1_CMD_1(PID_velocity_realize_1(calculatied_velocity[0]*1.1f,1),PID_velocity_realize_1(calculatied_velocity[1],2),PID_velocity_realize_1(calculatied_velocity[2],3),0);
}

void world_velocity_mode_run(void)
{	
	my_angle_measure.filtered_yaw+=my_angle_measure.filtered_yaw_omega*DT;
	rotate_matrix_calc(-my_angle_measure.filtered_yaw*PI/180.0f);
	calc_buffer[0]=receive_data[0];
	calc_buffer[1]=receive_data[1];
	calc_buffer[2]=angular_velocity_PID(receive_data[2]);
	matrix_multiply(rotate_matrix,calc_buffer,calc_buffer_2nd);
	matrix_multiply(inverse_solution_matrix,calc_buffer_2nd,calculatied_velocity);
	CAN1_CMD_1(PID_velocity_realize_1(calculatied_velocity[0],1),PID_velocity_realize_1(calculatied_velocity[1],2),PID_velocity_realize_1(calculatied_velocity[2],3),0);
}

void world_displacement_mode_run(void)
{
	
		static float last_target[3]={0};
		static const float threshold = 0.0001f;
		if(change_mode_flag==0x04)
		{
			reset_motor_position(motor_can1,3);
			change_mode_flag=0;
		}
		
		get_rnd_count_and_diaplacement();
		
		if(fabsf(last_target[0] - receive_data[0]) > threshold||fabsf(last_target[1] - receive_data[1]) > threshold||fabsf(last_target[2] - receive_data[2]) > threshold)
		{
			
			for(uint8_t i=0;i<3;++i)
			{
				spatium_designer_set_target(&xyw_spatium_designer[i],receive_data[i]);
				last_target[i]=receive_data[i];
			}
			
		}
		
		if(xyw_spatium_designer[0].finished==False||xyw_spatium_designer[1].finished==False||xyw_spatium_designer[2].finished==False)
		{
			for(uint8_t i=0;i<3;++i)
			{
				spatium_designer_update(&xyw_spatium_designer[i]);
			}
			displacement_control_with_omega(xyw_spatium_designer[0].current_v,xyw_spatium_designer[1].current_v,xyw_spatium_designer[2].current_v);
			calc_buffer[0]=my_displacement_pid.out[0];
			calc_buffer[1]=my_displacement_pid.out[1];
			calc_buffer[2]=my_displacement_pid.out[2];
			matrix_multiply(inverse_solution_matrix,calc_buffer,calculatied_velocity);
			CAN1_CMD_1(PID_velocity_realize_1(calculatied_velocity[0],1),PID_velocity_realize_1(calculatied_velocity[1],2),PID_velocity_realize_1(calculatied_velocity[2],3),0);
			if(msg_mode==CIRCUIT_SEND_ENABLE) not_got_it_the_point_yet();
		}
		else{
			displacement_control_with_omega(receive_data[0],receive_data[1],receive_data[2]);
			calc_buffer[0]=my_displacement_pid.out[0];
			calc_buffer[1]=my_displacement_pid.out[1];
			calc_buffer[2]=my_displacement_pid.out[2];
			matrix_multiply(inverse_solution_matrix,calc_buffer,calculatied_velocity);
			CAN1_CMD_1(PID_velocity_realize_1(calculatied_velocity[0],1),PID_velocity_realize_1(calculatied_velocity[1],2),PID_velocity_realize_1(calculatied_velocity[2],3),0);//完全依赖解算方案
			
			if(msg_mode==CIRCUIT_SEND_ENABLE) got_it_the_point();
		}
		
}

void cybergear_control(void)
{
	switch(cyber_gear_knock_flag)
	{
		case 1:{
			MI_motor_Control(&MI_Motor[2],-0.4,-3.6,28.2,21,2);
			cyber_gear_knock_flag=0;
		}break;
		
		case 2:{
			MI_motor_Control(&MI_Motor[2],2,0.7,29,300,4.4);
			cyber_gear_knock_flag=0;
		}break;
		
		default:break;
	}
}

void msg_control(void)
{
	
		if(msg_mode==CIRCUIT_SEND_ENABLE)//msg_mode==CIRCUIT_SEND_ENABLE
	  {
		  
			  tx_buffer[0]=motor_can1[0].speed_rpm;
			  tx_buffer[1]=motor_can1[0].speed_rpm >> 8;
			  tx_buffer[2]=motor_can1[1].speed_rpm;
			  tx_buffer[3]=motor_can1[1].speed_rpm >> 8;
			  tx_buffer[4]=motor_can1[2].speed_rpm;
			  tx_buffer[5]=motor_can1[2].speed_rpm >> 8;
			  HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
		  
	  }
	
}
void delta_enable(void){
    if(motor_can1[4].total_angle<-47514||
         motor_can1[5].total_angle<-47514||
       motor_can1[6].total_angle<-47514
        )
    {
            CAN1_CMD_2
           (
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,5),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,6),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,7),
        
            0
        
           );
     }
    else if(motor_can1[4].total_angle>8191||
         motor_can1[5].total_angle>8191||
       motor_can1[6].total_angle>8191
        )
    {
        CAN1_CMD_2
           (
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,5),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,6),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,7),
        
            0
        
           );
    }
    else if(motor_can1[4].total_angle-motor_can1[5].total_angle>8191 ||
         motor_can1[4].total_angle-motor_can1[5].total_angle<-8191  ||
       motor_can1[4].total_angle-motor_can1[6].total_angle>8191 ||
       motor_can1[4].total_angle-motor_can1[6].total_angle<-8191  ||
         motor_can1[5].total_angle-motor_can1[6].total_angle>8191 ||
       motor_can1[5].total_angle-motor_can1[6].total_angle<-8191  
        )
    {
        CAN1_CMD_2
           (
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,5),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,6),
        
            pid_call_1(-22.0f/360.0f*8191.0f*3591.0f/187.0f,7),
        
            0
        
           );
    }
}

	void delta_control(void)
{
    if(delta_mode)
    {
//        DeltaInversekinematic(receive_data[3],receive_data[4],receive_data[5],D_theta);
        if (receive_data[3]>90&&receive_data[3]>90&&receive_data[3]>90) {
       receive_data[3] = 90;
       receive_data[4] = 90;
       receive_data[5] = 90;  // 如果解无效，角度最大  
             CAN1_CMD_2
           (
        
            pid_call_1(-(receive_data[3]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
        
            pid_call_1(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
        
            pid_call_1(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
        
            0
        
           );
//			float num1 = -(receive_data[3]+22)/360*8191*3591/187;
     }
        else if (receive_data[3]<-12 && receive_data[3]<-12 && receive_data[3]<-12) {
       receive_data[3] = -12;
       receive_data[4] = -12;
       receive_data[5] = -12;  // 如果解无效，角度最小  
             CAN1_CMD_2
           (
        
            pid_call_1(-(receive_data[3]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
        
            pid_call_1(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
        
            pid_call_1(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
        
            0
        
           );
     }
		else{
			uint8_t spatium=delta_position[0]-receive_data[3];
			if(spatium>0)/*delta fall*/
			{
				
				CAN1_CMD_2
			 (
			
			  PID_delta_call_down(-(receive_data[3]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
			
			  PID_delta_call_down(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
			
			  PID_delta_call_down(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
			
			  0
			
			 );
			}else/*delta rise*/ 
			CAN1_CMD_2
			 (
			
			  PID_delta_call_up(-(receive_data[3]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,5),
			
			  PID_delta_call_up(-(receive_data[4]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,6),
			
			  PID_delta_call_up(-(receive_data[5]+22.0f)/360.0f*8191.0f*3591.0f/187.0f,7),
			
			  0
			
			 );
             
			get_delta_position();
//			delta_enable();
        }
        
    }
    
}
