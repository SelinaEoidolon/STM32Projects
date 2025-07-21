#include "include.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

int mi200 = 0;
int mi250 = 0;
int mi270 = 0;
uint8_t mi_flag = 0;
uint8_t down_flag = 0;
extern uint8_t send_flag;
uint8_t mi_mode_flag;	
uint8_t mi_high_add_flag;
uint8_t mi_speed_flag;
//uint8_t qigang_flag;

typedef enum {
    SHOT_IDLE = 0,
    SHOT_UP,
    SHOT_WAIT_UP,
    SHOT_DOWN,
    SHOT_WAIT_DOWN,
    SHOT_HIT,
    SHOT_DONE
} ShotState_t;

static ShotState_t shotState = SHOT_IDLE;
static uint32_t shotTickStart = 0;
float num1 = 0;
float sys_flag = 0;
void user_init(void)
{
	
	HAL_Delay(3000);
	PID_devices_Init();
//	PID_delta_init();
	
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	
	CAN_Start(&hcan1);
	CAN_Start(&hcan2);
	
	HAL_UART_Receive_IT(&huart1,buffer,1);
	HAL_UART_Receive_IT(&huart2,&hwt_rx_temp,1);
	HAL_UART_Receive_IT(&huart4,rx_buffs_ykq,1);
	DWT_Init();
	delay_init();
	
	inverse_solution_matrix_init();
	solution_matrix_init();
	HAL_TIM_Base_Start_IT(&htim2);
	
	xy_displacement_pid_config(1,0,0,150,2000);//kp,kd,ki,max_iout,max_out
	z_displacement_pid_config(2,0,0,184,3000);//角度单独给出
	
	vel_designer_init(body_acc_design,1500);
	xy_spatium_designer_init(xyw_spatium_designer,4800,1200);//这是对xy轴的规划，单位可以视作电机转速在三个轴上的分量，也还是rpm，电机转子的每2.41rpm,对应了轮子线速度1mm/s
	spatium_designer_init(&xyw_spatium_designer[2],2000,500);//角度轨迹单独调参
	
	MI_motor_Init(&MI_Motor[2],&MI_CAN_2,2);
	MI_motor_Enable(&MI_Motor[2]);
  MI_motor_SetMechPositionToZero(&MI_Motor[2]);	

}

//void control_loop(void) {
//    // 设置目标速度（例如来自遥控器）
//    SPlanner_SetTarget(&vx_planner, ykq_xyw[0]);
//    SPlanner_SetTarget(&vy_planner, ykq_xyw[1]);
//    SPlanner_SetTarget(&w_planner, ykq_xyw[2]);

//    // 获取平滑后的速度
//    calc_buffer[0] = SPlanner_Update(&vx_planner);
//    calc_buffer[1] = SPlanner_Update(&vy_planner);
//    calc_buffer[2]  = SPlanner_Update(&w_planner);
//    vel_control
//    // 送入逆运动学公式或轮速计算
//	
//}


void local_velocity_mode_run_with_angle_displacement(void)
{
	calc_buffer[0]=receive_data[0]+ykq_xyw[0];
	calc_buffer[1]=receive_data[1]+ykq_xyw[1];
	calc_buffer[2]=angular_displacement_PID(receive_data[2]+ykq_xyw[2]);
	my_angle_measure.yaw+=my_angle_measure.yaw_omega*DT;
	feedforword_control(body_acc_design,calc_buffer);
	vel_control(calc_buffer);
}

void world_velocity_mode_run(void)
{	
//	static float last_target_vx = 0.0f;
//	static float last_target_vy = 0.0f;
//	static float last_target_w  = 0.0f;
	rotate_matrix_calc(-my_angle_measure.yaw*PI/180.0f);
	my_angle_measure.yaw+=my_angle_measure.filtered_yaw_omega*DT;
//	//*****************************************
//	// 死区抑制（可选但推荐）
//	if (fabs(ykq_xyw[0]) < 0.01f) ykq_xyw[0] = 0;
//	if (fabs(ykq_xyw[1]) < 0.01f) ykq_xyw[1] = 0;
//	if (fabs(ykq_xyw[2]) < 0.01f) ykq_xyw[2] = 0;

//	// ? 只有在目标改变时才设置目标速度（防止每次重启规划器）
//	if (fabs(ykq_xyw[0] - last_target_vx) > 1e-3f) {
//		SPlanner_SetTarget(&vx_planner, ykq_xyw[0]);
//		last_target_vx = ykq_xyw[0];
//	}
//	if (fabs(ykq_xyw[1] - last_target_vy) > 1e-3f) {
//		SPlanner_SetTarget(&vy_planner, ykq_xyw[1]);
//		last_target_vy = ykq_xyw[1];
//	}
//	if (fabs(ykq_xyw[2] - last_target_w) > 1e-3f) {
//		SPlanner_SetTarget(&w_planner, ykq_xyw[2]);
//		last_target_w = ykq_xyw[2];
//	}

//    // 获取平滑后的速度
//    calc_buffer[0] = SPlanner_Update(&vx_planner);
//    calc_buffer[1] = SPlanner_Update(&vy_planner);
//    calc_buffer[2]  = SPlanner_Update(&w_planner);
	//*******************************************
	calc_buffer[0]=ykq_xyw[0];
	calc_buffer[1]=ykq_xyw[1];
	calc_buffer[2]=ykq_xyw[2];
	matrix_multiply(rotate_matrix,calc_buffer,calc_buffer_2nd);
	vel_control(calc_buffer_2nd);
}

void world_displacement_mode_run(void)
{
	
		static float last_target[3]={0};
		static const float threshold = 1.0f;
		
		if(change_mode_flag==0x04)
		{
			reset_motor_position(motor_can1,3);
			rnd_count_and_diaplacement_reset();
			last_target[0]=0;
			last_target[1]=0;
			last_target[2]=0;
			change_mode_flag=0;
		}
		
		get_rnd_count_and_diaplacement();
		
		if(fabsf(last_target[0] - receive_data[0]) > threshold||fabsf(last_target[1] - receive_data[1]) > threshold||fabsf(last_target[2] - receive_data[2]) > threshold)
		{
			
			for(uint8_t i=0;i<2;++i)//角度控制不能用下面那个线量转化角量的控制，要单独写，所以这里给 i 给出 <2 的表达 
			{
				spatium_designer_set_target(&xyw_spatium_designer[i],(receive_data[i]/*目标坐标*/-displace_buffer[i]/*当前坐标*/)*2.41f/*线量转化为角量*/);
				last_target[i]=receive_data[i];
			}
			spatium_designer_set_target(&xyw_spatium_designer[2],angle_error(receive_data[2]/*目标坐标*/,displace_buffer[2]/*当前坐标*/));//本身就是角量不需要转换，但是要划归到+-180
		}
		
		spatium_designer_update(&xyw_spatium_designer[0]);//先更新再检测，在运动末尾比起先检测再更新快了响应速度快了一个控制周期
		if(xyw_spatium_designer[0].finished==True)
		{
			x_displacement_control(receive_data[0]);
			calc_buffer[0]=my_displacement_pid.out[0];
		}else calc_buffer[0]=xyw_spatium_designer[0].current_v;//x轴控制
		
		spatium_designer_update(&xyw_spatium_designer[1]);
		if(xyw_spatium_designer[1].finished==True)
		{
			y_displacement_control(receive_data[1]);
			calc_buffer[1]=my_displacement_pid.out[1];
		}else calc_buffer[1]=xyw_spatium_designer[1].current_v;//y
		
		spatium_designer_update(&xyw_spatium_designer[2]);
		if(xyw_spatium_designer[2].finished==True)
		{
			theta_displacement_control(receive_data[2]);
			calc_buffer[2]=my_displacement_pid.out[2];
		}else calc_buffer[2]=xyw_spatium_designer[2].current_v;//z
		
		vel_control(calc_buffer);
		
		if(xyw_spatium_designer[0].finished==False||xyw_spatium_designer[1].finished==False||xyw_spatium_designer[2].finished==False)
		{
			if(msg_mode==CIRCUIT_SEND_ENABLE) not_got_it_the_point_yet();
		}else {if(msg_mode==CIRCUIT_SEND_ENABLE) got_it_the_point_already();}
		
}

void pure_local_velocity_mode_run(void)
{
//	static float curren_angle_speed=0;
//	static float current_angle=0;
//	
//	calc_buffer[0]=receive_data[0]+ykq_xyw[0];
//	calc_buffer[1]=receive_data[1]+ykq_xyw[1];
//	
//	curren_angle_speed=(receive_data[2]+ykq_xyw[2])*6*187/3591;//通信获得期望转子速度单位rpm，转化为轮子的度每秒。
//	current_angle+=curren_angle_speed*DT;
//	current_angle = fmodf(current_angle + 180.0f, 360.0f);
//	if (current_angle < 0) current_angle += 360.0f;
//	current_angle -= 180.0f;
//	
//	calc_buffer[2]=angular_displacement_PID(current_angle);
	vel_control(ykq_xyw);
}

void cybergear_control(void)
{
	
	switch(cyber_gear_knock_flag)	
	{	

		case 1:{
//			//项目一发球9.16

//         // 先执行预备动作
////         MI_motor_Control(&MI_Motor[2], -3, -5.3, 28.2, 21, 2.5); // 预备动作
//         MI_motor_Control(&MI_Motor[2],-3,-4.8,28.2,21,2.5);//预备   9.16m偏左0.7 
//         HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);//上升
//				 delay_ms(200);
//				 sys_flag = HAL_GetTick();
//				 num1 = 2;
//			   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//下降

//			   delay_ms(270);
//			   MI_motor_Control(&MI_Motor[2],2,0.9,29,300,4.4);//击球
//				 cyber_gear_knock_flag = 0;
////         // 置位状态机，启动非阻塞击球流程
//			if(qigang_flag == 1)HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);  //上升
//			if(qigang_flag == 2)HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//下降
//			cyber_gear_knock_flag = 0;

		}break;
		
		case 2:{
//			MI_motor_Control(&MI_Motor[2],-3,-5.35,28.2,21,2.5);//预备  估计>9.5
//			MI_motor_Control(&MI_Motor[2],-3,-5.0,28.2,21,2.5);//预备  估计>9.5
//      MI_motor_Control(&MI_Motor[2],-3,-4.6,28.2,21,2.5);//预备  估计>9.5
//			MI_motor_Control(&MI_Motor[2],-3,-4.0,28.2,21,2.5);
			switch(mi_high_add_flag){
				case 0:{
					MI_motor_Control(&MI_Motor[2],-3,-4.2,28.2,21,2.5);
				}break;
				case 1:{
					MI_motor_Control(&MI_Motor[2],-3,-4.2,28.2,21,2.5);
				}break;
				case 2:{
					MI_motor_Control(&MI_Motor[2],-3,-4.2,28.2,21,2.5);
				}break;
				case 3:{
					MI_motor_Control(&MI_Motor[2],-3,-4.2,28.2,21,2.5);
				}break;
				default:break;
			}
			
			cyber_gear_knock_flag = 0;
			mi_mode_flag = 1;
			
		}break;
		
		case 3:{
//			MI_motor_Control(&MI_Motor[2],-3,-4.45,28.2,21,2.5);//预备   估计（5m.6m）
//			MI_motor_Control(&MI_Motor[2],-3,-4.05,28.2,21,2.5);//预备   估计（5m.6m）
//      MI_motor_Control(&MI_Motor[2],-3,-3.75,28.2,21,2.5);//预备  估计>9.5
			switch(mi_high_add_flag){
				case 0:{
					MI_motor_Control(&MI_Motor[2],-3,-4.0,28.2,21,2.5);
				}break;
				case 1:{
					MI_motor_Control(&MI_Motor[2],-3,-4.0,28.2,21,2.5);
				}break;
				case 2:{
					MI_motor_Control(&MI_Motor[2],-3,-4.0,28.2,21,2.5);
				}break;
				case 3:{
					MI_motor_Control(&MI_Motor[2],-3,-4.0,28.2,21,2.5);
				}break;
				default:break;
			}
			
			
			cyber_gear_knock_flag = 0;
			mi_mode_flag = 2;
			
		}break;
		
		case 4:{
//			MI_motor_Control(&MI_Motor[2],-3,-3.8,28.2,21,2.5);//4.5m预备   低压4.47  高压4.6
//			MI_motor_Control(&MI_Motor[2],-3,-3.4,28.2,21,2.5);//预备  估计>9.5
			switch(mi_high_add_flag){
				case 0:{
					MI_motor_Control(&MI_Motor[2],-3,-3.8,28.2,21,2.5);
				}break;
				case 1:{
					MI_motor_Control(&MI_Motor[2],-3,-3.8,28.2,21,2.5);
				}break;
				case 2:{
					MI_motor_Control(&MI_Motor[2],-3,-3.8,28.2,21,2.5);
				}break;
				case 3:{
					MI_motor_Control(&MI_Motor[2],-3,-3.8,28.2,21,2.5);
				}break;
				default:break;
			}
			
			
			
			cyber_gear_knock_flag = 0;
			mi_mode_flag = 3;

		}break;
		
    case 5:{
			 MI_motor_Control(&MI_Motor[2],2,0.9,29,300,4.4);//纯击球		
			 cyber_gear_knock_flag = 0;


		}break;
		case 6:{
			 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);//上升
			 delay_ms(200);			 
			 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//下降
			 if(mi_mode_flag == 1) {
			    if(mi_speed_flag == 0) delay_ms(290);
			    if(mi_speed_flag == 1) delay_ms(290);
			    if(mi_speed_flag == 2) delay_ms(290);
				  if(mi_speed_flag == 3) delay_ms(290);
			 }
			 if(mi_mode_flag == 2)  {
			    if(mi_speed_flag == 0) delay_ms(290);//270-290
			    if(mi_speed_flag == 1) delay_ms(280);//275
			    if(mi_speed_flag == 2) delay_ms(280);//280
				  if(mi_speed_flag == 3) delay_ms(260);	//285
			 }
			 if(mi_mode_flag == 3) {
			    if(mi_speed_flag == 0) delay_ms(300);//300
			    if(mi_speed_flag == 1) delay_ms(280);
			    if(mi_speed_flag == 2) delay_ms(280);
				  if(mi_speed_flag == 3) delay_ms(260);				
			 }
			 if(mi_speed_flag == 0)
				 MI_motor_Control(&MI_Motor[2],2,0.9*1.02,29*1.02,300,4.4);//击球
			 if(mi_speed_flag == 1)
				 MI_motor_Control(&MI_Motor[2],2,0.9,29,300,4.4);//击球
			 if(mi_speed_flag == 2)
				 MI_motor_Control(&MI_Motor[2],2,0.9*0.9,29*0.9,300,4.4);//击球				 
			 if(mi_speed_flag == 3)
				 MI_motor_Control(&MI_Motor[2],2,0.9*0.8,29*0.8,300,4.4);//击球				 
			 if(mi_speed_flag == 4)
				 MI_motor_Control(&MI_Motor[2],2,0.9*0.7,29*0.7,300,4.4);//击球
				 
//			 MI_motor_Control(&MI_Motor[2],2,0.9,29,300,4.4);//击球
			 cyber_gear_knock_flag = 0;
			 mi_mode_flag = 0;
			 
			 

		}break;
		case 7:{
					MI_motor_Control(&MI_Motor[2],-3,-4.3,28.2,21,2.5);


		}break;
		default:break;
	}
}

void msg_control(void)
{
	
	switch (send_flag){
		
		case 2:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 3:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 4:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		case 5:{
			 HAL_UART_Transmit_IT(&huart1,&send_flag,1);
		}break;
		default:{}break;
	}
	send_flag = 0;
	
}
//void Shot_Task_Handler(void)
//{
//    switch (shotState)
//    {
//        case SHOT_IDLE:
//            // 不做事，等待启动
//            break;

//        case SHOT_UP:
//            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // 上升
//            shotTickStart = HAL_GetTick();
//            shotState = SHOT_WAIT_UP;
//            break;

//        case SHOT_WAIT_UP:
//            if (HAL_GetTick() - shotTickStart >= 200)
//            {
//                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // 下降
//                shotTickStart = HAL_GetTick();
//                shotState = SHOT_WAIT_DOWN;
//            }
//            break;

//        case SHOT_WAIT_DOWN:
//            if (HAL_GetTick() - shotTickStart >= 270)
//            {
//                shotState = SHOT_HIT;
//            }
//            break;

//        case SHOT_HIT:
//            MI_motor_Control(&MI_Motor[2], 2, 0.7, 29, 300, 4.4); // 击球动作
//            shotState = SHOT_DONE;
//            break;

//        case SHOT_DONE:
//            // 如果你想让动作循环，可以重置为 SHOT_IDLE 或其它状态
//            shotState = SHOT_IDLE;
//            break;

//        default:
//            break;
//    }
//}

