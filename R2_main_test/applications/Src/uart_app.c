#include "include.h"
#include "uart_app.h"

extern motor_measure_t motor_can1[8];

uint8_t step=0;

uint8_t run_mode=0x01;
uint8_t controller=0;// 0->共同控制 | 1->手动控制
uint8_t delta_mode=0;
uint8_t msg_mode=CIRCUIT_SEND_DISABLE;

uint8_t crc_code=0;
uint8_t declearation=0;
uint8_t count=0;

uint8_t tx_buffer[6];
uint8_t buffer[4]={0};
uint8_t rx_buffs_ykq[12];
float receive_data[6]={0};

uint8_t cyber_gear_knock_flag=0;

angle_measure my_angle_measure={0};

uint8_t sum_ykq=0;
int step_ykq =0;
int cnt_ykq=0;

uint8_t change_mode_flag=0x00;

uint8_t commander=0;

HWT_ParserState hwt_state = HWT_WAIT_HEAD_0x50;
uint8_t angle_buffer[8];  // 用于数据帧接收
uint8_t hwt_rx_temp;      // 单字节缓冲


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		switch(step)
		{
			
			case 0:
			{
				
				waiting_for_pack_head();
				
			}break;
			
			case 1:
			{
				
				judgement_of_declearation();
				
				
			}break;
			
			case 2:
			{
				
				receiving_datas();
				
			}break;
			
			case 3:
			{
				crc8_inspecting();
				
			}break;
			
			case 4:
			{
				
				waiting_for_package_tail();
				
			}break;

			case CONTROLLER_PACKAGE_BEGINNING:
			{
				commander=buffer[0];
				commander_package_analysis(buffer[0]);
				step=4;
				HAL_UART_Receive_IT(&huart1,buffer,1);
				
			}break;
			
			default:step=0;break;
		}
		
	}
	
	else if(huart->Instance==USART2)
	{
		
		switch (hwt_state)
		{
			case HWT_WAIT_HEAD_0x50:
				if (hwt_rx_temp == 0x50)
					hwt_state = HWT_WAIT_CMD_0x03;
				HAL_UART_Receive_IT(&huart2, &hwt_rx_temp, 1);
				break;

			case HWT_WAIT_CMD_0x03:
				if (hwt_rx_temp == 0x03)
					hwt_state = HWT_WAIT_ID_0x06;
				else if (hwt_rx_temp == 0x50)
					hwt_state = HWT_WAIT_CMD_0x03;  // 重启状态机
				else
					hwt_state = HWT_WAIT_HEAD_0x50;
				HAL_UART_Receive_IT(&huart2, &hwt_rx_temp, 1);
				break;

			case HWT_WAIT_ID_0x06:
				if (hwt_rx_temp == 0x06) {
					hwt_state = HWT_RECV_DATA;
					HAL_UART_Receive_IT(&huart2, angle_buffer, 8); // 接收完整数据
				} else {
					hwt_state = HWT_WAIT_HEAD_0x50;
					HAL_UART_Receive_IT(&huart2, &hwt_rx_temp, 1);
				}
				break;

			case HWT_RECV_DATA:
				// 数据接收完成后才会进入这个 case
				{
					if(hwt_what_type==1){
						int16_t raw_yaw = (int16_t)((angle_buffer[4] << 8) | angle_buffer[5]);
						my_angle_measure.yaw = raw_yaw / 32768.0f * 180.0f;
						hwt_what_type=0;
					}
					else if(hwt_what_type==2){
						int16_t raw_yaw_omega = (int16_t)((angle_buffer[4] << 8) | angle_buffer[5]);
						my_angle_measure.yaw_omega =my_angle_measure.yaw_omega = raw_yaw_omega / 32768.0f * 2000.0f;
						my_angle_measure.filtered_yaw_omega= low_pass_filter(my_angle_measure.yaw_omega,my_angle_measure.filtered_yaw_omega,0.3);
						hwt_what_type=0;
					}

					// 重置状态机
					hwt_state = HWT_WAIT_HEAD_0x50;
					HAL_UART_Receive_IT(&huart2, &hwt_rx_temp, 1);
				}
				break;
			
				default:break;
			}
				
		}
	
	else if(huart->Instance==UART4)
	{
		
		crc(rx_buffs_ykq[0]);
		HAL_UART_Receive_IT(&huart4,rx_buffs_ykq,1);
		
	}

	
}

void send_command_package(uint8_t command)
{
	
//	tx_buffer[0]=0xa5;
//	tx_buffer[1]=0x00;
//	tx_buffer[2]=command;
//	tx_buffer[3]=0x5a;
//	HAL_UART_Transmit_IT(&huart1,tx_buffer,4);
	
}

void got_it_the_point_already(void)
{
	
	tx_buffer[0]=0x01;
	HAL_UART_Transmit_IT(&huart1,tx_buffer,1);
	
}

void not_got_it_the_point_yet(void)
{
	
	tx_buffer[0]=0x00;
	HAL_UART_Transmit_IT(&huart1,tx_buffer,1);
	
}

void send_angle_velocity(void)
{
	
//	tx_buffer[0]=;
//	tx_buffer[1]=;
//	tx_buffer[2]=;
//	tx_buffer[3]=;
//	tx_buffer[4]=;
//	tx_buffer[5]=;
//	HAL_UART_Transmit_IT(&huart1,tx_buffer,6);
	
}
/*	
					my_angle_measure.roll= ((int16_t)( angle_buffer[2]<<8 )|angle_buffer[1])/32768.0f*180.0f;
					my_angle_measure.pitch=((int16_t)( angle_buffer[4]<<8 )|angle_buffer[3])/32768.0f*180.0f;
					my_angle_measure.yaw=  ((int16_t)( angle_buffer[6]<<8 )|angle_buffer[5])/32768.0f*180.0f;
*/
