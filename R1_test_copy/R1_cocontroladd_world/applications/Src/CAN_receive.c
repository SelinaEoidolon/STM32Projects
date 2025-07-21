#include "include.h"

extern volatile uint16_t angle ;
extern volatile uint16_t data;
extern volatile float absoluteAngle ;
extern volatile uint8_t DataFrame[8];
//1




extern CAN_TxHeaderTypeDef txMsg;//发送配置结构体
extern CAN_RxHeaderTypeDef rxMsg;//发送接收结构体
extern uint8_t rx_data[8];       //接收数据
extern uint32_t Motor_Can_ID;    //接收数据电机ID
extern uint8_t byte[4];          //转换临时数据
extern uint32_t send_mail_box;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];
float motor_out1;
CAN_TxHeaderTypeDef can_tx_message;
uint8_t	can_send_data[8];

void get_motor_measure(motor_measure_t *ptr,uint8_t data[])                                                     
    {   
        (ptr)->last_angle = (ptr)->angle;                                                          
        (ptr)->angle = data[0] << 8 | data[1];           
        (ptr)->speed_rpm = data[2] << 8 | data[3];
        (ptr)->given_current = data[4] << 8 | data[5]; 
        (ptr)->temperature = data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

					if(ptr->angle - ptr->last_angle > 4096)
						ptr->round_cnt --;
					else if (ptr->angle - ptr->last_angle < -4096)
						ptr->round_cnt ++;
					ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
    }
		
				
		
void get_motor_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0]<<8 |data[1] ;
	ptr->offset_angle = ptr->angle;
}



#define ABS(x)	( (x>0) ? (x) : (-x) )


void get_total_angle(motor_measure_t *p)
	{
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}
	
void reset_motor_position(motor_measure_t*this,uint8_t num)
{
	for(uint8_t i=0;i<num;++i){
		this->msg_cnt=0;
		this->round_cnt=0;
		this->last_angle=0;
		this->total_angle=0;
		this->angle=0;
		this->offset_angle=0;
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data_1[8];
		uint8_t rx_data_2[8];
	
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_1);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can1[i].msg_cnt++ <= 50	?	get_motor_offset(&motor_can1[i], rx_data_1) : get_motor_measure(&motor_can1[i], rx_data_1);
//				get_motor_measure(&motor_can1[i], rx_data_1);多余了
				break;
			}
			default:
			{
			break;
			}	
		}
	}
		
	else if(hcan->Instance==CAN2)
	{
		


        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_2);//接收数据
        if(rx_header.IDE == CAN_ID_STD){
			
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_ALL_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can2[i].msg_cnt++ <= 50	?	get_motor_offset(&motor_can2[i], rx_data_2) : get_motor_measure(&motor_can2[i], rx_data_2);
				get_motor_measure(&motor_can2[i], rx_data_2);
				break;
			}
			default:
			{
			break;
			}	
		}
		}
		else if(rx_header.IDE == CAN_ID_EXT){
			CAN_RxHeaderTypeDef rx_header;
			RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
			uint8_t rx_data[8];

			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_2);

			memcpy(&RxCAN_info,&rx_header.ExtId,4);//将扩展标识符的内容解码到缓存区，获取通信类型

			if(RxCAN_info.communication_type == 0){//通信类型0的反馈帧解码
				RxCAN_info_type_0_s RxCAN_info_type_0;
				memcpy(&RxCAN_info_type_0,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型0的对应内容
				//memcpy(&RxCAN_info_type_0.MCU_id,(void *)rx_data,8);//获取MCU标识符
				OutputData.data_3 = RxCAN_info_type_0.motor_id;
			}else if(RxCAN_info.communication_type == 2){//通信类型2的反馈帧解码
				RxCAN_info_type_2_s RxCAN_info_type_2;
						memcpy(&RxCAN_info_type_2,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
				MI_motor_RxDecode(&RxCAN_info_type_2,rx_data);//通信类型2的数据解码

				MI_Motor[RxCAN_info_type_2.motor_id].RxCAN_info = RxCAN_info_type_2;
				MI_Motor[RxCAN_info_type_2.motor_id].motor_mode_state = RxCAN_info_type_2.mode_state;

			}else if(RxCAN_info.communication_type == 17){//通信类型17的反馈帧解码
				RxCAN_info_type_17_s RxCAN_info_type_17;
						memcpy(&RxCAN_info_type_17,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型17的对应内容
				//memcpy(&RxCAN_info_type_17.index,(void *)&rx_data[0],2);//获取查找的参数索引码
				//memcpy(&RxCAN_info_type_17.param,(void *)&rx_data[4],4);//获取查找的参数信息
				OutputData.data_3 = RxCAN_info_type_17.motor_id;
				OutputData.data_5 = RxCAN_info_type_17.index;
				OutputData.data_6 = RxCAN_info_type_17.param;
			}
		//    OutputData.data_1 = MI_Motor[1].RxCAN_info.angle;
				OutputData.data_1 = MI_Motor[2].RxCAN_info.angle;
			OutputData.data_2 = RxCAN_info.communication_type;
		}
	}
//     motor_controlmode(&mi_motor[0], 12, 12.5, 30, 20,1);
		
	
		
 }


 
 
 
 void CAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x200;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor1 >> 8;
 can_send_data[1] = motor1;
 can_send_data[2] = motor2 >> 8;
 can_send_data[3] = motor2;
 can_send_data[4] = motor3 >> 8;
 can_send_data[5] = motor3;
 can_send_data[6] = motor4 >> 8;
 can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}




 void CAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x1FF;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor5 >> 8;
 can_send_data[1] = motor5;
 can_send_data[2] = motor6 >> 8;
 can_send_data[3] = motor6;
 can_send_data[4] = motor7 >> 8;
 can_send_data[5] = motor7;
 can_send_data[6] = motor8 >> 8;
 can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}



 void CAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x200;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor1 >> 8;
 can_send_data[1] = motor1;
 can_send_data[2] = motor2 >> 8;
 can_send_data[3] = motor2;
 can_send_data[4] = motor3 >> 8;
 can_send_data[5] = motor3;
 can_send_data[6] = motor4 >> 8;
 can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}


 void CAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x1FF;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor5 >> 8;
 can_send_data[1] = motor5;
 can_send_data[2] = motor6 >> 8;
 can_send_data[3] = motor6;
 can_send_data[4] = motor7 >> 8;
 can_send_data[5] = motor7;
 can_send_data[6] = motor8 >> 8;
 can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}



