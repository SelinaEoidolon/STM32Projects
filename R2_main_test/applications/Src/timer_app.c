#include "include.h"
__IO uint8_t time_up=0;
//extern int mi200 ;
//extern int mi250 ;
//extern int mi270 ;
//extern uint8_t mi_flag;
//extern uint8_t down_flag;
// float flag=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	  
	

//	control_loop();
//	mi200++;
//  mi250++;
//	mi270++;
//	
//	if(mi200>201) mi200 = 0;
//	if(mi250>251) mi250 = 0;
//	if(mi270>271) mi270 = 0;
	
	  if (htim->Instance == TIM2) {
        // 1ms��ʱ�¼�
        // ����������
			  time_up=1;
//			flag=flag+0.001;
    }
//    else if (htim->Instance == TIM3) {
//        // 200ms��ʱ�¼�
//        // ����������
//			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//�½�
//			  down_flag = 1;
//			
//    }
//    else if (htim->Instance == TIM4) {
//        // 250ms��ʱ�¼�
//        // ����������
//			  MI_motor_Control(&MI_Motor[2],2,0.7,29,300,4.4);//����		
//				mi_flag = 1;		
//    }
//    else if (htim->Instance == TIM5) {
//        // 270ms��ʱ�¼�
//        // ����������
//			  MI_motor_Control(&MI_Motor[2],2,0.7,29,300,4.4);//����
//        mi_flag = 1;			
//    }
}
	
	
	
	
	
	
