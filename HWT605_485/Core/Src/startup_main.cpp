#include "startup_main.h"
#include "bsp_delay.h"
#include "usart.h"

extern uint8_t hwt605buf[1];

void startup_main(void)
{
//	bsp_delay.f4.Init(168);
	HAL_UART_Receive_IT(&huart3,hwt605buf, 1);
// HAL_UART_Receive_DMA(&huart3, hwt605buf, 1);
	
#if isRTOS==1    	//如果是裸机开发
	for(;;)  //等同于while(true)
	{

	}
#endif
}