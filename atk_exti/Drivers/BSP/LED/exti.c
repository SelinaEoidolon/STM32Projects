
#include "exti.h"

void exti_init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct ;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitTypeDef GPIO_InitStruct2 ;
	
	GPIO_InitStruct2.Pin = GPIO_PIN_13;//指示灯
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);
	
	GPIO_InitTypeDef GPIO_InitStruct3 ;

	GPIO_InitStruct3.Pin = GPIO_PIN_10;//高电平引脚
  GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct3.Pull = GPIO_NOPULL;
  GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct3);
	
	
	HAL_NVIC_SetPriority(EXTI4_IRQn,2,0);//设置优先级
	
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);//使能中断
	
	
	
} 


void EXTI4_IRQHandler(void)
{
	
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == 1)
		{
				HAL_GPIO_WritePin(GPIOC,  GPIO_PIN_13,GPIO_PIN_RESET);
		}
		
	}
}






