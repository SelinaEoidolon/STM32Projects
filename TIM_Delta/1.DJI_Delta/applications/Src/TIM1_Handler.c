#include "main.h"

uint8_t step;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	++step;
	if(step==90)
	{
		step=0;
	}
}





