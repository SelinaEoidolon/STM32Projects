#include "main.h"

uint8_t step;
uint8_t move;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	++step;
	++move;
	if(step==45)
	{
		step=0;
	}
	
}





