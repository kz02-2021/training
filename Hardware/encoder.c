#include "encoder.h"
int GetEncoder(int TIMx )
{
		int value;
		switch(TIMx )
		{
			case 2:value=(short )__HAL_TIM_GetCounter (&htim2);__HAL_TIM_SetCounter (&htim2 ,0);break ;
			case 4:value=(short )__HAL_TIM_GetCounter (&htim4);__HAL_TIM_SetCounter (&htim4 ,0);break ;
			default :break ;
		}
		return value ;
		
		
}
