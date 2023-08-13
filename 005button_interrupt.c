/*
 * 005button_interrupt.c
 *
 *  Created on: Aug 12, 2023
 *      Author: murattuzun
 */

#include <stdint.h>
#include "stm32f411xx.h"

int main()
{

	GPIO_Handle_t GPIO_LED, GPIO_Btn;
		GPIO_LED.pGPIOx = GPIOD;
		GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_TYPE_LS;
		GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_TYPE_PP;
		GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_TYPE_NONE;

		GPIO_PeriClockControl(GPIOD,ENABLE);
		GPIO_Init(&GPIO_LED);

		GPIO_Btn.pGPIOx = GPIOD;
		GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
		GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_TYPE_FS;

		GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU_PD_TYPE_UP;

		GPIO_PeriClockControl(GPIOD,ENABLE);
		GPIO_Init(&GPIO_Btn);

		//IRQ Configuration
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

		while(1);

}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
