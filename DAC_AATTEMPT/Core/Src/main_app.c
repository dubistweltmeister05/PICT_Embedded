/*
 * main_app.c
 *
 *  Created on: Jan 21, 2026
 *      Author: HP
 */

#include "stm32f4xx.h"                  // Device header

int main(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;
	DAC1->CR |= DAC_CR_MAMP1_3 | DAC_CR_MAMP1_1 | DAC_CR_MAMP1_0;
	DAC1->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0;
	DAC1->CR |= DAC_CR_TEN1;
	DAC1->CR |= DAC_CR_BOFF1;
	DAC1->CR |= DAC_CR_EN1;

	while (1) {

		for (volatile int i = 0; i < 4095; i++) {
			DAC1->DHR12R1 = i;
			DAC1->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

		}

	}

}
