/*
 * Copyright 2022 NXP
 * NXP confidential.
 * This software is owned or controlled by NXP and may only be used strictly
 * in accordance with the applicable license terms.  By expressly accepting
 * such terms or by downloading, installing, activating and/or otherwise using
 * the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to
 * be bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software.
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_nvic.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"

#include <cr_section_macros.h>

// ADC
#define ADC_CONVERSION_RATE 100000

// GPDMA
#define GPDMA_CHANNEL_0 0
#define GPDMA_CHANNEL_1 1
#define GPDMA_BUFFER_SIZE 16

// SYSTICK & TIMER
#define SYSTICK_TIME_IN_US 100 // 0.1[ms]
#define TIMER_TIME_IN_US 100 // 0.1[ms]
#define DEBOUNCE_DELAY_CYCLES 2000 // 200[ms]

// ADC variables
uint32_t static adcValue_0;
uint32_t static adcValue_1;
uint32_t static adcValue_2;
uint32_t static adcValue_4;

// GPDMA variables
GPDMA_LLI_Type static adcLLI_0;
GPDMA_LLI_Type static adcLLI_1;
uint32_t static adcBuffer_0[GPDMA_BUFFER_SIZE];
uint32_t static adcBuffer_1[GPDMA_BUFFER_SIZE];

// General variables
uint32_t static debounceCounter_0 = 0;
uint32_t static debounceCounter_1 = 0;
uint32_t static debounceCounter_2 = 0;
uint32_t static debounceCounter_3 = 0;
uint32_t static buttonState_0 = 0;
uint32_t static buttonState_1 = 0;
uint32_t static buttonState_2 = 0;
uint32_t static buttonState_3 = 0;

void configADC();
void configDAC();
void configEINT();
void configGPDMA();
void configGPIO();
void configSysTick();
void configTimer();

void readADC();
void writeDAC();

int main() {
	SystemInit();
	configADC();
	configDAC();
	configEINT();
	//configGPDMA();
	//configGPIO();
	//configSysTick();
	configTimer();
	while (1) {
		writeDAC();
	}
	return 0;
}

/*
 * CONFIGURATION METHODS
 */

void configADC() {
	LPC_PINCON->PINSEL1 &= ~(3 << 14);	// Clear P0.23
	LPC_PINCON->PINSEL1 |=  (1 << 14);	// Set P0.23 as AD0.0
	LPC_PINCON->PINSEL1 &= ~(3 << 16);	// Clear P0.24
	LPC_PINCON->PINSEL1 |=  (1 << 16);	// Set P0.24 as AD0.1
	LPC_PINCON->PINSEL1 &= ~(3 << 18);	// Clear P0.25
	LPC_PINCON->PINSEL1 |=  (1 << 18);	// Set P0.25 as AD0.2
	LPC_PINCON->PINSEL3 |=  (3 << 28);	// Set P1.30 as AD0.4

	LPC_SC->PCONP |= (1 << 12);
	LPC_SC->PCLKSEL0 &= ~(3 << 24);
	LPC_SC->PCLKSEL0 |=  (3 << 24);
	LPC_ADC->ADCR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (124 << 8) | (0 << 16) | (1 << 21);
	LPC_ADC->ADINTEN = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4);
	NVIC_EnableIRQ(ADC_IRQn);
	LPC_ADC->ADCR |= (1 << 16);
}

void configDAC() { }

void configEINT() {
	LPC_PINCON->PINSEL4 &= ~(3 << 20);	// Clear P2.10
	LPC_PINCON->PINSEL4 |=  (1 << 20);	// Set P2.10 as EINT0
	LPC_PINCON->PINMODE4 &= ~(3 << 20);	// Set P2.10 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1 << 10);	// Set P2.10 as INPUT
	LPC_SC->EXTINT |= (1 << 0);			// Set EINT0 as external interrupt
	LPC_SC->EXTMODE |= (1 << 0);		// Set EINT0 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1 << 0);		// Set EINT0 interruption in falling edge
	NVIC_EnableIRQ(EINT0_IRQn);

	LPC_PINCON->PINSEL4 &= ~(3 << 22);	// Clear P2.11
	LPC_PINCON->PINSEL4 |=  (1 << 22);	// Set P2.11 as EINT1
	LPC_PINCON->PINMODE4 &= ~(3 << 22);	// Set P2.11 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1 << 11);	// Set P2.11 as INPUT
	LPC_SC->EXTINT |= (1 << 1);			// Set EINT1 as external interrupt
	LPC_SC->EXTMODE |= (1 << 1);		// Set EINT1 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1 << 1);		// Set EINT1 interruption in falling edge
	NVIC_EnableIRQ(EINT1_IRQn);

	LPC_PINCON->PINSEL4 &= ~(3 << 24);	// Clear P2.12
	LPC_PINCON->PINSEL4 |=  (1 << 24);	// Set P2.12 as EINT2
	LPC_PINCON->PINMODE4 &= ~(3 << 24);	// Set P2.12 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1 << 12);	// Set P2.12 as INPUT
	LPC_SC->EXTINT |= (1 << 2);			// Set EINT2 as external interrupt
	LPC_SC->EXTMODE |= (1 << 2);		// Set EINT2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1 << 2);		// Set EINT2 interruption in falling edge
	NVIC_EnableIRQ(EINT2_IRQn);

	LPC_PINCON->PINSEL4 &= ~(3 << 26);	// Clear P2.12
	LPC_PINCON->PINSEL4 |=  (1 << 26);	// Set P2.12 as EINT2
	LPC_PINCON->PINMODE4 &= ~(3 << 26);	// Set P2.12 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1 << 13);	// Set P2.12 as INPUT
	LPC_SC->EXTINT |= (1 << 3);			// Set EINT2 as external interrupt
	LPC_SC->EXTMODE |= (1 << 3);		// Set EINT2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1 << 3);		// Set EINT2 interruption in falling edge
	NVIC_EnableIRQ(EINT3_IRQn);
}

void configGPDMA() { }

void configGPIO() {
	LPC_PINCON->PINSEL4 &= ~(3 << 0);	// Set P2.0 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 0);	// Set P2.0 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 0);		// Set P2.0 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3 << 2);	// Set P2.1 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 2);	// Set P2.1 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 1);		// Set P2.1 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3 << 4);	// Set P2.2 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 4);	// Set P2.2 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 2);		// Set P2.2 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3 << 6);	// Set P2.3 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 6);	// Set P2.3 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 3);		// Set P2.3 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3 << 8);	// Set P2.4 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 8);	// Set P2.4 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 4);		// Set P2.4 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3 << 10);	// Set P2.5 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2 << 10);	// Set P2.5 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1 << 5);		// Set P2.5 as OUTPUT

	LPC_GPIO2->FIOCLR |= (1 << 0);		// Set P0.0 in LOW
	LPC_GPIO2->FIOCLR |= (1 << 1);		// Set P0.1 in LOW
	LPC_GPIO2->FIOCLR |= (1 << 2);		// Set P0.2 in LOW
	LPC_GPIO2->FIOCLR |= (1 << 3);		// Set P0.3 in LOW
	LPC_GPIO2->FIOCLR |= (1 << 4);		// Set P0.4 in LOW
	LPC_GPIO2->FIOCLR |= (1 << 5);		// Set P0.5 in LOW
}

void configSysTick() {
	SysTick->LOAD = (SystemCoreClock / 1000000) * SYSTICK_TIME_IN_US - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1 << 0) | (1 << 1) | (1 << 2); // Enable SysTick counter, enable SysTick interruptions and select internal clock
	NVIC_EnableIRQ(SysTick_IRQn);
}

void configTimer() { }

/*
 * INTERRUPTION HANDLERS
 */

void ADC_IRQHandler() {
	// Check which channel triggered the interruption and store its value
	if (LPC_ADC->ADSTAT & (1 << 0)) {
		adcValue_0 = (LPC_ADC->ADDR0 >> 4) & 0xFFF;
	}
	if (LPC_ADC->ADSTAT & (1 << 1)) {
		adcValue_1 = (LPC_ADC->ADDR1 >> 4) & 0xFFF;
	}
	if (LPC_ADC->ADSTAT & (1 << 2)) {
		adcValue_2 = (LPC_ADC->ADDR2 >> 4) & 0xFFF;
	}
	if (LPC_ADC->ADSTAT & (1 << 4)) {
		adcValue_3 = (LPC_ADC->ADDR3 >> 4) & 0xFFF;
	}
	// Clean the global interruption by reading the global data register
	volatile uint32_t dummy = LPC_ADC->ADGDR;
	(void)dummy;
}

void DMA_IRQHandler() { }

void EINT0_IRQHandler() {
	if (debounceCounter_0 == 0) {
		debounceCounter_0 = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		buttonState_0 =! buttonState_0;
	}
	LPC_SC->EXTINT = (1 << 0); // Clear the interruption flag
}

void EINT1_IRQHandler() {
	if (debounceCounter_1 == 0) {
		debounceCounter_1 = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		buttonState_1 =! buttonState_1;
	}
	LPC_SC->EXTINT = (1 << 1); // Clear the interruption flag
}

void EINT2_IRQHandler() {
	if (debounceCounter_2 == 0) {
		debounceCounter_2 = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		buttonState_2 =! buttonState_2;
	}
	LPC_SC->EXTINT = (1 << 2); // Clear the interruption flag
}

void EINT3_IRQHandler() {
	if (debounceCounter_3 == 0) {
		debounceCounter_3 = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		buttonState_3 =! buttonState_3;
	}
	LPC_SC->EXTINT = (1 << 3); // Clear the interruption flag
}

void SysTick_Handler() {
	if (debounceCounter_0 > 0) {
		debounceCounter_0--; // Decrement the debounce counter
	}
	if (debounceCounter_1 > 0) {
		debounceCounter_1--; // Decrement the debounce counter
	}
	if (debounceCounter_2 > 0) {
		debounceCounter_2--; // Decrement the debounce counter
	}
	if (debounceCounter_3 > 0) {
		debounceCounter_3--; // Decrement the debounce counter
	}
}

void TIMER0_IRQHandler() { }

/*
 * GENERAL METHODS
 */

void readADC() { }

void writeDAC() {
	if (buttonState_0) {
		LPC_DAC->DACR = (adcValue_0 >> 2);
	} else {
		LPC_DAC->DACR = (adcValue_1 >> 2);
	}
}
