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

// PIN CONNECTIONS
#define ADC_CHANNEL_0_PIN 23 // P0.23
#define ADC_CHANNEL_1_PIN 24 // P0.24
#define ADC_CHANNEL_2_PIN 25 // P0.25
#define ADC_CHANNEL_4_PIN 27 // P0.27
#define DAC_PIN 26 // P0.26
#define EINT_0_PIN 10 // P2.10
#define EINT_1_PIN 11 // P2.11
#define EINT_2_PIN 12 // P2.12
#define EINT_3_PIN 13 // P2.13
#define GPIO_0_PIN 0 // P2.0
#define GPIO_1_PIN 1 // P2.1
#define GPIO_2_PIN 2 // P2.2
#define GPIO_3_PIN 3 // P2.3
#define GPIO_4_PIN 4 // P2.4
#define GPIO_5_PIN 5 // P2.5

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
uint32_t static adcChannelSelection = 0;
uint32_t static adcValue_0;
uint32_t static adcValue_1;

// GPDMA variables
GPDMA_LLI_Type static adcLLI_0;
GPDMA_LLI_Type static adcLLI_1;
uint32_t static adcBuffer_0[GPDMA_BUFFER_SIZE];
uint32_t static adcBuffer_1[GPDMA_BUFFER_SIZE];

// General variables
uint32_t static debounceCounter_0 = 0;
uint32_t static buttonState_0 = 0;

void configADC();
void configDAC();
void configEINT();
void configGPDMA();
void configGPIO();
void configNVIC();
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
	configNVIC();
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
	/*
	PINSEL_CFG_Type PinCfg;
	// P0.23 as ADC
	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = ADC_CHANNEL_0_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	// P0.24 ad ADC
	PinCfg.Pinnum = ADC_CHANNEL_1_PIN;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, SENSOR_PIN, 0);
	ADC_Init(LPC_ADC, ADC_CONVERSION_RATE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_1, ENABLE);
	ADC_BurstCmd(LPC_ADC, ENABLE);
	*/
	// --- Configuración de pines ---
	// P0.23 -> AD0.0
	LPC_PINCON->PINSEL1 &= ~(3 << 14);   // Limpio bits
	LPC_PINCON->PINSEL1 |=  (1 << 14);   // Función 01 -> AD0.0
	// P0.24 -> AD0.1
	LPC_PINCON->PINSEL1 &= ~(3 << 16);
	LPC_PINCON->PINSEL1 |=  (1 << 16);   // Función 01 -> AD0.1

	// --- Alimentar el ADC ---
	LPC_SC->PCONP |= (1 << 12);          // PCLK_ADC encendido

	// --- Configurar reloj del ADC ---
	// PCLK_ADC = CCLK / 8  → (ejemplo)
	LPC_SC->PCLKSEL0 &= ~(3 << 24);      // PCLK_ADC bits [25:24]
	LPC_SC->PCLKSEL0 |=  (3 << 24);      // 3 = CCLK/8

	// Queremos 100 kHz de frecuencia de conversión aprox.
	// ADCR bits [15:8] = CLKDIV -> Fadc = PCLK / (CLKDIV + 1)
	// Si PCLK = 12.5 MHz (100 MHz / 8), entonces CLKDIV = 124 → 100 kHz
	LPC_ADC->ADCR = (1 << 0)   |          // SEL: canal 0 habilitado
					(1 << 1)   |          // SEL: canal 1 habilitado
					(124 << 8) |          // CLKDIV
					(0 << 16)  |          // BURST = 0 (por ahora)
					(1 << 21);            // PDN = 1 (ADC encendido)

	// --- Activar interrupciones ---
	LPC_ADC->ADINTEN = (1 << 0) | (1 << 1);   // Habilitar interrupciones por canal 0 y 1
	NVIC_EnableIRQ(ADC_IRQn);

	// --- Activar modo BURST (conversiones continuas automáticas) ---
	LPC_ADC->ADCR |= (1 << 16);  // BURST = 1
}

void configDAC() {
	PINSEL_CFG_Type PinCfg;
	// P0.26 as DAC
	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum  = DAC_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_2;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, DAC_PIN, 1);
	DAC_Init(LPC_DAC);
	DAC_CONVERTER_CFG_Type dacCfg;
	dacCfg.DBLBUF_ENA = DISABLE;
	dacCfg.CNT_ENA = DISABLE;
	dacCfg.DMA_ENA = DISABLE;
	DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
}

void configEINT() {
	PINSEL_CFG_Type PinCfg;
	// P2.10 as EINT3
	PinCfg.Portnum = PINSEL_PORT_2;
	PinCfg.Pinnum  = EINT_0_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << EINT_0_PIN), 1);
	LPC_SC->EXTINT |= (1<<0); // Enable EINT0 as external interrupt
	LPC_SC->EXTMODE |= (1<<0); // Set EINT0 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<0); // Set EINT0 interruption in falling edge
	// P2.11 as EINT3
	PinCfg.Pinnum  = EINT_1_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << EINT_1_PIN), 1);
	LPC_SC->EXTINT |= (1<<1); // Enable EINT1 as external interrupt
	LPC_SC->EXTMODE |= (1<<1); // Set EINT1 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<1); // Set EINT1 interruption in falling edge
	// P2.12 as EINT3
	PinCfg.Pinnum  = EINT_2_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << EINT_2_PIN), 1);
	LPC_SC->EXTINT |= (1<<2); // Enable EINT2 as external interrupt
	LPC_SC->EXTMODE |= (1<<2); // Set EINT2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<2); // Set EINT2 interruption in falling edge
	// P2.13 as EINT3
	PinCfg.Pinnum  = EINT_3_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << EINT_3_PIN), 1);
	LPC_SC->EXTINT |= (1<<3); // Enable EINT3 as external interrupt
	LPC_SC->EXTMODE |= (1<<3); // Set EINT3 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<3); // Set EINT3 interruption in falling edge
}

void configGPDMA() {
	GPDMA_Init();
	GPDMA_Channel_CFG_Type DMACfg;
	DMACfg.ChannelNum = GPDMA_CHANNEL_0;
	DMACfg.TransferSize = GPDMA_BUFFER_SIZE;
	DMACfg.TransferWidth = GPDMA_WIDTH_WORD;
	DMACfg.SrcMemAddr = 0;
	DMACfg.DstMemAddr = (uint32_t)&adcBuffer_0;
	DMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
	DMACfg.SrcConn = GPDMA_CONN_ADC;
	DMACfg.DstConn = 0;
	DMACfg.DMALLI = (uint32_t)&adcLLI_0;
	GPDMA_Setup(&DMACfg);
	adcLLI_0.SrcAddr = LPC_ADC->ADDR0;
	adcLLI_0.DstAddr = (uint32_t)&adcBuffer_0[0];
	adcLLI_0.NextLLI = (uint32_t)&adcLLI_1;
	adcLLI_0.Control = (GPDMA_BUFFER_SIZE & 0xfff) | (2<<18) | (2<<21) | (1<<25) | (1<<27);
	adcLLI_1.SrcAddr = LPC_ADC->ADDR0;
	adcLLI_1.DstAddr = (uint32_t)&adcBuffer_1[0];
	adcLLI_1.NextLLI = (uint32_t)&adcLLI_0;
	adcLLI_1.Control = (GPDMA_BUFFER_SIZE & 0xfff) | (2<<18) | (2<<21) | (1<<25) | (1<<27);
	GPDMA_ChannelCmd(GPDMA_CHANNEL_0, ENABLE);
}

void configGPIO() {
	PINSEL_CFG_Type PinCfg;
	// P2.0 as GPIO
	PinCfg.Portnum = PINSEL_PORT_2;
	PinCfg.Pinnum  = GPIO_0_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_0;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << GPIO_0_PIN), 1);
	// P2.1 as GPIO
	PinCfg.Pinnum  = GPIO_1_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << GPIO_1_PIN), 1);
	// P2.2 as GPIO
	PinCfg.Pinnum  = GPIO_2_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << GPIO_2_PIN), 1);
	// P2.3 as GPIO
	PinCfg.Pinnum  = GPIO_3_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << GPIO_3_PIN), 1);
	// P2.4 as GPIO
	PinCfg.Pinnum  = GPIO_4_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << GPIO_4_PIN), 1);
	// P2.5 as GPIO
	PinCfg.Pinnum  = GPIO_5_PIN;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(3, (1 << GPIO_5_PIN), 1);
	// Set output in LOW as starting point
	GPIO_ClearValue(0, (1 << GPIO_0_PIN));
	GPIO_ClearValue(3, (1 << GPIO_1_PIN));
	GPIO_ClearValue(3, (1 << GPIO_2_PIN));
	GPIO_ClearValue(3, (1 << GPIO_3_PIN));
	GPIO_ClearValue(3, (1 << GPIO_4_PIN));
	GPIO_ClearValue(3, (1 << GPIO_5_PIN));
}

void configNVIC() {
	NVIC_EnableIRQ(ADC_IRQn);
    //NVIC_EnableIRQ(DMA_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
	//NVIC_EnableIRQ(EINT1_IRQn);
	//NVIC_EnableIRQ(EINT2_IRQn);
	//NVIC_EnableIRQ(EINT3_IRQn);
	//NVIC_EnableIRQ(SysTick_IRQn); // -NOT USED-
	NVIC_EnableIRQ(TIMER0_IRQn);
}

void configSysTick() {
	SysTick->LOAD = (SystemCoreClock / 1000000) * SYSTICK_TIME_IN_US - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<0) | (1<<1) | (1<<2); // Enable SysTick counter, enable SysTick interruptions and select internal clock
}

void configTimer() {
	TIM_TIMERCFG_Type timerCfg;
	TIM_MATCHCFG_Type matchCfg;
	timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
	timerCfg.PrescaleValue  = 1;
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);
	matchCfg.MatchChannel = 0;
	matchCfg.IntOnMatch = ENABLE;
	matchCfg.StopOnMatch = DISABLE;
	matchCfg.ResetOnMatch = ENABLE;
	matchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	matchCfg.MatchValue = TIMER_TIME_IN_US;
	TIM_ConfigMatch(LPC_TIM0, &matchCfg);
	TIM_Cmd(LPC_TIM0, ENABLE);
}

/*
 * INTERRUPTION HANDLERS
 */

void ADC_IRQHandler() {
	// Comprobamos el flag de cada canal
	if (LPC_ADC->ADSTAT & (1 << 0)) {
		// Leo valor de canal 0
		adcValue_0 = (LPC_ADC->ADDR0 >> 4) & 0xFFF;
	}
	if (LPC_ADC->ADSTAT & (1 << 1)) {
		// Leo valor de canal 1
		adcValue_1 = (LPC_ADC->ADDR1 >> 4) & 0xFFF;
	}

	// Limpiar la interrupción global (lectura del registro la limpia)
	volatile uint32_t dummy = LPC_ADC->ADGDR;
	(void)dummy;
}

void DMA_IRQHandler() {
	GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, GPDMA_CHANNEL_0);
}

void EINT0_IRQHandler() {
	if (debounceCounter_0 == 0) {
		debounceCounter_0 = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		buttonState_0 =! buttonState_0;
	}
	EXTI_ClearEXTIFlag(EXTI_EINT0); // Clear the interruption flag
}

void EINT1_IRQHandler() { }

void EINT2_IRQHandler() { }

void EINT3_IRQHandler() { }

void SysTick_Handler() {
	if (debounceCounter_0 > 0) {
		debounceCounter_0--; // Decrement the debounce counter
	}
}

void TIMER0_IRQHandler() {
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)) {
		TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
		if (debounceCounter_0 > 0) {
			debounceCounter_0--; // Decrement the debounce counter
		}
	}
}

/*
 * GENERAL METHODS
 */

void readADC() { }

void writeDAC() {
	if (buttonState_0 == 0) {
		DAC_UpdateValue(LPC_DAC, (adcValue_0 >> 2));
	} else {
		DAC_UpdateValue(LPC_DAC, (adcValue_1 >> 2));
	}
}
