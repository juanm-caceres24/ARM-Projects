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
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"

#include <cr_section_macros.h>

#define SENSOR_PIN 23 // P0.23

#define LED_0_PIN 22 // P0.22 RED-LED
#define LED_1_PIN 25 // P3.25 GREEN-LED
#define LED_2_PIN 26 // P3.26 BLUE-LED

#define TIME_IN_US 100000 // Timer interval in microseconds

#define ADC_CONVERSION_RATE 200000

uint32_t static sensor_value = 0;
uint32_t static consecutive_counter = 0;

void configADC();
void configGPIO();
void configNVIC();
void configTimer();
void updateOutput();

int main() {
	SystemInit();
	configADC();
	configGPIO();
	configNVIC();
	configTimer();
	while (1) {
		updateOutput();
	}
	return 0;
}

/*
 * CONFIGURATION METHODS
 */

void configADC() {
	PINSEL_CFG_Type PinCfg;

	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = SENSOR_PIN; // ADC on P0.23
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, POTENTIOMETER_0_PIN, 0);

	ADC_Init(LPC_ADC, ADC_CONVERSION_RATE);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);
}

void configGPIO() {
	PINSEL_CFG_Type PinCfg;
	// P0.22 as GPIO
	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum  = LED_0_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_0;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	// P3.25 as GPIO
	PinCfg.Portnum = PINSEL_PORT_3;
	PinCfg.Pinnum  = LED_1_PIN;
	PINSEL_ConfigPin(&PinCfg);
	// P3.26 as GPIO
	PinCfg.Portnum = PINSEL_PORT_3;
	PinCfg.Pinnum  = LED_2_PIN;
	PINSEL_ConfigPin(&PinCfg);
	// set P0.22 as output
	GPIO_SetDir(0, (1 << LED_0_PIN), 1);
	// set P3.25 & P3.26
	GPIO_SetDir(3, (1 << LED_1_PIN) | (1 << LED_2_PIN), 1);
	// Set output in LOW as starting point
	GPIO_ClearValue(0, LED_0_PIN);
	GPIO_ClearValue(3, LED_1_PIN);
	GPIO_ClearValue(3, LED_2_PIN);
}

void configNVIC() {
    NVIC_EnableIRQ(ADC_IRQn);
}

void configTimer() {
	TIM_TIMERCFG_Type timerCfg;
	TIM_MATCHCFG_Type matchCfg;

	timerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
	timerCfg.PrescaleValue  = 1;
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerCfg);

	matchCfg.MatchChannel = 1;
	matchCfg.IntOnMatch   = DISABLE;
	matchCfg.ResetOnMatch = ENABLE;
	matchCfg.StopOnMatch  = DISABLE;
	matchCfg.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	matchCfg.MatchValue   = TIME_IN_US / 2;
	TIM_ConfigMatch(LPC_TIM0, &matchCfg);

	TIM_Cmd(LPC_TIM0, ENABLE);
}

/*
 * INTERRUPTION HANDLERS
 */

void ADC_IRQHandler() {
	sensor_value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
}

/*
 * GENERAL METHODS
 */

void updateOutput() {
	if (sensor_value <= 4096 * 0.4) {
		GPIO_ClearValue(0, LED_0_PIN);
		GPIO_SetValue(3, LED_1_PIN);
		GPIO_ClearValue(3, LED_2_PIN);
		consecutive_counter = 0;
	} else if (sensor_value > 4096 * 0.4 && sensor_value <= 4096 * 0.6) {
		GPIO_ClearValue(0, LED_0_PIN);
		GPIO_ClearValue(3, LED_1_PIN);
		GPIO_SetValue(3, LED_2_PIN);
		consecutive_counter = 0;
	} else {
		if (consecutive_counter >= 10) {
			GPIO_SetValue(0, LED_0_PIN);
			GPIO_ClearValue(3, LED_1_PIN);
			GPIO_ClearValue(3, LED_2_PIN);
		}
		consecutive_counter++;
	}
}
