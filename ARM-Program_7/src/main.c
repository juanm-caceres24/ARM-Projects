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

#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"

#include <cr_section_macros.h>

#define BUTTON_0_PIN 10 // P2.10
#define BUTTON_1_PIN 11 // P2.11

#define POTENTIOMETER_0_PIN 23 // P0.23
#define POTENTIOMETER_1_PIN 25 // P0.25

#define LED_0_PIN 26 // P0.26

#define TIME_IN_US 100 // Timer interval in microseconds
#define DEBOUNCE_DELAY_CYCLES 2000 // Cycles of TIME_IN_US that the button will be ignored (2000 * 100us = 200ms)

#define ADC_CONVERSION_RATE 200000

uint32_t static debounce_0_counter = 0; // Decrement counter of cycles of TIME_IN_US
uint32_t static debounce_1_counter = 0;
uint32_t static button_0_state = 0;
uint32_t static button_1_state = 0;

uint32_t static potentiometer_0_value = 0;
uint32_t static potentiometer_1_value = 0;

uint32_t static channel_selection = 0;

void configGPIO();
void configEINT();
void configSysTick();
void configTimer_0();
void configADC();
void configDAC();
void configGPDMA();
void configNVIC();

int main() {
	SystemInit();
	configGPIO();
	configEINT();
	configSysTick();
	configTimer_0();
	configADC();
	configDAC();
	configGPDMA();
	configNVIC();
	while (1) {
		if (button_0_state == 0 && button_1_state == 0) {
			DAC_UpdateValue(LPC_DAC, potentiometer_0_value);
		}
		if (button_0_state == 1 && button_1_state == 0) {
			DAC_UpdateValue(LPC_DAC, potentiometer_1_value);
		}
		if (button_0_state == 0 && button_1_state == 1) {
			DAC_UpdateValue(LPC_DAC, potentiometer_0_value + potentiometer_1_value);
		}
		if (button_0_state == 1 && button_1_state == 1) {
			DAC_UpdateValue(LPC_DAC, 0x3FF - potentiometer_0_value);
		}
	}
	return 0;
}

/*
 * CONFIGURATION METHODS
 */

void configGPIO() {

	/*
	 * BUTTONS
	 */
	PINSEL_CFG_Type PinCfg;

	PinCfg.Portnum = PINSEL_PORT_2;
	PinCfg.Pinnum = BUTTON_0_PIN; // EINT on P2.10
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, BUTTON_0_PIN, 0);

	PinCfg.Portnum = PINSEL_PORT_2;
	PinCfg.Pinnum = BUTTON_1_PIN; // EINT on P2.11
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, BUTTON_1_PIN, 0);
}

void configEINT() {
	LPC_SC->EXTINT |= (1<<0); // Enable EINT0 as external interrupt
	LPC_SC->EXTMODE |= (1<<0); // Set EINT0 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<0); // Set EINT0 interruption in falling edge

	LPC_SC->EXTINT |= (1<<1); // Enable EINT1 as external interrupt
	LPC_SC->EXTMODE |= (1<<1); // Set EINT1 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<1); // Set EINT1 interruption in falling edge
}

void configSysTick() {
	SysTick->LOAD = (SystemCoreClock / 1000000) * TIME_IN_US - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<0) | (1<<1) | (1<<2); // Enable SysTick counter, enable SysTick interruptions and select internal clock
}

void configTimer_0() {

}

void configADC() {
	PINSEL_CFG_Type PinCfg;

	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = POTENTIOMETER_0_PIN; // ADC on P0.23
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, POTENTIOMETER_0_PIN, 0);

	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = POTENTIOMETER_1_PIN; // ADC on P0.25
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, POTENTIOMETER_1_PIN, 0);

	ADC_Init(LPC_ADC, ADC_CONVERSION_RATE);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN2, ENABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_2, DISABLE);
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
}

void configDAC() {
	PINSEL_CFG_Type PinCfg;

	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = LED_0_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_2;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	GPIO_SetDir(2, LED_0_PIN, 1);
	PINSEL_ConfigPin(&PinCfg);

	DAC_Init(LPC_DAC);
}

void configGPDMA() {

}

void configNVIC() {
    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT1_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
}

/*
 * INTERRUPTION HANDLERS
 */

void EINT0_IRQHandler() {
    if (debounce_0_counter == 0) {
        debounce_0_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        button_0_state =! button_0_state;
    }
    LPC_SC->EXTINT |= (1<<0);
}

void EINT1_IRQHandler() {
    if (debounce_1_counter == 0) {
    	debounce_1_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
    	button_1_state =! button_1_state;
    }
    LPC_SC->EXTINT |= (1<<1);
}

void SysTick_Handler() {
    if (debounce_0_counter > 0) {
        debounce_0_counter--; // Decrement the debounce counter
    }
    if (debounce_1_counter > 0) {
        debounce_1_counter--; // Decrement the debounce counter
    }
}

void ADC_IRQHandler() {
	if (channel_selection == 0) {
		potentiometer_0_value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0) >> 2;
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, DISABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_2, ENABLE);
		channel_selection = 1;
	} else {
		potentiometer_1_value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_2) >> 2;
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_2, DISABLE);
		ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
		channel_selection = 0;
	}

	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
}

/*
 * GENERAL METHODS
 */


