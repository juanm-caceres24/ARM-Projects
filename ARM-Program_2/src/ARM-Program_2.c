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

#include <cr_section_macros.h>

void configPorts();
void configSysTick();
void configEINT();
void configNVIC();
void setDutyCycle();
void writePort();
void setOutputHIGH();
void setOutputLOW();

static uint32_t TIME_IN_MS = 2;
static uint32_t RATIO = 2000;
static uint32_t counter = 0;
static uint32_t duty_cycle;
static uint32_t value = 0;
static uint32_t rebound_set;
static uint32_t rebound_counter;

int main(void) {
	SystemInit();		// system initialization
	configPorts();		// port configuration
	configSysTick();	// systick configuration
	configEINT();		// external interrupts configuration
	configNVIC();		// interruptions configuration
    while(1) { }
    return 0 ;
}

void configPorts() {
	LPC_PINCON->PINSEL4 &= ~(3<<20);	// clear P2.10
	LPC_PINCON->PINSEL4 |=  (1<<20);	// set P2.10 as EINT0
	LPC_PINCON->PINSEL4 &= ~(3<<22);	// clear P2.11
	LPC_PINCON->PINSEL4 |=  (1<<22);	// set P2.11 as EINT1
	LPC_PINCON->PINSEL4 &= ~(3<<24);	// clear P2.11
	LPC_PINCON->PINSEL4 |=  (1<<24);	// set P2.12 as EINT2
	LPC_PINCON->PINSEL4 &= ~(3<<0);		// set P2.0 as GPIO
	LPC_PINCON->PINSEL4 &= ~(3<<2);		// set P2.1 as GPIO
	LPC_PINCON->PINSEL4 &= ~(3<<4);		// set P2.2 as GPIO
	LPC_PINCON->PINSEL7 &= ~(3<<18);	// set P3.25 as GPIO
	LPC_PINCON->PINMODE4 &= ~(3<<20);	// set P2.10 with PULL-UP
	LPC_PINCON->PINMODE4 &= ~(3<<22);	// set P2.11 with PULL-UP
	LPC_PINCON->PINMODE4 &= ~(3<<24);	// set P2.12 with PULL-UP
	LPC_PINCON->PINMODE4 &= ~(2<<0);	// set P2.0 neither PULL-UP nor PULL-DOWN
	LPC_PINCON->PINMODE4 &= ~(2<<2);	// set P2.1 neither PULL-UP nor PULL-DOWN
	LPC_PINCON->PINMODE4 &= ~(2<<4);	// set P2.2 neither PULL-UP nor PULL-DOWN
	LPC_PINCON->PINMODE7 &= ~(2<<18);	// set P3.25 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR &= ~(1<<10);		// set P2.10 as INPUT
	LPC_GPIO2->FIODIR &= ~(1<<11);		// set P2.11 as INPUT
	LPC_GPIO2->FIODIR &= ~(1<<12);		// set P2.12 as INPUT
	LPC_GPIO2->FIODIR |= (1<<0);		// set P2.0 as OUTPUT
	LPC_GPIO2->FIODIR |= (1<<1);		// set P2.1 as OUTPUT
	LPC_GPIO2->FIODIR |= (1<<2);		// set P2.2 as OUTPUT
	LPC_GPIO3->FIODIR |= (1<<25);		// set P3.25 as OUTPUT
}

void configSysTick() {
	SysTick->LOAD = (SystemCoreClock / 1000) * TIME_IN_MS - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<0) | (1<<1) | (1<<2);
}

void SysTick_Handler() {
	rebound_counter += 1;
	counter += 1;
	if (counter > 7) {
		counter = 0;
	}
	if (counter <= value && value != 0) {
		setOutputHIGH();
	}
	else {
		setOutputLOW();
	}
}

void configEINT() {
	LPC_SC->EXTINT |= (1<<0);		// set PORT 2 as external interrupt
	LPC_SC->EXTMODE |= (1<<0);		// set PORT 2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<0);	// set PORT 2 interruption in falling edge
	LPC_SC->EXTINT |= (1<<1);		// set PORT 2 as external interrupt
	LPC_SC->EXTMODE |= (1<<1);		// set PORT 2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<1);	// set PORT 2 interruption in falling edge
	LPC_SC->EXTINT |= (1<<2);		// set PORT 2 as external interrupt
	LPC_SC->EXTMODE |= (1<<2);		// set PORT 2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<2);	// set PORT 2 interruption in falling edge
}

void EINT0_IRQHandler() {
	if (rebound_counter - 100 > rebound_set) {
		rebound_set = rebound_counter;
		if (value < 7) {
			value += 1;
		}
		writePort();
		setDutyCycle();
	}
	LPC_SC->EXTINT |= (1<<0);
}

void EINT1_IRQHandler() {
	if (rebound_counter - 100 > rebound_set) {
		rebound_set = rebound_counter;
		if (value > 0) {
			value -= 1;
		}
		writePort();
		setDutyCycle();
	}
	LPC_SC->EXTINT |= (1<<1);
}

void EINT2_IRQHandler() {
	if (rebound_counter - 100 > rebound_set) {
		rebound_set = rebound_counter;
		value = 0;
		writePort();
		setDutyCycle();
	}
	LPC_SC->EXTINT |= (1<<2);
}

void configNVIC() {
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}

void setDutyCycle() {
	duty_cycle = value * 100 / 7;
}

void writePort() {
	uint32_t value_tmp = value;
	if (value_tmp > 3) {
		LPC_GPIO2->FIOSET |= (1<<0);
		value_tmp -= 4;
	}
	else {
		LPC_GPIO2->FIOCLR |= (1<<0);
	}
	if (value_tmp > 1) {
		LPC_GPIO2->FIOSET |= (1<<1);
		value_tmp -= 2;
	}
	else {
		LPC_GPIO2->FIOCLR |= (1<<1);
	}
	if (value_tmp > 0) {
		LPC_GPIO2->FIOSET |= (1<<2);
	}
	else {
		LPC_GPIO2->FIOCLR |= (1<<2);
	}
}

void setOutputLOW() {
	LPC_GPIO3->FIOSET |= (1<<25);
}

void setOutputHIGH() {
	LPC_GPIO3->FIOCLR |= (1<<25);
}
