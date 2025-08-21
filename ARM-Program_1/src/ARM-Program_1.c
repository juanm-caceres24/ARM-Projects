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
void delay(uint32_t);
void setDutyCycle();
void readPort();
void setOutputHIGH();
void setOutputLOW();

static uint32_t RATIO = 100000;
static uint32_t duty_cycle;
static uint32_t value;

int main(void) {

	SystemInit();	// system initialization
	configPorts();	// port configuration

    while(1) {
    	readPort();
    	setDutyCycle();
    	setOutputHIGH();
    	delay(duty_cycle * RATIO);
    	setOutputLOW();
    	delay((100 - duty_cycle) * RATIO);
    }

    return 0 ;
}

void configPorts() {
	LPC_PINCON->PINSEL0 &= ~(3<<30);	// set P0.15 GPIO			0b00111111111111111111111111111111
	LPC_PINCON->PINSEL1 &= ~(15<<0);	// set P0.16 & P0.17 GPIO	0b11111111111111111111111111110000

	LPC_PINCON->PINMODE0 |= (3<<30);	// set P0.15 PULL-DOWN
	//LPC_PINCON->PINMODE0 &= ~(3<<30);	// set P0.15 PULL-UP

	//LPC_PINCON->PINMODE1 |= (3<<0);	// set P0.16 PULL-DOWN
	LPC_PINCON->PINMODE1 &= ~(3<<0);	// set P0.16 PULL-UP

	//LPC_PINCON->PINMODE1 |= (3<<2);	// set P0.17 PULL-DOWN
	LPC_PINCON->PINMODE1 &= ~(3<<2);	// set P0.17 PULL-UP

	LPC_GPIO0->FIODIR &= ~(7<<15);		// set P0.15 & P0.16 & P0.17 INPUT

	LPC_PINCON->PINSEL7 &= ~(3<<18);	// set P0.25 GPIO
	LPC_PINCON->PINMODE7 &= ~(3<<18);	// set P0.25 PULL-UP
	LPC_GPIO3->FIODIR |= (1<<25);		// set P0.25 OUTPUT
}

void delay(uint32_t delay){
	for (volatile uint32_t i = 0; i < delay; i++) {}
}

void setDutyCycle() {
	duty_cycle = value * 100 / 7;
}

void readPort() {
	value = 0;
	if (LPC_GPIO0->FIOPIN & (1<<15)) {
		value += 1;
	}
	if (LPC_GPIO0->FIOPIN & (1<<16)) {
		value += 2;
	}
	if (LPC_GPIO0->FIOPIN & (1<<17)) {
		value += 4;
	}
}

void setOutputHIGH() {
	LPC_GPIO3->FIOSET |= (1<<25);
}

void setOutputLOW() {
	LPC_GPIO3->FIOCLR |= (1<<25);
}
