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

#define TIME_IN_US 100 // 0.1ms
#define DATA_BASE ((uint32_t*)0x2007C000)
#define DATA_LIMIT ((uint32_t*)0x2007000F)
#define READ_PERIOD_CYCLES 2000 // 200ms
#define WRITE_PERIOD_CYCLES 4000 // 400ms
#define READ_BIT_CYCLES 4

uint32_t static debounce_0 = 0;
uint32_t static debounce_1 = 0;
uint32_t static debounce_2 = 0;
uint32_t static debounce_3 = 0;
uint32_t static *read_data_pointer = DATA_BASE;
uint32_t static *write_data_pointer = DATA_BASE;
uint32_t static update_write = 0;
uint32_t static update_read = 0;
uint32_t static read_period_counter = 0;
uint32_t static write_period_counter = 0;
uint32_t static value = 0;
uint32_t static bit_read_counter = 0;
uint32_t static clock_led_status = 0;

void configPorts();
void configEINT();
void configNVIC();
void configSysTick();
void updateWrite();
void updateRead();
void toggleLED1();

int main(void) {
	SystemInit();
	configPorts();
	configEINT();
	configNVIC();
	configSysTick();
	while (1) {
		if (update_write) {
			updateWrite();
		}
		if (update_read) {
			updateRead();
		}
	}
    return 0 ;
}

/*
 * CONFIGURATION METHODS
 */

void configPorts() {
	LPC_PINCON->PINSEL4 &= ~(3<<20); // Clear P2.10
	LPC_PINCON->PINSEL4 |=  (1<<20); // Set P2.10 as EINT0
	LPC_PINCON->PINMODE4 &= ~(3<<20); // Set P2.10 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1<<10); // Set P2.10 as INPUT

	LPC_PINCON->PINSEL4 &= ~(3<<22); // Clear P2.11
	LPC_PINCON->PINSEL4 |=  (1<<22); // Set P2.11 as EINT1
	LPC_PINCON->PINMODE4 &= ~(3<<22); // Set P2.11 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1<<11); // Set P2.11 as INPUT

	LPC_PINCON->PINSEL4 &= ~(3<<24); // Clear P2.12
	LPC_PINCON->PINSEL4 |=  (1<<24); // Set P2.12 as EINT2
	LPC_PINCON->PINMODE4 &= ~(3<<24); // Set P2.12 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1<<12); // Set P2.12 as INPUT

	LPC_PINCON->PINSEL4 &= ~(3<<26); // Clear P2.13
	LPC_PINCON->PINSEL4 |=  (1<<26); // Set P2.13 as EINT3
	LPC_PINCON->PINMODE4 &= ~(3<<26); // Set P2.13 with PULL-UP
	LPC_GPIO2->FIODIR &= ~(1<<13); // Set P2.13 as INPUT

	LPC_PINCON->PINSEL4 &= ~(3<<0); // Set P2.0 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<0); // Set P2.0 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<0); // Set P2.0 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<2); // Set P2.1 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<2); // Set P2.1 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<1); // Set P2.1 as OUTPUT
}

void configEINT() {
	LPC_SC->EXTINT |= (1<<0); // Enable EINT0 as external interrupt
	LPC_SC->EXTMODE |= (1<<0); // Set EINT0 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<0); // Set EINT0 interruption in falling edge

	LPC_SC->EXTINT |= (1<<1); // Enable EINT1 as external interrupt
	LPC_SC->EXTMODE |= (1<<1); // Set EINT1 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<1); // Set EINT1 interruption in falling edge

	LPC_SC->EXTINT |= (1<<2); // Enable EINT2 as external interrupt
	LPC_SC->EXTMODE |= (1<<2); // Set EINT2 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<2); // Set EINT2 interruption in falling edge

	LPC_SC->EXTINT |= (1<<3); // Enable EINT3 as external interrupt
	LPC_SC->EXTMODE |= (1<<3); // Set EINT3 as edge sensitive
	LPC_SC->EXTPOLAR &= ~(1<<3); // Set EINT3 interruption in falling edge
}

void configNVIC() {
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

void configSysTick() {
	SysTick->LOAD = (SystemCoreClock / 1000000) * TIME_IN_US - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<0) | (1<<1) | (1<<2); // Enable SysTick counter, enable SysTick interruptions and select internal clock
}

/*
 * INTERRUPTIONS HANDLERS
 */

void EINT0_IRQHandler() {
	if (debounce_0 == 0) {
		debounce_0 = 1;
		value += 1;
	}
	LPC_SC->EXTINT |= (1<<0);
}

void EINT1_IRQHandler() {
	if (debounce_1 == 0) {
		debounce_1 = 1;
		value += 2;
	}
	LPC_SC->EXTINT |= (1<<1);
}

void EINT2_IRQHandler() {
	if (debounce_2 == 0) {
		debounce_2 = 1;
		value += 4;
	}
	LPC_SC->EXTINT |= (1<<2);
}

void EINT3_IRQHandler() {
	if (debounce_3 == 0) {
		debounce_3 = 1;
		value += 8;
	}
	LPC_SC->EXTINT |= (1<<3);
}

void SysTick_Handler() {
	read_period_counter++;
	if (read_period_counter >= READ_PERIOD_CYCLES) {
		read_period_counter = 0;
		update_read = 1;
	}
	write_period_counter++;
	if (write_period_counter >= WRITE_PERIOD_CYCLES) {
		write_period_counter = 0;
		debounce_0 = 0;
		debounce_1 = 0;
		debounce_2 = 0;
		debounce_3 = 0;
		update_write = 1;
	}
}

/*
 * GENERAL METHODS
 */

void updateWrite() {
	*write_data_pointer = value;
	write_data_pointer++;
	if (write_data_pointer >= DATA_LIMIT) {
		write_data_pointer = DATA_BASE;
	}
	value = 0;
	update_write = 0;
}

void updateRead() {
	uint32_t value_tmp = *read_data_pointer & 0x01;
	*read_data_pointer >>= 1;
	if (value_tmp) {
		LPC_GPIO2->FIOSET |= (1<<0);
	} else {
		LPC_GPIO2->FIOCLR |= (1<<0);
	}
	bit_read_counter++;
	if (bit_read_counter >= READ_BIT_CYCLES) {
		bit_read_counter = 0;
		read_data_pointer++;
		if (read_data_pointer >= DATA_LIMIT) {
			read_data_pointer = DATA_BASE;
		}
	}
	toggleLED1();
	update_read = 0;
}

void toggleLED1() {
	if (clock_led_status) {
		LPC_GPIO2->FIOCLR = (1<<1);
	} else {
		LPC_GPIO2->FIOSET = (1<<1);
	}
	clock_led_status = !clock_led_status;
}
