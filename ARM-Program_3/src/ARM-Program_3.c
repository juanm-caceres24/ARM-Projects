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

#define BUTTON_0_PIN (1<<10) // P2.10
#define BUTTON_1_PIN (1<<11) // P2.11
#define BUTTON_2_PIN (1<<12) // P2.12
#define BUTTON_3_PIN (1<<13) // P2.13
#define LED_0_PIN (1<<0) // P2.0
#define LED_1_PIN (1<<1) // P2.1
#define TIME_IN_US 100 // Period of the SysTick interruptions in microseconds (0.1ms)
#define DEBOUNCE_DELAY_CYCLES 2000 // Times of TIME_IN_US to ignore inputs considered rebounds of the input (200ms)
#define PWM_CYCLES 100 // Times of TIME_IN_US to consider a PWM full-cycle (10ms)
#define MAX_BRIGHTNESS 5

uint32_t debounce_0_counter = 0;
uint32_t debounce_1_counter = 0;
uint32_t debounce_2_counter = 0;
uint32_t debounce_3_counter = 0;
uint32_t pwm_0_counter = 0;
uint32_t pwm_1_counter = 0;
uint32_t led_0_brightness = MAX_BRIGHTNESS;
uint32_t led_1_brightness = MAX_BRIGHTNESS;

void configPorts();
void configEINT();
void configNVIC();
void configSysTick();
void configADC();
void updateLED0();
void updateLED1();

/*
 * MAIN
 */

int main(void) {
	SystemInit();
	configPorts();
	configEINT();
	configNVIC();
	configSysTick();
	while (1) {
		updateLED0();
		updateLED1();
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

void configADC() {

}

/*
 * INTERRUPTIONS HANDLERS
 */

void EINT0_IRQHandler() {
	if (debounce_0_counter == 0) {
		debounce_0_counter = DEBOUNCE_DELAY_CYCLES;
		if (led_0_brightness > 0) {
			led_0_brightness--;
		}
	}
	LPC_SC->EXTINT |= (1<<0);
}

void EINT1_IRQHandler() {
	if (debounce_1_counter == 0) {
		debounce_1_counter = DEBOUNCE_DELAY_CYCLES;
		if (led_0_brightness < MAX_BRIGHTNESS) {
			led_0_brightness++;
		}
	}
	LPC_SC->EXTINT |= (1<<1);
}

void EINT2_IRQHandler() {
	if (debounce_2_counter == 0) {
		debounce_2_counter = DEBOUNCE_DELAY_CYCLES;
		if (led_1_brightness > 0) {
			led_1_brightness--;
		}
	}
	LPC_SC->EXTINT |= (1<<2);
}

void EINT3_IRQHandler() {
	if (debounce_3_counter == 0) {
		debounce_3_counter = DEBOUNCE_DELAY_CYCLES;
		if (led_1_brightness < MAX_BRIGHTNESS) {
			led_1_brightness++;
		}
	}
	LPC_SC->EXTINT |= (1<<3);
}

void SysTick_Handler() {
	if (debounce_0_counter > 0) {
		debounce_0_counter--; // Decrement the debounce counter
	}
	if (debounce_1_counter > 0) {
		debounce_1_counter--; // Decrement the debounce counter
	}
	if (debounce_2_counter > 0) {
		debounce_2_counter--; // Decrement the debounce counter
	}
	if (debounce_3_counter > 0) {
		debounce_3_counter--; // Decrement the debounce counter
	}
	pwm_0_counter++;
	if (pwm_0_counter >= PWM_CYCLES) {
		pwm_0_counter = 0; // Reset the PWM counter after a full cycle
	}
	pwm_1_counter++;
	if (pwm_1_counter >= PWM_CYCLES) {
		pwm_1_counter = 0; // Reset the PWM counter after a full cycle
	}
}

/*
 * GENERAL METHODS
 */

void updateLED0() {
	if (pwm_0_counter < (led_0_brightness * (PWM_CYCLES / MAX_BRIGHTNESS))) {
		LPC_GPIO2->FIOSET |= LED_0_PIN; // -GENERIC VALUE-
	} else {
		LPC_GPIO2->FIOCLR |= LED_0_PIN; // -GENERIC VALUE-
	}
}

void updateLED1() {
	if (pwm_1_counter < (led_1_brightness * (PWM_CYCLES / MAX_BRIGHTNESS))) {
		LPC_GPIO2->FIOSET |= LED_1_PIN; // -GENERIC VALUE-
	} else {
		LPC_GPIO2->FIOCLR |= LED_1_PIN; // -GENERIC VALUE-
	}
}
