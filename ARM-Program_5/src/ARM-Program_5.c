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

#define PWM_0_PIN (1<<0) // P2.0
#define PWM_1_PIN (1<<1) // P2.1
#define MOTOR_0_DIRECTION_0_PIN (1<<2) // P2.2
#define MOTOR_0_DIRECTION_1_PIN (1<<3) // P2.3
#define MOTOR_1_DIRECTION_0_PIN (1<<4) // P2.4
#define MOTOR_1_DIRECTION_1_PIN (1<<5) // P2.5

#define TIME_IN_US 100 // Timer interval in microseconds
#define DEBOUNCE_DELAY_CYCLES 2000 // Cycles of TIME_IN_US that the button will be ignored (2000 * 100us = 200ms)
#define PWM_CYCLES 100 // Number of cycles (of TIME_IN_US) to consider a PWM cycle (100 * 100us = 10ms)

#define MAX_THROTTLE 4 // Maximum throttle level

uint32_t static debounce_0_counter = 0; // Decrement counter of cycles of TIME_IN_US
uint32_t static debounce_1_counter = 0;
uint32_t static debounce_2_counter = 0;
uint32_t static debounce_3_counter = 0;
uint32_t static debounce_4_counter = 0;
uint32_t static debounce_5_counter = 0;
uint32_t static motor_0_working_state = 0; // Working state of Motor 0 (0 = off, 1 = on)
uint32_t static motor_1_working_state = 0; // Working state of Motor 1 (0 = off, 1 = on)
uint32_t static motor_0_direction = 0; // Direction of Motor 0
uint32_t static motor_1_direction = 0; // Direction of Motor 1
uint32_t static motor_0_throttle = 0; // Throttle level of Motor 0
uint32_t static motor_1_throttle = 0; // Throttle level of Motor 1
uint32_t static pwm_0_counter = 0; // Counter for PWM cycles of Motor 0
uint32_t static pwm_1_counter = 0; // Counter for PWM cycles of Motor 1
uint32_t static motor_selection = 0; // Selection of the controlled motor

void configPorts();
void configEINT();
void configNVIC();
void configSysTick();
void updateMotor0();
void updateMotor1();

int main() {
	SystemInit();
	configPorts();
	configEINT();
	configNVIC();
	configSysTick();
	while (1) {
		updateMotor0();
		updateMotor1();
	}
	return 0;
}

/*
 * CONFIGURATION METHODS
 */

void configPorts() {

	/*
	 * BUTTONS
	 */
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

	/*
	 * CONTROL LEDS
	 */
	LPC_PINCON->PINSEL4 &= ~(3<<0); // Set P2.0 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<0); // Set P2.0 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<0); // Set P2.0 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<2); // Set P2.1 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<2); // Set P2.1 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<1); // Set P2.1 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<4); // Set P2.2 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<4); // Set P2.2 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<2); // Set P2.2 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<6); // Set P2.3 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<6); // Set P2.3 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<3); // Set P2.3 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<8); // Set P2.4 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<8); // Set P2.4 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<4); // Set P2.4 as OUTPUT

	LPC_PINCON->PINSEL4 &= ~(3<<10); // Set P2.5 as GPIO
	LPC_PINCON->PINMODE4 &= ~(2<<10); // Set P2.5 neither PULL-UP nor PULL-DOWN
	LPC_GPIO2->FIODIR |= (1<<5); // Set P2.5 as OUTPUT
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
 * INTERRUPTION HANDLERS
 */

void EINT0_IRQHandler() {
    if (debounce_0_counter == 0) {
        debounce_0_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (motor_selection) {
        	motor_selection = 0;
        } else {
        	motor_selection = 1;
        }
    }
    LPC_SC->EXTINT |= (1<<0);
}

void EINT1_IRQHandler() {
    if (debounce_1_counter == 0) {
        debounce_1_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (motor_selection == 0) {
        	if (motor_0_working_state) {
        		motor_0_working_state = 0;
        	} else {
        		motor_0_working_state = 1;
        	}
        } else {
        	if (motor_1_working_state) {
				motor_1_working_state = 0;
			} else {
				motor_1_working_state = 1;
			}
        }
    }
    LPC_SC->EXTINT |= (1<<1);
}

void EINT2_IRQHandler() {
	if (debounce_2_counter == 0) {
		debounce_2_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
		if (motor_selection == 0) {
			if (motor_0_direction == 0) { // If direction is forward
				if (motor_0_throttle > 0) {
					motor_0_throttle--; // Decrease throttle
				} else {
					motor_0_direction = 1; // Change direction to reverse if throttle is zero
				}
			} else { // If direction is reverse
				if (motor_0_throttle < MAX_THROTTLE) {
					motor_0_throttle++; // Increase throttle
				}
			}
		} else {
			if (motor_1_direction == 0) { // If direction is forward
				if (motor_1_throttle > 0) {
					motor_1_throttle--; // Decrease throttle
				} else {
					motor_1_direction = 1; // Change direction to reverse if throttle is zero
				}
			} else { // If direction is reverse
				if (motor_1_throttle < MAX_THROTTLE) {
					motor_1_throttle++; // Increase throttle
				}
			}
		}
	}
	LPC_SC->EXTINT |= (1<<2);
}

void EINT3_IRQHandler() {
    if (debounce_3_counter == 0) {
        debounce_3_counter = DEBOUNCE_DELAY_CYCLES; // Set the debounce counter
        if (motor_selection == 0) {
        	if (motor_0_direction == 0) { // If direction is forward
				if (motor_0_throttle < MAX_THROTTLE) {
					motor_0_throttle++; // Increase throttle
				}
			} else { // If direction is reverse
				if (motor_0_throttle > 0) {
					motor_0_throttle--; // Decrease throttle
				} else {
					motor_0_direction = 0; // Change direction to forward if throttle is zero
				}
			}
        } else {
        	if (motor_1_direction == 0) { // If direction is forward
				if (motor_1_throttle < MAX_THROTTLE) {
					motor_1_throttle++; // Increase throttle
				}
			} else { // If direction is reverse
				if (motor_1_throttle > 0) {
					motor_1_throttle--; // Decrease throttle
				} else {
					motor_1_direction = 0; // Change direction to forward if throttle is zero
				}
			}
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
    if (debounce_4_counter > 0) {
        debounce_4_counter--; // Decrement the debounce counter
    }
    if (debounce_5_counter > 0) {
        debounce_5_counter--; // Decrement the debounce counter
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

void updateMotor0() {
    if (motor_0_direction) {
        LPC_GPIO2->FIOCLR |= MOTOR_0_DIRECTION_0_PIN;
        LPC_GPIO2->FIOSET |= MOTOR_0_DIRECTION_1_PIN;
    } else {
    	LPC_GPIO2->FIOSET |= MOTOR_0_DIRECTION_0_PIN;
    	LPC_GPIO2->FIOCLR |= MOTOR_0_DIRECTION_1_PIN;
    }
    if (motor_0_working_state) {
        if (pwm_0_counter < (motor_0_throttle * (PWM_CYCLES / MAX_THROTTLE))) {
            LPC_GPIO2->FIOSET |= PWM_0_PIN;
        } else {
        	LPC_GPIO2->FIOCLR |= PWM_0_PIN;
        }
    } else {
    	LPC_GPIO2->FIOCLR |= PWM_0_PIN;
    }
}

void updateMotor1() {
	if (motor_1_direction) {
		LPC_GPIO2->FIOCLR |= MOTOR_1_DIRECTION_0_PIN;
		LPC_GPIO2->FIOSET |= MOTOR_1_DIRECTION_1_PIN;
	} else {
		LPC_GPIO2->FIOSET |= MOTOR_1_DIRECTION_0_PIN;
		LPC_GPIO2->FIOCLR |= MOTOR_1_DIRECTION_1_PIN;
	}
	if (motor_1_working_state) {
		if (pwm_1_counter < (motor_1_throttle * (PWM_CYCLES / MAX_THROTTLE))) {
			LPC_GPIO2->FIOSET |= PWM_1_PIN;
		} else {
			LPC_GPIO2->FIOCLR |= PWM_1_PIN;
		}
	} else {
		LPC_GPIO2->FIOCLR |= PWM_1_PIN;
	}
}
