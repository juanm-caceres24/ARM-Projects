#include "LPC17xx.h"

#define LED	(1<<28) // -00010000000000000000000000000000-

void delay(uint32_t times);

int main(void) {

	// PORT configuration

	// PINSEL
	LPC_PINCON->PINSEL1 &= ~(0x3<<24); // set digital -11111100111111111111111111111111-

	// FIODIR
	LPC_GPIO0->FIODIR |= LED; // set output -00010000000000000000000000000000-

	// main loop
	while (1) {

		// turn on LED
		LPC_GPIO0->FIOSET |= LED;
		delay(1000);

		// turn off LED
		LPC_GPIO0->FIOCLR |= LED;
		delay(1500);
	}

	// program end
    return 0;
}

// delay subroutine
void delay(uint32_t times) {
	for(uint32_t i = 0; i < times; i++)
		for(uint32_t j = 0; j < times; j++);
}
