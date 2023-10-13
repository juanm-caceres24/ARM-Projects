#include "LPC17xx.h"

#define LED	(1<<28)	// -00010000000000000000000000000000-
#define BTN	(1<<27)	// -00001000000000000000000000000000-

int main(void) {

	// PORT configuration

	// PINSEL
	LPC_PINCON->PINSEL1 &= ~(0x3<<22);	// set digital -1111111001111111111111111111111-
	LPC_PINCON->PINSEL1 &= ~(0x3<<24);	// set digital -11111100001111111111111111111111-

	// FIODIR
	LPC_GPIO0->FIODIR |= LED;	// set output -00000000010000000000000000000000-
	LPC_GPIO0->FIODIR &= ~BTN;	// set input -00000000010000000000000000000000-

	// main loop
	while (1) {
		if (LPC_GPIO0->FIOPIN == (LPC_GPIO0->FIOPIN | BTN)) {
			LPC_GPIO0->FIOCLR |= LED;	// turn off LED
		} else {
			LPC_GPIO0->FIOSET |= LED;	// turn on LED
		}
	}

	// program end
    return 0;
}
