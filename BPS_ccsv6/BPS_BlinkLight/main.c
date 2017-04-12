#include <msp430.h> 

/*
 * main.c
 */
int main(void) {
	 WDTCTL = WDT_MDLY_32;                     // WDT 32ms, SMCLK, interval timer
	  SFRIE1 |= WDTIE;                          // Enable WDT interrupt
	  P1DIR |= 0x01;                            // Set P1.0 to output direction

	  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
	  __no_operation();                         // For debugger
	}

	// Watchdog Timer interrupt service routine
	#pragma vector=WDT_VECTOR
	__interrupt void WDT_ISR(void)
	{
	  P1OUT ^= 0x01;                            // Toggle P1.0 (LED)
	}
