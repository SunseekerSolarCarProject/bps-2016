/*
 * 
 * Test will send data to CAN
 *
 */

#include <stdio.h>
#include <msp430x54xa.h>
#include <ctype.h>
#include <math.h>
#include <float.h>
#include "BPSmain.h"
#include "can.h"
unsigned char count = 0;

/*=================================== **MAIN** =========================================*/
//////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	/* Stop watchdog timer */
	WDTCTL = WDTPW + WDTHOLD;             

	/* Init I/O PORT Setting */
	io_init();
	
	/* Configure HF and LF clocks */
	clock_init();

	canspi_init();
	can_init();

	/* Setup TIMERA0 for 1s interrupts */
	TA1CCTL0 = CCIE; 						   //CCR0 interrupt enabled
	TA1CCR0 = 32768;						   //interrupt every 32768 clock cycles (1 second)
	TA1CTL = TASSEL__ACLK + MC_1 + TACLR;      //ACLK, cont mode, clear tar
	
	__bis_SR_register(GIE); 				   //enable global interrupts
	__no_operation();
	
	while(1)								   //loop forever
	{
		can.address = 0x481;
		can.data.data_u8[7] = 'A';
		can.data.data_u8[6] = 'B';
		can.data.data_u8[5] = 'C';
		can.data.data_u8[4] = 'D';
		can.data.data_u32[0] = DEVICE_SERIAL;
		can_transmit();

	}
}
/*================================ ** END MAIN** =========================================*/
////////////////////////////////////////////////////////////////////////////////////////////



/*================================ TIMER1_A0 Interrupt ==================================*/
//Timer A0 interrupt service routine

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) //LED1 Flash
{
	//Blink LED0
	P1OUT ^= LED0;
	
	//Blink LED2:5 
	switch(count)
	{
		case 0:
		P6OUT ^= LED2;
		break;
		case 1:
		P6OUT ^= LED3;
		break;
		case 2:
		P6OUT ^= LED4;
		break;
		case 3:
		P6OUT ^= LED5;
		break;
	}
	count++;
	if(count == 4)
	{
		count = 0;
	}



}
/*===============================END TIMER1_A0 Interrupt ================================*/




