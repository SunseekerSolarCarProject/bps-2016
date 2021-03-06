/*
 *  Originally Written for BPS2012 PCB Development
 * 
 *  Modified for BPS_V2 2015 by Scott Haver
 *
 *  Clock and microcontroller Initialization RS232
 */

// Include files
#include<msp430x54xa.h>
#include "BPSmain.h"
#include "RS232.h"

/*********************************************************************************/
// BPS to PC External RS-232 (voltage isolated)
/*********************************************************************************/
void BPS2PC_init(void)
{
    UCA3CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA3CTL1 |= UCSSEL_2;                     // SMCLK
//    UCA3BRW = 833;                            // 8 MHz 9600 (see User's Guide)
    UCA3BRW = 208;                            // 8 MHz 38400 (see User's Guide)
    UCA3MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
	
	UCA3IFG &= ~UCTXIFG;			//Clear Xmit and Rec interrupt flags
	UCA3IFG &= ~UCRXIFG;
	UCA3CTL1 &= ~UCSWRST;			// initalize state machine
//	UCA3ABCTL |= UCABDEN;			// automatic baud rate 
//  UCA3IE |= UCRXIE|UCTXIE;		//enable TX & RX interrupt
}

void BPS2PC_putchar(char data)
{
    while((UCA3IFG & UCTXIFG) == 0);
    UCA3TXBUF = data; 
}

unsigned char BPS2PC_getchar(void)
{
	char i = 100;

    while((UCA3IFG & UCRXIFG) == 0 && i != 0)
    {
    	i--;
    }
    return(UCA3RXBUF);
}

int BPS2PC_puts(char *str)
{
	int i;
    char ch;
    i = 0;

    while((ch=*str)!= '\0')
    {
    	BPS2PC_putchar(ch);
    	str++;
    	i++;
    }
    BPS2PC_putchar(0x0A);
    BPS2PC_putchar(0x0D);
    
    return(i);
}

int BPS2PC_gets(char *ptr)
{
    int i;
    i = 0;
    while (1) {
          *ptr = BPS2PC_getchar();
          if (*ptr == 0x0D){
             *ptr = 0;
             return(i);
          }
          else 
          {
          	ptr++;
          	i++;
          }
     }
}

void BPS2PC_put_int(void)
{
    extern char *putPC_ptr;
    extern char put_status_PC;
    char ch;
    
    ch = *putPC_ptr++;

	if (ch == '\0')
	{
		UCA3IE &= ~UCTXIE;
		put_status_PC = FALSE;
	}
	else
	{
		UCA3TXBUF = ch;
		UCA3IE |= UCTXIE;
		put_status_PC = TRUE;
	}
}

