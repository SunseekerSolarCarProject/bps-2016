/*
 *  ADC SPI Initialization
 *
 *  Originally Written for BPS_V1 2014
 *
 *  Modified for BPS_V2 2015 by Scott Haver
 *
 */
 
#include <msp430x54xa.h>
#include "ad7739_func.h"

 //#######################################################//

 /* Function Notation Description 
  *
  * adc1 + adc2 + adc3 = adc_bus1  (UCA0)   //BUS1
  * adc4 + adc5 + adc6 = adc_bus2  (UCB1)   //BUS2
  * adc7 = misc_adc (UCA1)                  //MISC
  *
  */
  //#######################################################//


/*============================= BUS1 USCA0 SPI Functions ======================================*/

// Initilize BUS1 UCA0 SPI PORT
 
void adc_bus1_spi_init( void )
{
	UCA0CTL1 |= UCSWRST;		//software reset
    UCA0CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC; // CKPL=11, SPI Master, 3-pin mode.
    UCA0CTL1 = UCSSEL1 | UCSSEL0 | UCSWRST; // SMCLK, Software reset

    // Baud Rate Select
    UCA0BR0 = 0x02;				// SPICLK (8 MHz)/8 = BRCLK (1 MHz)
    UCA0BR1 = 0x00;				// SPICLK/8 = BRCLK
 
	UCA0STAT = 0x00;			//not in loopback mode
            
    UCA0CTL1 &= ~UCSWRST; // SPI enable
    
    //UCA0IE |= UCTXIE | UCRXIE; 	// Interrupt enables
      
}

/*
 *  Transmits data on UCA0 SPI connection
 *	- Busy waits until entire shift is complete
 */
void adc_bus1_transmit( unsigned char data )
{
	UCA0TXBUF = data;
	while(( UCA0IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	UCA0RXBUF;
}

/*
 * Exchanges data on UCA0 SPI connection
 *	- Busy waits until entire shift is complete
 */
unsigned char adc_bus1_exchange( unsigned char data )
{  
    UCA0TXBUF = data;
	while(( UCA0IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	return (UCA0RXBUF);
}
/*================================== End BUS1 USCA0 SPI Functions ======================================*/

/*============================= BUS2 USCB1 SPI Functions ======================================*/

// Initilize BUS1 UCB1 SPI PORT
 
void adc_bus2_spi_init( void )
{
	UCB1CTL1 |= UCSWRST;		//software reset
    UCB1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC; // CKPL=11, SPI Master, 3-pin mode.
    UCB1CTL1 = UCSSEL1 | UCSSEL0 | UCSWRST; // SMCLK, Software reset

    // Baud Rate Select
    UCB1BR0 = 0x02;				// SPICLK (8 MHz)/8 = BRCLK (1 MHz)
    UCB1BR1 = 0x00;				// SPICLK/8 = BRCLK
 
	UCB1STAT = 0x00;			//not in loopback mode
            
    UCB1CTL1 &= ~UCSWRST; // SPI enable
    
    //UCB1IE |= UCTXIE | UCRXIE; 	// Interrupt enables
      
}

/*
 *  Transmits data on UCB1 SPI connection
 *	- Busy waits until entire shift is complete
 */
void adc_bus2_transmit( unsigned char data )
{
	UCB1TXBUF = data;
	while(( UCB1IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	UCB1RXBUF;
}

/*
 * Exchanges data on UCB1 SPI connection
 *	- Busy waits until entire shift is complete
 */
unsigned char adc_bus2_exchange( unsigned char data )
{  
    UCB1TXBUF = data;
	while(( UCB1IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	return (UCB1RXBUF);
}
/*================================== End BUS2 USCB1 SPI Functions ======================================*/


/*============================= MISC USCA1 SPI Functions ======================================*/

// Initilize UCA1 SPI Port
 
void adc_misc_spi_init( void )
{
	UCA1CTL1 |= UCSWRST;		//software reset
    UCA1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC; // CKPL=11, SPI Master, 3-pin mode.
    UCA1CTL1 = UCSSEL1 | UCSSEL0 | UCSWRST; // SMCLK, Software reset

    // Baud Rate Select
    UCA1BR0 = 0x02;				// SPICLK (8 MHz)/8 = BRCLK (1 MHz)
    UCA1BR1 = 0x00;				// SPICLK/8 = BRCLK
 
	UCA1STAT = 0x00;			//not in loopback mode
    UCA1MCTL = 0x00;			// No modulation
            
    UCA1CTL1 &= ~UCSWRST; 		// SPI enable
    
	//UCA1IE |= UCTXIE | UCRXIE; 	// Interrupt enables
        
}

/*
 * Transmits data on UCA1 SPI connection
 *	- Busy waits until entire shift is complete
 */
void adc_misc_transmit( unsigned char data )
{
	UCA1TXBUF = data;
	while(( UCA1IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	UCA1RXBUF;
}

/*
 * Exchanges data on UCA1 SPI connection
 *	- Busy waits until entire shift is complete
 */
unsigned char adc_misc_exchange( unsigned char data )
{  
    UCA1TXBUF = data;
	while(( UCA1IFG & UCRXIFG ) == 0x00 );	// Wait for Rx completion (implies Tx is also complete)
	return (UCA1RXBUF);
}

/*================================== End MISC USCA1 SPI Functions ======================================*/

