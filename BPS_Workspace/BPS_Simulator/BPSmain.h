/*
 *
 * BPS MAIN.H
 * Originally Created for BPS_V1 2012
 * Revised for BPS_V2 2015 by Scott Haver
 * 2015 Western Michigan University Sunseeker
 *
 */

#ifndef BPSMAIN_H_
#define BPSMAIN_H_

#include <msp430x54xa.h>
#include "LTC6803.h"
#include "RS232.h"
#include "ad7739_func.h"
#include <stdio.h>

void io_init(void);
void clock_init(void);

void timerB_init(void);


static inline void delay(void)
{
  volatile int jj;
  volatile int ii;
  for (ii = 0; ii < 4; ii++)
  {
    for (jj = 0; jj < 1000; jj++)
    {
      asm(" nop"); //The space is necessary or else the assember things nop is a label!
    }
  }
}

// Event timing  
#define SMCLK_RATE		8000000	// Hz
#define ACLK_RATE		32768	// Hz
#define TICK_RATE		100		// Hz
#define CM_STATUS_COUNT			13			// Number of ticks per event: ~0.133 sec
#define LTC_STATUS_COUNT		13			// Number of ticks per event: ~0.133 sec
#define TEXT_COMMS_COUNT	 100*7			// Number of ticks per event: 7 sec
#define CAN_COMMS_COUNT		100*10			// Number of ticks per event: 11 sec
#define EXTPC_COMMS_COUNT	100*29			// Number of ticks per event: 29 sec

// C == 5.25*6 = 31.5. Discharge 2C, Charge 1C
// Hopefulley - 60 AMps (-60 mV to +31.5 mV at the shunt)
#define KI_DISCHARGE  	+60000.0
#define KI_CHARGE  		-31500.0

// ADC scaling constants */
#define CURRENT_FULL_SCALE    16777216.0
#define CURRENT_I_SCALE      50.0 * 2500.0  // Scale to be in mA

/******************** Pin Definitions *************************/

// PORT 1
#define LED0		      	0x01
#define LED1			    0x02
#define BUTTON1				0x04
#define BUTTON2 			0x08
#define CAN_RSTn	        0x10
#define CAN_CSn		        0x20
#define CAN_RX0n			0x40
#define CAN_RX1n		    0x80
#define P1_UNUSED			0x00

// PORT 2
#define CAN_INTn			0x01
#define ADC1_RDY			0x02
#define ADC2_RDY			0x04
#define ADC3_RDY			0x08
#define ADC4_RDY			0x10 
#define ADC5_RDY			0x20  
#define ADC6_RDY			0x40  
#define ADC7_RDY			0x80  
#define P2_UNUSED			0x00

// PORT 3
#define ADC_bus1_CLK		0x01  
#define CAN_SIMO			0x02  
#define CAN_SOMI			0x04  
#define CAN_SCLK			0x08
#define ADC_bus1_MOSI		0x10  
#define ADC_bus1_MISO		0x20  
#define ADC_misc_CLK		0x40  
#define ADC_bus2_MOSI		0x80  
#define P3_UNUSED			0x00

// PORT 4
#define ADC_CS1				0x01
#define ADC_CS2				0x02
#define ADC_CS3				0x04
#define ADC_CS4				0x08
#define ADC_CS5				0x10
#define ADC_CS6				0x20
#define ADC_CS7			    0x40
#define ADC_RSTn			0x80
#define P4_UNUSED			0x00

// PORT 5
#define P50					0x01
#define P51					0x02
#define XT2IN				0x04
#define XT2OUT				0x08
#define ADC_bus2_MISO		0x10
#define ADC_bus2_CLK		0x20
#define ADC_misc_MOSI	    0x40
#define ADC_misc_MISO		0x80
#define P5_UNUSED			0x01 | 0x02 

// PORT 6
#define LED2				0x01
#define LED3				0x02
#define LED4				0x04
#define LED5				0x08
#define RELAY_BATT			0x10
#define RELAY_ARRAY			0x20
#define RELAY_MCPC			0x40
#define RELAY_MC		    0x80
#define P6_UNUSED			0x00

// PORT 7
#define XT1IN				0x01
#define XT1OUT				0x02
#define P72					0x04
#define P73					0x08
#define RELAY_SCPC			0x10
#define RELAY_SC			0x20
#define EXT_RELAY_SCPC		0x40
#define EXT_RELAY_MCPC		0x80
#define P7_UNUSED			0x04 | 0x08 

// PORT 8
#define P80				    0x01
#define P81			    	0x02
#define P82			        0x04
#define P83					0x08
#define P84					0x10
#define LT_CS3		    	0x20
#define LT_CS2		    	0x40
#define LT_CS1			    0x80
#define P8_UNUSED			0x01 | 0x02 | 0x04 | 0x08 | 0x10

// PORT 9
#define LT_SCLK_1			0x01
#define LT_MOSI_2			0x02
#define LT_MISO_2			0x04
#define LT_SCLK_2			0x08
#define LT_MOSI_1			0x10
#define LT_MISO_1			0x20
#define P96				    0x40
#define p97				    0x80
#define P9_UNUSED			0x40 | 0x80

// PORT 10
#define P100				0x01
#define LT_MOSI_3			0x02
#define LT_MISO_3			0x04
#define LT_SCLK_3			0x08
#define EXT_TX				0x10
#define EXT_RX				0x20
#define P106				0x40
#define P107     			0x80
#define P10_UNUSED			0x01 | 0x40 | 0x80

// PORT 11
#define ACLK_TEST			0x01
#define MCLK_TEST			0x02
#define SMCLK_TEST		   	0x04

// PORT J
#define JTAG_TDO			0x01
#define JTAG_TDI			0x02
#define JTAG_TMS			0x04
#define JTAG_TCK			0x08

// Constant Definitions
#define	TRUE				1
#define FALSE				0

/*
 *   Relay notation:
 *
 *	 MC     : Motor Controller 
 *	 MCPC   : Motor Controller Pre-Charge
 *   SC     : Super CAP
 * 	 SCPC   : Super CAP Pre-Charge
 *   BATT   : Battery Contactor
 *   ARRAY  : Array Contactor
 *   EXT_   : external relay on precharge board
 *
*/


//Relay ON/OFF definitions
#define relay_batt_close  		P6OUT |= RELAY_BATT;
#define relay_batt_open 		P6OUT &= ~RELAY_BATT;
#define relay_array_close 		P6OUT |= RELAY_ARRAY
#define relay_array_open  		P6OUT &= ~RELAY_ARRAY
#define relay_mcpc_close  		P6OUT |= RELAY_MCPC
#define relay_mcpc_open   		P6OUT &= ~RELAY_MCPC
#define relay_mc_close   		P6OUT |= RELAY_MC
#define relay_mc_open    		P6OUT &= ~RELAY_MC

#define relay_scpc_close    	P7OUT |= RELAY_SCPC
#define relay_scpc_open      	P7OUT &= ~RELAY_SCPC
#define relay_sc_close			P7OUT |= RELAY_SC
#define relay_sc_open			P7OUT &= ~RELAY_SC
#define ext_relay_scpc_close 	P7OUT |= EXT_RELAY_SCPC
#define ext_relay_scpc_open		P7OUT &= ~EXT_RELAY_SCPC
#define ext_relay_mcpc_close	P7OUT |= EXT_RELAY_MCPC
#define ext_relay_mcpc_open		P7OUT &= ~EXT_RELAY_MCPC

/*
 *   LED Notation:
 *
 *	 LED_ERROR_ON    : Turn on Error Light
 *	 LED_ERROR_OFF   : Turn off Error Light
 *	 LED_NORMALOP_ON : Turn on normal operation light
 *	 LED_NORMALOP_OFF: Turn off normal operation light
 *   LED_INIT_ON     : Turn on initialization light
 * 	 LED_INIT_OFF    : Turn off initialization light
 *   LED_PC_ON       : Turn on pre-charge light
 *   LED_PC_OFF      : Turn off pre-charge light
 *
*/

#define LED_ERROR_ON  		P1OUT |= LED0;
#define LED_ERROR_OFF 		P1OUT &= ~LED0;
#define LED_NORMALOP_ON  	P1OUT |= LED1;
#define LED_NORMALOP_OFF 	P1OUT &= ~LED1;
#define LED_INIT_ON   		P6OUT &= ~LED2;	//active low
#define LED_INIT_OFF  		P6OUT |=  LED2;
#define LED_PC_ON	  		P6OUT &= ~LED3;	//active low
#define LED_PC_OFF    	    P6OUT |=  LED3;



#endif /*BPSMAIN_H_*/
