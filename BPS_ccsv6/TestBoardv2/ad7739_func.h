#ifndef AD7739_FUNC_H_
#define AD7739_FUNC_H_

/*
 * Bazuin ADC header file for an AD7734
 *
 * Modified: B. Bazuin 5/7/2009
 *
 * Modified for BPS_V2 2015 by Scott Haver
 *
 * Sunseeker 2015
 *
 */
//#######################################################//

 /* Function Notation Description 
  *
  * adc1 + adc2 + adc3 = adc_bus1  (UCA0)   //BUS1
  * adc4 + adc5 + adc6 = adc_bus2  (UCB1)   //BUS2
  * adc7 = misc_adc (UCA1)                  //MISC
  *
  */
  //#######################################################//
  
 
// MSP430 SPI interface header file

// Public Function Prototypes

//BUS1
extern void adc_bus1_spi_init( void );
extern void adc_bus1_transmit( unsigned char data );
extern unsigned char adc_bus1_exchange( unsigned char data );
 
//BUS2
extern void adc_bus2_spi_init( void );                         
extern void adc_bus2_transmit( unsigned char data );
extern unsigned char adc_bus2_exchange( unsigned char data );

//MISC ADC
extern void adc_misc_spi_init( void );                         
extern void adc_misc_transmit( unsigned char data );
extern unsigned char adc_misc_exchange( unsigned char data );

// Public Function Prototypes BUS1
void adc_bus1_reset(void);
extern char adc_bus1_init(void);
void adc_bus1_ctset(char adc_tcword, char adc_channel, char adc_device);
extern char adc_bus1_selfcal(char adc_device);
extern signed long  adc_bus1_zselfcal(char adc_channel, char adc_device);
extern signed long  adc_bus1_fselfcal(char adc_channel, char adc_device);
extern signed long adc_bus1_in(char adc_channel, char adc_device);
extern signed long adc_bus1_indump(char adc_channel, char adc_device);
extern void adc_bus1_contconv_start(char adc_channel, char adc_device);
extern void adc_bus1_idle(char adc_channel, char adc_device);
extern void adc_bus1_read_convert(char adc_channel, char adc_device);
extern void adc_bus1_io(char adc_device);  
extern char adc1_status(void);
extern char adc2_status(void);
extern char adc3_status(void);
extern char adc_bus1_chstatus(char adc_channel, char adc_device);

// Public Function Prototypes BUS2
void adc_bus2_reset(void);
extern char adc_bus2_init(void);
void adc_bus2_ctset(char adc_tcword, char adc_channel, char adc_device);
extern char adc_bus2_selfcal(char adc_device);
extern signed long  adc_bus2_zselfcal(char adc_channel, char adc_device);
extern signed long  adc_bus2_fselfcal(char adc_channel, char adc_device);
extern signed long adc_bus2_in(char adc_channel, char adc_device);
extern signed long adc_bus2_indump(char adc_channel, char adc_device);
extern void adc_bus2_contconv_start(char adc_channel, char adc_device);
extern void adc_bus2_idle(char adc_channel, char adc_device);
extern void adc_bus2_read_convert(char adc_channel, char adc_device);
extern void adc_bus2_io(char adc_device);  
extern char adc4_status(void);
extern char adc5_status(void);
extern char adc6_status(void);
extern char adc_bus2_chstatus(char adc_channel, char adc_device);

// Public Function Prototypes MISC
void adc_misc_reset(void);
extern char adc_misc_init(void);
void adc_misc_ctset(char adc_tcword, char adc_channel);
extern char adc_misc_run_config(void);
extern char adc_misc_selfcal(void);
extern char  adc_misc_zselfcal(char adc_channel);
extern char  adc_misc_fselfcal(char adc_channel);
extern signed long adc_misc_in(char adc_channel);
extern signed long adc_misc_indump(char adc_channel);
extern void adc_misc_convert(void);
extern void adc_misc_contconv_start(char adc_channel);
extern void adc_misc_idle(char adc_channel);
unsigned long adc_misc_read_convert(char adc_channel);

extern void adc_misc_io(void);  
extern char adc_misc_status(void);
extern char adc_misc_chstatus(char adc_channel);

extern char adc_temp_check(void);

// SPI port interface macros
#define adc1_select		P4OUT &= ~ADC_CS1;
#define adc1_deselect	P4OUT |= ADC_CS1;
#define adc2_select		P4OUT &= ~ADC_CS2;
#define adc2_deselect	P4OUT |= ADC_CS2;
#define adc3_select		P4OUT &= ~ADC_CS3;
#define adc3_deselect	P4OUT |= ADC_CS3;
#define adc4_select		P4OUT &= ~ADC_CS4;
#define adc4_deselect	P4OUT |= ADC_CS4;
#define adc5_select		P4OUT &= ~ADC_CS5;
#define adc5_deselect	P4OUT |= ADC_CS5;
#define adc6_select		P4OUT &= ~ADC_CS6;
#define adc6_deselect	P4OUT |= ADC_CS6;
#define adc7_select		P4OUT &= ~ADC_CS7;
#define adc7_deselect	P4OUT |= ADC_CS7;

// variables
/* FWRATE Settings (1.5x the following): 
 * 2 - 84 us, 9 - 230 us, 10 - 251 us, 17 - 397 us
 * 46 - 1001 us, 127 - 2689 us
 * est. 44 - 500 us, 60 - 666 us, 
 */
#define FWRATE 60 

// ADC Registers

/* ADC Comm
0 0 0 0 0 0 Communications Register during a Write Operation (W)
0 0 0 0 0 1 I/O Port (R/W)
0 0 0 0 1 0 Revision (R)
0 0 0 0 1 1 Test (R/W)
0 0 0 1 0 0 ADC Status (R)
0 0 0 1 0 1 Checksum (R/W) 
0 0 0 1 1 0 ADC Zero-Scale Cal (R/W)
0 0 0 1 1 1 ADC Full Scale (R/W)

0 0 1 0 0 0 Channel Data 0
to
0 0 1 1 1 1 Channel Data 7

0 1 0 0 0 0 Channel Zero-Scale Cal 0
to
0 1 0 1 1 1 Channel Zero-Scale Cal 7

0 1 1 0 0 0 Channel Full-Scale Cal 0
to
0 1 1 0 1 1 Channel Full-Scale Cal 7

1 0 0 0 0 0 Channel Status 0
to
1 0 0 1 1 1 Channel Status 7

1 0 1 0 0 0 Channel Setup 0
to
1 0 1 0 1 1 Channel Setup 7

1 1 0 0 0 0 Channel Conversion Time 0
to
1 1 0 0 1 1 Channel Conversion Time 7

1 1 1 0 0 0 Channel Mode
to
1 1 1 1 1 1 Channel Mode
*/

#define ADC_COMM 0x00		//Write Only
#define ADC_IOPORT 0x01
#define ADC_REV 0x02		//Read Only
#define ADC_TEST 0x03		//Read Only
#define ADC_STATUS 0x04		//Read Only
#define ADC_CHECKSUM 0x05
#define ADC_ZSCAL 0x06
#define ADC_FS 0x07

#define ADC_DATA0 0x08
#define ADC_DATA1 0x09
#define ADC_DATA2 0x0A
#define ADC_DATA3 0x0B
#define ADC_DATA4 0x0C
#define ADC_DATA5 0x0D
#define ADC_DATA6 0x0E
#define ADC_DATA7 0x0F
#define ADC_ZSCAL0 0x10
#define ADC_ZSCAL1 0x11
#define ADC_ZSCAL2 0x12
#define ADC_ZSCAL3 0x13
#define ADC_ZSCAL4 0x14
#define ADC_ZSCAL5 0x15
#define ADC_ZSCAL6 0x16
#define ADC_ZSCAL7 0x17
#define ADC_FS0 0x18
#define ADC_FS1 0x19
#define ADC_FS2 0x1A
#define ADC_FS3 0x1B
#define ADC_FS4 0x1C
#define ADC_FS5 0x1D
#define ADC_FS6 0x1E
#define ADC_FS7 0x1F
#define ADC_STATUS0 0x20
#define ADC_STATUS1 0x21
#define ADC_STATUS2 0x22
#define ADC_STATUS3 0x23
#define ADC_STATUS4 0x24
#define ADC_STATUS5 0x25
#define ADC_STATUS6 0x26
#define ADC_STATUS7 0x27
#define ADC_SETUP0 0x28
#define ADC_SETUP1 0x29
#define ADC_SETUP2 0x2A
#define ADC_SETUP3 0x2B
#define ADC_SETUP4 0x2C
#define ADC_SETUP5 0x2D
#define ADC_SETUP6 0x2E
#define ADC_SETUP7 0x2F
#define ADC_CT0 0x30
#define ADC_CT1 0x31
#define ADC_CT2 0x32
#define ADC_CT3 0x33
#define ADC_CT4 0x34
#define ADC_CT5 0x35
#define ADC_CT6 0x36
#define ADC_CT7 0x37
#define ADC_MODE0 0x38
#define ADC_MODE1 0x39
#define ADC_MODE2 0x3A
#define ADC_MODE3 0x3B
#define ADC_MODE4 0x3C
#define ADC_MODE5 0x3D
#define ADC_MODE6 0x3E
#define ADC_MODE7 0x3F

#define ADC_COMM_RD 0x40
#define ADC_COMM_WRn ~(0x40)

//ADC_IOPORT Configs
#define ADCPO 0x80
#define ADCP1 0x40
#define ADCPODIR 0x20	//0-output, 1-input
#define ADCP1DIR 0x10	//0-output, 1-input
#define RDYFN 0x08		//0-any unread, 1-all unread
#define RDYPWR 0x04		//0-high power mode, 1-low power mode (4 MHz)
#define ADCSYNC 0x01	//0-P1 digital IO, 1-P1 used as sync

//ADC_STATUS Configs
#define RDY0 0x01
#define RDY1 0x02
#define RDY2 0x04
#define RDY3 0x08
#define RDY4 0x10
#define RDY5 0x20
#define RDY6 0x40
#define RDY7 0x80

//ADC_STATUS0,1,2,3,4,5,6,7 Configs
#define CH210 0x70
#define SP0 0x10
#define SP1 0x08
#define NOREF 0x04
#define SIGN 0x02
#define OVR 0x01

//ADC_SETUP0,1,2,3,4,5,6,7 Configs
#define BUFOFF 0x80		// 0
#define COM1 0x40		// 0
#define COM0 0x20		// 0
#define STSTOPT 0x10	// 1
#define ENABLE 0x08		// enable = 1, disable = 0
#define RNG0 0x01		// +/- 2.5V
#define RNG1 0x02		// +/- .0625
#define RNG2 0x04		// +2.5
	


//ADC_CT0,1,2,3,4,5,6,7 Configs, conversion time register
#define CHOP 0x80
#define FW 0x7F

//ADC_MODE0,1,2,3,4,5,6,7 Configs
#define MIDLE 0x00
#define MCONT 0x20
#define MSINGLE 0x40
#define MPWRDWN 0x60
#define MZSELFCAL 0x80
#define MFSELFCAL 0xA0
#define MZSSYSCAL 0xC0
#define MFSSYSCAL 0xE0

#define CLKDIS 0x10
#define DUMP 0x08
#define CONTRD 0x04
#define BIT24_16n 0x02
#define CLAMP 0x01


#endif /*AD7739_FUNC_H_*/
