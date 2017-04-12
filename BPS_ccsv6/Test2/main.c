/*
 * Battery Protection Software for BPS2015 PCB Development
 * Based on previous Sunseeker code.
 * Some functions are based on code originally written by
 * Tritium Pty Ltd. See functional modules for Tritium code.
 *
 * Western Michigan University
 * Sunseeker BPS 2015 Version 1
 * Created by Scott Haver November 2015
 *
 *
 */


/*
 * RS232 COMMANDS:
 *
 * 1) battery temps
 * 2) battery volts
 * 3) battery current
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <msp430x54xa.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "BPSmain.h"
#include "RS232.h"
#include "ad7739_func.h"
#include "LTC6803.h"


#define MAX_TEMP_DISCHARGE 		0x337F38 		//60 Degree C
#define MAX_TEMP_CHARGE   		0x413A8E		//45 Degree C

#define MAX_CURRENT_DISCHARGE	0xFAE147		//60A    @ 2.45 volts across batt shunt
#define MAX_CURRENT_CHARGE		0x3F7CED		//-31.5A @ .62 volts across batt shunt

//LTC Variables
volatile unsigned char ltc1_errflag = 0x00;		//set if LTC1 error
volatile unsigned char ltc2_errflag = 0x00; 	//set if LTC2 error
volatile unsigned char ltc3_errflag = 0x00; 	//set if LTC3 error
unsigned char ltc_error = 0;					//combined error calc
volatile unsigned char batt1,batt2,batt3;		//stores return value of flag read
volatile unsigned char status_flag = FALSE;		//status flag set on timer B
volatile unsigned char batt_KILL = FALSE;		//set to isolate batt
int mode_count = 0;								//used for sequencing
unsigned int ltc1_cv[12];						//holds ltc1 cell volts
unsigned int ltc2_cv[12];						//holds ltc2 cell volts
unsigned int ltc3_cv[12];						//holds ltc3 cell volts

//ADC Temperature Variables
volatile unsigned long temperature_adc[40];		//stores adc temperatures
volatile unsigned long current;					//keeps track of current direction across batt shunt
volatile unsigned char ch;						//used for ch switching of adc
volatile unsigned char dev;						//used for dev switching of
volatile unsigned char i;						//used for counting
volatile unsigned char temp_flag = FALSE;		//used for temp measurement timing

//PreCharge Variables
unsigned long SIG1;								//measures BATT voltage
unsigned long SIG2;								//measures precharge shunt resistor voltage
unsigned long SIG3;								//measures voltage after MC contactor
volatile char PC_Complete = FALSE;				//TRUE when caps are charged
volatile char MC_Complete = FALSE;				//TRUE when MC contactor is closed
unsigned int compare_sig;						//stores signal comparison value

//RS232 Variables
char command[30];								//stores rs232 commands
char buff[30];									//buff array to hold sprintf string
unsigned char rs232_count = 0;					//counts characters received
char batt_temp[30] = "battery temps\r";			//command to read batt temperatures
char batt_temp_status = 0;						//TRUE if battery temps is sent
char batt_volt[30] = "battery volts\r";			//command to read cell volts
char batt_volt_status = 0;						//TRUE if battery volts is sent
char batt_current[30] ="battery current\r";		//command to read battery current
char batt_current_status = 0;					//TRUE if battery current is sent
float volt;										//variable to store adc volt conv
float temp;										//temperature calculation
int temp1;										//integer value
int temp2;										//tenths place
int temp3;										//hundredths place



/*=================================== **MAIN** =========================================*/
//////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	unsigned int i;

	enum MODE
	{
		INITIALIZE,
		SELFCHECK,
		BPS2BPSCHECK,
		BPSREADY,
		ARRAYREADY,
		CANCHECK,
		PRECHARGE,
		NORMALOP,
		SHUTDOWN,
		CHARGE,
		ERRORMODE
	}bpsMODE;

	WDTCTL = WDTPW | WDTHOLD | WDTSSEL__ACLK; 	// Stop watchdog timer to prevent time out reset
	_DINT();     		    					//disables interrupts

	//open all relays
	relay_batt_open;
	relay_array_open;
	relay_mcpc_open;
	relay_mc_open;
	ext_relay_mcpc_open;

	bpsMODE = INITIALIZE;
	LED_INIT_ON;								//INIT LED ON

	io_init();									//enable IO pins
	clock_init();								//Configure HF and LF clocks
	timerB_init();								//init timer B

	BPS2PC_init();								//init RS232

	//adc bus1 initializations
	adc_bus1_spi_init();
	adc_bus1_init();
	adc_bus1_selfcal(1);
	adc_bus1_read_convert(0,1);
	adc_bus1_selfcal(2);
	adc_bus1_read_convert(0,2);
	adc_bus1_selfcal(3);
	adc_bus1_read_convert(0,3);
	//adc bus2 initializations
	adc_bus2_spi_init();
	adc_bus2_init();
	adc_bus2_selfcal(1);
	adc_bus2_read_convert(0,1);
	adc_bus2_selfcal(2);
	adc_bus2_read_convert(0,2);
	adc_bus2_selfcal(3);
	adc_bus2_read_convert(0,3);
	//adc misc initializations
	adc_misc_spi_init();
	adc_misc_init();
	adc_misc_selfcal();
	adc_misc_read_convert(0);

	i = 50000;									// SW Delay
	do i--;
	while (i != 0);

	//LTC1 Configure
	LTC1_init();
	LTC1_init();
	ltc1_errflag = 0x00;
	//LTC2 Configure
	LTC2_init();
	LTC2_init();
	ltc2_errflag = 0x00;
	//LTC3 Configure
	LTC3_init();
	LTC3_init();
	ltc3_errflag = 0x00;
	
    /*Enable Interrupts*/
	UCA3IE |= UCRXIE;						//RS232 receive interrupt
	__bis_SR_register(GIE); 				//enable global interrupts
	__no_operation();						//for compiler
	
	LED_INIT_OFF;							//LED_INIT OFF
 	bpsMODE = SELFCHECK;

	while(TRUE)								//loop forever
	{
		if(bpsMODE == SHUTDOWN)
		{
			//open all relays
			relay_batt_open;
			relay_array_open;
			relay_mcpc_open;
			relay_mc_open;
			ext_relay_mcpc_open;
		}
		if(status_flag)
		{
			status_flag = FALSE;
			mode_count++;
			batt_KILL = FALSE;

			if(bpsMODE == ERRORMODE)
			{
				//look for CAN message to end errormode
				bpsMODE = SELFCHECK;

			}
			else if(bpsMODE == SHUTDOWN)
			{
				//open all relays
				relay_batt_open;
				relay_array_open;
				relay_mcpc_open;
				relay_mc_open;
				ext_relay_mcpc_open;
			}
			else									//normal periodic sequence based on mode_count
			{
				//batt1
				batt1 = LTC1_Read_Config();
				batt1 |= LTC1_Read_Flags();
				if(batt1 != 0x00)
				{
					batt_KILL = TRUE;
					batt1=0x00;
				}
				//batt2
				batt2 = LTC2_Read_Config();
				batt2 |= LTC2_Read_Flags();
				if(batt2 != 0x00)
				{
					batt_KILL = TRUE;
					batt2=0x00;
				}
				//batt3
				batt3 = LTC3_Read_Config();
				batt3 |= LTC3_Read_Flags();
				if(batt3 != 0x00)
				{
					batt_KILL = TRUE;
					batt3=0x00;
				}
				// Periodic Measurements
				switch(mode_count)
				{
					case 1:
					  ltc1_errflag = 0x00;
					  ltc2_errflag = 0x00;
					  ltc3_errflag = 0x00;
					break;
					case 2:
					  LTC1_Start_ADCCV();
					break;
					case 3:
					  batt1 = LTC1_Read_Voltages();
					  if (batt1 != 0x00)
					  {
						ltc1_errflag = TRUE;
					  }
					break;
					case 4:
					  LTC2_Start_ADCCV();
					break;
					case 5:
					  batt2 = LTC2_Read_Voltages();
					  if (batt2 != 0x00)
					  {
						 ltc2_errflag = TRUE;
					  }
					break;
					case 6:
					 LTC3_Start_ADCCV();
					break;
					case 7:
					  batt3 = LTC3_Read_Voltages();
					  if (batt3 != 0x00)
					  {
						  ltc3_errflag = TRUE;
					  }
					break;
					case 8:
						LTC1_Start_ADCCV();
					break;
					default:
					  mode_count = 0;
					break;
				}	// END Switch mode_count

				ltc_error = ltc1_errflag | ltc2_errflag | ltc3_errflag;

				if (ltc_error != 0x00 && (bpsMODE != SELFCHECK))
				{
					batt_KILL = TRUE; // LTC Error
					ltc_error = FALSE;
				}
				if (mode_count == 0x08)
				{
					mode_count = 0;

					switch(bpsMODE)
					{
					case SELFCHECK:
						if(batt_KILL != 0x00)
						{
							batt_KILL = FALSE;									//reset  batt_KILL
							LED_NORMALOP_OFF;									//LED NORMALOP OFF
							char i;
							for(i = 0; i < 2; i++)								//re-init ltcs reset flags
							{
								LTC1_init();
								LTC2_init();
								LTC3_init();
								ltc1_errflag = 0x00;
								ltc2_errflag = 0x00;
								ltc3_errflag = 0x00;
							}
							//adc bus1 initializations							//re-init adcs and calibrate
							adc_bus1_spi_init();
							adc_bus1_init();
							adc_bus1_selfcal(1);
							adc_bus1_read_convert(0,1);
							adc_bus1_selfcal(2);
							adc_bus1_read_convert(0,2);
							adc_bus1_selfcal(3);
							adc_bus1_read_convert(0,3);
							//adc bus2 initializations
							adc_bus2_spi_init();
							adc_bus2_init();
							adc_bus2_selfcal(1);
							adc_bus2_read_convert(0,1);
							adc_bus2_selfcal(2);
							adc_bus2_read_convert(0,2);
							adc_bus2_selfcal(3);
							adc_bus2_read_convert(0,3);
							//adc misc initializations
							adc_misc_spi_init();
							adc_misc_init();
							adc_misc_selfcal();
							adc_misc_read_convert(0);

						}
						else
						{
							bpsMODE = BPSREADY;
						}
					break;

					case BPSREADY:
						relay_batt_close;										//BPS Power Enabled
						bpsMODE = ARRAYREADY;
					break;

					case ARRAYREADY:
						relay_array_close;										//Array Connection Enabled
						bpsMODE = CANCHECK;
					break;

					case CANCHECK:
						//check for CAN message start PRECHARGE
						bpsMODE = PRECHARGE;
					break;

					case PRECHARGE:

						LED_PC_ON;												//turn PC LED ON

						relay_mcpc_close;										//close MCPC contactor
						ext_relay_mcpc_close;									//close external pc relay to read signals

						//start cont conv for PC signals
						adc_misc_contconv_start(5);								//battery signal 			(SIGNAL 1)
						adc_misc_contconv_start(6);								//precharge resistor signal (SIGNAL 2)
						adc_misc_contconv_start(7);								//motor contactor signal    (SIGNAL 3)

						if(PC_Complete == FALSE)								//if caps are not charged
						{
							while(SIG1 == 0) SIG1 = adc_misc_read_convert(5);	//wait for valid reading SIG1
							SIG2 = adc_misc_read_convert(6);					//get SIG2

							compare_sig = abs(SIG1 - SIG2);						//compare SIG1 & SIG2

							if(compare_sig < 0xFFFF && SIG2 > 0xFFF)			//check comparison is small and SIG2 is not noise
							{
								PC_Complete = TRUE;
							}
							else
							{
								PC_Complete = FALSE;
							}

						}

						else if(MC_Complete == FALSE && PC_Complete == TRUE)	//only run loop when PC_Complete = TRUE
						{
							relay_mcpc_open;			  						//open MCPC contactor
							relay_mc_close;				  						//close MC contactor


							while(SIG1 == 0) SIG1 = adc_misc_read_convert(5);	//wait for valid reading SIG1
							SIG3 = adc_misc_read_convert(7);					//get SIG3 reading

							compare_sig = abs(SIG1 - SIG3);						//compare SIG1 and SIG2

							if(compare_sig < 0xFFFF && SIG3 > 0xFFF)			//ensure motor contactor is closed
							{
								MC_Complete = TRUE;
								//PC is complete at this point
								ext_relay_mcpc_open;							//open external PC relay
								bpsMODE = NORMALOP;
								LED_PC_OFF;										//turn precharge LED OFF
							}
							else
							{
								MC_Complete = FALSE;
							}

						}
					break;

					case NORMALOP:
						LED_NORMALOP_ON;
					break;

					case CHARGE:
						//
					break;
					}//end switch(bpsMODE)

				}//end if(mode_count == 0x08)

			}//end else

		}//end if(status_flag)


		//////////////////////////CHECK ADC TEMP LIMITS//////////////////////////////////////////
		if(temp_flag)
		{
			temp_flag = FALSE;

			//start adc battery temp continuous conversions
			for(dev = 1; dev < 4; dev++)								//device {1:3}
			{
				for(ch = 0; ch < 8; ch++)								//channel {0:7}
				{
					adc_bus1_contconv_start(ch,dev);
					adc_bus2_contconv_start(ch,dev);
				}
			}

			//start continuous conversion for shunt measurement
			adc_misc_contconv_start(4);
			//start continous conversion for misc temperatures
			adc_misc_contconv_start(2);
			adc_misc_contconv_start(3);


			///////////////BATTERY TEMPS
			//read adc bus1 device 1 temperatures
			for(i = 1; i < 8; i++)
			{
				temperature_adc[8-i] = adc_bus1_read_convert(i,1);		//store temp {1:7}
			}
			//read adc bus1 device 2 temperatures
			for(i = 1; i < 8; i++)
			{
				temperature_adc[15-i] = adc_bus1_read_convert(i,2);		//store temp {8:14}
			}
			//read adc bus1 device 3 temperatures
			for(i = 1; i < 8; i++)
			{
				temperature_adc[22-i] = adc_bus1_read_convert(i,3);		//store temp {15:21}
			}
			//read adc bus2 device 1 temperatures
			for(i = 1; i < 8; i++)
			{
				temperature_adc[29-i] = adc_bus2_read_convert(i,1);		//store temp {22:28}
			}
			//read adc bus2 device 2 temperatures
			for(i = 1; i < 8; i++)
			{
				temperature_adc[36-i] = adc_bus2_read_convert(i,2);		//store temp {29:35}
			}

			///////////ADDITIONAL TEMPS
			temperature_adc[36] = adc_bus2_read_convert(7,3);			//store inlet temp  {36}
			temperature_adc[37] = adc_bus2_read_convert(6,3);			//store outlet temp {37}

			temperature_adc[38] = adc_misc_read_convert(3);				//store misc temp 	{38}
			temperature_adc[39] = adc_misc_read_convert(2);				//store misc temp 	{39}

			/////////CHECK LIMITS

			//get current direction across batt shunt
			for(i = 5; i > 0; i--)
			{
				current = adc_misc_read_convert(4);						//check multiple times to avoid invalid data
			}

			//check temperature and current limits if discharging
			if(current >= 0x800000)										//adc > 1.25V  (DISCHARGING)
			{
				if(current >= MAX_CURRENT_DISCHARGE)					//over current check
				{
					batt_KILL = TRUE;
				}
				else
				{
					for(i = 1; i < 36; i++)								//check temperature cells {1:35}
					{
						if(temperature_adc[i] <= MAX_TEMP_DISCHARGE)	//temp max is 60 degree C discharging
						{
							batt_KILL = TRUE;
						}
					}
				}
			}

			//check temperature and current limits if charging
			else if(current < 0x800000)									//adc < 1.25V  (CHARGING)
			{
				if(current <= MAX_CURRENT_CHARGE)						//over current check
				{
					batt_KILL = TRUE;
				}
				else
				{
					for(i = 1; i < 36; i++)								//check temperature cells {1:35}
					{
						if(temperature_adc[i] <= MAX_TEMP_CHARGE)		//temp max is 45 degree C charging
						{
							batt_KILL = TRUE;
						}
					}
				}
			}
		}
	   ////////////////////////////////////////////////////////////////////////////////////////////////


		/////////////////////////////////////////HANDLE RS232//////////////////////////////////////////

		if(batt_temp_status)											//cell temperatures
		{
			batt_temp_status = 0;
			BPS2PC_puts("\nBATTERY TEMPERATURES:");
			BPS2PC_puts("MAX Discharge Temp = 60 Degree C");
			BPS2PC_puts("MAX Charge Temp = 45 Degree C\n");
			for(i = 1; i < 40; i++)
			{
				volt = 2.5*(temperature_adc[i] / 16777215.0);
				temp = 116.0 - (111.39*volt);
				temp1 = (int)temp;
				temp2 = (int)(temp * 10) % 10;
				temp3 = (int)(temp * 100) %10;

				sprintf(buff,"Temp %d = %d.%d%d Degree C",i,temp1,temp2,temp3);
				BPS2PC_puts(buff);
			}
		}

		else if(batt_volt_status)										//cell voltages
		{

			batt_volt_status = 0;
			BPS2PC_puts("\nBATTERY CELL VOLTAGES:");
			BPS2PC_puts("MAX CELL VOLTAGE 4.176 V");
			BPS2PC_puts("MIN CELL VOLTAGE 2.808 V\n");
			for(i = 0; i < 12; i++)
			{
				ltc1_cv[i] = get_cell_voltage(i,1);						//cell i ltc1
				ltc2_cv[i] = get_cell_voltage(i,2);						//cell i ltc2
				ltc3_cv[i] = get_cell_voltage(i,3);						//cell i ltc3

				ltc1_cv[i] = (ltc1_cv[i] - 512);
				ltc2_cv[i] = (ltc2_cv[i] - 512);
				ltc3_cv[i] = (ltc3_cv[i] - 512);
			}
			for(i = 0; i < 12; i++)
			{
				temp = ltc1_cv[i] / 666.66;								//conversion to volts
				temp1 = (int)temp;										//integer value
				temp2 = (int)(temp * 10) % 10;							//tenths place
				temp3 = (int)(temp * 100) %10;							//hundredths place

				sprintf(buff, "Cell %d = %d.%d%d Volts",i,temp1,temp2,temp3);
				BPS2PC_puts(buff);
			}
			for(i = 0; i < 12; i++)
			{
				temp = ltc2_cv[i] / 666.66;
				temp1 = (int)temp;
				temp2 = (int)(temp * 10) % 10;
				temp3 = (int)(temp * 100) %10;

				sprintf(buff, "Cell %d = %d.%d%d Volts",i+12,temp1,temp2,temp3);
				BPS2PC_puts(buff);
			}
			for(i = 0; i < 12; i++)
			{
				temp = ltc3_cv[i] / 666.66;
				temp1 = (int)temp;
				temp2 = (int)(temp * 10) % 10;
				temp3 = (int)(temp * 100) %10;

				sprintf(buff, "Cell %d = %d.%d%d Volts",i + 24,temp1,temp2,temp3);
				BPS2PC_puts(buff);
			}


		}

		else if(batt_current_status)								//battery current
		{
			batt_current_status = 0;

			BPS2PC_puts("\nBATTERY CURRENT:");
			BPS2PC_puts("MAX CURRENT DISCHARGE 60A");
			BPS2PC_puts("MAX CURRENT CHARGE -31.5A\n");

			temp = 2.5*(current / 16777215.0);
			temp = 50*(temp - 1.25);
			temp1 = (int)temp;
			temp2 = abs((int)(temp * 10) % 10);
			temp3 = abs((int)(temp * 100) %10);

			sprintf(buff, "Battery Current = %d.%d%d Amps",temp1,temp2,temp3);
			BPS2PC_puts(buff);
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//handle batt_KILL error

		if(batt_KILL)
		{
			LED_ERROR_ON;
			//open all relays
			relay_batt_open;
			relay_array_open;
			relay_mcpc_open;
			relay_mc_open;
			ext_relay_mcpc_open;

			PC_Complete = FALSE;
			MC_Complete = FALSE;

			LED_NORMALOP_OFF;

			bpsMODE = SELFCHECK;
		}
		else
		{
			LED_ERROR_OFF;
		}


	} //end while(TRUE)

}
/*================================ ** END MAIN** =========================================*/
////////////////////////////////////////////////////////////////////////////////////////////

/*
* Initialise Timer B
*	- Provides timer tick timebase at 100 Hz
*/
void timerB_init( void )
{
  TBCTL = CNTL_0 | TBSSEL_1 | ID_3 | TBCLR;		// ACLK/8, clear TBR
  TBCCR0 = (ACLK_RATE/8/TICK_RATE);				// Set timer to count to this value = TICK_RATE overflow
  TBCCTL0 = CCIE;								// Enable CCR0 interrrupt
  TBCTL |= MC_1;								// Set timer to 'up' count mode
}

/*
* Timer B CCR0 Interrupt Service Routine
*	- Interrupts on Timer B CCR0 match at 10Hz
*	- Sets Time_Flag variable
*/
/*
* GNU interropt symantics
* interrupt(TIMERB0_VECTOR) timer_b0(void)
*/
#pragma vector = TIMERB0_VECTOR
__interrupt void timer_b0(void)
{
	static unsigned int status_count = LTC_STATUS_COUNT;
	static unsigned int temp_count = LTC_STATUS_COUNT/2;

	status_count--;
	temp_count--;
    if( status_count == 0 )
    {
    	P6OUT ^= LED4;
    	status_count = LTC_STATUS_COUNT;
    	status_flag = TRUE;
    }
    if(temp_count == 0)
    {
    	temp_count = LTC_STATUS_COUNT/2;
    	temp_flag = TRUE;
    }

}

//RS232 Interrupt
#pragma vector = USCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
{
	UCA3IV = 0x00;

	command[rs232_count] = BPS2PC_getchar();		//get character
	BPS2PC_putchar(command[rs232_count]);			//put character

	if(command[rs232_count] == 0x0D)				//if return
	{
		rs232_count = 0x00;							//reset counter
		if(strcmp(batt_temp,command) == 0) batt_temp_status = 1;
		else if(strcmp(batt_volt,command) == 0) batt_volt_status = 1;
		else if(strcmp(batt_current, command) == 0) batt_current_status = 1;
		else
		{
			batt_temp_status = 0;
			batt_volt_status = 0;
			batt_current_status = 0;
		}

		for(i = 0; i < 31; i++)
		{
			command[i] = 0;							//reset command
		}
		BPS2PC_putchar(0x0A);						//new line
		BPS2PC_putchar(0x0D);						//return
	}
	else if(command[rs232_count] != 0x7F)			//if not backspace
	{
		rs232_count++;
	}
	else
	{
		rs232_count--;
	}

}


