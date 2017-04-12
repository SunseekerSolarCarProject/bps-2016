/*
  *   LTC6803.c
 *  
 *
 *  Created by Byron Izenbaard on 6/8/11.
 *  Modified by Kenwood Hoben on 5/26/12
 *
 *  Modified for BPS_V2 by Scott Haver on 11/3/15
 *
 *  Copyright 2015 Western Michigan University Sunseeker. All rights reserved.
 *
 */
 
 
 // GStandard Include Files
#include <msp430x54xa.h>
#include "BPSmain.h"
#include "LTC6803.h"
 
// Global Variables
static const unsigned char InitialPEC = 0x41;
//store ltc cell voltages
volatile int ltc1_CV[12];
volatile int ltc2_CV[12];
volatile int ltc3_CV[12];
	
static const unsigned char LTADDR = 0x80;
static const unsigned char LTADDR_PEC= 0x49;
// 130msec Comp Period, 13ms ADC Time
// WDTB=GPIO1=1 (ref enable)
static const unsigned char CFGR0 = WDT | GPIO2 | GPIO1 | LVLPL | CDC_3;
static const unsigned char CFGR1 = 0x00;
static const unsigned char CFGR2 = 0x00;
static const unsigned char CFGR3a = 0x00;		//LTC1 - 12 cells
static const unsigned char CFGR3b = 0x00;		//LTC2 - 12 cells
static const unsigned char CFGR3c = 0x80;		//LTC3 - 11 cells
// Vuv = (Hex-31)*16*1.5mV, each step is 0.024 V
static const unsigned char CFGR4 = 0x48;  		// Vuv = 1 Volt
// Vov = (Hex-32)*16*1.5mV
static const unsigned char CFGR5 = 0x9D;   		// Vov = 1.5 Volts

char LTC1_init(void) {
	volatile int read_error=0;
	
	// Initialize the SPI Port
	LTC1SPI_init();
	
	//Configuration
	LTC1_Config();
	read_error = LTC1_Read_Config();
	
	if(read_error==1) {LED_ERROR_ON;}
	else {LED_ERROR_OFF;}

	return(read_error);
}

char LTC2_init(void) {
	volatile int read_error=0;
	
	// Initialize the SPI Port
	LTC2SPI_init();

	//Configuration
	LTC2_Config();
	read_error = LTC2_Read_Config();
	
	if(read_error==1) {LED_ERROR_ON;}
	else {LED_ERROR_OFF;}
	return(read_error);
}

char LTC3_init(void) 
{
	volatile int read_error=0;
	
	// Initialize the SPI Port
	LTC3SPI_init();
	
	//Configuration
	LTC3_Config();
	read_error = LTC3_Read_Config();
	
	if(read_error==1) {LED_ERROR_ON;}
	else {LED_ERROR_OFF;}
	return(read_error);
}


void LTC1_Config(void)
{
	unsigned char comp_PEC;
	
	comp_PEC = Calculate_PEC(CFGR0,InitialPEC);
	comp_PEC = Calculate_PEC(CFGR1,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR2,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR3a,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR4,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR5,comp_PEC);

	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(WRCFG);
	LTC1_transmit(WRCFG_PEC);

	LTC1_transmit(CFGR0);
	LTC1_transmit(CFGR1);
	LTC1_transmit(CFGR2);
	LTC1_transmit(CFGR3a);
	LTC1_transmit(CFGR4);
	LTC1_transmit(CFGR5);
	LTC1_transmit(comp_PEC);
		
	LTC1_deselect;   //Pull CSBI high
}

unsigned char LTC1_Read_Config(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_CFG[6],READ_CFG_PEC;
	unsigned char ltc1_errflag = 0x00;
	
	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(RDCFG);
	LTC1_transmit(RDCFG_PEC);

	READ_CFG[0]=LTC1_exchange(0xFF);
	READ_CFG[1]=LTC1_exchange(0xFF);
	READ_CFG[2]=LTC1_exchange(0xFF);
	READ_CFG[3]=LTC1_exchange(0xFF);
	READ_CFG[4]=LTC1_exchange(0xFF);
	READ_CFG[5]=LTC1_exchange(0xFF);
	READ_CFG_PEC=LTC1_exchange(0xFF);
	
	LTC1_deselect;   //Pull CSBI high
	
	comp_PEC = Calculate_PEC(READ_CFG[0],InitialPEC);
	comp_PEC = Calculate_PEC(READ_CFG[1],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[2],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[3],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[4],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[5],comp_PEC);
	
	if (READ_CFG_PEC==comp_PEC)
	{
		if (READ_CFG[0] == CFGR0) return(0);
		else
		{
			ltc1_errflag |= 0x10;
			return(1);
		}
	}
	else
	{
		ltc1_errflag |= 0x40;
		return(1);
	}
}

void LTC2_Config(void)
{
	unsigned char comp_PEC;
	
	comp_PEC = Calculate_PEC(CFGR0,InitialPEC);
	comp_PEC = Calculate_PEC(CFGR1,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR2,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR3b,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR4,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR5,comp_PEC);

	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(WRCFG);
	LTC2_transmit(WRCFG_PEC);

	LTC2_transmit(CFGR0);
	LTC2_transmit(CFGR1);
	LTC2_transmit(CFGR2);
	LTC2_transmit(CFGR3b);
	LTC2_transmit(CFGR4);
	LTC2_transmit(CFGR5);
	LTC2_transmit(comp_PEC);
		
	LTC2_deselect;   //Pull CSBI high
}

unsigned char LTC2_Read_Config(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_CFG[6],READ_CFG_PEC;
	unsigned char ltc2_errflag = 0x00;
	
	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(RDCFG);
	LTC2_transmit(RDCFG_PEC);

	READ_CFG[0]=LTC2_exchange(0xFF);
	READ_CFG[1]=LTC2_exchange(0xFF);
	READ_CFG[2]=LTC2_exchange(0xFF);
	READ_CFG[3]=LTC2_exchange(0xFF);
	READ_CFG[4]=LTC2_exchange(0xFF);
	READ_CFG[5]=LTC2_exchange(0xFF);
	READ_CFG_PEC=LTC2_exchange(0xFF);
	
	LTC2_deselect;   //Pull CSBI high
	
	comp_PEC = Calculate_PEC(READ_CFG[0],InitialPEC);
	comp_PEC = Calculate_PEC(READ_CFG[1],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[2],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[3],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[4],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[5],comp_PEC);
	
	if (READ_CFG_PEC==comp_PEC)
	{
		if (READ_CFG[0] == CFGR0) return(0);
		else 
		{
			ltc2_errflag |= 0x20;
			return(1);
		}
	}
	else
	{
		ltc2_errflag |= 0x80;
		return(1);
	}
}

void LTC3_Config(void)
{
	unsigned char comp_PEC;
	
	comp_PEC = Calculate_PEC(CFGR0,InitialPEC);
	comp_PEC = Calculate_PEC(CFGR1,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR2,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR3c,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR4,comp_PEC);
	comp_PEC = Calculate_PEC(CFGR5,comp_PEC);

	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(WRCFG);
	LTC3_transmit(WRCFG_PEC);

	LTC3_transmit(CFGR0);
	LTC3_transmit(CFGR1);
	LTC3_transmit(CFGR2);
	LTC3_transmit(CFGR3c);
	LTC3_transmit(CFGR4);
	LTC3_transmit(CFGR5);
	LTC3_transmit(comp_PEC);
		
	LTC3_deselect;   //Pull CSBI high
}

unsigned char LTC3_Read_Config(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_CFG[6],READ_CFG_PEC;
	unsigned char LTC3_errflag = 0x00;
	
	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(RDCFG);
	LTC3_transmit(RDCFG_PEC);

	READ_CFG[0]=LTC3_exchange(0xFF);
	READ_CFG[1]=LTC3_exchange(0xFF);
	READ_CFG[2]=LTC3_exchange(0xFF);
	READ_CFG[3]=LTC3_exchange(0xFF);
	READ_CFG[4]=LTC3_exchange(0xFF);
	READ_CFG[5]=LTC3_exchange(0xFF);
	READ_CFG_PEC=LTC3_exchange(0xFF);
	
	LTC3_deselect;   //Pull CSBI high
	
	comp_PEC = Calculate_PEC(READ_CFG[0],InitialPEC);
	comp_PEC = Calculate_PEC(READ_CFG[1],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[2],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[3],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[4],comp_PEC);
	comp_PEC = Calculate_PEC(READ_CFG[5],comp_PEC);
	
	if (READ_CFG_PEC==comp_PEC)
	{
		if (READ_CFG[0] == CFGR0) return(0);
		else
		{
			LTC3_errflag |= 0x10;
			return(1);
		}
	}
	else
	{
		LTC3_errflag |= 0x40;
		return(1);
	}
}

unsigned char LTC1_PollInts(void)
{
	volatile unsigned char int_status;
	
	int_status = 0;
	
	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(PLINT);
	LTC1_transmit(PLINT_PEC);
	delay();
	
	if ((P9IN & LT_MISO_1) != LT_MISO_1) int_status = 1;
	
	LTC1_deselect;   //Pull CSBI high
	
	return(int_status);	
}

unsigned char LTC2_PollInts(void)
{
	volatile unsigned char int_status;
	
	int_status = 0;
	
	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(PLINT);
	LTC2_transmit(PLINT_PEC);
	delay();
	
	if ((P9IN & LT_MISO_2) != LT_MISO_2) int_status = 1;
	
	LTC2_deselect;   //Pull CSBI high
	
	return(int_status);	
}

unsigned char LTC3_PollInts(void)
{
	volatile unsigned char int_status;
	
	int_status = 0;
	
	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(PLINT);
	LTC3_transmit(PLINT_PEC);
	delay();
	
	if ((P9IN & LT_MISO_1) != LT_MISO_1) int_status = 1;
	
	LTC3_deselect;   //Pull CSBI high
	
	return(int_status);	
}

unsigned char LTC1_Read_Flags(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_FLGR_PEC;
	volatile unsigned char OV_flag, UV_flag;
	unsigned char ltc1_FLGR[3], ltc1_errflag;
	ltc1_errflag = 0;
	
	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(RDFLG);
	LTC1_transmit(RDFLG_PEC);

	ltc1_FLGR[0]=LTC1_exchange(0xFF);
	ltc1_FLGR[1]=LTC1_exchange(0xFF);
	ltc1_FLGR[2]=LTC1_exchange(0xFF);
	READ_FLGR_PEC=LTC1_exchange(0xFF);
	
	LTC1_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = Calculate_PEC(ltc1_FLGR[0],InitialPEC);
	comp_PEC = Calculate_PEC(ltc1_FLGR[1],comp_PEC);
	comp_PEC = Calculate_PEC(ltc1_FLGR[2],comp_PEC);
	
	if (READ_FLGR_PEC!=comp_PEC)
	{
		ltc1_errflag |= 0x40;
		return(1);
	}
	
	OV_flag = (ltc1_FLGR[0] & 0xAA)>>1;
	OV_flag |= (ltc1_FLGR[1] & 0xAA);
	OV_flag |= (ltc1_FLGR[2] & 0xAA);
	
	UV_flag = (ltc1_FLGR[0] & 0x55);
	UV_flag |= (ltc1_FLGR[1] & 0x55)<<1;
	UV_flag |= (ltc1_FLGR[2] & 0x55);

// Error flag conditions	
	if(OV_flag != 0x00) ltc1_errflag |= 0x02;
	if(UV_flag != 0x00) ltc1_errflag |= 0x01;
	
	if (ltc1_errflag == 0x00) return(0);
	else return(1);
}

unsigned char LTC2_Read_Flags(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_FLGR_PEC;
	volatile unsigned char OV_flag, UV_flag;
	unsigned char ltc2_FLGR[3];
	unsigned char ltc2_errflag = 0x00;
	
	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(RDFLG);
	LTC2_transmit(RDFLG_PEC);

	ltc2_FLGR[0]=LTC2_exchange(0xFF);
	ltc2_FLGR[1]=LTC2_exchange(0xFF);
	ltc2_FLGR[2]=LTC2_exchange(0xFF);
	READ_FLGR_PEC=LTC2_exchange(0xFF);
	
	LTC2_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = Calculate_PEC(ltc2_FLGR[0],InitialPEC);
	comp_PEC = Calculate_PEC(ltc2_FLGR[1],comp_PEC);
	comp_PEC = Calculate_PEC(ltc2_FLGR[2],comp_PEC);
	
	if (READ_FLGR_PEC!=comp_PEC)
	{
		ltc2_errflag |= 0x80;
		return(1);
	}
	
	OV_flag = (ltc2_FLGR[0] & 0xAA)>>1;
	OV_flag |= (ltc2_FLGR[1] & 0xAA);
	OV_flag |= (ltc2_FLGR[2] & 0xAA);
	
	UV_flag = (ltc2_FLGR[0] & 0x55);
	UV_flag |= (ltc2_FLGR[1] & 0x55)<<1;
	UV_flag |= (ltc2_FLGR[2] & 0x55);

// Error flag conditions	
	if(OV_flag != 0x00) ltc2_errflag |= 0x08;
	if(UV_flag != 0x00) ltc2_errflag |= 0x04;

	if (ltc2_errflag == 0x00) return(0);
	else return(1);
}

unsigned char LTC3_Read_Flags(void)
{
	unsigned char comp_PEC;
	volatile unsigned char READ_FLGR_PEC;
	volatile unsigned char OV_flag, UV_flag;
	unsigned char LTC3_FLGR[3];
	unsigned char ltc3_errflag = 0x00;
	
	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(RDFLG);
	LTC3_transmit(RDFLG_PEC);

	LTC3_FLGR[0]=LTC3_exchange(0xFF);
	LTC3_FLGR[1]=LTC3_exchange(0xFF);
	LTC3_FLGR[2]=LTC3_exchange(0xFF);
	READ_FLGR_PEC=LTC3_exchange(0xFF);
	
	LTC3_deselect;   //Pull CSBI high
	
// Validate communications
	comp_PEC = Calculate_PEC(LTC3_FLGR[0],InitialPEC);
	comp_PEC = Calculate_PEC(LTC3_FLGR[1],comp_PEC);
	comp_PEC = Calculate_PEC(LTC3_FLGR[2],comp_PEC);
	
	if (READ_FLGR_PEC!=comp_PEC)
	{
		ltc3_errflag |= 0x40;
		return(1);
	}
	
	OV_flag = (LTC3_FLGR[0] & 0xAA)>>1;
	OV_flag |= (LTC3_FLGR[1] & 0xAA);
	OV_flag |= (LTC3_FLGR[2] & 0xAA);
	
	UV_flag = (LTC3_FLGR[0] & 0x55);
	UV_flag |= (LTC3_FLGR[1] & 0x55)<<1;
	UV_flag |= (LTC3_FLGR[2] & 0x55);

// Error flag conditions
	if(OV_flag != 0x00) ltc3_errflag |= 0x02;
	if(UV_flag != 0x00) ltc3_errflag |= 0x01;
	
	if (ltc3_errflag == 0x00) return(0);
	else return(1);
}

void LTC1_Clear_ADCCV(void)		// Command require 1 msec to operate
{
	LTC1_select;  //Pull CSBI low
	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(STCVAD_Clear_FF);
	LTC1_transmit(STCVAD_Clear_FF_PEC);
	LTC1_deselect;   //Pull CSBI high
}

void LTC1_Start_ADCCV(void)
{
	LTC1_select;  //Pull CSBI low
	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(STCVAD_All);
	LTC1_transmit(STCVAD_All_PEC);
	LTC1_deselect;   //Pull CSBI high
}

unsigned char LTC1_Read_Voltages(void)
{
	unsigned char i;
    unsigned char READ_CVR[18];
	volatile unsigned char READ_CVR_PEC;
	unsigned char ltc1_errflag = 0x00;
	
	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(RDCV);
	LTC1_transmit(RDCV_PEC);

	for (i=0;i<=17;i++)	READ_CVR[i]=LTC1_exchange(0xFF);
	READ_CVR_PEC=LTC1_exchange(0xFF);
	
	LTC1_deselect;   //Pull CSBI high
	
// Validate communications
	unsigned char comp_PEC = InitialPEC;
	for (i=0;i<=17;i++) comp_PEC = Calculate_PEC(READ_CVR[i],comp_PEC);
	
	if (READ_CVR_PEC!=comp_PEC)
	{
		ltc1_errflag |= 0x40;
		return(1);
	}
	
	for (i=0;i<=5;i++)
	{
		ltc1_CV[2*i]  = ((int)(READ_CVR[3*i+1] & 0x0F))<<8;
		ltc1_CV[2*i] |= (int)READ_CVR[3*i];
		ltc1_CV[2*i+1]  = ((int)READ_CVR[3*i+2])<<4;
		ltc1_CV[2*i+1] |= ((int)(READ_CVR[3*i+1] & 0xF0))>>4;
	}
	
	return(0);
}

void LTC2_Clear_ADCCV(void)		// Command require 1 msec to operate
{
	LTC2_select;  //Pull CSBI low
	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(STCVAD_Clear_FF);
	LTC2_transmit(STCVAD_Clear_FF_PEC);
	LTC2_deselect;   //Pull CSBI high
}

void LTC2_Start_ADCCV(void)
{
	LTC2_select;  //Pull CSBI low
	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(STCVAD_All);
	LTC2_transmit(STCVAD_All_PEC);
	LTC2_deselect;   //Pull CSBI high
}

unsigned char LTC2_Read_Voltages(void)
{
	unsigned char comp_PEC, i;
	unsigned char READ_CVR[18];
	volatile unsigned char READ_CVR_PEC;
	unsigned char ltc2_errflag = 0x00;
	
	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(RDCV);
	LTC2_transmit(RDCV_PEC);

	for (i=0;i<=17;i++)	READ_CVR[i]=LTC2_exchange(0xFF);
	READ_CVR_PEC=LTC2_exchange(0xFF);
	
	LTC2_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = InitialPEC;
	for (i=0;i<=17;i++) comp_PEC = Calculate_PEC(READ_CVR[i],comp_PEC);
	
	if (READ_CVR_PEC!=comp_PEC)
	{
		ltc2_errflag |= 0x80;
		return(1);
	}
	
	for (i=0;i<=5;i++)
	{
		ltc2_CV[2*i]  = ((int)(READ_CVR[3*i+1] & 0x0F))<<8;
		ltc2_CV[2*i] |= (int)READ_CVR[3*i];
		ltc2_CV[2*i+1]  = ((int)READ_CVR[3*i+2])<<4;
		ltc2_CV[2*i+1] |= ((int)(READ_CVR[3*i+1] & 0xF0))>>4;
	}
	
	return(0);
}

void LTC3_Clear_ADCCV(void)		// Command require 1 msec to operate
{
	LTC3_select;  //Pull CSBI low
	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(STCVAD_Clear_FF);
	LTC3_transmit(STCVAD_Clear_FF_PEC);
	LTC3_deselect;   //Pull CSBI high
}

void LTC3_Start_ADCCV(void)
{
	LTC3_select;  //Pull CSBI low
	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(STCVAD_All);
	LTC3_transmit(STCVAD_All_PEC);
	LTC3_deselect;   //Pull CSBI high
}

unsigned char LTC3_Read_Voltages(void)
{
	unsigned char comp_PEC, i;
	unsigned char READ_CVR[18];
	volatile unsigned char READ_CVR_PEC;
	unsigned char LTC3_errflag = 0x00;
	
	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(RDCV);
	LTC3_transmit(RDCV_PEC);

	for (i=0;i<=17;i++)	READ_CVR[i]=LTC3_exchange(0xFF);
	READ_CVR_PEC=LTC3_exchange(0xFF);
	
	LTC3_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = InitialPEC;
	for (i=0;i<=17;i++) comp_PEC = Calculate_PEC(READ_CVR[i],comp_PEC);
	
	if (READ_CVR_PEC!=comp_PEC)
	{
		LTC3_errflag |= 0x40;
		return(1);
	}
	
	for (i=0;i<=5;i++)
	{
		ltc3_CV[2*i]  = ((int)(READ_CVR[3*i+1] & 0x0F))<<8;
		ltc3_CV[2*i] |= (int)READ_CVR[3*i];
		ltc3_CV[2*i+1]  = ((int)READ_CVR[3*i+2])<<4;
		ltc3_CV[2*i+1] |= ((int)(READ_CVR[3*i+1] & 0xF0))>>4;
	}
	


	return(0);
}

void LTC1_Start_ADCTEMP(void)
{
	LTC1_select;  //Pull CSBI low
	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(STTMPAD_All);
	LTC1_transmit(STTMPAD_All_PEC);
	LTC1_deselect;   //Pull CSBI high
}

unsigned char LTC1_Read_TempReg(void)
{
	unsigned char comp_PEC, i;
	unsigned char READ_TMPR[5];
	volatile unsigned char READ_TMPR_PEC;
	 int ltc1_ETMP[2], ltc1_ITMP;
	 unsigned char ltc1_errflag = 0x00;
	
	//Transmit
	LTC1_select;  //Pull CSBI low

	LTC1_transmit(LTADDR);
	LTC1_transmit(LTADDR_PEC);
	LTC1_transmit(RDTMP);
	LTC1_transmit(RDTMP_PEC);

	for (i=0;i<=4;i++)	READ_TMPR[i]=LTC1_exchange(0xFF);
	READ_TMPR_PEC=LTC1_exchange(0xFF);
	
	LTC1_deselect;   //Pull CSBI high
	
// Validate communications
	comp_PEC = InitialPEC;
	for (i=0;i<=4;i++) comp_PEC = Calculate_PEC(READ_TMPR[i],comp_PEC);
	
	if (READ_TMPR_PEC!=comp_PEC)
	{
		ltc1_errflag |= 0x40;
		return(1);
	}
	
	ltc1_ETMP[0]  = ((int)(READ_TMPR[1] & 0x0F))<<8;
	ltc1_ETMP[0] |=  (int)READ_TMPR[0];
	ltc1_ETMP[1]  = ((int)READ_TMPR[2])<<4;
	ltc1_ETMP[1] |= ((int)(READ_TMPR[1] & 0xF0))>>4;
	ltc1_ITMP     = ((int)(READ_TMPR[4] & 0x0F))<<8;
	ltc1_ITMP    |=  (int)READ_TMPR[3];
	
	return(0);
}

void LTC2_Start_ADCTEMP(void)
{
	LTC2_select;  //Pull CSBI low
	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(STTMPAD_All);
	LTC2_transmit(STTMPAD_All_PEC);
	LTC2_deselect;   //Pull CSBI high
}

unsigned char LTC2_Read_TempReg(void)
{
	unsigned char comp_PEC, i;
	unsigned char READ_TMPR[5];
	volatile unsigned char READ_TMPR_PEC;
	 int ltc2_ETMP[2], ltc2_ITMP;
	 unsigned char ltc2_errflag = 0x00;
	
	//Transmit
	LTC2_select;  //Pull CSBI low

	LTC2_transmit(LTADDR);
	LTC2_transmit(LTADDR_PEC);
	LTC2_transmit(RDTMP);
	LTC2_transmit(RDTMP_PEC);

	for (i=0;i<=4;i++)	READ_TMPR[i]=LTC2_exchange(0xFF);
	READ_TMPR_PEC=LTC2_exchange(0xFF);
	
	LTC2_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = InitialPEC;
	for (i=0;i<=4;i++) comp_PEC = Calculate_PEC(READ_TMPR[i],comp_PEC);
	
	if (READ_TMPR_PEC!=comp_PEC)
	{
		ltc2_errflag |= 0x80;
		return(1);
	}
	
	ltc2_ETMP[0]  = ((int)(READ_TMPR[1] & 0x0F))<<8;
	ltc2_ETMP[0] |=  (int)READ_TMPR[0];
	ltc2_ETMP[1]  = ((int)READ_TMPR[2])<<4;
	ltc2_ETMP[1] |= ((int)(READ_TMPR[1] & 0xF0))>>4;
	ltc2_ITMP     = ((int)(READ_TMPR[4] & 0x0F))<<8;
	ltc2_ITMP    |=  (int)READ_TMPR[3];
	
	return(0);
}

void LTC3_Start_ADCTEMP(void)
{
	LTC3_select;  //Pull CSBI low
	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(STTMPAD_All);
	LTC3_transmit(STTMPAD_All_PEC);
	LTC3_deselect;   //Pull CSBI high
}

unsigned char LTC3_Read_TempReg(void)
{
	unsigned char comp_PEC, i;
	unsigned char READ_TMPR[5];
	volatile unsigned char READ_TMPR_PEC;
	 int LTC3_ETMP[2], LTC3_ITMP;
	 unsigned char LTC3_errflag = 0x00;
	
	//Transmit
	LTC3_select;  //Pull CSBI low

	LTC3_transmit(LTADDR);
	LTC3_transmit(LTADDR_PEC);
	LTC3_transmit(RDTMP);
	LTC3_transmit(RDTMP_PEC);

	for (i=0;i<=4;i++)	READ_TMPR[i]=LTC3_exchange(0xFF);
	READ_TMPR_PEC=LTC3_exchange(0xFF);
	
	LTC3_deselect;   //Pull CSBI high
	
// Validate communications	
	comp_PEC = InitialPEC;
	for (i=0;i<=4;i++) comp_PEC = Calculate_PEC(READ_TMPR[i],comp_PEC);
	
	if (READ_TMPR_PEC!=comp_PEC)
	{
		LTC3_errflag |= 0x40;
		return(1);
	}
	
	LTC3_ETMP[0]  = ((int)(READ_TMPR[1] & 0x0F))<<8;
	LTC3_ETMP[0] |=  (int)READ_TMPR[0];
	LTC3_ETMP[1]  = ((int)READ_TMPR[2])<<4;
	LTC3_ETMP[1] |= ((int)(READ_TMPR[1] & 0xF0))>>4;
	LTC3_ITMP     = ((int)(READ_TMPR[4] & 0x0F))<<8;
	LTC3_ITMP    |=  (int)READ_TMPR[3];
	
	return(0);
}

unsigned char Calculate_PEC(unsigned char theCommand, unsigned char CurrentPEC) 
{
	unsigned char bit_mask[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
	unsigned char Final_PEC;
	int DIN,IN0,IN1,IN2,i,n,bit_number = 0;
	int PEC_bit[8];
	int bit[8];
	
	for(n=0;n<8;n++) {
		bit[n] = (theCommand & bit_mask[n]) != 0x00;
	}

//  Step 1: Initialize PEC
//	unsigned char CurrentPEC = 0x41;

	for(i=0;i<8;i++) {
		PEC_bit[i] = (CurrentPEC & bit_mask[i]) != 0x00;
	}

	for(bit_number = 7;bit_number>(-1);bit_number--) {
	// Step 2
	DIN = bit[bit_number];
	IN0 = (DIN ^ PEC_bit[7]);
	IN1 = (PEC_bit[0] ^ IN0);
	IN2 = (PEC_bit[1] ^ IN0);

	// Step 3: Update 8-bit PEC
	PEC_bit[7] = PEC_bit[6];
	PEC_bit[6] = PEC_bit[5];
	PEC_bit[5] = PEC_bit[4];
	PEC_bit[4] = PEC_bit[3];
	PEC_bit[3] = PEC_bit[2];
	PEC_bit[2] = IN2;
	PEC_bit[1] = IN1;
	PEC_bit[0] = IN0;
	}

	Final_PEC = (PEC_bit[7] << 7) | (PEC_bit[6] << 6) | (PEC_bit[5] << 5) | (PEC_bit[4] << 4) | (PEC_bit[3] << 3) | (PEC_bit[2] << 2) | (PEC_bit[1] << 1) | (PEC_bit[0]);
	return (Final_PEC);
}

unsigned int get_cell_voltage(char cell, char ltc)
{
	char i = 0;
	switch(ltc)
	{
	case 1:
		for(i = 0; i <= 11; i++)
		{
			if(i == cell)
			{
				return(ltc1_CV[i]);
			}
		}
		break;
	case 2:
		for(i = 0; i <= 11; i++)
		{
			if(i == cell)
			{
				return(ltc2_CV[i]);
			}
		}
	break;
	case 3:
		for(i = 0; i <= 11; i++)
		{
			if(i == cell)
			{
				return(ltc3_CV[i]);
			}
		}
		break;
	}
}

