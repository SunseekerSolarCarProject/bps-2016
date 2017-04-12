/*
 *  LTC6803.h
 *  
 *  Created by Byron Izenbaard on 6/8/11.
 *  Modified by Kenwood Hoben on 5/26/12
 *
 *  Modified for BPS_V2 by Scott Haver on 11/3/15
 *
 *  Copyright 2015 Western Michigan University Sunseeker. All rights reserved.
 *
 */
 
#ifndef LTC6803_H_
#define LTC6803_H_

// Functional Prototypes
 char LTC1_init(void);
void LTC1_Config(void);
unsigned char LTC1_Read_Config(void);
 unsigned char LTC1_Read_Flags(void);
 unsigned char LTC1_PollInts(void);

void LTC1_Start_ADCCV(void);
unsigned char LTC1_Read_Voltages(void);
void LTC1_Start_ADCTEMP(void);
unsigned char LTC1_Read_TempReg(void);

 char LTC2_init(void);
void LTC2_Config(void);
unsigned char LTC2_Read_Config(void);
 unsigned char LTC2_Read_Flags(void);
 unsigned char LTC2_PollInts(void);

void LTC2_Start_ADCCV(void);
unsigned char LTC2_Read_Voltages(void);
void LTC2_Start_ADCTEMP(void);
unsigned char LTC2_Read_TempReg(void);

 char LTC3_init(void);
void LTC3_Config(void);
unsigned char LTC3_Read_Config(void);
 unsigned char LTC3_Read_Flags(void);
 unsigned char LTC3_PollInts(void);

void LTC3_Start_ADCCV(void);
unsigned char LTC3_Read_Voltages(void);
void LTC3_Start_ADCTEMP(void);
unsigned char LTC3_Read_TempReg(void);

unsigned char Calculate_PEC(unsigned char theCommand, unsigned char CurrentPEC);
unsigned int get_cell_voltage(char cell, char ltc);

// LTC SPI Functional Prototypes
void LTC1SPI_init(void);
void LTC1_transmit(unsigned char data);
unsigned char LTC1_exchange(unsigned char data);

void LTC2SPI_init(void);
void LTC2_transmit(unsigned char data);
unsigned char LTC2_exchange(unsigned char data);

void LTC3SPI_init(void);
void LTC3_transmit(unsigned char data);
unsigned char LTC3_exchange(unsigned char data);

// SPI port interface macros
#define LTC1_select		P8OUT &= ~LT_CS1
#define LTC1_deselect	P8OUT |= LT_CS1
#define LTC2_select		P8OUT &= ~LT_CS2
#define LTC2_deselect	P8OUT |= LT_CS2
#define LTC3_select		P8OUT &= ~LT_CS3
#define LTC3_deselect	P8OUT |= LT_CS3

// LTC6803 DATA SHEET PAGE 22-23 TABLE 9 COMMAND CODES AND PEC BYTES
// Write Configuration Register Group
#define WRCFG				0x01
#define WRCFG_PEC			0xC7

// Read Configuration Register Group
#define RDCFG				0x02
#define RDCFG_PEC			0xCE

// Read All Cell Voltage Group
#define RDCV				0x04
#define RDCV_PEC			0xDC

// Read Cell Voltages 1-4
#define RDCVA				0x06
#define RDCVA_PEC			0xD2

// Read Cell Voltages 5-8
#define RDCVB				0x08
#define RDCVB_PEC			0xF8

// Read Cell Voltages 9-12
#define	RDCVC				0x0A
#define RDCVC_PEC			0xF6

// Read Flag Register Group
#define RDFLG				0x0C
#define RDFLG_PEC			0xE4

// Read Tempperature Register Group
#define RDTMP				0x0E
#define RDTMP_PEC			0xEA

// Start Cell Voltage ADC Conversions and Poll Status
#define STCVAD_All					0x10
#define STCVAD_All_PEC				0xB0
#define STCVAD_Cell_1				0x11
#define STCVAD_Cell_1_PEC			0xB7
#define STCVAD_Cell_2				0x12
#define STCVAD_Cell_2_PEC			0xBE
#define STCVAD_Cell_3				0x13
#define STCVAD_Cell_3_PEC			0xB9
#define STCVAD_Cell_4				0x14
#define STCVAD_Cell_4_PEC			0xAC
#define STCVAD_Cell_5				0x15
#define STCVAD_Cell_5_PEC			0xAB
#define STCVAD_Cell_6				0x16
#define STCVAD_Cell_6_PEC			0xA2
#define STCVAD_Cell_7				0x17
#define STCVAD_Cell_7_PEC			0xA5
#define STCVAD_Cell_8				0x18
#define STCVAD_Cell_8_PEC			0x88
#define STCVAD_Cell_9				0x19
#define STCVAD_Cell_9_PEC			0x8F
#define STCVAD_Cell_10				0x1A
#define STCVAD_Cell_10_PEC			0x86
#define STCVAD_Cell_11				0x1B
#define STCVAD_Cell_11_PEC			0x81
#define STCVAD_Cell_12				0x1C
#define STCVAD_Cell_12_PEC			0x94
#define STCVAD_Clear_FF   			0x1D  // Begin Kenwood 3/9/12
#define STCVAD_Clear_FF_PEC			0x93
#define STCVAD_Self_Test1   		0x1E
#define STCVAD_Self_Test1_PEC		0x9A
#define STCVAD_Self_Test2			0x1F
#define STCVAD_Self_Test2_PEC		0x9D

// Start Open-Wire ADC Converstions and Poll Status
#define STOWAD_All					0x20
#define STOWAD_All_PEC				0x20
#define STOWAD_Cell_1				0x21
#define STOWAD_Cell_1_PEC			0x27
#define STOWAD_Cell_2				0x22
#define STOWAD_Cell_2_PEC			0x2E
#define STOWAD_Cell_3				0x23
#define STOWAD_Cell_3_PEC			0x29
#define STOWAD_Cell_4				0x24
#define STOWAD_Cell_4_PEC			0x3C
#define STOWAD_Cell_5				0x25
#define STOWAD_Cell_5_PEC			0x3B
#define STOWAD_Cell_6				0x26
#define STOWAD_Cell_6_PEC			0x32
#define STOWAD_Cell_7				0x27
#define STOWAD_Cell_7_PEC			0x35
#define STOWAD_Cell_8				0x28
#define STOWAD_Cell_8_PEC			0x18
#define STOWAD_Cell_9				0x29
#define STOWAD_Cell_9_PEC			0x1F
#define STOWAD_Cell_10				0x2A
#define STOWAD_Cell_10_PEC			0x16
#define STOWAD_Cell_11				0x2B
#define STOWAD_Cell_11_PEC			0x11
#define STOWAD_Cell_12				0x2C
#define STOWAD_Cell_12_PEC			0x4

//Start Temperature ADC Converstions and Poll Status
#define STTMPAD_All					0x30
#define STTMPAD_All_PEC				0x50
#define STTMPAD_al1			0x31
#define STTMPAD_al1_PEC		0x57
#define STTMPAD_al2			0x32
#define STTMPAD_al2_PEC		0x5E
#define STTMPAD_Internal			0x33
#define STTMPAD_Internal_PEC		0x59
#define STTMPAD_Self_Test_1			0x3E
#define STTMPAD_Self_Test_1_PEC		0x7A
#define STTMPAD_Self_Test_2			0x3F
#define STTMPAD_Self_Test_2_PEC		0x7D

// Poll ADC Converter Status
#define PLADC						0x40
#define PLADC_PEC					0x07

// Poll Interrupt Status
#define PLINT						0x50
#define PLINT_PEC					0x77

// Start Diagnose and Poll Status
#define DAGN						0x52
#define DAGN_PEC					0x79

// Read Diagnostic Register
#define RDDGNR						0x52
#define RDDGNR_PEC					0x6B

// Start Cell Voltage ADC Conversions and Poll Status, with Discharge Permitted
#define STCVDC_All				    0x60
#define STCVDC_All_PEC				0xE7
#define STCVDC_Cell_1				0x61
#define STCVDC_Cell_1_PEC			0xE0
#define STCVDC_Cell_2				0x62	
#define STCVDC_Cell_2_PEC			0xE9
#define STCVDC_Cell_3				0x63
#define STCVDC_Cell_3_PEC			0xEE
#define STCVDC_Cell_4				0x64
#define STCVDC_Cell_4_PEC			0xFB
#define STCVDC_Cell_5				0x65
#define STCVDC_Cell_5_PEC			0xFC
#define STCVDC_Cell_6				0x66
#define STCVDC_Cell_6_PEC			0xF5
#define STCVDC_Cell_7				0x67
#define STCVDC_Cell_7_PEC			0xF2
#define STCVDC_Cell_8				0x68
#define STCVDC_Cell_8_PEC			0xDF
#define STCVDC_Cell_9				0x69
#define STCVDC_Cell_9_PEC			0xD8
#define STCVDC_Cell_10				0x6A
#define STCVDC_Cell_10_PEC			0xD1
#define STCVDC_Cell_11				0x6B
#define STCVDC_Cell_11_PEC			0xD6
#define STCVDC_Cell_12				0x6C
#define STCVDC_Cell_12_PEC			0xC3

// Start Open-Wire ADC Conversions and Poll Status, with Discharge Permitted
#define STOWDC_All				0x70
#define STOWDC_All_PEC				0x97
#define STOWDC_Cell_1				0x71
#define STOWDC_Cell_1_PEC			0x90
#define STOWDC_Cell_2				0x72
#define STOWDC_Cell_2_PEC			0x99
#define STOWDC_Cell_3				0x73
#define STOWDC_Cell_3_PEC			0x9E
#define STOWDC_Cell_4				0x74
#define STOWDC_Cell_4_PEC			0x8B
#define STOWDC_Cell_5				0x75
#define STOWDC_Cell_5_PEC			0x8C
#define STOWDC_Cell_6				0x76
#define STOWDC_Cell_6_PEC			0x85
#define STOWDC_Cell_7				0x77
#define STOWDC_Cell_7_PEC			0x82
#define STOWDC_Cell_8				0x78
#define STOWDC_Cell_8_PEC			0xAF
#define STOWDC_Cell_9				0x79
#define STOWDC_Cell_9_PEC			0xA8
#define STOWDC_Cell_10				0x7A
#define STOWDC_Cell_10_PEC			0xA1
#define STOWDC_Cell_11				0x7B
#define STOWDC_Cell_11_PEC			0xA6
#define STOWDC_Cell_12				0x7C
#define STOWDC_Cell_12_PEC			0xB3

// Configuration Register Group
#define CDC_0						0x00
#define CDC_1						0x01
#define CDC_2						0x02
#define CDC_3						0x03
#define CDC_4						0x04
#define CDC_5						0x05
#define CDC_6						0x06
#define CDC_7						0x07
#define CELL10						0x08
#define LVLPL						0x10
#define GPIO1						0x20
#define GPIO2						0x40
#define WDT							0x80
#define DCC1						0x01
#define DCC2						0x02
#define DCC3						0x04
#define DCC4						0x08
#define DCC5						0x10
#define DCC6						0x20
#define DCC7						0x40
#define DCC8						0x80
#define DCC9						0x01
#define DCC10						0x02
#define DCC11						0x04
#define DCC12						0x08
#define MC1I						0x10
#define MC2I						0x20
#define MC3I						0x40
#define MC4I						0x80
#define MC5I						0x01
#define MC6I						0x02
#define MC7I						0x04
#define MC8I						0x08
#define MC9I						0x10
#define MC10I						0x20
#define MC11I						0x40
#define MC12I						0x80
#define VUV_0						0x01   // Should VUV and VOV comparisons start at 0x00? I think so. Should we control using bits or voltages?
#define VUV_1						0x02
#define VUV_2						0x04
#define VUV_3						0x08
#define VUV_4						0x10
#define VUV_5						0x20
#define VUV_6						0x40
#define VUV_7						0x80
#define VOV_0						0x01
#define VOV_1						0x02
#define VOV_2						0x04
#define VOV_3						0x08
#define VOV_4						0x10
#define VOV_5						0x20
#define VOV_6						0x40
#define VOV_7						0x80

// Cell Voltage (CV) Register Group
#define C1V_0						0x01
#define C1V_1						0x02
#define C1V_2						0x04
#define C1V_3						0x08
#define C1V_4						0x10
#define C1V_5						0x20
#define C1V_6						0x40
#define C1V_7						0x80
#define C1V_8						0x01
#define C1V_9						0x02
#define C1V_10						0x04
#define C1V_11						0x08
#define C2V_0						0x10
#define C2V_1						0x20
#define C2V_2						0x40
#define C2V_3						0x80
#define C2V_4						0x01
#define C2V_5						0x02
#define C2V_6						0x04
#define C2V_7						0x08
#define C2V_8						0x10
#define C2V_9						0x20
#define C2V_10						0x40
#define C2V_11						0x80
#define C3V_0						0x01
#define C3V_1						0x02
#define C3V_2						0x04
#define C3V_3						0x08
#define C3V_4						0x10
#define C3V_5						0x20
#define C3V_6						0x40
#define C3V_7						0x80
#define C3V_8						0x01
#define C3V_9						0x02
#define C3V_10						0x04
#define C3V_11						0x08
#define C4V_0						0x10
#define C4V_1						0x20
#define C4V_2						0x40
#define C4V_3						0x80
#define C4V_4						0x01
#define C4V_5						0x02
#define C4V_6						0x04
#define C4V_7						0x08
#define C4V_8						0x10
#define C4V_9						0x20
#define C4V_10						0x40
#define C4V_11						0x80
#define C5V_0						0x01
#define C5V_1						0x02
#define C5V_2						0x04
#define C5V_3						0x08
#define C5V_4						0x10
#define C5V_5						0x20
#define C5V_6						0x40
#define C5V_7						0x80
#define C5V_0						0x01
#define C5V_9						0x02
#define C5V_10						0x04
#define C5V_11						0x08
#define C6V_0						0x10
#define C6V_1						0x20
#define C6V_2						0x40
#define C6V_3						0x80
#define C6V_4						0x01
#define C6V_5						0x02
#define C6V_6						0x04
#define C6V_7						0x08
#define C6V_8						0x10
#define C6V_9						0x20
#define C6V_10						0x40
#define C6V_11						0x80
#define C7V_0						0x01
#define C7V_1						0x02
#define C7V_2						0x04
#define C7V_3						0x08
#define C7V_4						0x10
#define C7V_5						0x20
#define C7V_6						0x40
#define C7V_7						0x80
#define C7V_8						0x01
#define C7V_9						0x02
#define C7V_10						0x04
#define C7V_11						0x08
#define C8V_0						0x10
#define C8V_1						0x20
#define C8V_2						0x40
#define C8V_3						0x80
#define C8V_4						0x01
#define C8V_5						0x02
#define C8V_6						0x04
#define C8V_7						0x08
#define C8V_8						0x10
#define C8V_9						0x20
#define C8V_10						0x40
#define C8V_11						0x80
#define C9V_0						0x01
#define C9V_1						0x02
#define C9V_2						0x04
#define C9V_3						0x08
#define C9V_4						0x10
#define C9V_5						0x20
#define C9V_6						0x40
#define C9V_7						0x80
#define C9V_8						0x01
#define C9V_9						0x02
#define C9V_10						0x04
#define C9V_11						0x08
#define C10V_0						0x10
#define C10V_1						0x20
#define C10V_2						0x40
#define C10V_3						0x80
#define C10V_4						0x01
#define C10V_5						0x02
#define C10V_6						0x04
#define C10V_7						0x08
#define C10V_8						0x10
#define C10V_9						0x20
#define C10V_10						0x40
#define C10V_11						0x80
#define C11V_0						0x01
#define C11V_1						0x02
#define C11V_2						0x04
#define C11V_3						0x08
#define C11V_4						0x10
#define C11V_5						0x20
#define C11V_6						0x40
#define C11V_7						0x80
#define C11V_8						0x01
#define C11V_9						0x02
#define C11V_10						0x04
#define C11V_11						0x08
#define C12V_0						0x10
#define C12V_1						0x20
#define C12V_2						0x40
#define C12V_3						0x80
#define C12V_4						0x01
#define C12V_5						0x02
#define C12V_6						0x04
#define C12V_7						0x08
#define C12V_8						0x10
#define C12V_9						0x20
#define C12V_10						0x40
#define C12V_11						0x80

// Flag (FLG) Register Group
#define C1UV						0x01
#define C1OV						0x02
#define C2UV						0x04
#define C2OV						0x08
#define C3UV						0x10
#define C4UV						0x40
#define C3OV						0x20
#define C4OV						0x80
#define C5UV						0x01
#define C5OV						0x02
#define C6UV						0x04
#define C6OV						0x08
#define C7UV						0x10
#define C7OV						0x20
#define C8UV						0x40
#define C8OV						0x80
#define C9UV						0x01
#define C9OV						0x02
#define C10UV						0x04
#define C10OV						0x08
#define C11UV						0x10
#define C11OV						0x20
#define C12UV						0x40
#define C12OV						0x80

// Temperature (TMP) Register Group
#define ETMP1_0						0x01
#define ETMP1_1						0x02
#define ETMP1_2						0x04
#define ETMP1_3						0x08
#define ETMP1_4						0x10
#define ETMP1_5						0x20
#define ETMP1_6						0x40
#define ETMP1_7						0x80
#define ETMP1_8						0x01
#define ETMP1_9						0x02
#define ETMP1_10					0x04
#define ETMP1_11					0x08
#define ETMP2_0						0x10
#define ETMP2_1						0x20
#define ETMP2_2						0x40
#define ETMP2_3						0x80
#define ETMP2_4						0x01
#define ETMP2_5						0x02
#define ETMP2_6						0x04
#define ETMP2_7						0x08
#define ETMP2_8						0x10
#define ETMP2_9						0x20
#define ETMP2_10					0x40
#define ETMP2_11					0x80

// Packet Error Code (PEC)
// Need to finish/redo this!!!
#define PEC_0						0x01
#define PEC_1						0x02
#define PEC_2						0x04
#define PEC_3						0x08
#define PEC_4						0x10
#define PEC_5						0x20
#define PEC_6						0x40
#define PEC_7						0x80


// Diagnostic Register Group
#define REF_0						0x01
#define REF_1						0x02
#define REF_2						0x04
#define REF_3						0x08
#define REF_4						0x10
#define REF_5						0x20
#define REF_6						0x40
#define REF_7						0x80
#define REF_8						0x01
#define REF_9						0x02
#define REF_10						0x04
#define REF_11						0x08
#define MUXFAIL						0x20
#define REV_0						0x40
#define REV_1						0x80


#endif /*LTC6803_H_*/
