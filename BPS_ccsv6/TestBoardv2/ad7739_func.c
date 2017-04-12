/*
* ADC converter code for the AD7734
* B.J. Bazuin 5/7/2009
* 
* Modified for ADC converter code for the AD7739
* B.J. Bazuin 6/12/2012
*
* Modified for BPS_V2 2015 by Scott Haver
*
* Sunseeker 2015
* 
*/

// header files
#include <msp430x54xa.h>
#include "BPSmain.h"
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
  

/*================================== ADC BUS1 Functions ======================================*/

void adc_bus1_reset(void)    //reset the ADC
{
  char i;
  // Sowtware Reset
  for (i=1;i<=3;i++)
  {
  	if(i==1) adc1_select;         
  	if(i==2) adc2_select;         
  	if(i==3) adc3_select;         
  	             
    adc_bus1_transmit(0x00);
    adc_bus1_transmit(0xFF);                
    adc_bus1_transmit(0xFF);               
    adc_bus1_transmit(0xFF);                
    adc_bus1_transmit(0xFF);    
    
    if(i==1) adc1_deselect;         
  	if(i==2) adc2_deselect;         
  	if(i==3) adc3_deselect; 
  }        
}

char adc_bus1_init(void)    //initialize BUS1
{
  char status_adc;
  char i,j;
  
  // reset the device to default conditions
  adc_bus1_reset();
  
  // IO Control Initialization
  
  for (i=1;i<=3;i++)
  {
  	if(i==1) adc1_select;         
  	if(i==2) adc2_select;         
  	if(i==3) adc3_select;    
  	
  	adc_bus1_transmit(ADC_IOPORT);             // Command Register write for IOPORT
    adc_bus1_transmit(ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
    
    if(i==1) adc1_deselect;         
  	if(i==2) adc2_deselect;         
  	if(i==3) adc3_deselect; 
  }        

  for (i=1;i<=3;i++)
  {
  	if(i==1) adc1_select;
  	if(i==2) adc2_select;
  	if(i==3) adc3_select;

  	adc_bus1_transmit(ADC_IOPORT);             // Command Register write for IOPORT
    adc_bus1_transmit(ADCPODIR | ADCP1DIR | ADCP1);	// P0 and P1 Outputs, Data P0=0 P1=1

    if(i==1) adc1_deselect;
  	if(i==2) adc2_deselect;
  	if(i==3) adc3_deselect;
  }

  // Initialize Channel 0-7
  
  for (j=0;j<=7;j++)
  {
  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc1_select;         
  		if(i==2) adc2_select;         
  		if(i==3) adc3_select;    
  	
    	adc_bus1_transmit(ADC_SETUP0 | j);      	// Command Register write for SETUP0
    	adc_bus1_transmit(ENABLE | RNG0 | RNG2 );       	// 0-2.5 V conversion, RDY in status, not-continuous conversion

    	if(i==1) adc1_deselect;         
  		if(i==2) adc2_deselect;         
  		if(i==3) adc3_deselect; 
  	  }

  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc1_select;         
  		if(i==2) adc2_select;         
  		if(i==3) adc3_select;    
  	
    	adc_bus1_transmit(ADC_CT0 | j);      	// Command Register write for TC0
    	adc_bus1_transmit(CHOP | FWRATE); 		// Use Chopping and FW = FWRATE

    	if(i==1) adc1_deselect;         
  		if(i==2) adc2_deselect;         
  		if(i==3) adc3_deselect; 
  	  }

  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc1_select;         
  		if(i==2) adc2_select;         
  		if(i==3) adc3_select;    
  	
    	adc_bus1_transmit(ADC_MODE0 | j);      	// Command Register write for MODE0
    	adc_bus1_transmit(MIDLE | CLKDIS | BIT24_16n);  // Idle, MCLKOUT disabled, 24 bit conv.
    	if(i==1) adc1_deselect;         
  		if(i==2) adc2_deselect;         
  		if(i==3) adc3_deselect; 
  	  }
  }
  
  status_adc=adc1_status();
  status_adc=adc2_status();
  status_adc=adc3_status();
  return(status_adc);
}

void adc_bus1_ctset(char adc_tcword, char adc_channel, char adc_device)    //Set the channel conversion time
{
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_CT0 | adc_channel);      	// Command Register write for TC0,1,2,3
  adc_bus1_transmit(adc_tcword);             // Use char provided

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
}

// Run self cal on all 8 channels
char adc_bus1_selfcal(char adc_device)	//Self-calibration (internal shorting)
{
  static volatile signed long cal_value1, cal_value2;
  char status_adc, i;

  for(i=0;i<=7;i++)		// zero cal for each channel
  {
  	cal_value1=adc_bus1_zselfcal(i,adc_device);
  }

  for(i=0;i<=7;i++)
  {
//  	cal_value2=adc1_fselfcal(i,adc_device);		// full scale cal for each channel
  }

  if(adc_device==1) status_adc=adc1_status();     
  if(adc_device==2) status_adc=adc2_status();     
  if(adc_device==3) status_adc=adc3_status();  
    
  return(status_adc); 
}

signed long adc_bus1_zselfcal(char adc_channel, char adc_device)	//Per channel zero self-calibration (internal shorting)
{
  static signed long cal_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
    
  // Set the channel MODE to zero-scale self cal
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_bus1_transmit(MZSELFCAL | CLKDIS | BIT24_16n); // Mode zero-Scale Self Cal with other mode bits

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC1_RDYn) == ADC1_RDYn);
  if(adc_device==2) while ( (P2IN & ADC2_RDYn) == ADC2_RDYn);
  if(adc_device==3) while ( (P2IN & ADC3_RDYn) == ADC3_RDYn);

  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_COMM_RD | ADC_ZSCAL | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus1_exchange(0x00);        
  abyte2 = adc_bus1_exchange(0x00);       
  abyte3 = adc_bus1_exchange(0x00);       

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  cal_value = 0;
  cal_value = abyte1;
  cal_value = (cal_value<<8) | abyte2;
  cal_value = (cal_value<<8) | abyte3;
  
  adc_bus1_idle(adc_channel,adc_device);
  return(cal_value);
}

signed long adc_bus1_fselfcal(char adc_channel, char adc_device)	//Per channel zero self-calibration (internal shorting)
{
  static signed long cal_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  // Set the channel MODE to zero-scale self cal
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_bus1_transmit(MFSELFCAL | CLKDIS | BIT24_16n); // Mode zero-Scale Self Cal with other mode bits

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC1_RDYn) == ADC1_RDYn);
  if(adc_device==2) while ( (P2IN & ADC2_RDYn) == ADC2_RDYn);
  if(adc_device==3) while ( (P2IN & ADC3_RDYn) == ADC3_RDYn);

  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_COMM_RD | ADC_FS | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus1_exchange(0x00);        
  abyte2 = adc_bus1_exchange(0x00);       
  abyte3 = adc_bus1_exchange(0x00);       

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  cal_value = 0;
  cal_value = abyte1;
  cal_value = (cal_value<<8) | abyte2;
  cal_value = (cal_value<<8) | abyte3;
  
  adc_bus1_idle(adc_channel,adc_device);
  return(cal_value);
}

signed long adc_bus1_in(char adc_channel, char adc_device)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus1_transmit(MSINGLE| CLKDIS | BIT24_16n);  // Mode single conversion with other mode bits

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
   
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC1_RDYn) == ADC1_RDYn);
  if(adc_device==2) while ( (P2IN & ADC2_RDYn) == ADC2_RDYn);
  if(adc_device==3) while ( (P2IN & ADC3_RDYn) == ADC3_RDYn);
 
  //Read the ADC Data
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus1_exchange(0x00);        
  abyte2 = adc_bus1_exchange(0x00);       
  abyte3 = adc_bus1_exchange(0x00);       

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  adc_value = 0;
  adc_value = abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}
signed long adc_bus1_indump(char adc_channel, char adc_device)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus1_transmit(MSINGLE| CLKDIS | BIT24_16n);  // Mode single conversion with other mode bits

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
   
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC1_RDYn) == ADC1_RDYn);
  if(adc_device==2) while ( (P2IN & ADC2_RDYn) == ADC2_RDYn);
  if(adc_device==3) while ( (P2IN & ADC3_RDYn) == ADC3_RDYn);
 
  //Read the ADC Data
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    

  adc_bus1_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte0 = adc_bus1_exchange(0x00);        
  abyte1 = adc_bus1_exchange(0x00);        
  abyte2 = adc_bus1_exchange(0x00);       
  abyte3 = adc_bus1_exchange(0x00);       

  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  adc_value = abyte0;
  adc_value = (adc_value<<8) | abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}

void adc_bus1_contconv_start(char adc_channel, char adc_device)	// read voltage from multiple channels
{
  // IO Control Initialization
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_bus1_transmit(RDYFN | ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  //Set the channel MODE to continuous conversion
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus1_transmit(MCONT| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
}

void adc_bus1_idle(char adc_channel, char adc_device)	// read voltage from multiple channels
{
  //Set the channel MODE to idle
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus1_transmit(MIDLE| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
}

void adc_bus1_read_convert(char adc_channel, char adc_device)
{
  extern signed long adc_voltage[8*7];
  extern unsigned char adc_stat[8*7];
  static volatile signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3;
  unsigned int index, i;
 
  for (i=adc_channel;i<=7;i++)
   {
    //Read the ADC Data
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
    adc_bus1_transmit(ADC_COMM_RD | ADC_DATA0 | i);  // Command Register for ADCCON
    abyte0 = adc_bus1_exchange(0x00);        
    abyte1 = adc_bus1_exchange(0x00);        
    abyte2 = adc_bus1_exchange(0x00);       
    abyte3 = adc_bus1_exchange(0x00);       
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
    adc_value = abyte0;
    adc_value = (adc_value<<8) | abyte1;
    adc_value = (adc_value<<8) | abyte2;
    adc_value = (adc_value<<8) | abyte3;

    index = ((int)adc_device-1)*8+i;

    adc_stat[index] = (char)((adc_value & 0xFF000000)>>24);
    adc_voltage[index] = (adc_value & 0x00FFFFFF);

    // Check overflow, negative clamps, positive rolls over
    if((adc_stat[index] & 0x01) == 0x01)
    {
    	if((adc_stat[index] & 0x02) != 0x02) adc_voltage[index] += 0x01000000;
    }

   }
}


void adc_bus1_io(char adc_device)	// Toggle the IO Pins
{
  
  char p1p2_adc;
  
  // IO Control Initialization
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_COMM_RD | ADC_IOPORT);	// Command to read DATA IO Register 
  p1p2_adc=adc_bus1_exchange(0xFF);                 // P2 P1 values with IO control
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  p1p2_adc ^= 0xC0;
  
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_IOPORT);             // Command write DATA IO Register      
  adc_bus1_transmit(p1p2_adc);                 // P2 P1 outputs with IO control
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
}


char adc1_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc1_select;                      
  adc_bus1_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus1_exchange(0x00);	// Read the status
  adc1_deselect;
  
  return(status_adc);        
}

char adc2_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc2_select;                      
  adc_bus1_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus1_exchange(0x00);	// Read the status
  adc2_deselect;
  
  return(status_adc);        
}

char adc3_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc3_select;                      
  adc_bus1_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus1_exchange(0x00);	// Read the status
  adc3_deselect;
  
  return(status_adc);        
}

char adc_bus1_chstatus(char adc_channel, char adc_device)	// Read the ADC Channel Status Register 
{
  char chstatus_adc;
  
  if(adc_device==1) adc1_select;         
  if(adc_device==2) adc2_select;         
  if(adc_device==3) adc3_select;    
  adc_bus1_transmit(ADC_COMM_RD | ADC_STATUS0 | adc_channel);	// Command to read Channel Status Register
  chstatus_adc=adc_bus1_exchange(0x00);	// Read the status
  if(adc_device==1) adc1_deselect;         
  if(adc_device==2) adc2_deselect;         
  if(adc_device==3) adc3_deselect;    
  
  return(chstatus_adc);        
}


/*================================== ADC BUS1 Functions - End - ======================================*/



/*================================== ADC BUS2 Functions ======================================*/

void adc_bus2_reset(void)    //reset the ADC
{
  char i;
  // Sowtware Reset
  for (i=1;i<=3;i++)
  {
  	if(i==1) adc4_select;         
  	if(i==2) adc5_select;         
  	if(i==3) adc6_select;         
  	             
    adc_bus2_transmit(0x00);
    adc_bus2_transmit(0xFF);                
    adc_bus2_transmit(0xFF);               
    adc_bus2_transmit(0xFF);                
    adc_bus2_transmit(0xFF);    
    
    if(i==1) adc4_deselect;         
  	if(i==2) adc5_deselect;         
  	if(i==3) adc6_deselect; 
  }        
}

char adc_bus2_init(void)    //initialize BUS1
{
  char status_adc;
  char i,j;
  
  // reset the device to default conditions
  adc_bus2_reset();
  
  // IO Control Initialization
  
  for (i=1;i<=3;i++)
  {
  	if(i==1) adc4_select;         
  	if(i==2) adc5_select;         
  	if(i==3) adc6_select;    
  	
  	adc_bus2_transmit(ADC_IOPORT);             // Command Register write for IOPORT
    adc_bus2_transmit(ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
    
    if(i==1) adc4_deselect;         
  	if(i==2) adc5_deselect;         
  	if(i==3) adc6_deselect; 
  }        
  
  // Initialize Channel 0-7
  
  for (j=0;j<=7;j++)
  {
  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc4_select;         
  		if(i==2) adc5_select;         
  		if(i==3) adc6_select;    
  	
    	adc_bus2_transmit(ADC_SETUP0 | j);      	// Command Register write for SETUP0
    	adc_bus2_transmit(ENABLE | RNG0 | RNG2 );       	// 0-2.5 V conversion, RDY in status, not-continuous conversion

    	if(i==1) adc4_deselect;         
  		if(i==2) adc5_deselect;         
  		if(i==3) adc6_deselect; 
  	  }

  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc4_select;         
  		if(i==2) adc5_select;         
  		if(i==3) adc6_select;    
  	
    	adc_bus2_transmit(ADC_CT0 | j);      	// Command Register write for TC0
    	adc_bus2_transmit(CHOP | FWRATE); 		// Use Chopping and FW = FWRATE

    	if(i==1) adc4_deselect;         
  		if(i==2) adc5_deselect;         
  		if(i==3) adc6_deselect; 
  	  }

  	  for (i=1;i<=3;i++)
  	  {
  	  	if(i==1) adc4_select;         
  		if(i==2) adc5_select;         
  		if(i==3) adc6_select;    
  	
    	adc_bus2_transmit(ADC_MODE0 | j);      	// Command Register write for MODE0
    	adc_bus2_transmit(MIDLE | CLKDIS | BIT24_16n);  // Idle, MCLKOUT disabled, 24 bit conv.
    	if(i==1) adc4_deselect;         
  		if(i==2) adc5_deselect;         
  		if(i==3) adc6_deselect; 
  	  }
  }
  
  status_adc=adc4_status();
  status_adc=adc5_status();
  status_adc=adc6_status();
  return(status_adc);
}

void adc_bus2_ctset(char adc_tcword, char adc_channel, char adc_device)    //Set the channel conversion time
{
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_CT0 | adc_channel);      	// Command Register write for TC0,1,2,3
  adc_bus2_transmit(adc_tcword);             // Use char provided

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
}

// Run self cal on all 8 channels
char adc_bus2_selfcal(char adc_device)	//Self-calibration (internal shorting)
{
  static volatile signed long cal_value1, cal_value2;
  char status_adc, i;

  for(i=0;i<=7;i++)		// zero cal for each channel
  {
  	cal_value1=adc_bus2_zselfcal(i,adc_device);
  }

  for(i=0;i<=7;i++)
  {
//  	cal_value2=adc4_fselfcal(i,adc_device);		// full scale cal for each channel
  }

  if(adc_device==1) status_adc=adc4_status();
  if(adc_device==2) status_adc=adc5_status();
  if(adc_device==3) status_adc=adc6_status();
    
  return(status_adc); 
}

signed long adc_bus2_zselfcal(char adc_channel, char adc_device)	//Per channel zero self-calibration (internal shorting)
{
  static signed long cal_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
    
  // Set the channel MODE to zero-scale self cal
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_bus2_transmit(MZSELFCAL | CLKDIS | BIT24_16n); // Mode zero-Scale Self Cal with other mode bits

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC4_RDYn) == ADC4_RDYn);
  if(adc_device==2) while ( (P2IN & ADC5_RDYn) == ADC5_RDYn);
  if(adc_device==3) while ( (P2IN & ADC6_RDYn) == ADC6_RDYn);

  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_COMM_RD | ADC_ZSCAL | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus2_exchange(0x00);        
  abyte2 = adc_bus2_exchange(0x00);       
  abyte3 = adc_bus2_exchange(0x00);       

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  cal_value = 0;
  cal_value = abyte1;
  cal_value = (cal_value<<8) | abyte2;
  cal_value = (cal_value<<8) | abyte3;
  
  adc_bus2_idle(adc_channel,adc_device);
  return(cal_value);
}

signed long adc_bus2_fselfcal(char adc_channel, char adc_device)	//Per channel zero self-calibration (internal shorting)
{
  static signed long cal_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  // Set the channel MODE to zero-scale self cal
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_bus2_transmit(MFSELFCAL | CLKDIS | BIT24_16n); // Mode zero-Scale Self Cal with other mode bits

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC4_RDYn) == ADC4_RDYn);
  if(adc_device==2) while ( (P2IN & ADC5_RDYn) == ADC5_RDYn);
  if(adc_device==3) while ( (P2IN & ADC6_RDYn) == ADC6_RDYn);

  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_COMM_RD | ADC_FS | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus2_exchange(0x00);        
  abyte2 = adc_bus2_exchange(0x00);       
  abyte3 = adc_bus2_exchange(0x00);       

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  cal_value = 0;
  cal_value = abyte1;
  cal_value = (cal_value<<8) | abyte2;
  cal_value = (cal_value<<8) | abyte3;
  
  adc_bus2_idle(adc_channel,adc_device);
  return(cal_value);
}

signed long adc_bus2_in(char adc_channel, char adc_device)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus2_transmit(MSINGLE| CLKDIS | BIT24_16n);  // Mode single conversion with other mode bits

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
   
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC4_RDYn) == ADC4_RDYn);
  if(adc_device==2) while ( (P2IN & ADC5_RDYn) == ADC5_RDYn);
  if(adc_device==3) while ( (P2IN & ADC6_RDYn) == ADC6_RDYn);
 
  //Read the ADC Data
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_bus2_exchange(0x00);        
  abyte2 = adc_bus2_exchange(0x00);       
  abyte3 = adc_bus2_exchange(0x00);       

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  adc_value = 0;
  adc_value = abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}
signed long adc_bus2_indump(char adc_channel, char adc_device)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus2_transmit(MSINGLE| CLKDIS | BIT24_16n);  // Mode single conversion with other mode bits

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
   
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Wait for the conversion to be done
  if(adc_device==1) while ( (P2IN & ADC4_RDYn) == ADC4_RDYn);
  if(adc_device==2) while ( (P2IN & ADC5_RDYn) == ADC5_RDYn);
  if(adc_device==3) while ( (P2IN & ADC6_RDYn) == ADC6_RDYn);
 
  //Read the ADC Data
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;

  adc_bus2_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte0 = adc_bus2_exchange(0x00);        
  abyte1 = adc_bus2_exchange(0x00);        
  abyte2 = adc_bus2_exchange(0x00);       
  abyte3 = adc_bus2_exchange(0x00);       

  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  adc_value = abyte0;
  adc_value = (adc_value<<8) | abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}

void adc_bus2_contconv_start(char adc_channel, char adc_device)	// read voltage from multiple channels
{
  // IO Control Initialization
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_bus2_transmit(RDYFN | ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  //Set the channel MODE to continuous conversion
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus2_transmit(MCONT| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
}

void adc_bus2_idle(char adc_channel, char adc_device)	// read voltage from multiple channels
{
  //Set the channel MODE to idle
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_bus2_transmit(MIDLE| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
}

void adc_bus2_read_convert(char adc_channel, char adc_device)
{
  extern signed long adc_voltage[8*7];
  extern unsigned char adc_stat[8*7];
  static volatile signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3;
  unsigned int index, i;
 
  for (i=adc_channel;i<=7;i++)
  {
    //Read the ADC Data
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
    adc_bus2_transmit(ADC_COMM_RD | ADC_DATA0 | i);  // Command Register for ADCCON
    abyte0 = adc_bus2_exchange(0x00);        
    abyte1 = adc_bus2_exchange(0x00);        
    abyte2 = adc_bus2_exchange(0x00);       
    abyte3 = adc_bus2_exchange(0x00);       
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
    adc_value = abyte0;
    adc_value = (adc_value<<8) | abyte1;
    adc_value = (adc_value<<8) | abyte2;
    adc_value = (adc_value<<8) | abyte3;

    index = ((int)adc_device-1)*8+i;

    adc_stat[24+index] = (char)((adc_value & 0xFF000000)>>24);
    adc_voltage[24+index] = (adc_value & 0x00FFFFFF);

    // Check overflow, negative clamps, positive rolls over
    if((adc_stat[24+index] & 0x01) == 0x01)
    {
    	if((adc_stat[24+index] & 0x02) != 0x02) adc_voltage[24+index] += 0x01000000;
    }
  }
}


void adc_bus2_io(char adc_device)	// Toggle the IO Pins
{
  
  char p1p2_adc;
  
  // IO Control Initialization
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_COMM_RD | ADC_IOPORT);	// Command to read DATA IO Register 
  p1p2_adc=adc_bus2_exchange(0xFF);                 // P2 P1 values with IO control
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  p1p2_adc ^= 0xC0;
  
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_IOPORT);             // Command write DATA IO Register      
  adc_bus2_transmit(p1p2_adc);                 // P2 P1 outputs with IO control
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
}


char adc4_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc4_select;                      
  adc_bus2_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus2_exchange(0x00);	// Read the status
  adc4_deselect;
  
  return(status_adc);        
}

char adc5_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc5_select;                      
  adc_bus2_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus2_exchange(0x00);	// Read the status
  adc5_deselect;
  
  return(status_adc);        
}

char adc6_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc6_select;                      
  adc_bus2_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_bus2_exchange(0x00);	// Read the status
  adc6_deselect;
  
  return(status_adc);        
}

char adc_bus2_chstatus(char adc_channel, char adc_device)	// Read the ADC Channel Status Register 
{
  char chstatus_adc;
  
  if(adc_device==1) adc4_select;
  if(adc_device==2) adc5_select;
  if(adc_device==3) adc6_select;
  adc_bus2_transmit(ADC_COMM_RD | ADC_STATUS0 | adc_channel);	// Command to read Channel Status Register
  chstatus_adc=adc_bus2_exchange(0x00);	// Read the status
  if(adc_device==1) adc4_deselect;
  if(adc_device==2) adc5_deselect;
  if(adc_device==3) adc6_deselect;
  
  return(chstatus_adc);        
}


/*================================== ADC BUS2 Functions - End - ======================================*/





/*================================== ADC MISC Functions ======================================*/

void adc_misc_reset()    //reset the misc ADC
{
  // Sowtware Reset
  adc7_select;                      
  adc_misc_transmit(0x00);
  adc_misc_transmit(0xFF);
  adc_misc_transmit(0xFF);
  adc_misc_transmit(0xFF);
  adc_misc_transmit(0xFF);
  adc7_deselect;                     
}

char adc_misc_init()    //initialize misc ADC
{
  char status_adc;
  char i;
  
  // reset the device to default conditions
  adc_misc_reset();
  
  // IO Control Initialization
  adc7_select;                      
  adc_misc_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_misc_transmit(ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  adc7_deselect;  
  
  // Initialize Channel 0-7
  
  for (i=0;i<=7;i++)
  {
    adc7_select;                      
    adc_misc_transmit(ADC_SETUP0 | i);      	// Command Register write for SETUP0
    adc_misc_transmit(ENABLE | RNG0 | RNG2);             // 0-2.5 V conversion, RDY in status, not-continuous conversion
    adc7_deselect;                      
  
    adc7_select;                      
    adc_misc_transmit(ADC_CT0 | i);      	// Command Register write for TC0
    adc_misc_transmit(CHOP | FWRATE); // Use Chopping and FW = FWRATE
    adc7_deselect;                      
  
    adc7_select;                      
    adc_misc_transmit(ADC_MODE0 | i);      	// Command Register write for MODE0
    adc_misc_transmit(MIDLE | CLKDIS | BIT24_16n);  // Idle, MCLKOUT disabled, 24 bit conv.
    adc7_deselect; 
  
    status_adc=adc_misc_status();
  }
  
//  status_adc=adc_misc_status();
  return(status_adc);
}

char adc_misc_run_config(void)    //Operational ADC4 configuration (read channels 4-7)
{
  char status_adc;
  char i;
  
  // Reconfigure Channel 0-7
  
  for (i=0;i<=3;i++)
  {
    adc7_select;                      
    adc_misc_transmit(ADC_SETUP0 | i);      	// Command Register write for SETUP0
    adc_misc_transmit( RNG0 | RNG2);            		// 0-2.5 V conversion, RDY in status, not-continuous conversion
    adc7_deselect;                      
  }
  
  for (i=4;i<=7;i++)
  {
    adc7_select;                      
    adc_misc_transmit(ADC_SETUP0 | i);      	// Command Register write for SETUP0
    adc_misc_transmit(ENABLE | RNG0 | RNG2);        // 0-2.5 V conversion, RDY in status, not-continuous conversion
    adc7_deselect;                      
  }
  
  status_adc=adc_misc_status();
  return(status_adc);
}


void adc_misc_ctset(char adc_tcword, char adc_channel)    //Set the channel conversion time
{
  adc7_select;                      
  adc_misc_transmit(ADC_CT0 | adc_channel);      	// Command Register write for TC0,1,2,3
  adc_misc_transmit(adc_tcword);             // Use char provided
  adc7_deselect;  
}


// Run self cal on all 8 channels
char adc_misc_selfcal(void)	//Self-calibration (internal shorting)
{
  char status_adc, i;

  for(i=0;i<=7;i++)		// zero cal for each channel
  {
  	status_adc=adc_misc_zselfcal(i);
  }

  for(i=0;i<=7;i++)
  {
  	status_adc=adc_misc_fselfcal(i);		// full scale cal for each channel
  }

  status_adc=adc_misc_status();
  
  return(status_adc); 
}

char adc_misc_zselfcal(char adc_channel)	//Per channel zero self-calibration (internal shorting)
{
  unsigned int i;    
  char status_adc;
  
  // Set the channel MODE to zero-scale self cal
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_misc_transmit(MZSELFCAL | CLKDIS | BIT24_16n | CLAMP); // Mode zero-Scale Self Cal with other mode bits
  adc7_deselect;  
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);

  status_adc=adc_misc_status();
  return(status_adc);  	
}

char adc_misc_fselfcal(char adc_channel)	//Per channel zero self-calibration (internal shorting)
{
  unsigned int i;    
  char status_adc;
  
  // Set the channel MODE to zero-scale self cal
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);      	// Command Register write for MODE
  adc_misc_transmit(MFSELFCAL | CLKDIS | BIT24_16n | CLAMP); // Mode zero-Scale Self Cal with other mode bits
  adc7_deselect;  
  
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);

  status_adc=adc_misc_status();
  return(status_adc);  	
}

signed long adc_misc_in(char adc_channel)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MSINGLE| CLKDIS | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
    
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);

  //Read the ADC Data
  adc7_select;                      
  adc_misc_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte1 = adc_misc_exchange(0x00);
  abyte2 = adc_misc_exchange(0x00);       
  abyte3 = adc_misc_exchange(0x00);       
  adc7_deselect;   
  
  adc_value = 0;
  adc_value = abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}

// In the dump mode, the opper 8 bits is the channel status.
signed long adc_misc_indump(char adc_channel)	// read voltage from Channel adc_channel (0-3)
{
  static signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3; 
  unsigned int i;    
  
  //Set the channel MODE to single conversion
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MSINGLE| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
    
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Read the ADC Data
  adc7_select;                      
  adc_misc_transmit(ADC_COMM_RD | ADC_DATA0 | adc_channel);  // Command Register for ADCCON
  abyte0 = adc_misc_exchange(0x00);        
  abyte1 = adc_misc_exchange(0x00);        
  abyte2 = adc_misc_exchange(0x00);       
  abyte3 = adc_misc_exchange(0x00);       
  adc7_deselect;   
  
  adc_value = abyte0;
  adc_value = (adc_value<<8) | abyte1;
  adc_value = (adc_value<<8) | abyte2;
  adc_value = (adc_value<<8) | abyte3;
  
  return(adc_value);
}

void adc_misc_convert(void)	// read voltage from multiple channels
{
  extern signed long adc_voltage[8*7];
  extern unsigned char adc_stat[8*7];
  static signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3; 
  unsigned int i; 
  
  // IO Control Initialization
  adc7_select;                      
  adc_misc_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_misc_transmit(RDYFN | ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  adc7_deselect;  
     
  
  //Set the channel MODE to continuous conversion
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MCONT| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
    
  i = 32;		// Short SW Delay
  do i--;
  while (i != 0);
  
  //Set the channel MODE to idle
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MIDLE| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
 
  for (i=0;i<=7;i++)
  {
    //Read the ADC Data
    adc7_select;                      
    adc_misc_transmit(ADC_COMM_RD | ADC_DATA0 | i);  // Command Register for ADCCON
    abyte0 = adc_misc_exchange(0x00);
    abyte1 = adc_misc_exchange(0x00);        
    abyte2 = adc_misc_exchange(0x00);       
    abyte3 = adc_misc_exchange(0x00);       
    adc7_deselect;   
  
    adc_value = abyte0;
    adc_value = (adc_value<<8) | abyte1;
    adc_value = (adc_value<<8) | abyte2;
    adc_value = (adc_value<<8) | abyte3;
  
    adc_stat[48+i] = (char)((adc_value & 0xFF000000)>>24);
    adc_voltage[48+i] = (adc_value & 0x00FFFFFF);
// Check overflow, negative clamps, positive rolls over   
    if((adc_stat[48+i] & 0x01) == 0x01)
    {
  	  if((adc_stat[48+i] & 0x02) != 0x02) adc_voltage[48+i] += 0x01000000;
    }
  }
  
  // IO Control Initialization
  adc7_select;                      
  adc_misc_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_misc_transmit(ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  adc7_deselect;  
  
}

void adc_misc_contconv_start(char adc_channel)	// read voltage from multiple channels
{
  // IO Control Initialization
  adc7_select;                      
  adc_misc_transmit(ADC_IOPORT);             // Command Register write for IOPORT
  adc_misc_transmit(RDYFN | ADCPODIR | ADCP1DIR | ADCPO);	// P0 and P1 Outputs, Data P0=1 P1=0
  adc7_deselect;  
  
  //Set the channel MODE to continuous conversion
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MCONT| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
}

void adc_misc_idle(char adc_channel)	// read voltage from multiple channels
{
  //Set the channel MODE to idle
  adc7_select;                      
  adc_misc_transmit(ADC_MODE0 | adc_channel);              // Command Register for MODE0,1,2,3
  adc_misc_transmit(MIDLE| CLKDIS | DUMP | BIT24_16n);  // Mode single conversion with other mode bits
  adc7_deselect; 
}

unsigned long adc_misc_read_convert(char adc_channel)
{
  extern signed long adc_voltage[8*7];
  extern unsigned char adc_stat[8*7];
  static signed long adc_value;
  static unsigned char abyte0, abyte1, abyte2, abyte3; 
  unsigned int i; 
 
  for (i=adc_channel;i<=7;i++)
  {
    //Read the ADC Data
    adc7_select;
    adc_misc_transmit(ADC_COMM_RD | ADC_DATA0 | i);  // Command Register for ADCCON
    abyte0 = adc_misc_exchange(0x00);
    abyte1 = adc_misc_exchange(0x00);        
    abyte2 = adc_misc_exchange(0x00);       
    abyte3 = adc_misc_exchange(0x00);       
    adc7_deselect;   
  
    adc_value = abyte0;
    adc_value = (adc_value<<8) | abyte1;
    adc_value = (adc_value<<8) | abyte2;
    adc_value = (adc_value<<8) | abyte3;
  
    adc_stat[48+i] = (char)((adc_value & 0xFF000000)>>24);
    adc_voltage[48+i] = (adc_value & 0x00FFFFFF);
// Check overflow, negative clamps, positive rolls over
    if((adc_stat[48+i] & 0x01) == 0x01)		// overflow
    {
  	  if((adc_stat[48+i] & 0x02) != 0x02) adc_voltage[48+i] += 0x01000000;
    }
  }
  return(adc_value);
}


void adc_misc_io(void)	// Toggle the IO Pins
{
  
  char p1p2_adc;
  
  // IO Control Initialization
  adc7_select;                      
  adc_misc_transmit(ADC_COMM_RD | ADC_IOPORT);	// Command to read DATA IO Register
  p1p2_adc=adc_misc_exchange(0xFF);                 // P2 P1 values with IO control
  adc7_deselect;     
  
  p1p2_adc ^= 0xC0;
  
  adc7_select;                      
  adc_misc_transmit(ADC_IOPORT);             // Command write DATA IO Register
  adc_misc_transmit(p1p2_adc);                 // P2 P1 outputs with IO control
  adc7_deselect; 
}


char adc_misc_status(void)	// Read the ADC Status Register 
{
  char status_adc;
  
  adc7_select;                      
  adc_misc_transmit(ADC_COMM_RD | ADC_STATUS);	// Command to read Status Register
  status_adc=adc_misc_exchange(0x00);	// Read the status
  adc7_deselect;
  
  return(status_adc);        
}

char adc_misc_chstatus(char adc_channel)	// Read the ADC Cahnnel Status Register 
{
  char chstatus_adc;
  
  adc7_select;                      
  adc_misc_transmit(ADC_COMM_RD | ADC_STATUS0 | adc_channel);	// Command to read Channel Status Register
  chstatus_adc=adc_misc_exchange(0x00);	// Read the status
  adc7_deselect;
  
  return(chstatus_adc);        
}


/*================================== ADC Check Functions - End - ======================================*/

char adc_temp_check(void)
{
	
	volatile float temp_voltage;
	unsigned char temp_Error;
	unsigned char terr;
	unsigned char i,j,index;

    extern long max_temp_voltage;
	extern float max_temp_val;
	extern int max_temp_idx;
	extern unsigned char max_temp_index;

	extern signed long adc_voltage[8*7];

	// Temp Locations: 1,2,3,4,5,6,7,  9,10,11,12,13,14,15,  17,18,19,20,21,22,23,
	// Temp Locations: 25,26,27,28,29,30,31  33,34,35,36,37,38,39, 46,47
	// Temp Locations: 50, 51

	// 2.5V Locations: 0,8,16,24,32,40,48
	// MC PC Locations: 0,8,16,24,32,40,48
	// SCAP Ref & Shunt Location: 45 & 41
	// SCAP PC Locations: 42, 43, 44
	// Bat Ref & Shunt Location: 49 & 52
	// MC PC Locations: 53, 54, 55

//	static const char TALL_table[40] = {
//			1,2,3,4,5,6,7,  9,10,11,12,13,14,15,  17,18,19,20,21,22,23,
//			25,26,27,28,29,30,31,  33,34,35,36,37,38,39, 46,47, 50, 51};
	static const char TCHECK[35] = {
			1,2,3,4,5,  9,10,11,12,13,14,15,  17,18,19,20,21,22,23,
			25,26,27,28,29,30,31,  33,34,35,36,37,38,39 ,46,47 };

	static const char VRCHECK[7] = {0,8,16,24,32,40,48};

	terr = 0x00;

	max_temp_index = 0;
	max_temp_voltage = 0;
	temp_Error = 0;
	
	for (j=0;j<=34;j++)
	{
		index = TCHECK[j];

		if(adc_voltage[index] > max_temp_voltage)
		{
			max_temp_voltage = adc_voltage[index];
			max_temp_index = index;
		}

		if(adc_voltage[index]<=0x004582EA) temp_Error = 1;// 45 deg
		if(adc_voltage[index]<=0x002da2b3) temp_Error = 2;// 60 deg
		if(adc_voltage[index]> 0x00B00000) temp_Error = 3;// no therm
		// Computed from data sheet +/-1%
		//0x2DA2B3 is 60 deg (140 F)
		//0x34BCFA is 55 deg (131 F)
		//0x4582EA is 45 deg (113 F)
		//0x6F286B is 25 deg ( 77 F)
		//0xA05D1A is  0 deg ( 32 F)
		//0xC47711 is Therm not connected

		temp_voltage = (float)adc_voltage[index] * 2.5 /16777216.0;
		

	}
	if (temp_Error>terr) terr = temp_Error;

	max_temp_val = (float)max_temp_voltage * 2.5 /16777216.0;
	max_temp_idx = (int)max_temp_index;

	temp_Error = 0;

	for (i=0;i<=5;i++)
	{
		index = VRCHECK[i];
		if(adc_voltage[index]<=0x00F5C28F) temp_Error = 4;// Ref<=2.4V
	}
	if (temp_Error>terr) terr = temp_Error;

	 // 0 is OK, 1 is >45C, 2 is >60C, 3 is no them, 4 is ref err

	return(terr);
}

