#ifndef RS232_PORTS_H_
#define RS232_PORTS_H_

//public declarations constants

static char RS232Cmd[5] = "+++\r\0";
static char RS232_Test1[13] = "Sunseeker \n\r\0";
static char RS232_Test2[9] = "2014. \n\r\0";
static char Parse_header[6][5] = {"LTC \0","ADC \0","ISH \0","ERR \0","BPS \0","BPC \0"};

/*********************************************************************************/
// BPS to PC External RS-232 (voltage isolated)
/*********************************************************************************/

void BPS2PC_init();
void BPS2PC_putchar(char data);
unsigned char BPS2PC_getchar(void);

int BPS2PC_gets(char *ptr);
int BPS2PC_puts(char *str);

void BPS2PC_put_int(void);


#endif /*RS232_PORTS_H_*/
