/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_16K.h"

enum{
	TARGET_CH0 = 0 ,
	TARGET_CH1 ,
	TARGET_CH2 ,
	TARGET_CH3 ,	
	
	TARGET_CH4 ,
	TARGET_CH5 ,
	TARGET_CH6 ,
	TARGET_CH7 ,

	TARGET_CH_DEFAULT	
}Channel_TypeDef;

//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

#define PWM_FREQ 								(15000ul)

#define TIMER_LOG_MS							(1000ul)
//#define ADC_SAMPLETIME_MS						(20ul)
#define GPIO_TOGGLE_MS							(500ul)

#define DUTY_MAX								(100ul)
#define DUTY_MIN								(1ul)

uint8_t 	u8TH0_Tmp = 0;
uint8_t 	u8TL0_Tmp = 0;

//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;


typedef enum{
	flag_LED = 0 ,
	
	flag_DEFAULT	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

uint8_t is_flag_set(Flag_Index idx)
{
	return BitFlag_READ(ReadBit(idx));
}

void set_flag(Flag_Index idx , uint8_t en)
{
	if (en)
	{
		BitFlag_ON(ReadBit(idx));
	}
	else
	{
		BitFlag_OFF(ReadBit(idx));
	}
}

void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}

void PWMx_CH2_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM2H = HIBYTE(res);
    PWM2L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWMx_CH1_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM1H = HIBYTE(res);
    PWM1L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWMx_CH0_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM0H = HIBYTE(res);
    PWM0L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWMx_Init(uint16_t uFrequency)
{
	uint32_t res = 0;

	P10_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM2_P10_OUTPUT_ENABLE;	

	P11_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM1_P11_OUTPUT_ENABLE;

	P12_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM0_P12_OUTPUT_ENABLE;
	
    PWM_IMDEPENDENT_MODE;
    PWM_CLOCK_DIV_16;

/*
	PWM frequency   = Fpwm/((PWMPH,PWMPL)+1) = (24MHz/2)/(PWMPH,PWMPL)+1) = 20KHz
*/	

	res = (24000000>>4);			// 2 ^ 4 = 16
	res = res/uFrequency;
	res = res - 1;	

    PWMPH = HIBYTE(res);
    PWMPL = LOBYTE(res);
}


void Timer0_IRQHandler(void)
{
//	static uint16_t LOG_TIMER = 0;
	static uint16_t CNT_TIMER = 0;
	static uint16_t CNT_GPIO = 0;

	if (CNT_GPIO++ >= GPIO_TOGGLE_MS)
	{		
		CNT_GPIO = 0;

	}	

	if (CNT_TIMER++ >= TIMER_LOG_MS)
	{		
		CNT_TIMER = 0;
//    	printf("LOG:%d\r\n",LOG_TIMER++);
//		send_UARTString("LOG_TIMER\r\n");
	}

}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void TIMER0_Init(void)
{
	uint16_t res = 0;

	ENABLE_TIMER0_MODE1;
	
	u8TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}


void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void UART0_Init(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

    UART0_Init();

	PWMx_Init(PWM_FREQ);								
			
	TIMER0_Init();	

	PWMx_CH0_SetDuty(20);
	PWMx_CH1_SetDuty(60);	
	PWMx_CH2_SetDuty(75);
	
    while(1)
    {

    }
}



