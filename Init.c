
#include "INIT.h" 
#include "MDR32Fx.h" 
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_rst_clk.h"

#include "Global.h"

void reset_pins(void)
 {
	NVIC_DisableIRQ(Timer1_IRQn); 
		 	  
	PORT_InitTypeDef PORTAInit;                   
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);   
	
	PORTAInit.PORT_Pin=PORT_Pin_1|PORT_Pin_2|PORT_Pin_3|PORT_Pin_4|PORT_Pin_5|PORT_Pin_8; 
	PORTAInit.PORT_OE=PORT_OE_OUT;	
	PORTAInit.PORT_MODE=PORT_MODE_DIGITAL;
	PORTAInit.PORT_PULL_UP=PORT_PULL_UP_OFF;          
	PORTAInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;       
	PORTAInit.PORT_PD=PORT_PD_DRIVER ; 
	PORTAInit.PORT_FUNC=PORT_FUNC_PORT;               
	PORTAInit.PORT_SPEED=PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTA,&PORTAInit);
	
  PORT_ResetBits(MDR_PORTA, PORT_Pin_1);  
  PORT_ResetBits(MDR_PORTA, PORT_Pin_2);   
  PORT_ResetBits(MDR_PORTA, PORT_Pin_3);   
  PORT_ResetBits(MDR_PORTA, PORT_Pin_4);   
	PORT_ResetBits(MDR_PORTA, PORT_Pin_5);   
  PORT_ResetBits(MDR_PORTA, PORT_Pin_8);   
 } 
   
/*НАСТРОЙКА ТАЙМЕР СЧЁТЧИКА ДЛЯ ШИМ*/
void timer_init(void)
{
  MDR_TIMER1->CNTRL=0;
  MDR_RST_CLK->PER_CLOCK |= (1 << 14); //разрешение тактирования Таймера 1, порта B
  MDR_RST_CLK->TIM_CLOCK |= (1 << 24); //разешение тактирования Таймера1 (EN)
  
	
  MDR_TIMER1->CNT =0; 
  MDR_TIMER1->PSG = 0x0;              //делитель частоты при счете основного счетчика PSG=0
  MDR_TIMER1->ARR = 0x800;            //основание счёта основоного счетчика 20кГц - период ШИМ
  MDR_TIMER1->IE |= (1 << 0);         //разрешено прерывание таймера по совпадению CNT и нуля
  MDR_TIMER1->CNTRL |=(1<< 6)|(1<< 7);//счётчик вверх/вниз
}
/*****************/

/*****************/
/*ИНИЦИАЛИЗАЦИЯ ТАКТИРОВАНИЯ CPU*/
void cpu_init (void)
{
	MDR_RST_CLK->HS_CONTROL = 1;					                     // вкл. генератора HSE (кварц 8 МГц)
	while(!( MDR_RST_CLK->CLOCK_STATUS & (1 << 2)));	         // выход HSE на режим
	MDR_RST_CLK->CPU_CLOCK	=	2;			                         //переключение CPU_C1 на HSE == 8МГц (см. errarta при такт от HSE)
	MDR_RST_CLK->PLL_CONTROL=	0x00000900 | (1 << 2) | (1 << 3);
	MDR_RST_CLK->PLL_CONTROL=	0x00000900 | (1 << 2);            
	while(!( MDR_RST_CLK->CLOCK_STATUS & (1 << 1)));
	MDR_RST_CLK->CPU_CLOCK	=	(1 << 8)|(1 << 2)|3;    //на выходе 80 МГц
}
/*****************/

/*НАСТРОЙКА ВЫЫОДОВ ПОРТА А ДЛЯ ШИМ*/
void porta_init(void)
{
	MDR_RST_CLK->PER_CLOCK |= (1 << 21);
		/*ПОРТ НА ШИМ*/ /*HIGH*/
	MDR_PORTA->OE |= (1 << 1);     
	MDR_PORTA->ANALOG |= (1 << 1); 
	MDR_PORTA->FUNC |= (2 << 2*1);  
  MDR_PORTA->PD &= ~(1 << 1);    
	MDR_PORTA->PWR |= (3 << 2*1);  
		/***********************/
	
		 /*ПОРТ НА ШИМ*/ /*LOW*/
	MDR_PORTA->OE |= (1 << 2); 
	MDR_PORTA->ANALOG |= (1 << 2);
	MDR_PORTA->FUNC |= (2 << 2*2); 
  MDR_PORTA->PD &=~(1 << 2); 
	MDR_PORTA->PWR |= (3 << 2*2); 	
     /***********************/
	
	    /*ПОРТ НА ШИМ*/  /*HIGH*/
	MDR_PORTA->OE |= (1 << 3); 
	MDR_PORTA->ANALOG |= (1 << 3);  
	MDR_PORTA->FUNC |= (2 << 2*3); 
  MDR_PORTA->PD &= ~(1 << 3); 
	MDR_PORTA->PWR |= (3 << 2*3);
     /***********************/
	 
		/*ПОРТ НА ШИМ*/  /*LOW*/
	MDR_PORTA->OE |= (1 << 4); 
	MDR_PORTA->ANALOG |= (1 << 4); 
	MDR_PORTA->FUNC |= (2 << 2*4);
  MDR_PORTA->PD &= ~(1 << 4); 
	MDR_PORTA->PWR |= (3 << 2*4); 
     /***********************/
		 
		/*ПОРТ НА ШИМ*/  /*HIGH*/
	MDR_PORTA->OE |= (1 << 5); 
	MDR_PORTA->ANALOG |= (1 << 5); 
	MDR_PORTA->FUNC |= (2 << 2*5); 
  MDR_PORTA->PD &= ~(1 << 5); 
	MDR_PORTA->PWR |= (3 << 2*5); 
		/***********************/
		
		/*ПОРТ НА ШИМ*/  /*LOW*/
	MDR_PORTA->OE |= (1 << 8); 
	MDR_PORTA->ANALOG |= (1 << 8);  
	MDR_PORTA->FUNC |= (2 << 2*8); 
  MDR_PORTA->PD &= ~(1 << 8); 
	MDR_PORTA->PWR |= (3 << 2*8); 		
		/***********************/
}
/*****************/

/*НАСТРОЙКА КАНАЛОВ ПОРТ А ДЛЯ ШИМ*/
void pwm_channel_init(void)
{
  MDR_TIMER1->CH1_CNTRL |= ((1<<11)|(1<<10)|(1<<0));
	MDR_TIMER1->CH1_CNTRL1 |=((1<<11)|(1<<10)|(1<<8)|(1<<3)|(1<<2)|(1<<0));        
	MDR_TIMER1->CH1_DTG |=(3<<0)|(30<<8);
	
	MDR_TIMER1->CH2_CNTRL |= ((1<<11)|(1<<10)|(1<<0));
	MDR_TIMER1->CH2_CNTRL1 |= ((1<<11)|(1<<10)|(1<<8)|(1<<3)|(1<<2)|(0<<1)|(1<<0));
	MDR_TIMER1->CH2_DTG |= (3<<0)|(30<<8);
	
	MDR_TIMER1->CH3_CNTRL |= ((1<<11)|(1<<10)|(1<<0));
  MDR_TIMER1->CH3_CNTRL1 |= ((1<<11)|(1<<10)|(1<<8)|(1<<3)|(1<<2)|(0<<1)|(1<<0));
	MDR_TIMER1->CH3_DTG |= (3<<0)|(30<<8);
}
/*****************/

/*ИНИЦИАЛИЗАЦИЯ UART*/
void uart_init (void)
{
	PORT_InitTypeDef PORTFInit;                   
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF,ENABLE);   
	/*RxD*/
	PORTFInit.PORT_Pin=PORT_Pin_0; 
	PORTFInit.PORT_OE=PORT_OE_IN;	
	PORTFInit.PORT_MODE=PORT_MODE_DIGITAL;
	PORTFInit.PORT_PULL_UP=PORT_PULL_UP_OFF;          
	PORTFInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;       
	PORTFInit.PORT_PD=PORT_PD_DRIVER ; 
	PORTFInit.PORT_FUNC=PORT_FUNC_OVERRID;               
	PORTFInit.PORT_SPEED=PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTF,&PORTFInit); 
	
	/*TxD*/
	PORTFInit.PORT_Pin=PORT_Pin_1; 
	PORTFInit.PORT_OE=PORT_OE_OUT;	
	PORTFInit.PORT_MODE=PORT_MODE_DIGITAL;
	PORTFInit.PORT_PULL_UP=PORT_PULL_UP_OFF;          
	PORTFInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;       
	PORTFInit.PORT_PD=PORT_PD_DRIVER ; 
	PORTFInit.PORT_FUNC=PORT_FUNC_OVERRID;               
	PORTFInit.PORT_SPEED=PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTF,&PORTFInit);
	
	MDR_RST_CLK->PER_CLOCK |= (1 << 7); 
  MDR_RST_CLK->UART_CLOCK = (0|(1 << 25));  
	
  /*Параметры делителя при частоте = 80000000Гц и скорости  = 115200*/
  MDR_UART2->IBRD = 0x2b; 
  MDR_UART2->FBRD = 0x1a; 
  
  MDR_UART2->LCR_H = ((0 << 1)|(3 << 5)|(0 << 7)); 

  MDR_UART2->CR = ((1 << 8)|(1 << 9)|1); 
 
  NVIC_EnableIRQ(UART2_IRQn);

  UART_ITConfig(MDR_UART2,UART_IT_RX,ENABLE);

  UART_Cmd(MDR_UART2 ,ENABLE);
}
/*****************/

/*ИНИЦИАЛИЗАЦИЯ ЦАП*/
void dac_init(void)
{
  MDR_RST_CLK->PER_CLOCK |= (1 << 18)|(1 << 25);
	MDR_PORTE->OE |= (1 << 0);     
	MDR_PORTE->ANALOG &=~(1 << 0); 
  MDR_PORTE->PD &=~(1 << 0);     
	MDR_PORTE->PWR |= (3 << 2*0);  

	MDR_PORTE->OE |= (1 << 9); 
	MDR_PORTE->ANALOG &=~(1 << 9); 
  MDR_PORTE->PD &=~(1 << 9); 
	MDR_PORTE->PWR |= (3 << 2*9);	

	MDR_DAC->CFG |=(1 << 2)|(1 << 3);
}
/*****************/

/*ИНИЦИАЛИЗАЦИЯ ПОРТОВ АЦП*/
void port_adc_init (void)
{
	PORT_InitTypeDef PORTDInit;                   
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD,ENABLE);   
	
	PORTDInit.PORT_Pin=PORT_Pin_7;                
	PORTDInit.PORT_OE=PORT_OE_IN;                 
	PORTDInit.PORT_PULL_UP=PORT_PULL_UP_OFF;      
	PORTDInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;  
	PORTDInit.PORT_FUNC=PORT_FUNC_PORT;           
  PORTDInit.PORT_MODE=PORT_MODE_ANALOG;          
	
	PORT_Init(MDR_PORTD,&PORTDInit);              
}
/*****************/
/*ИНИЦИАЛИЗАЦИЯ АЦП*/
void adc_init(void)	
{   
	ADCx_InitTypeDef ADC1;
	
	ADC_InitTypeDef ADC;   
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_RST_CLK|RST_CLK_PCLK_ADC,ENABLE);
	
	ADC_StructInit(&ADC);  
	ADC.ADC_StartDelay=1;  
	ADC_Init(&ADC);        
	
	ADCx_StructInit(&ADC1);                   
	ADC1.ADC_ChannelNumber=ADC_CH_ADC7;       
	ADC1.ADC_ClockSource=ADC_CLOCK_SOURCE_CPU;
	ADC1.ADC_Prescaler=ADC_CLK_div_8;         
	ADC1_Init(&ADC1);                         
	
	NVIC_EnableIRQ(ADC_IRQn);                 
	NVIC_SetPriority(ADC_IRQn,2);             
	ADC1_ITConfig(ADCx_IT_END_OF_CONVERSION,ENABLE);
	ADC1_Cmd(ENABLE);                         
	ADC1_Start();                           
}
/*****************/
/*ИНИЦИАЛИЗАЦИЯ ПОРТОВ КЛЮЧЕЙ (перед инициализацией на эти каналы ШИМ)*/
void init_ports(void)
{
  PORT_InitTypeDef PORTAInit;                   
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);   
	
	PORTAInit.PORT_Pin=PORT_Pin_0|PORT_Pin_2|PORT_Pin_3|PORT_Pin_5|PORT_Pin_8; 
	PORTAInit.PORT_OE=PORT_OE_OUT;	
	PORTAInit.PORT_MODE=PORT_MODE_DIGITAL;
	PORTAInit.PORT_PULL_UP=PORT_PULL_UP_OFF;          
	PORTAInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;       
	PORTAInit.PORT_PD=PORT_PD_DRIVER ; 
	PORTAInit.PORT_FUNC=PORT_FUNC_PORT;               
	PORTAInit.PORT_SPEED=PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTA,&PORTAInit); 
	
	PORT_ResetBits(MDR_PORTA, PORT_Pin_0);
	PORT_ResetBits(MDR_PORTA, PORT_Pin_2);
	PORT_ResetBits(MDR_PORTA, PORT_Pin_3);
	PORT_ResetBits(MDR_PORTA, PORT_Pin_5);
	PORT_ResetBits(MDR_PORTA, PORT_Pin_8);
	/*************************************/
	
 
  MDR_RST_CLK->PER_CLOCK |= (1 << 23);
	
/*кнопки на отладочной плате МИЛАНДР*/
		/*LEFT*/
	MDR_PORTC->OE &=~(1 << 13); 
	MDR_PORTC->ANALOG |= (1 << 13);  
	MDR_PORTC->FUNC &=~(1 << 2*13); 
	MDR_PORTC->PULL |=  (1 << 13); 
  MDR_PORTC->PD &=~(1 << 13); 
	MDR_PORTC->PWR |= (3 << 2*13);
	    /*RIGHT*/
	MDR_PORTC->OE &= ~(1 << 14); 
	MDR_PORTC->ANALOG |= (1 << 14);  
	MDR_PORTC->FUNC &=~(1 << 2*14); 
	MDR_PORTC->PULL |=  (1 << 14); 
  MDR_PORTC->PD &= ~(1 << 14); 
	MDR_PORTC->PWR |= (3 << 2*14);
	    /*SELECT*/
	MDR_PORTC->OE &=~(1 << 10); 
	MDR_PORTC->ANALOG |= (1 << 10);  
	MDR_PORTC->FUNC &=~(1 << 2*10); 
	MDR_PORTC->PULL |=  (1 << 10); 
  MDR_PORTC->PD &=~(1 << 10); 
	MDR_PORTC->PWR |= (3 << 2*10);	
	    /*DOWN*/
	MDR_PORTC->OE &= ~(1 << 12); 
	MDR_PORTC->ANALOG |= (1 << 12);  
	MDR_PORTC->FUNC &=~(1 << 2*12); 
	MDR_PORTC->PULL |=  (1 << 12); 
  MDR_PORTC->PD &= ~(1 << 12); 
	MDR_PORTC->PWR |= (3 << 2*12);	
	     /*UP*/
	MDR_PORTC->OE &=~(1 << 11); 
	MDR_PORTC->ANALOG |= (1 << 11);  
	MDR_PORTC->FUNC &=~(1<< 2*11); 
	MDR_PORTC->PULL |=  (1 << 11); 
  MDR_PORTC->PD &= ~(1 << 11); 
	MDR_PORTC->PWR |= (3 << 2*11);
}

void system_init(void)
{
 void reset_pins(void);
 void cpu_init(void); 
 void uart_init(void);
 void dac_init(void); 
 void port_adc_init(void);	
 void init_ports(void); 
 void timer_init(void);
 void porta_init(void);
 void pwm_channel_init(void);
 Nrotor=Nred*Kred; //вычисление кол-ва необходимых оборотов двигателя 
}