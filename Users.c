
#include "Users.h"
#include "MDR32Fx.h" 
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_rst_clk.h"
#include "Init.h" 
#include "Global.h"
#include "math.h"

/*****************/
 /*ПОДПРОГРАММА ПРОВЕРКИ НАПРЯЖЕНИЯ */
void check_supply_voltage(void)
{
  PORT_InitTypeDef PORTAInit;                   /*объявляем структуру инициализации порта A*/
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);   /*включаем тактирование порта A;*/
	
	PORTAInit.PORT_Pin=PORT_Pin_1|PORT_Pin_4; 
	PORTAInit.PORT_OE=PORT_OE_OUT;	
	PORTAInit.PORT_MODE=PORT_MODE_DIGITAL;
	PORTAInit.PORT_PULL_UP=PORT_PULL_UP_OFF;          
	PORTAInit.PORT_PULL_DOWN=PORT_PULL_DOWN_OFF;       
	PORTAInit.PORT_PD=PORT_PD_DRIVER ; 
	PORTAInit.PORT_FUNC=PORT_FUNC_PORT;               
	PORTAInit.PORT_SPEED=PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTA,&PORTAInit);
	
  PORT_ResetBits(MDR_PORTA, PORT_Pin_1);   //0 на выход верхнего ключа
  PORT_ResetBits(MDR_PORTA, PORT_Pin_4);   //0 на выход нижнего  ключа

  int clock=0;//-счётчик для реализации длительностей открытия и закрытия ключей 
  int poll=0; //-счётчик импульсов для измерения тока
 
  while (poll<400)
  {
	  PORT_SetBits(MDR_PORTA, PORT_Pin_1);//открытие верхнего
	  PORT_SetBits(MDR_PORTA, PORT_Pin_4);//и нижнего ключа разных полумостов
		
	  while (clock<150)
	  {
	   clock++;
	   if (clock==122)
	    ADC1_Start(); //опрос АЦП
	  }
     clock=0;
	 
	  PORT_ResetBits(MDR_PORTA, PORT_Pin_1);/*закрытие*/
	  PORT_ResetBits(MDR_PORTA, PORT_Pin_4);/*ключей*/
	 
	  while (clock<350)
	  {
	    clock++; //формирование паузы перед следующим импульсом
	  }
	  clock=0;
	  PORT_ResetBits(MDR_PORTA, PORT_Pin_0);
	  SUPPLY_VOLTAGE[poll]=ADC_Result;//запись результвтов опроса в массив
	  poll++;
  }
   poll=0;
	
   float MEDIUM_SUPPLY_VOLTAGE;//значение среднего арифметического из массива
   while(poll<400)
   {
     MEDIUM_SUPPLY_VOLTAGE=MEDIUM_SUPPLY_VOLTAGE+SUPPLY_VOLTAGE[poll];//накопление результатов опроса
     poll++;
   }
	 
	 poll=0;
   MEDIUM_SUPPLY_VOLTAGE=(MEDIUM_SUPPLY_VOLTAGE/400);//вычисление среднего результата
   MEDIUM_SUPPLY_VOLTAGE=0.000000003*powf(MEDIUM_SUPPLY_VOLTAGE,3.5518);/* функция перевода значений АЦП в значения напряжения !!!!!!!!!!!!!(ДЛЯ ДРУГОГО ЖЕЛЕЗА НЕОБХОДИМО РЕАЛИЗОВАТЬ СВОЮ ФУНКЦИЮ ) */
   RESOLUTION=(1/MEDIUM_SUPPLY_VOLTAGE)*61440;	//формирование значения разрешения ШИМ, исходя из напряжения питания
	
   if(RESOLUTION>2048)
      RESOLUTION=2048; //ограничение разрешения ШИМ  
}

/*****************/
				
/*****************/
/*ОБРАБОТКА ПРИНЯТЫХ ДАННЫХ*/		
void usart2_processing(void)
{
	if (last_command==Work_buf[4]) //проверка условия новизны принятой команды
      flags.bit.new_command=0; //не новая 
	else 
	    flags.bit.new_command=1; //новая
	
	if(Work_buf[4]==0xAA)	//команда об открытии
	 {
		 if (flags.bit.open_close!=3&&flags.bit.new_command==1||flags.bit.open_close==4) //провекра  условия  "а не открыт ли уже"  и условия новой команды
	    {
		   check_supply_voltage();//ПОДПРОГРАММА ПРОВЕРКИ НАПРЯЖЕНИЯ 
		   flags.bit.end_of_check=1;//инициализируем каналы ШИМ				
	     flags.bit.GO=1;          //разрешение формирования ШИМ сигнала
		   flags.bit.direction=1;   //направление открытия
	     N=0;           //обнуляем число оборотов
	     flags.bit.state=1; //выставляем флаг на разгон
      }
   }
	  
   if(Work_buf[4]==0xBB) //команда остановки	 
   {  
     MDR_TIMER1->CCR1=0; 
	   MDR_TIMER1->CCR2=0;
	   MDR_TIMER1->CCR3=0;	
	   w=0;
     U=0;	
	   flags.bit.GO=0;    //запрет вращения 
	   N=0;               //обнуление кол-ва сделанных оборотов
	   flags.bit.state=1; //выставляем флаг на разгон
	   flags.bit.emergency_mode=0;
	   reset_pins();    
	 }
	 
	 if(Work_buf[4]==0xFF)  //команда о закрытии	
	 {
		 if (flags.bit.open_close!=1&&flags.bit.new_command==1||flags.bit.open_close==2) //провекра  условия  "а не закрыт ли уже"  и условия новой команды
		 {
		   check_supply_voltage();   //ПОДПРОГРАММА ПРОВЕРКИ НАПРЯЖЕНИЯ 
       flags.bit.end_of_check=1; //инициализируем каналы ШИМ 
		   flags.bit.GO=1;           //разрешение формирования ШИМ сигнала
		   flags.bit.direction=0;    //направление закрытия
	     N=0;                      //обнуляем число оборотов 
	     flags.bit.state=1;        //выставляем флаг на разгон
		 }
	 }

	 if(Work_buf[5]==0xEE)        //включение аварийного режима работы	
		flags.bit.emergency_mode=1; //выставление флага включения "аварийного режима"/    
}

/*****************/
/*****************/
/*ФОРМИРОВАНИЕ И ОТПРАВКА ДАННЫХ*/
void usart2_send_packet(void)
{   
	/*!!!!!ПРИ ИСПОЛЬЗОВАНИИ ИНТЕРФЕЙСА RS-485 ПРОПИСАТЬ включение/выключение ПИНА на ПРИЕМ/ПЕРЕДАЧУ!!!!!!*/
	static int TxCounter = 0;
	static uint8_t USART2_bufTX[TxBufferSize];
	while (TxCounter<TxBufferSize)
	{
	  USART2_bufTX[TxCounter]=Work_buf[TxCounter];//запись в буферы передач данных из рабочих буферов
	  if (TxCounter==5)                           //условие передачи 
	   {
	     USART2_bufTX[TxCounter]=USART2_bufTX[TxCounter-1];
			 
		   if (flags.bit.open_close==3)        //проверка условия "открыт"
				 USART2_bufTX[TxCounter]=0xdd;     //отправка байта "открыт"
		   
		   if (flags.bit.open_close==4)
				USART2_bufTX[TxCounter]=0x66;      // не дошёл до концевика
		   
		   if (flags.bit.open_close==2)
				USART2_bufTX[TxCounter]=0x66;     // не дошёл до концевика
		   
		   if (flags.bit.open_close==1)       // провекра условия "закрыт"
		    USART2_bufTX[TxCounter]=0xcc;     // отправка байта "закрыт"
		    
		   if (Work_buf[4]==0xBB)             // проверка команды "стоп"
		    USART2_bufTX[TxCounter]=0xBB;     // отправка байта "стоп"
		     
			 if(Work_buf[5]==0xEE&&flags.bit.open_close==0) //проверка команды "аварийный режим"
				USART2_bufTX[TxCounter]=0xEE;                 //отправка байта "аварийный режим" 
	   }
	   TxCounter++;
	 }
	TxCounter = 0;
	while (TxCounter<TxBufferSize)
	{
    MDR_UART2->DR=USART2_bufTX[TxCounter]; 
	  while (UART_GetFlagStatus(MDR_UART2,UART_FLAG_BUSY));//ожидание окончания передачи для передачи следующего
	  TxCounter++;
	}	
}
/*****************/
void flags_and_button_poll(void)
{
   if(flags.bit.end_of_check==1) /*проверка флага окончания опроса АЦП*/
		 {
	     timer_init();	  
	     porta_init();   
	     pwm_channel_init();	
	     MDR_TIMER1->CNTRL = (1<<0); 
	     NVIC_EnableIRQ(Timer1_IRQn); 
	     flags.bit.end_of_check=0;	
		 }
		 
		 if(flags.bit.rec==1)     //обработка флага окончания приёма по UART 
		 { 
		   usart2_processing();//ОБРАБОТКА ПРИНЯТЫХ ДАННЫХ
		   usart2_send_packet();//ФОРМИРОВАНИЕ И ОТПРАВКА ДАННЫХ
		   flags.bit.rec = 0;	 
		 }

		 /*ОПРОС КОНЦЕВИКОВ В ЦИКЛЕ*/
		 if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_13)==0) //зажат ли концевик
		 {
			 if ( flags.bit.direction==1) //проверка условия движения в направлении именно этого концевика 
    	 {	 
				 flags.bit.open_close=3;    //выставляем флаг об открытии
			   flags.bit.GO=0;            //запрещаем вращение
			   N=0;                       //сбрасываем количество сделанных оборотов
				 reset_pins();              //сброс пинов для проверки напряжения
			   flags.bit.state=1;         //выставляем флаг разгона для следуещего включения
				 flags.bit.emergency_mode=0;//убираем флаг аварийного режима
			 }				 
		 }
		   
		 if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)==0)  //зажат ли концевик
		 {
			 if(flags.bit.direction==0)    //проверка условия движения в направлении именно этого концевика 
			  {
				  flags.bit.open_close=1;    //выставляем флаг о закрытии
			    flags.bit.GO=0;            //запрещаем вращение
			    N=0;                       //сбрасываем количество сделанных оборотов
				  reset_pins();              //сброс пинов для проверки напряжения
			    flags.bit.state=1;         //выставляем флаг разгона для следуещего включения
				  flags.bit.emergency_mode=0;//убираем флаг аварийного режима
			  }				
		 }
		 
		  if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)==1&&PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_13)==1&&N!=0) /*отжаты ли концевики, идёт ли вращение*/
			  flags.bit.open_close=0; //выставляем флаг об отсутствии нажатия концевиков
		     /****/

	//------------------------------------------------------------------------------------------	 
		 
 if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)==0) //кнопка реверса направления вращения
   {
		int kk=0;
		if(flags.bit.reverse==1)
	   {	
			 NVIC_DisableIRQ(Timer1_IRQn);
	     while(kk<200000)
	      {
	        kk=kk+1;
	        MDR_TIMER1->CCR1=0;
	        MDR_TIMER1->CCR2=0;
	        MDR_TIMER1->CCR3=0; 
		    }
        flags.bit.direction=0;
	      NVIC_EnableIRQ(Timer1_IRQn);
		    kk=0;
	   }
		 
  	 if(flags.bit.reverse==0)
	   {					
	     NVIC_DisableIRQ(Timer1_IRQn);
		   while(kk<200000)
		   {
         kk=kk+1;
	       MDR_TIMER1->CCR1=0;
	       MDR_TIMER1->CCR2=0;
	       MDR_TIMER1->CCR3=0;
		   }
	    flags.bit.direction=1;
	    NVIC_EnableIRQ(Timer1_IRQn);
		  kk=0;
	   }

	   switch(flags.bit.reverse)
	   {
		  case 0: flags.bit.reverse=1; break;
	    case 1: flags.bit.reverse=0; break;
	   }
	 }
}

/*ТАБЛИЦЫ И ФУКЦИЯ ДЛЯ ВЫЧИСЛЕНИЯ КОНТРОЛЬНОЙ СУММЫ ПО MODBUS*/

const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
 
const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t crc16(uint8_t *buffer, int buffer_length)
{
  uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
  uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
  unsigned int i;       /* will index into CRC lookup */
    /* pass through message buffer */
  while (buffer_length--) 
	{
    i = crc_hi ^ *buffer++; /* calculate the CRC  */
    crc_hi = crc_lo ^ table_crc_hi[i];
    crc_lo = table_crc_lo[i];
  }
   return (crc_hi << 8 | crc_lo);
}
