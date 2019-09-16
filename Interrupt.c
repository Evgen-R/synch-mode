
#include "MDR32Fx.h" // Device header 
#include "Interrupt.h"
#include "mclib.h"    
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_rst_clk.h"
#include "Init.h"
#include "Global.h"
#include "Users.h"

/*******************************************************************************
* Function Name : TIM1_UP_TIM1_IRQHandler  TIM_IT_Update
* Description :   Самая главная процедура программы (обработка прерывания)
*     TIM_IT_Update на периоде треугольного ШИМ-а может генерироваться дважды.
*     Перед нулевым и перед максимальным кодами в счетчике таймера.
*     Но поглощающий прерывания счетчик реверсов может их прореживать.
*     Что влияет на шаг дискретизации системы управления (на TIMESTEP).
*******************************************************************************/
/*ОБРАБОТКА ПРЕРЫВАНИЯ ОКОНЧАНИЯ ПЕРИОДАШИМ СИГНАЛА, ФОРМИРОВАНИЕ ШИМ СИГНАЛА*/

 void Timer1_IRQHandler(void) 
{
  if(flags.bit.GO==1) //ПРОВЕРКА ФЛАГА РАЗРЕШЕНИЯ ВРАЩЕНИЯ
		{	 
				 if(N<Nrotor)
		     {
		    	/*РАЗГОН ДО МАКСИМАЛЬНОГО ЗАДАННОГО ЗНАЧЕНИЯ*/
		        if(w<0.999&&flags.bit.state==1)
		          w=w+DELTA_W;  
		        
				    if(w>=0.999&&flags.bit.state==1)
				      flags.bit.state=2;
  //------------------------------------------------------------------------- 
            /* ВРАЩЕНИЕ С МАКСИМАЛЬНО ЗАДАННОЙ СКОРОСТЬЮ*/
				    if(flags.bit.state==2)
				    {
					    if(N>=Nrotor-N_RAZG)
				        flags.bit.state=3;
				    }
	//--------------------------------------------------------------------------
            /*СНИЖЕНИЕ СКОРОСТИ ДО 0*/
				    if(w>0&&flags.bit.state==3)  
		          w=w-DELTA_W;
						
				    if(w<=0&&flags.bit.state==3)
			       {
					     w=0; 
					     N=N+0.1;
			       }

				     N=N+w*DELTA_T; 
		     }
	//---------------------------------------------------------------------------- 
			   if(N>=Nrotor&&flags.bit.state==3) //ПРОВКРКА ЧИСЛА ВЫПОЛНЕННЫХ ОБОРОТОВ
         {
				   switch(flags.bit.direction)
			       { 
				        case 1: flags.bit.open_close=4;
						    break;
					      case 0: flags.bit.open_close=2;  
						    break; 
					   }

				   flags.bit.GO=0;
				   N=0; 			  
					 reset_pins();  
				   flags.bit.emergency_mode=0;
				   flags.bit.state=1;  
         } 
	       /*ВОЛЬТ ЧАСТОТНЫЙ ЗАКОН*/
			  	U=A*w+Uo; 
	//----------------------------------------------------------------------------
	        if (flags.bit.emergency_mode==1)//проверка включения аварийного режима
	          U=Aavar*w+Uoavar;
	       
	        float TEMPv=U;	
					
          if (TEMPv>=1)
		       TEMPv=0.999;
	       
	        if (TEMPv<=-1)	
	         TEMPv=-0.999;
	        
					*dq2ab.q=_IQ(TEMPv);

	        if(flags.bit.direction==1)
           *Rotor.yS=_IQ(w);
	       
					if(flags.bit.direction==0)
           *Rotor.yS=_IQ(-w);
    //------------------------------------------------------------------------------
    //  Наблюдатель углового положения ротора
    // ------------------------------------------------------------------------------
    // Angle = Speed * (100 * PI) * (POLES/2) * TIMESTEP * 1/s, [0.0, 2*PI]
    // где: Speed - в относительных единицах (pu) - [-1.0, +1.0]
    //      Angle - в абсолютных единицах (рад)   - [0.0,  2*PI]
    // ИЛИ
    // Angle = Speed * (100 / 2) * (POLES/2) * TIMESTEP * 1/s =
    //       = Speed *       BASE_FREQ       * TIMESTEP * 1/s, [0.0, 1.0]
    // где: Speed - в относительных единицах (pu) - [-1.0, +1.0]
    //      Angle - в относительных единицах (pu) - [ 0.0,  1.0]
    // ------------------------------------------------------------
    //Rotor.Angle=_IQ(0.9); //+= _IQmpy(*Rotor.yS, _IQ(BASE_FREQ * TIMESTEP)); // 0.8 uS
	  Rotor.Angle+= _IQmpy(*Rotor.yS, _IQ(BASE_FREQ * TIMESTEP));
    if (Rotor.Angle > _IQ(1.0)) Rotor.Angle -= _IQ(1.0);
    if (Rotor.Angle < _IQ(0.0)) Rotor.Angle += _IQ(1.0);
    // Внимание! Перегрузка: _IQtoIQ15(_IQ(1.0)) === 0x8000
    ROTOR_GetTrigonometic(&Rotor);
    // 2.4 uS
    // ------------------------------------------------------------------------------
    //  Преобразуем управляющие сигналы Регуляторов тока в 2-x фазную систему напряжений
    // ------------------------------------------------------------------------------
    dq2ab.a = _IQmpy(*dq2ab.d, *dq2ab.Cosine) - _IQmpy(*dq2ab.q, *dq2ab.Sine);
    dq2ab.b = _IQmpy(*dq2ab.q, *dq2ab.Cosine) + _IQmpy(*dq2ab.d, *dq2ab.Sine);
    // 3.8 uS  
    // ------------------------------------------------------------------------------
    //  Преобразуем 2-x фазную систему напряжений в 3-x фазную (для питания электромотора)
    // ------------------------------------------------------------------------------
    _iq temp_v1 = _IQdiv2(*ab2uvw.a); // 0.8660254037844386 = sqrt(3)/2
    _iq temp_v2 = _IQmpy(*ab2uvw.b, _IQ(0.8660254037844386));
    ab2uvw.u = -(*ab2uvw.a);
    ab2uvw.v = temp_v1 - temp_v2;
    ab2uvw.w = temp_v1 + temp_v2;
    // 4.7 uS
    // ------------------------------------------------------------------------------
    //  Преобразуем 3-x фазную синусоидальную последовательность в ... и будет SVPWM
    // ------------------------------------------------------------------------------
   if (SetPnt.SVPWM_Enable) 
		{
        if (ab2uvw.u < ab2uvw.v) 
			{
            if (ab2uvw.u < ab2uvw.w) 
				{
                ab2uvw.v = ab2uvw.v - (_IQ(1.0) + ab2uvw.u);
                ab2uvw.w = ab2uvw.w - (_IQ(1.0) + ab2uvw.u);
                ab2uvw.u = _IQ(-1.0);
                } 
				else 
				{
                ab2uvw.u = ab2uvw.u - (_IQ(1.0) + ab2uvw.w);
                ab2uvw.v = ab2uvw.v - (_IQ(1.0) + ab2uvw.w);
                ab2uvw.w = _IQ(-1.0);
                }
            }
		else if (ab2uvw.v < ab2uvw.w) 
			{
            ab2uvw.w = ab2uvw.w - (_IQ(1.0) + ab2uvw.v);
            ab2uvw.u = ab2uvw.u - (_IQ(1.0) + ab2uvw.v);
            ab2uvw.v = _IQ(-1.0);
            } 
			else if (ab2uvw.w < ab2uvw.v) 
			{  
            ab2uvw.u = ab2uvw.u - (_IQ(1.0) + ab2uvw.w);
            ab2uvw.v = ab2uvw.v - (_IQ(1.0) + ab2uvw.w);
            ab2uvw.w = _IQ(-1.0);
            } 
				else 
			{
            ab2uvw.u = _IQ(-1.0);
            ab2uvw.v = _IQ(-1.0);
            ab2uvw.w = _IQ(-1.0);
      }
		} 
    // SVM: +1.9 .. 2.4 uS = 6.6 .. 7.0 uS
    // ------------------------------------------------------------------------------
    //  Ограничиваем сигналы (чтоб не было перегрузки в макросе _IQtoIQ15)
    // ------------------------------------------------------------------------------
    if (ab2uvw.u > _IQ(0.99999)) ab2uvw.u = _IQ(0.99999);
    if (ab2uvw.u < _IQ(-1.0000)) ab2uvw.u = _IQ(-1.0000);
    if (ab2uvw.v > _IQ(0.99999)) ab2uvw.v = _IQ(0.99999);
    if (ab2uvw.v < _IQ(-1.0000)) ab2uvw.v = _IQ(-1.0000);
    if (ab2uvw.w > _IQ(0.99999)) ab2uvw.w = _IQ(0.99999);   
    if (ab2uvw.w < _IQ(-1.0000)) ab2uvw.w = _IQ(-1.0000);
    // +1.5 uS | SVM: +1.6 .. 1.82 uS | PWM: +1.68 .. 1.86 uS   
	  //Обновляем Регистры Сравнения 3-х каналов Таймера
		/*!!!!!!НЕ ЗАБЫТЬ ПРОПИСАТЬ RESOLUTION ПРИ ВКЛ. РЕЖИМА ПРОВЕРКИ НАПРЯЖЕНИЯ!!!!!*/
	   MDR_TIMER1->CCR1=2048-_Q15toBASE(_IQtoIQ15(ab2uvw.w), RESOLUTION);
	   MDR_TIMER1->CCR2=2048-_Q15toBASE(_IQtoIQ15(ab2uvw.v), RESOLUTION);  
	   MDR_TIMER1->CCR3=2048-_Q15toBASE(_IQtoIQ15(ab2uvw.u), RESOLUTION);
     }
   MDR_TIMER1->STATUS = ~(1 << 0);
   NVIC_ClearPendingIRQ(Timer1_IRQn);
}

/*ОБРАБОТКА ПРЕРЫВАНИЯ АЦП*/
void ADC_IRQHandler()
{
  if (ADC_GetITStatus(ADC1_IT_END_OF_CONVERSION))
	{
	  ADC_Result=ADC1_GetResult();
	  ADC_Result &=0x00FFF;                  //запись результата
	  PORT_SetBits(MDR_PORTA, PORT_Pin_0);   // ножка использовалась для индикации окончания опроса
	}
	 // MDR_DAC->DAC1_DATA=10*ADC_Result;
    NVIC_ClearPendingIRQ(ADC_IRQn);
} 
/************/
/*ПРЕРЫВАНИЕ ПО ПРИЁМУ UART2*/
void UART2_IRQHandler(void)
{
    if(UART_GetITStatus (MDR_UART2, UART_IT_RX))
     {	
        static uint8_t USART2_bufRX[RxBufferSize];//Буфер приема
  			UART_ClearITPendingBit(MDR_UART2,UART_IT_RX);
		    uint16_t data = MDR_UART2->DR; //запись пришедшего байта
			  last_command=Work_buf[4]; //запись предыдущей команды
				if (RxCounter<9)
	      {
			    USART2_bufRX[RxCounter] = data; //запись данных в буфер
			    RxCounter++;                    //инкрементирование счётчика принятых байт
			  }
				if(RxCounter==9)
			  {					
			     USART2_bufRX[RxCounter] = data; 
			     while (RxCounter>0||RxCounter==0)
				   { 
             Work_buf[RxCounter] = USART2_bufRX[RxCounter];//запись в рабочие буферы
				     RxCounter=RxCounter-1;
				   }
				   RxCounter=0;  //обнуление счётчика буфера приема
				 //  CRC=crc16(Work_buf,6);
					
					 if (Work_buf[0]==0xAA&&Work_buf[1]==0xAA&&Work_buf[8]==0x55&&Work_buf[9]==0x55) //проверка стартовых и стоповых байтов 
				   {
					  if ((Work_buf[6]<<8|Work_buf[7])==crc16(Work_buf,6)); //проверка контрольной суммы
					   {
							 if (ADR_2==Work_buf[3])  //проверка адреса назначения
							   flags.bit.rec = 1; //выставление флага окончания приёма      
					   }
				   }     
		     }	
		  }	
}
/*****************/
	 