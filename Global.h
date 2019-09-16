#include "mclib.h"

#ifndef GLOBAL_H
#define GLOBAL_H
 
/* Private define ------------------------------------------------------------*/
#define SYSCLK          80000000                      //Частота системного таймера
#define PWM_COUNTER     2048                          // 2048 + 2048 (счёт вверх и вниз) == 19531,25 Гц
#define PWM_FREQUENCY   (0.5 * SYSCLK / PWM_COUNTER)  // div2 4 TRANGLE
#define PWM_RESOLUTION  PWM_COUNTER                   // Разрешение падает с ростом частоты
#define TIMESTEP        (1.0 / PWM_FREQUENCY)         // Шаг дискретизации ИУ, Ротаторов (сек)

#define RxBufferSize	10		// входной пакет RS-485 (USART1)
#define TxBufferSize	10		// выходной пакет RS-485 (USART1)
//-----------------------------------------------------------

/*МЕХАНИЧЕСКИЕ ПАРАМЕТРЫ*/
#define BASE_FREQ  4.0   // максимальная частота оборотов электрического поля (Hz) , механические обороты=(эл.обороты)/(число пар полюсов)
#define Trazg 1          // время разгона до максимально значения заданной скорости  (сек)
#define p 2              // число пар полюсов
#define Nred 10          // количество необходимых оборотов вала редуктора
#define Kred 1           // коэффициент редукции
//-----------------------------------------------------------
#define ADR_2 0x1E       //Адресс получателя 
/*ПАРАМЕТРЫ ВОЛЬТ-ЧАСТОТНОГО ЗАКОНА*/
#define Uo 0.04          //начальная ставка напряжения /*(Ro*Ce*M)/(1.5*Cm)*/
#define A  0.2           //коэффициент зависимости оборотов и напряжения /*Ce*/
/*РЕЖИМ АВАРИЙНОГО ВКЛЮЧЕНИЯ*/
#define Uoavar 0.12      //начальная ставка напряжения в аварийном режиме работы
#define Aavar  0.2       //коэффициент зависимости оборотов и напряжения в аварийном режиме работы
 
#define DELTA_W (512/(Trazg*10000000))
#define DELTA_T (512*(BASE_FREQ/p)/10000000)
#define N_RAZG  (Trazg*0.5*(BASE_FREQ/p))
//-----------------------------------------------------------
/*ФЛАГИ*/

typedef union {
unsigned int all_flags;
struct flags_mark {
	                   unsigned int direction:1; //
	                   unsigned int end_of_check:1;//
	                   unsigned int reverse:1;//
	                   unsigned int state:2;//
	                   unsigned int open_close:3;//
	                   unsigned int emergency_mode:1;//
                   	 unsigned int new_command:1; //флаг новой команды
	                   unsigned int rec:1;//
	                   unsigned int GO:1;//флаг разрешения формирования ШИМ сигнала
                  } bit;
		           } u_flags;

extern u_flags flags;
							 
extern int RESOLUTION; //разрешение для ШИМ в зависимости от напряжения
extern int ADC_Result;  //ПЕРЕМЕННЫЕ ДЛЯ АЦП
extern float SUPPLY_VOLTAGE[];//массив для записи результатов опроса АЦП
extern float U;  //переменная напряжения, доля от максимального [0...1]
extern float w;  //переменная скорости, доля от максимального [0...1]
extern float Nrotor;  // число оборотов ротора двигателя=Nred*Kred (вычисляется в main())
extern float N;       //промежуточная переменная значения числа оборотов
							 
extern uint8_t last_command;
extern uint8_t RxCounter;
extern uint8_t Work_buf[RxBufferSize];
	
// Глобальные уставки Цифровой системы управления для СДПМ (PMSM / BLAC)
extern SETPOINTS_TypeDef SetPnt ;
// Интегратор Угла и другие координаты, связанные с положением ротора
extern ROTOR_TypeDef           Rotor;
// Инверсный преобразователь Парка (dq2ab == dc2ac)
extern PIPARK_TypeDef          dq2ab;
// Инвертирующий преобразователь Кларка (преобразователь числа фаз - 2Ph_2_3Ph)
extern PICLARKE_TypeDef        ab2uvw;
   
void PMSM_CntrlUnit_CreateWires(void);
#endif //GLOBAL_H
