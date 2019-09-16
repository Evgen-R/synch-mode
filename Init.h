
#ifndef INIT_H 
#define INIT_H
void reset_pins(void);
void cpu_init(void); 
void uart_init(void);
void dac_init(void); 	
void port_adc_init(void);
void adc_init(void);  
void init_ports(void); 
void timer_init(void);
void porta_init(void);
void pwm_channel_init(void);
void system_init(void);
#endif //INIT_H
