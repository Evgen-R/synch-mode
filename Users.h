#include "MDR32Fx.h"
#ifndef USERS_H
#define USERS_H

uint16_t crc16(uint8_t *buffer, int buffer_length);
void check_supply_voltage(void);
void usart2_processing(void);
void usart2_send_packet(void);
void flags_and_button_poll(void);
#endif //USERS_H
