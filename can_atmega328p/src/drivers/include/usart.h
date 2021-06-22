#ifndef _USART_H_
#define _USART_H_

#include <avr/io.h>

extern void usart_init(void);
extern void usart_transmit(uint8_t dataByte);
extern void usart_transmit(const char *dataByte);

#endif