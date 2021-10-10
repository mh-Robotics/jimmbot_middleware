#include "usart.h"

#include <avr/io.h>

#define USART_BAUDRATE 115200 // Desired Baud Rate
#define BAUD_PRESCALER ((F_CPU / 4 / USART_BAUDRATE - 1) / 4)

// https://www.avrfreaks.net/sites/default/files/forum_attachments/AVR%20Oscillator%20vs%20Serial%20Error.pdf

#define ASYNCHRONOUS (0 << UMSEL00) // USART Mode Selection

#define DISABLED (0 << UPM00)
#define EVEN_PARITY (2 << UPM00)
#define ODD_PARITY (3 << UPM00)
#define PARITY_MODE DISABLED // USART Parity Bit Selection

#define ONE_BIT (0 << USBS0)
#define TWO_BIT (1 << USBS0)
#define STOP_BIT ONE_BIT // USART Stop Bit Selection

#define FIVE_BIT (0 << UCSZ00)
#define SIX_BIT (1 << UCSZ00)
#define SEVEN_BIT (2 << UCSZ00)
#define EIGHT_BIT (3 << UCSZ00)
#define DATA_BIT EIGHT_BIT // USART Data Bit Selection

void usart_init(void) {
  // Set Baud Rate
  UBRR0H = BAUD_PRESCALER >> 8;
  UBRR0L = BAUD_PRESCALER;

  // Set Frame Format
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;

  // Enable Receiver and Transmitter
  UCSR0B = (0 << RXEN0) | (1 << TXEN0);
}

void usart_transmit(uint8_t dataByte) {
  while ((UCSR0A & (1 << UDRE0)) == 0) {
  }; // Do nothing until UDR is ready
  UDR0 = dataByte;
}

void usart_transmit(const char *dataByteArray) {
  int index = 0;
  while (dataByteArray[index] != '\0') {
    usart_transmit(dataByteArray[index++]);
  }
}