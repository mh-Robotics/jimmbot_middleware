#include "millis.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#define clockCyclesToMicroseconds(a) (((a)*1000L) / (F_CPU / 1000L))
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

ISR(TIMER0_OVF_vect) {
  unsigned long m = timer0_millis;
  unsigned char f = timer0_fract;

  m += MILLIS_INC;
  f += FRACT_INC;
  if (f >= FRACT_MAX) {
    f -= FRACT_MAX;
    m += 1;
  }

  timer0_fract = f;
  timer0_millis = m;
  timer0_overflow_count++;
}

void millis_init(void) {
  TCNT0 = 0;
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  TIMSK0 = (1 << TOIE0);
}

unsigned long millis_get(void) {
  unsigned long m;
  uint8_t oldSREG = SREG;

  cli();
  m = timer0_millis;
  SREG = oldSREG;
  sei();

  return m;
}