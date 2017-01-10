/*
  ---------------------------
  micros_1us
  author : vincent jaunet
  date : 10-01-2013
  ---------------------------

  Description :
  Replacements for the Arduino micros() and millis function with better than
  4us resolution. Using the Timer 2 so that other tlibrairies using
  micros/millis/delay etc would still work.

  Copyright (c) <2017> <Vincent Jaunet>
  License GNU Lesser GPL

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/

#include "micros_1us.h"

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer2 ticks every 8 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROS_PER_TIMER2_OVERFLOW (clockCyclesToMicroseconds(8 * 256))


// the whole number of milliseconds per timer2 overflow
#define MILLIS_INCVAL (MICROS_PER_TIMER2_OVERFLOW / 1000)

// the fractional number of milliseconds per timer2 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INCVAL ((MICROS_PER_TIMER2_OVERFLOW % 1000) >> 3)
#define FRACT_MAXVAL (1000 >> 3)

volatile unsigned long timer2_overflow_count = 0;
volatile unsigned long timer2_millis = 0;
static   unsigned char timer2_fract = 0;

ISR(TIMER2_OVF_vect)
{
  // copy these to local variables so they can be stored in registers
  // (volatile variables must be read from memory on every access)
  unsigned long m = timer2_millis;
  unsigned char f = timer2_fract;

  m += MILLIS_INCVAL;
  f += FRACT_INCVAL;
  if (f >= FRACT_MAXVAL) {
    f -= FRACT_MAXVAL;
    m += 1;
  }

  timer2_fract = f;
  timer2_millis = m;
  timer2_overflow_count++;
}

unsigned long m1us_millis()
{
  unsigned long m;
  uint8_t oldSREG = SREG;

  // disable interrupts while we read timer2_millis or we might get an
  // inconsistent value (e.g. in the middle of a write to timer2_millis)
  cli();
  m = timer2_millis;
  SREG = oldSREG;

  return m;
}

unsigned long m1us_micros() {
  unsigned long m;
  uint8_t oldSREG = SREG, t;


  cli();
  m = timer2_overflow_count;
#if defined(TCNT2)
  t = TCNT2;
#elif defined(TCNT2L)
  t = TCNT2L;
#else
#error TIMER 2 not defined
#endif

#ifdef TIFR2
  if ((TIFR2 & _BV(TOV2)) && (t < 255))
    m++;
#else
  if ((TIFR & _BV(TOV2)) && (t < 255))
    m++;
#endif

  SREG = oldSREG;

  return ((m << 8) + t)/2;
}


void m1us_init()
{

  // set timer 0 mode
#if defined(TCCR2A)
  TCCR2A = 0;
#else
#error Timer 2 normal mode error
#endif

  // set timer 2 prescale factor to 8
#if defined(TCCR2B) && defined(CS20) && defined(CS21) && defined(CS22)
  cbi(TCCR2B, CS20);
  sbi(TCCR2B, CS21);
  cbi(TCCR2B, CS22);
#else
#error Timer 2 prescale factor 64 not set correctly
#endif

  // enable timer 0 overflow interrupt
#if defined(TIMSK2) && defined(TOIE2)
  sbi(TIMSK2, TOIE2);
#else
#error  Timer 2 overflow interrupt not set correctly
#endif

  // //enalbe interrupt again
  // cli();

}
