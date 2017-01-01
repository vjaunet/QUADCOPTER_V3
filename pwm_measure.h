/**********************************************

   author : vincent JAUNET
   mail : vincent.jaunet@hotmail.fr
   date : 27-12-2016

   Description:

  simple interrupt service routines
  for PWM pulse length measurements


  Copyright (c) <2014> <Vincent Jaunet>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
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
***********************************************/
#ifndef _PWM_MEASURE_
#define _PWM_MEASURE_

#if (!defined(THROTTLE_IN_PIN)) || (!defined(YAW_IN_PIN)) ||	\
  (!defined(PITCH_IN_PIN)) || (!defined(ROLL_IN_PIN))
//#error "PWM_measure : Please define PWM input pins"
//define some default values
#define THROTTLE_IN_PIN 6
#define YAW_IN_PIN 7
#define PITCH_IN_PIN 8
#define ROLL_IN_PIN  9
#endif

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
extern uint32_t ulThrottleStart;
extern uint32_t ulYawStart;
extern uint32_t ulPitchStart;
extern uint32_t ulRollStart;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
extern volatile uint16_t unThrottleInShared;
extern volatile uint16_t unYawInShared;
extern volatile uint16_t unPitchInShared;
extern volatile uint16_t unRollInShared;

void calcThrottle(void);
void calcYaw(void);
void calcPitch(void);
void calcRoll(void);

#endif
