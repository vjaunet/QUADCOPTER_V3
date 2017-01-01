/**********************************************

   author : vincent JAUNET
   mail : vincent.jaunet@hotmail.fr
   date : 10-01-2013

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

#include <Arduino.h>
#include "pwm_measure.h"

//variable defintion :
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;


void calcThrottle(){
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH){
    ulThrottleStart = micros(); }
  else {
    // else it must be a falling edge, so lets get the time and subtract
    // the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
  }
}

void calcYaw(){
  if(digitalRead(YAW_IN_PIN) == HIGH){
      ulYawStart = micros();
    }  else {
      unYawInShared = (uint16_t)(micros() - ulYawStart);
    }
}

void calcPitch(){
  if(digitalRead(PITCH_IN_PIN) == HIGH){
      ulPitchStart = micros();
    }  else {
      unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    }
}

void calcRoll(){
  if(digitalRead(ROLL_IN_PIN) == HIGH){
      ulRollStart = micros();
    }  else {
      unRollInShared = (uint16_t)(micros() - ulRollStart);
    }
}
