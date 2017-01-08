/*================================================================================

  Human Machine Interface class:

  Author : Vincent Jaunet
  date   : 04/01/2016

  Copyright (c) <2016> <Vincent Jaunet>

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

================================================================================*/

#include <Arduino.h>
#include "hmi.h"

//default constructor
HMI::HMI(){

  _freq=1;
  _DC  =50;

  // initialize the output
  DDRD |= (1<<PD3);

}

void HMI::set_timing(uint8_t Freq, uint8_t DC){
  //Freq   : Blinking Frequency in Hertz
  //DC     : BLink Duty Cycle
  //----------------------------------------------------

  if (Freq == _freq && DC== _DC) return;

  // Reccord the input values
  _freq = Freq; _DC=DC;

}

void HMI::on(){
  PORTD |= (1<<PD3);
}

void HMI::off(){
  PORTD &= ~(1<<PD3);
}

void HMI::blink(uint8_t Nblink){
  //Nblink : number of expected blinks
  //----------------------------------------------------

  uint32_t period_ms= (uint32_t) 1000/_freq;

  for (uint8_t iblink=0; iblink<Nblink; iblink++){
    PORTD &= ~(1<<PD3);
    uint32_t now=millis();
    while(millis()-now < period_ms*(100-_DC)/100);

    PORTD |= (1<<PD3);
    now=millis();
    while(millis()-now < period_ms*_DC/100);
  }
}

void HMI::blink(uint8_t Nblink,uint8_t Freq, uint8_t DC){
  //blink until decided
  //----------------------------------------------------
  uint8_t freq_Save=_freq;
  uint8_t DC_save=_DC;

  _freq=Freq;
  _DC=DC;
  blink(Nblink);

  _freq=freq_Save;
  _DC=DC_save;

}
