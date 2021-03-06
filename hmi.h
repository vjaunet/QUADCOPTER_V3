/*================================================================================

  Human Machine Interface class:

  Use of Timer2 to perform blinking LED
  Uses the OCR2B PWM output (a.k.a PD3 or Arduino PIN 3)

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

#ifndef _HMI_CLASS_
#define  _HMI_CLASS_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

class HMI {

 private :
  uint8_t _freq;
  uint8_t _DC;
  bool auto_blink;

 public:
  HMI();
  void set_timing(uint8_t Freq, uint8_t DC);
  void blink(uint8_t Nblink);
  void blink(uint8_t Nblink,uint8_t Freq, uint8_t DC);
  void blink_auto_on();
  void blink_auto_off();
  void on();
  void toggle();
  void off();


};

#endif
