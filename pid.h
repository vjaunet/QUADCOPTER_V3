/*

   PID Controller

   author : vincent JAUNET
   mail : vincent.jaunet@hotmail.fr
   date : 10-01-2013

   Description:
   the PID class is a collection of routine necessary
   to perform a PID control based on setpoints (remote control),
   inputs (measured attitude).
   Can be used for any type of system and features :
   - Derivative on measurement
   - Windsup of integral errors

   Reference :
   -"A Simple PID Controller with Adaptive Parameter in a dsPIC; Case of Study"
     http://www.aedie.org/9CHLIE-paper-send/337_CHAINHO.pdf

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

*/

#ifndef pid
#define pid

#include <Arduino.h>
#include <math.h>

class PID {

private:
  //PID constants
  float m_Kp;
  float m_Ki;
  float m_Kd;

  //PID variables
  float m_err;
  float m_last_err;
  float m_sum_err;
  float m_ddt_err;
  float m_lastInput;
  float m_outmax;
  float m_outmin;
  float m_output;
  uint32_t m_last_loop_time;
  float m_deltaT;

  //methods
  void  windup_reset();
  float get_deltaT();

public:
  PID();
  PID(float,float,float);
  float update_pid_std(float setpt, float input);
  void  updateKpKi(float setpt, float input);
  void  set_Kpid(float, float, float);
  void  set_windup_bounds(float, float);
  void  reset();

  //vars
  float setpoint;
};

#endif
