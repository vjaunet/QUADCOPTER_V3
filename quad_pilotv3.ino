/*===================================================================

  Author : Vincent Jaunet

  This sketch is made for a Quadcopter controller on a atmega328
  - read four chanel pwm input
  - read attitude from MPU6050
  - compute the PIDs
  - ouput ESC values on 4 pins

  based on :
  -- rcarduino.blogspot.com
  -- i2cDev, MPU6050_MOTION...
  -- // https://github.com/NicoHood/PinChangeInterrupt


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
  =====================================================================*/

// To printdebug information on screen
//#define _DEBUG
//#define _DEBUG_RC
//#define _DEBUG_YPR
//#define _DEBUG_PID
//#define _DEBUG_ESC

//====================================================================

#define XMODE
#define PID_STAB //PID_RATE

// sensitivity factors
#define K_YAW 260.f
#define K_PITCH_ROLL 120.f

//Failsafe values
#define THR_SAFE 1300
#define PITCH_SAFE 0.0f
#define ROLL_SAFE 0.0f
bool failsafe = false;


//PID Constants
#define yaw_rate_kp 5.0f
#define rate_kp 1.4f
#define rate_ki 0.01f
#define rate_kd 0.02f

#define stab_kp 2.5f
#define stab_ki 0.01f
#define stab_kd 0.5f

//min and max RC values
#define THR_MIN 905
#define RC_MIN 1000.f
#define RC_MAX 2000.f


//====================================================================

// some other constants defined here to
// speed up the computations
#define cs45 0.7071067811865f
const float PR_fact  =K_PITCH_ROLL/(RC_MAX-RC_MIN);
const float Y_fact   =K_YAW/(RC_MAX-RC_MIN);
const float YPR_center=(RC_MAX+RC_MIN)/2.0f;

//====================================================================

#include <PinChangeInterrupt.h>
#include <Servo.h>
#include "pid.h"
#include "dmp.h"

PID yprSTAB[2];
PID yprRATE[3];

DMP imu;

//For ease of programming
#define THR_RC 0
#define YAW_RC 1
#define PITCH_RC 2
#define ROLL_RC 3

// Assign your RC channel input pinss
#define THROTTLE_IN_PIN 5 //4
#define YAW_IN_PIN 6      //7
#define PITCH_IN_PIN 7    //5
#define ROLL_IN_PIN  8    //6

// Assign your ESC output pins
#define FL_MOTOR_OUT_PIN 9
#define FR_MOTOR_OUT_PIN 10
#define BL_MOTOR_OUT_PIN 11
#define BR_MOTOR_OUT_PIN 12

//define Servo variables
#define SERVO_NUM 4
Servo MOTOR[SERVO_NUM];
uint16_t ESC[SERVO_NUM];

//define rc data table
#define CHAN_NUM 4
float rc_data[CHAN_NUM];

/*************************************
 // New delay routine
 // allowing for interrupts
 **************************************/
void delay_millis(uint32_t duration){
  uint32_t last=millis();
  while(millis()-last<duration);
}


/*************************************
 //   Human Machine Interface
 **************************************/
//NOTE : Use of TImers would be more efficient
//and would leave the main loop to continue...
#include "hmi.h"
HMI led;

/*************************************
 // New 1us precision micros function
 **************************************/
#include "micros_1us.h"


/*****************************************
         Receiver PWM Stuff
*****************************************/
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;

volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

volatile uint32_t last_reception;

bool check_user_start(void)
// The sticks must be placed
//  bottom center for 500ms to allow flying
{
  if (unThrottleInShared < 1200 &&
      unYawInShared      < 1300 &&
      unPitchInShared    < 1300 &&
      unRollInShared     > 1300 ) {

    uint32_t t_old = millis();
    while (millis()-t_old < 500 ){
      //wait to be sure that sticks are still in position
    }

    // if same we can start
    if (unThrottleInShared < 1200 &&
        unYawInShared      < 1300 &&
        unPitchInShared    < 1300 &&
        unRollInShared     > 1300 ) {

      return true;
    }
  }

  return false;
}

bool check_receiver(void)
//if no YPR interrupt occured for the last 1s
//return false -> the receiver is OFF
{
  if (micros()-last_reception > 1000000) return false;
  return true;
}

void calcThrottle(){
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH){
    ulThrottleStart = m1us_micros(); }
  else {
    // else it must be a falling edge, so lets get the time and subtract
    // the time of the rising edge
    // this gives us the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (m1us_micros() - ulThrottleStart);
  }
}

void calcYaw(){
  if(digitalRead(YAW_IN_PIN) == HIGH){
    ulYawStart = m1us_micros();
  }  else {
    uint32_t now   = m1us_micros();
    unYawInShared  = (uint16_t)(now - ulYawStart);
    last_reception = now;

  }
}

void calcPitch(){
  if(digitalRead(PITCH_IN_PIN) == HIGH){
    ulPitchStart = m1us_micros();
  }  else {
    unPitchInShared = (uint16_t)(m1us_micros() - ulPitchStart);
  }
}

void calcRoll(){
  if(digitalRead(ROLL_IN_PIN) == HIGH){
    ulRollStart = m1us_micros();
  }  else {
    unRollInShared = (uint16_t)(m1us_micros() - ulRollStart);
  }
}




/*$===================================$
  $           setup function          $
  $===================================$*/
void setup()
{
#if defined(_DEBUG) ||defined(_DEBUG_RC) ||	\
  defined(_DEBUG_PID) ||defined(_DEBUG_YPR)||	\
  defined(_DEBUG_ESC) || defined(_DEBUG_TIMER)
  Serial.begin(57600);
#endif

   /***************************************************************
    Init the micros_1us timer 2 usage
  ***************************************************************/
  m1us_init();

  /***************************************************************
     Set the HMI error indication timing
  ***************************************************************/
  led.set_timing(5,50); //5 Hz, 50% DutyCycle

  /***********************************************
    Tell the user we started
  ***********************************************/
  led.blink(20,20,50);
  led.on();

  /****************************************************************
    PWM measurement settings
  *****************************************************************/

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  attachPinChangeInterrupt(digitalPinToPCINT(THROTTLE_IN_PIN), calcThrottle,CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(YAW_IN_PIN), calcYaw,CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(PITCH_IN_PIN), calcPitch,CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ROLL_IN_PIN), calcRoll,CHANGE);


  /***********************************************
          Set servo values to min
  ***********************************************/
  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  MOTOR[0].attach(FL_MOTOR_OUT_PIN);
  MOTOR[1].attach(FR_MOTOR_OUT_PIN);
  MOTOR[2].attach(BR_MOTOR_OUT_PIN);
  MOTOR[3].attach(BL_MOTOR_OUT_PIN);

  for (int i=0;i<SERVO_NUM;i++)
    MOTOR[i].writeMicroseconds(0);


  /***********************************************
          Set PID constant values
  ***********************************************/
  //manual initialization of PID constants
  for (int i=0;i<DIM-1;i++){
    yprSTAB[i].set_Kpid(stab_kp, stab_ki, stab_kd);
  }

  yprRATE[YAW].set_Kpid(yaw_rate_kp, 0.0, 0.0);
  for (int i=1;i<DIM;i++){
    yprRATE[i].set_Kpid(rate_kp, rate_ki, rate_kd);
  }

  //we should certainly set the windup bound here
  // TO DO

  /********************************************
    MPU-6050 Digital Motion Processing
  *********************************************/
  delay_millis(100); //wait for IMU to boot up
  uint8_t err=imu.set_up();
  if (err != 0) {
    while(1){
      led.blink(err);
      delay_millis(2*err*200);
    }
  }

  /***********************************************
    Check emitter/receiver connected and not in failssafe
  ***********************************************/
  while(!check_receiver()){
    led.blink(5);
    delay_millis(500);
  }

  /***********************************************
    Check the battery voltage
  ***********************************************/
  // TODO
  // batt_volt=AnalogRead(A0);


  /***********************************************
    Do nothing until the user has decided to
  ***********************************************/
  led.set_timing(1,50); //1 Hz, 50% DutyCycle

  while(!check_user_start()){
    // check for DMP fifo overflow
    // by this time the dmp should have reached stability
    imu.getAttitude();
    led.blink(1);
  }


  /***********************************************
    Tell the user we are ready
  ***********************************************/
  led.blink(20,20,50);

#ifdef _DEBUG
  Serial.println("-- QuadCopter  Ready --");
  delay_millis(2000);
#endif

}

void loop()
{

  /*****************************************
        Get the current batt voltage
  *****************************************/
  // TODO : AnalogRead a value every 10s or so

  //static uint16_t Nloop=0;  Nloop++;
  // if (Nloop == 1000 ) {
  //   batt_volt=AnalogRead(A0);
  // }
  // if (batt_voltage<3.7) led.toggle();
  // if (batt_voltage<3.2) failsafe=true;

  /*****************************************
             Update  RC inputs
  *****************************************/
  //Check  not in failsafe
  if ( failsafe ){
    rc_data[THR_RC]   = THR_SAFE;
  }else {
    rc_data[THR_RC]   = (float) unThrottleInShared;
  }

  rc_data[YAW_RC]   = (float) unYawInShared;
  rc_data[PITCH_RC] = (float) unPitchInShared;
  rc_data[ROLL_RC]  = (float) unRollInShared;

// #ifdef _DEBUG_RC
//   Serial.print(unThrottleInShared);
//   Serial.print(" ");
//   Serial.print(unYawInShared);
//   Serial.print(" ");
//   Serial.print(unPitchInShared);
//   Serial.print(" ");
//   Serial.print(unRollInShared);
//   Serial.print(" ");
// #endif
#ifdef _DEBUG_RC
  for (int i=1;i<CHAN_NUM;i++){
    Serial.print(" ");
    char val[6];
    dtostrf(rc_data[i],6,2,val);
    Serial.print(val);
  }
  Serial.println(" ");
#endif


  rc_data[YAW_RC]   = -(rc_data[YAW_RC]  - YPR_center)* Y_fact;
  rc_data[PITCH_RC] = (rc_data[PITCH_RC] - YPR_center)*PR_fact;
  rc_data[ROLL_RC]  = (rc_data[ROLL_RC]  - YPR_center)*PR_fact;



#ifdef XMODE
  //Switch to Xmode instead of +mode
  //orders are given in a ref frame rotated by 45deg.
  float Pitch_RC, Roll_RC;
  Pitch_RC=   rc_data[PITCH_RC]*cs45 +  rc_data[ROLL_RC]*cs45;
  Roll_RC =  -rc_data[PITCH_RC]*cs45 +  rc_data[ROLL_RC]*cs45;
  rc_data[PITCH_RC] = Pitch_RC;
  rc_data[ROLL_RC]  = Roll_RC;
#endif


  /*****************************************
        Get the current QUAD's attitude
  *****************************************/
  uint32_t start=micros();
  while(imu.getAttitude()<0){
    //if it takes too long -> go for failsafe
    if (micros()-start > 1000000UL){
      failsafe=true;
      break;
    }
  };

#ifdef _DEBUG_YPR
  for (int i=0;i<DIM;i++){
    Serial.print(" ");
    char val[6];
    dtostrf(imu.ypr[i],6,2,val);
    Serial.print(val);
  }
  Serial.print(" ");
  //  delay_millis(50);
#endif

  /**************************************
         PID Computations
  ***************************************/

  float PIDout[DIM];

#ifdef _DEBUG
  Serial.print(" ");
  char val[6];
  dtostrf(yprRATE[0].get_deltaT()*1000,6,2,val);
  Serial.print(val);
#endif

  //STABLE PID :
#ifdef PID_STAB

  //do not update the PID while THR is low
  //we want to be able to hold the quad avoiding
  //the motor to spin because of some tilt
  if (rc_data[THR_RC] <=  THR_MIN) {

    //Force Ouput to zero
    for (int i=0;i<DIM;i++)  PIDout[i] = 0.0f;

    //reset the PIDs
    for (int i=0;i<DIM;i++)   yprRATE[i].reset();
    for (int i=0;i<DIM-1;i++) yprSTAB[i].reset();

  } else {  //Compute PID

    // yaw is rate PID only --
    PIDout[YAW] = rc_data[YAW_RC];
    // pitch and roll have to be updated --
    PIDout[PITCH] = yprSTAB[0].
      update_pid_std(rc_data[PITCH_RC],imu.ypr[PITCH]);
    PIDout[ROLL] = yprSTAB[1].
      update_pid_std(rc_data[ROLL_RC],imu.ypr[ROLL]);

    //rate pid on all the channels
    for (int i=0;i<DIM;i++){
      PIDout[i] =
        yprRATE[i].update_pid_std(PIDout[i],
                                  imu.gyro[i]);
    }

    //Compensate loss of Thrust due to angle of drone
    rc_data[THR_RC] = rc_data[THR_RC]
      /cos(imu.ypr[ROLL]/180*M_PI)
      /cos(imu.ypr[PITCH]/180*M_PI);
  }

#endif

  //RATE PID only:
#ifdef PID_RATE
  for (int i=0;i<DIM;i++){
    PIDout[i] =
      yprRATE[i].update_pid_std(rc_data[i+1],
                                imu.gyro[i]);
  }
#endif

#ifdef _DEBUG_PID
  for (int i=0;i<DIM;i++){
    Serial.print(" ");
    char val[6];
    dtostrf(PIDout[i],6,2,val);
    Serial.print(val);
  }
  Serial.println(" ");
#endif


  /**************************************
           write ESC output
  ***************************************/

  ESC[1] = (uint16_t)(rc_data[0] + PIDout[ROLL]  - PIDout[YAW]);
  ESC[3] = (uint16_t)(rc_data[0] - PIDout[ROLL]  - PIDout[YAW]);
  ESC[0] = (uint16_t)(rc_data[0] - PIDout[PITCH] + PIDout[YAW]);
  ESC[2] = (uint16_t)(rc_data[0] + PIDout[PITCH] + PIDout[YAW]);

  for (int i=0;i<SERVO_NUM;i++)
    MOTOR[i].writeMicroseconds(ESC[i]);

#ifdef _DEBUG_ESC
  for (int i=0;i<SERVO_NUM;i++){
    Serial.print(" ");
    Serial.print(ESC[i]);
  }
  Serial.println(" ");
  delay_millis(20);
#endif


}
