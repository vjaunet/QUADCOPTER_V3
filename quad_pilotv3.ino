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

-MOSFET : IRLZ34N

=====================================================================*/

// To printdebug information on screen
#define _DEBUG

//====================================================================

#define XMODE
#define PID_STAB //PID_RATE

// sensitivity factors
#define K_YAW 260
#define K_PITCH_ROLL 120

#define RC_MIN 1000
#define RC_MAX 2000

//PID Constants
#define yaw_rate_kp 5.0f
#define rate_kp 1.4f
#define rate_ki 0.01f
#define rate_kd 0.02f

#define stab_kp 2.5f
#define stab_ki 0.01f
#define stab_kd 0.5f

//====================================================================

// some other constants defined here to
// speed up the computations
#define cs45 0.707106781186548f
const float PR_fact  =K_PITCH_ROLL/(RC_MAX-RC_MIN);
const float Y_fact   =K_YAW/(RC_MAX-RC_MIN);
const float YPR_center=(RC_MAX-RC_MIN)/2.;

//====================================================================

#include <PinChangeInterrupt.h>
#include <Servo.h>
#include <Wire.h>
#include "pid.h"
#include "dmp.h"

PID yprSTAB[2];
PID yprRATE[3];
static uint32_t last_loop_time=0;

DMP imu;

//LED pin for Communicating
#define LED_PIN 2

//For ease of programming
#define THR_RC 0
#define YAW_RC 1
#define PITCH_RC 2
#define ROLL_RC 3

// Assign your RC channel input pinss
#define THROTTLE_IN_PIN 5
#define YAW_IN_PIN 6
#define PITCH_IN_PIN 7
#define ROLL_IN_PIN  8
#include "pwm_measure.h"

//++++ servo defines ++++
//Number of servos
#define SERVO_NUM 4
//define Servo variables
Servo MOTOR[SERVO_NUM];

//++++ RC defines ++++
#define RC_MIN 1000
// Assign your ESC output pins
#define FL_MOTOR_OUT_PIN 9
#define FR_MOTOR_OUT_PIN 10
#define BL_MOTOR_OUT_PIN 11
#define BR_MOTOR_OUT_PIN 12
//define rc data table
#define CHAN_NUM 4
uint16_t rc_data[CHAN_NUM];


/*****************************************
         Check if the user want to fly
 *****************************************/
bool check_user_start(void)
// The sticks must be placed
//  bottom center for 500ms to allow flying
{
  if (unThrottleInShared < 1200 &&
      unYawInShared      < 1200 &&
      unPitchInShared    < 1200 &&
      unRollInShared     > 1200 ) {

    uint32_t t_old = millis();
    while (millis()-t_old < 500 ){
      //wait to be sure that sticks are still in position
    }

    // if same we can start
    if (unThrottleInShared < 1200 &&
	unYawInShared      < 1200 &&
	unPitchInShared    < 1200 &&
	unRollInShared     > 1200 ) {

      //change LED status
      digitalWrite(LED_PIN, HIGH);
      return true;
    }
  }

  return false;
}


// New delay routine
// allowing for interrupts
void delay_millis(uint32_t duration){
  uint32_t last=millis();
  while(millis()-last<duration);
}

void blink_fast(uint8_t times=3)
{
  for (uint8_t i=0;i<times;i++){
  digitalWrite(LED_PIN,0);
  delay_millis(50);
  digitalWrite(LED_PIN,1);
  delay_millis(50);
  }
}


//setup function
void setup()
{

  pinMode(LED_PIN, OUTPUT);
  blink_fast(2);

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
  MOTOR[2].attach(BL_MOTOR_OUT_PIN);
  MOTOR[3].attach(BR_MOTOR_OUT_PIN);

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

  /********************************************
    MPU-6050 Digital Motion Processing
  *********************************************/
  imu.set_up();

  /***********************************************
    Do nothing until the user has decided to
  ***********************************************/

  // while(!check_user_start()){
  //   // check for DMP fifo overflow
  //   // by this time the dmp should have reached stability
  //   imu.getAttitude();
  //   blink_fast(5);
  // }

#ifdef _DEBUG
  Serial.begin(115200);
  Serial.println("-- QuadCopter  Ready --");
  delay_millis(2000);
#endif

  blink_fast(5);

}

void loop()
{

  /*****************************************
        Get the current QUAD's attitude
  *****************************************/
  while(imu.getAttitude()<0);
// #ifdef _DEBUG
//   display_3f("",imu.ypr);
//   delay_millis(500);
// #endif

  /*****************************************
             Update  RC inputs
  *****************************************/
  rc_data[THR_RC]   = unThrottleInShared;
  rc_data[YAW_RC]   = unYawInShared;
  rc_data[PITCH_RC] = unPitchInShared;
  rc_data[ROLL_RC]  = unRollInShared;

#ifdef _DEBUG
  Serial.print("RC: ");
  Serial.print(unThrottleInShared);
  Serial.print(" ");
  Serial.print(unYawInShared);
  Serial.print(" ");
  Serial.print(unPitchInShared);
  Serial.print(" ");
  Serial.println(unRollInShared);
#endif


  rc_data[YAW_RC]   = -(rc_data[YAW_RC]  - YPR_center)* Y_fact;
  rc_data[PITCH_RC] = (rc_data[PITCH_RC] - YPR_center)*PR_fact;
  rc_data[ROLL_RC]  = (rc_data[ROLL_RC]  - YPR_center)*PR_fact;

#ifdef XMODE
  //Switch to Xmode instead of +mode
  //orders are given in a ref frame rotated by 90deg.
   rc_data[PITCH_RC] =   rc_data[PITCH_RC]*cs45 +  rc_data[ROLL_RC]*cs45;
   rc_data[ROLL_RC]  = - rc_data[PITCH_RC]*cs45 +  rc_data[ROLL_RC]*cs45;
#endif

  //Compensate lost of Thrust due to angle of drone
  rc_data[THR_RC] = rc_data[THR_RC]/cos(imu.ypr[ROLL]/180*M_PI)
    /cos(imu.ypr[PITCH]/180*M_PI);


  /**************************************
         PID Computations
  ***************************************/

  float PIDout[DIM];

  //store timing values
  float deltaT = 1e-6*((float) (micros()-last_loop_time));
  last_loop_time=micros();

// #ifdef _DEBUG
//   Serial.print("loop us: ");
//   Serial.println(micros());
// #endif


  //STABLE PID :
#ifdef PID_STAB

  // yaw is rate PID only --
  // pitch and roll have to be updated
  PIDout[YAW] = rc_data[YAW_RC];
  PIDout[PITCH] = yprSTAB[0].
    update_pid_std(rc_data[PITCH_RC],imu.ypr[PITCH],deltaT);
  PIDout[ROLL] = yprSTAB[1].
    update_pid_std(rc_data[ROLL_RC],imu.ypr[ROLL],deltaT);

  //rate pid on all the channels
  for (int i=0;i<DIM;i++){
        PIDout[i] =
  	  yprRATE[i].update_pid_std(PIDout[i],
  				    imu.gyro[i],
  				    deltaT);
  }

#endif

  //RATE PID :
#ifdef PID_RATE
  for (int i=0;i<DIM;i++){
    PIDout[i] =
      yprRATE[i].update_pid_std(rc_data[i+1],
				imu.gyro[i],
				deltaT);
  }
#endif

  /**************************************
           write ESC output
  ***************************************/

  uint16_t ESC[SERVO_NUM];

  if (rc_data[0] < 900) {
    // Ensure that the motor don't spin when THR is low
    // the Qaad is easier to grab this way
    for (int i=0;i<SERVO_NUM;i++){
      ESC[i] = RC_MIN;
    }
  } else {
    ESC[1] = (uint16_t)(rc_data[0] + PIDout[ROLL]  + PIDout[YAW]);
    ESC[3] = (uint16_t)(rc_data[0] - PIDout[ROLL]  + PIDout[YAW]);
    ESC[0] = (uint16_t)(rc_data[0] + PIDout[PITCH] - PIDout[YAW]);
    ESC[2] = (uint16_t)(rc_data[0] - PIDout[PITCH] - PIDout[YAW]);
  }

  for (int i=0;i<SERVO_NUM;i++)
    MOTOR[i].writeMicroseconds(ESC[i]);

}
