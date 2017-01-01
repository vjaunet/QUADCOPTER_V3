#ifndef dmp_
#define dmp_

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

#include <math.h>
#include <stdint.h>
#include "helper_3dmath.h"
#include "MPU6050.h"

class DMP {

 private:

  MPU6050 mpu;

  // MPU control/status vars
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation
                          //(0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  int32_t g[3];              // [x, y, z]            gyro vector
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector

  bool dmpReady;         // set true if DMP init was successful
  float lastval[3];
  float m_ypr_off[3];    //dmp offsets

 public:
  DMP();
  int getAttitude();
  void set_up();
  void initialize();
  bool initialized;

  float ypr[3];
  float gyro[3];
};

#endif
