/*
  ---------------------------
  DMP class
  author : vincent jaunet
  date : 10-01-2013
  ---------------------------

  Description :
  The DMP class is mainly a wrapper to the MPU6050
  one from github/PiBits and Jeff Rowberg <jeff@rowberg.net>

  It defines the main functions to :
  -set up the I2C communication through I2Cdev
  -Initialize the measurements and retrieve Offset values
  -Get the attitude of the drone

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

#include "dmp.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

DMP::DMP()
{
  dmpReady=true;
  initialized = false;

  for (int i=0;i<DIM;i++){
    lastval[i]=10;
    m_ypr_off[i]=0.0;
  }

}


//---------------------------
//         mpu setup
//---------------------------
void DMP::set_up() {
  // initialize device
  mpu.initialize();

  // verify connection
  if (!mpu.testConnection()){
    while(1);
  }

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // printf("DMP Initialization failed (code %d)\n", devStatus);
    while(1);
  }
}

void DMP::initialize(){
  //todo
  initialized = true;
}


int DMP::getAttitude()
{
  if (!dmpReady) return -1;

  // wait for FIFO count > 42 bits
  do {
    fifoCount = mpu.getFIFOCount();
  }while (fifoCount<42);

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    return -1;

    // otherwise, check for DMP data ready interrupt
    //(this should happen frequently)
  } else  {
    //read packet from fifo
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    for (int i=0;i<DIM;i++){
      //offset removal
      ypr[i]-=m_ypr_off[i];

      //scaling for output in degrees
      ypr[i]*=180/M_PI;

    }

    //unwrap yaw when it reaches 180
    ypr[0] = wrap_180(ypr[0]);

    //change sign of ROLL, MPU is attached upside down
    ypr[2]*=-1.0;

    mpu.dmpGetGyro(g, fifoBuffer);

    //0=gyroX, 1=gyroY, 2=gyroZ
    //swapped to match Yaw,Pitch,Roll
    //Scaled from deg/s to get tr/s
    for (int i=0;i<DIM;i++){
      gyro[i]   = (float)(g[DIM-i-1])/131.0/360.0;
    }

    return 0;

  }
}
