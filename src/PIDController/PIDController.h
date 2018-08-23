#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <PID_Bowler.h>
#include <mbed.h>
#include "MA702.h"
#include "RunEvery.h"

// Initial values for the PID gains
#define kp 0.005
#define ki 0
#define kd 0
#define vkp 1

#define vkd 0

#define SENSOR_SUM 4.0


class PIDimp : public PIDBowler
{
 public:

  // when the constructor is called with no parameters, do nothing
  PIDimp(){}

  // constructor taking in the hardware objects
  PIDimp(PwmOut * D_1, PwmOut * D_2, MA702 * myEncoder, AnalogIn * currentSensor);

  // Functions inherited from PIDBowler
  float getPosition();
  void setOutputLocal(float);
  float resetPosition(float);
  void onPidConfigureLocal();
  void MathCalculationPosition(float);
  void MathCalculationVelocity(float);
  PidLimitEvent * checkPIDLimitEvents();
  float getMs();
  void setTorque(float);

  // Class public attributes
  MA702 * encoder;             // list of encoders
  PwmOut * motorD1;                // list of servo motors
  PwmOut * motorD2;
  AnalogIn * currentSense;          // list of load cells


  // Class private attributes
 private:
  float runningValues[(int)SENSOR_SUM];
  float runningTotal;
  int runningTotalIndex;
};

#endif
