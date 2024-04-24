/*
  Adeept Balance 2WD library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
  Author: TOM
*/


#ifndef ADEEPT_BALANCE2WD_H_
#define ADEEPT_BALANCE2WD_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class Adeept_Balance2WD
{
public:
  double speedPiOut(double kps,double kis,double kds,int f,int b,double p0);
  float turnSpin(int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,double kpturn,double kdturn,float Gyroz);
  void pwma(double speedoutput,float rotationoutput,float angle,float angle6,int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,
						int f,int b,float accelz,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB);
	int pulseright = 0;
	int pulseleft = 0;
	int posture=0;
	int stopl = 0;
	int stopr = 0;
	double angleoutput=0,pwm1 = 0, pwm2 = 0;
private:
	float speeds_filterold;//Speed filtering
	float positions;       
	int turnmax = 0;       //Rotate the output amplitude
	int turnmin = 0;       //Rotate the output amplitude
	float turnout = 0;
	int flag1 = 0;
	int flag2 = 0;
	int flag3 = 0;
	int flag4 = 0;
};
#endif
//
// END OF FILE
//