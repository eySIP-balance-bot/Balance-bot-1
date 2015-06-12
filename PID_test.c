#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
//#include "lcd.c"
#include "adxl.h"

#define Setpoint 0



/*working variables*/
double lastTime=0 ,lastTime2=0 ,lastTime3=0;
double Input, Output;
double errSum=0, lastErr=0 ,lastErr2=0 ,lastErr3=0;
double kp, ki, kd;



void Compute()
{
	/*How long since we last calculated*/
	double timeChange = (double)millis();
	
	/*Compute all the working error variables*/
	double error = Setpoint - Input;
	errSum = (error * timeChange)+(lastErr*lastTime)+(lastErr2*lastTime2);
	double dErr = (error - lastErr) / timeChange;
	
	/*Compute PID Output*/
	Output = kp * error + ki * errSum + kd * dErr;
	
	/*Remember some variables for next time*/
	lastErr2=lastErr;
	lastErr = error;
	
	lastTime2=lastTime;
	lastTime = timeChange;
	
	
}

void SetTunings(double Kp, double Ki, double Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

int main(void)
{
	pr_int(2,1,acc_angle(),3);
	SetTunings(1,1,1);
	while(1)
	{
		Input=(double)acc_angle();
		Compute();
		pr_int(1,1,Output,5);
	}	
}