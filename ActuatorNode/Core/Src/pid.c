#include "pid.h"
#include "motor.h"
extern PIDInfor pidInfor;

float PID(float refSpeed, float currSpeed)
{
	if (pidInfor.timeCountPID >= 100)
	{
		pidInfor.error = refSpeed - currSpeed;
		pidInfor.currT = HAL_GetTick();
		pidInfor.deltaT = (pidInfor.currT - pidInfor.prevT) / 1000; // convert millisecond to second
		pidInfor.prevT = pidInfor.currT;

		pidInfor.errorIntergral += pidInfor.error;
		// Limit error Intergral
		if (pidInfor.errorIntergral > MAX_ERR_INTEGRAL)
		{
			pidInfor.errorIntergral = MAX_ERR_INTEGRAL;
		}
		else if (pidInfor.errorIntergral < MIN_ERR_INTEGRAL)
		{
			pidInfor.errorIntergral = MIN_ERR_INTEGRAL;
		}
		// Calculate output PID
		pidInfor.outputPID = pidInfor.kp*pidInfor.error +
							 pidInfor.ki*pidInfor.errorIntergral*pidInfor.deltaT +
							 pidInfor.kd*(pidInfor.error - pidInfor.prevError)/pidInfor.deltaT;
		// Limit output PID
		if (pidInfor.outputPID > MAX_OUTPUT_PID)
		{
			pidInfor.outputPID = MAX_OUTPUT_PID;
		}
		else if (pidInfor.outputPID < MIN_OUTPUT_PID)
		{
			pidInfor.outputPID = MIN_OUTPUT_PID;
		}
		pidInfor.prevError = pidInfor.error;
		pidInfor.timeCountPID = 0;
	}
	return pidInfor.outputPID;
}
