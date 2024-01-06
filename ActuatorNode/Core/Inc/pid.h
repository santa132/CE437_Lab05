#ifndef INC_PID_H_
#define INC_PID_H_
#include "main.h"

#define MAX_ERR_INTEGRAL 10
#define MIN_ERR_INTEGRAL -10
#define MAX_OUTPUT_PID 20
#define MIN_OUTPUT_PID -20

typedef struct{
	float kp;
	float ki;
	float kd;

	float errorIntergral;
	float prevError;
	float error;

	float currT;
	float prevT;
	float deltaT;

	float outputPID;

	int timeCountPID;
} PIDInfor;

/*
 * @brief apply PID algorithm for DC motor
 * @param refSpeed: reference velocity
 * @param currSpeed: current velocity
 * @return velocity control output
 */
float PID(float refSpeed, float currSpeed);

#endif /* INC_PID_H_ */
