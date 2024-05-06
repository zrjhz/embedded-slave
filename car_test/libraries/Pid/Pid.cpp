#include "Pid.h"


_Pid Pid;

_Pid::_Pid()
{
}

_Pid::~_Pid()
{
}

void _Pid::PidData_Clear(void)
{
    I = 0;
    D = 0;
    PID_value = 0;
    previous_error = 0;
}

void _Pid::PidData_Set(float error, float value)
{
    I = 0;
    D = 0;
    PID_value = value;
    previous_error = error;
}

void _Pid::Calculate_pid(float inputError)
{
    error = inputError;
    P = error;
    I = I + error;
    D = error - previous_error;

    I = constrain_float(I, -200, 200); // 积分限幅

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    PID_value = constrain_float(PID_value, -50, 50);

    previous_error = error;
}

float _Pid::constrain_float(float x, float a, float b)
{
	if ((x >= a) && (x <= b))
	{
		return x;
	}
	else
	{
		return (x < a) ? a : b;
	}
}