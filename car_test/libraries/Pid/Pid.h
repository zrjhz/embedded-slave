// Pid.h

#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class _Pid
{
public:
    _Pid();
    ~_Pid();
    float PID_value;
    void Calculate_pid(float inputError);
    void PidData_Clear(void);
    void PidData_Set(float error, float value);
    float constrain_float(float x, float a, float b);

private:
    float Kp = 16.5, Ki = 0.0, Kd = 5.0;
    float error = 0, P = 0, I = 0, D = 0;
    float previous_error = 0;
};

extern _Pid Pid;
#endif
