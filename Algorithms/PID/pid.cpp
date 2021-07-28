#include "pid.h"

float PID::PID_Output(float p_real, float p_exp) {
    float error = p_exp - p_real;
    float output;

    integralSum += error;
    //Limit
    if(integralSum > integralLimit) integralSum = integralLimit;
    if(integralSum < -integralLimit) integralSum = -integralLimit;

    //PID
    output = error * kp +
             (error - lastError) * kd +
             integralSum * ki;

    //Limit
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
    //Flash error
    lastError = error;
    return output;
}

PID::PID(float p_kp, float p_ki, float  p_kd, float p_errorMax, float p_oMax, float p_integralLimit) {
    kp = p_kp;
    ki = p_ki;
    kd = p_kd;
    errorMax = p_errorMax;
    outputMax = p_oMax;
    outputMin = -p_oMax;
    integralLimit = p_integralLimit;
}
