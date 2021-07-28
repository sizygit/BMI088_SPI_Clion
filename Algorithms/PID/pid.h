/*
 * @brief Integral separation PID algorithm
 */
#ifndef __PID_H
#define __PID_H



class PID
{
    float kp;
    float ki;
    float kd;
    float lastError{};
    float preError{};
    float errorMax;
    float outputMax;
    float outputMin;

    float integralSum = 0;
    float integralLimit;

public:
    PID(float p_kp, float p_ki, float  p_kd, float p_errorMax, float p_oMax, float p_integralLimit);
    virtual float PID_Output(float p_real, float p_exp);
};


#endif