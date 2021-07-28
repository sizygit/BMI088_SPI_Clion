#include "bsp_imu_pwm.h"


void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = (pwm);

}
