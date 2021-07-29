//
// Created by szy on 2021/7/28.
//
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_Task.h"



#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "pid.h"


#include "math.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float_32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


void AHRS_init(float_32 quat[4], float_32 accel[3], float_32 mag[3]);
void AHRS_update(float_32 quat[4], float_32 time, float_32 gyro[3], float_32 accel[3], float_32 mag[3]);
void get_angle(float_32 quat[4], float_32 *yaw, float_32 *pitch, float_32 *roll);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


extern bmi088_real_data_t bmi088_real_data;
extern ist8310_real_data_t ist8310_real_data;


static uint8_t first_temperate;





/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_Task(void const * argument)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
    while(ist8310_init())
    {
        osDelay(5);
    }
    while(BMI088_init())
    {
        osDelay(5);
    }
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim3);  //enable the tim3 interrupt

    //get the handle of task
    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    for(;;)
    {
        osDelay(1);

    }
    /* USER CODE END INS_Task */
}


/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
/*static void imu_temp_control(float_32 temp)
{
    uint16_t tempPWM;
    float_32 tempout;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        tempout = imu_temp_pid.PID_Output(temp,45.0f);
        if (tempout < 0.0f)
            tempout = 0.0f;
        tempPWM = (uint16_t)tempout;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                //imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        ist8310_read_mag(ist8310_real_data.mag);
    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
/*
static void imu_cmd_spi_dma(void)
{

  //开启陀螺仪的DMA传输
  if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
      && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
      gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
      gyro_update_flag |= (1 << IMU_SPI_SHFITS);

      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
      SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
      return;
  }
  //开启加速度计的DMA传输
  if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
     && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
      accel_update_flag &= ~(1 << IMU_DR_SHFITS);
      accel_update_flag |= (1 << IMU_SPI_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
      SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
      return;
  }




  if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
     && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
  {
      accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
      accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
      SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
      return;
  }
}


void DMA2_Stream2_IRQHandler(void)
{

  if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
  {
      __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

      //gyro read over
      //陀螺仪读取完毕
      if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
      {
          gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
          gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

          HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

      }

      //accel read over
      //加速度计读取完毕
      if(accel_update_flag & (1 << IMU_SPI_SHFITS))
      {
          accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
          accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

          HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
      }
      //temperature read over
      //温度读取完毕
      if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
      {
          accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
          accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

          HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
      }

      imu_cmd_spi_dma();

      if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
      {
          gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
          gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
          __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
      }
  }
}


*/
