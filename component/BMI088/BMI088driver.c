#include <cmsis_os.h>
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "spi.h"

float_32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float_32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

bmi088_real_data_t bmi088_real_data;

uint8_t sendbuf_none[8] = {0x55 ,0x55, 0x55, 0x55, 0x55 ,0x55 ,0x55 ,0x55};
uint8_t DMA_buf[8] = {0};


extern osSemaphoreId BinaryNSLowHandle;
#if defined(BMI088_USE_SPI)
/**                 Multiple read operations (burst-read)
 *  are possible by keeping CSB low and continuing the data transfer(i.e. continuing to toggle SCK).
 *  Only the first register address has to be written. Addresses are automatically incremented after
 *  each read access as long as CSB stays active low.
 *
 */

/**                 accelerometer part(normal part)
 *  In case of read operations of the accelerometer part, the requested data is not sent immediately, but
 *  instead first a dummy byte is sent, and after this dummy byte the actual reqested register content is
 *  transmitted
 *  ----when in read mode for accelerometer part,the accel data is
 *  Bit #0: Read/Write bit
 *  Bit #1-7: Address AD(6:0)
 *  Bit #8-15:
 *      o When in write mode, these are the data SDI, which will be written into the address.
 *      o When in read mode, these bits contain unpredictable values, and the user has to
 *         read Bit #16-23 to get the actual data from the reading address.
 */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }





/**                 gyroscope part(special paet)
 *  For single byte read as well as write operations, 16-bit protocols are used. The SPI interface also
 *  supports multiple-byte read operations (burst-read).
 *  ----The data bits are used as follows:
 *  Bit #0: Read/Write bit. When 0, the data SDI is written into the chip. When 1, the data
 *          SDO from the chip is read.
 *  Bit #1-7: Address AD(6:0).
 *  Bit #8-15: when in write mode, these are the data SDI, which will be written into the
 *             address. When in read mode, these are the data SDO, which are read from the address.
 *
 */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

static void BMI088_accel_read_muli_reg_DMA(uint8_t reg,uint8_t *data,uint8_t len);
static void BMI088_gyro_read_muli_reg_DMA(uint8_t reg,uint8_t *data,uint8_t len);

#elif defined(BMI088_USE_IIC)
#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}





void BMI088_read(float_32 gyro[3], float_32 accel[3], float_32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
   // failed in DMA
      /* BMI088_accel_read_muli_reg_DMA(BMI088_ACCEL_XOUT_L, DMA_buf, 6);
       while(xSemaphoreTake(BinaryNSLowHandle,portMAX_DELAY) != pdTRUE);
           BMI088_ACCEL_NS_H();
       bmi088_raw_temp = (int16_t)((DMA_buf[1]) << 8) | DMA_buf[0];
       accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
       bmi088_raw_temp = (int16_t)((DMA_buf[3]) << 8) | DMA_buf[2];
       accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
       bmi088_raw_temp = (int16_t)((DMA_buf[5]) << 8) | DMA_buf[4];
       accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

       BMI088_gyro_read_muli_reg_DMA(BMI088_GYRO_CHIP_ID, DMA_buf, 8);
       while (xSemaphoreTake(BinaryNSLowHandle,portMAX_DELAY) != pdTRUE);
           BMI088_GYRO_NS_H();
       if(DMA_buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
       {
           bmi088_raw_temp = (int16_t)((DMA_buf[3]) << 8) | DMA_buf[2];
           gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
           bmi088_raw_temp = (int16_t)((DMA_buf[5]) << 8) | DMA_buf[4];
           gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
           bmi088_raw_temp = (int16_t)((DMA_buf[7]) << 8) | DMA_buf[6];
           gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
       }

       BMI088_accel_read_muli_reg_DMA(BMI088_TEMP_M, DMA_buf, 2);


       while (xSemaphoreTake(BinaryNSLowHandle,portMAX_DELAY) != pdTRUE);
           BMI088_ACCEL_NS_H();
       bmi088_raw_temp = (int16_t)((DMA_buf[0] << 3) | (DMA_buf[1] >> 5));
       if (bmi088_raw_temp > 1023)
       {
           bmi088_raw_temp -= 2048;
       }

       *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;*/

}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

static void BMI088_accel_read_muli_reg_DMA(uint8_t reg,uint8_t *data,uint8_t len)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte((reg) | 0x80);

    BMI088_read_write_byte(reg | 0x80);   /* wait for unpredictable values*/
    HAL_SPI_TransmitReceive_DMA(&hspi1,sendbuf_none,data,len);
    /** now DMA begin to work,but we don't know whether the SPI is over or not
     * so we can't low the relevant NS **/

}
static void BMI088_gyro_read_muli_reg_DMA(uint8_t reg,uint8_t *data,uint8_t len)
{
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg | 0x80);

    HAL_SPI_TransmitReceive_DMA(&hspi1,sendbuf_none,data,len);  //from ID res
    /** now DMA begin to work,but we don't know whether the SPI is over or not
     * so we can't low the relevant NS **/
}


#elif defined(BMI088_USE_IIC)

#endif
