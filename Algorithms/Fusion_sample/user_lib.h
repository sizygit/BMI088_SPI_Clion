#ifndef USER_LIB_H
#define USER_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include "main.h"
typedef __packed struct
{
    float_32 input;        //输入数据
    float_32 out;          //输出数据
    float_32 min_value;    //限幅最小值
    float_32 max_value;    //限幅最大值
    float_32 frame_period; //时间间隔
} ramp_function_source_t;

typedef __packed struct
{
    float_32 input;        //输入数据
    float_32 out;          //滤波输出的数据
    float_32 num[1];       //滤波参数
    float_32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
//快速开方
extern float_32 invSqrt(float_32 num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float_32 frame_period, float_32 max, float_32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float_32 input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float_32 frame_period, const float_32 num[1]);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float_32 input);
//绝对限制
extern void abs_limit(float_32 *num, float_32 Limit);
//判断符号位
extern float_32 sign(float_32 value);
//浮点死区
extern float_32 float_32_deadline(float_32 Value, float_32 minValue, float_32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern float_32 float_32_constrain(float_32 Value, float_32 minValue, float_32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern float_32 loop_float_32_constrain(float_32 Input, float_32 minValue, float_32 maxValue);
//角度 °限幅 180 ~ -180
extern float_32 theta_format(float_32 Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_32_constrain((Ang), -PI, PI)

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */