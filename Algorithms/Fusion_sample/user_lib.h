#ifndef USER_LIB_H
#define USER_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include "main.h"
typedef __packed struct
{
    float_32 input;        //��������
    float_32 out;          //�������
    float_32 min_value;    //�޷���Сֵ
    float_32 max_value;    //�޷����ֵ
    float_32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    float_32 input;        //��������
    float_32 out;          //�˲����������
    float_32 num[1];       //�˲�����
    float_32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern float_32 invSqrt(float_32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, float_32 frame_period, float_32 max, float_32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, float_32 input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float_32 frame_period, const float_32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float_32 input);
//��������
extern void abs_limit(float_32 *num, float_32 Limit);
//�жϷ���λ
extern float_32 sign(float_32 value);
//��������
extern float_32 float_32_deadline(float_32 Value, float_32 minValue, float_32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern float_32 float_32_constrain(float_32 Value, float_32 minValue, float_32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern float_32 loop_float_32_constrain(float_32 Input, float_32 minValue, float_32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern float_32 theta_format(float_32 Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_float_32_constrain((Ang), -PI, PI)

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */