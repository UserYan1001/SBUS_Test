/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/
#include "adc.h"

// У׼ϵ������
#define ADC_REVISE         1.0f    // ADCУ׼ϵ��
#define BATTERY_VOLTAGE_RATIO 11.0f // ��ص�ѹ��ѹ��

/**
 * @brief  ��ʼ����ص�ѹ����ͨ��(PC0 - ADC2_CH10)
 * @param  ��
 * @retval ��
 */
void BatteryVoltage_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ʹ��ADC2��GPIOCʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    // ����PC0Ϊģ�����루��ص�ѹ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ��λADC2����
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, DISABLE);

    // ADCͨ������
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ����ADC2����
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC2, &ADC_InitStructure);

    // ����ADC2ͨ����PC0��ӦADC_Channel_10��
    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);

    // ʹ��ADC2
    ADC_Cmd(ADC2, ENABLE);
}

/**
 * @brief  ��ȡ��ص�ѹԭʼADֵ
 * @param  ��
 * @retval 12λADת�����
 */
uint16_t BatteryVoltage_GetRawAD(void)
{
    // ����ADC2ת��
    ADC_SoftwareStartConv(ADC2);
    
    // �ȴ�ת�����
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
    
    // ����ת�����
    return ADC_GetConversionValue(ADC2);
}

/**
 * @brief  ��ȡ��ص�ѹֵ���Ŵ�100����������������
 * @param  ��
 * @retval ��ѹֵ * 100 (��λ��V)
 */
uint16_t BatteryVoltage_GetVol_X100(void)
{
    uint16_t adValue = BatteryVoltage_GetRawAD();
    // �����ѹ����ѹ�������ο���ѹ3.3V��12λADC���ֵ4095��
    return (uint16_t)((3.3f * BATTERY_VOLTAGE_RATIO * ADC_REVISE * adValue) / 4095.0f * 100.0f);
}

/**
 * @brief  ��ʼ�������������ͨ��
 *         PA2(ADC1_CH2)��PA3(ADC1_CH3)��PA4(ADC1_CH4)��PC1(ADC1_CH11)
 * @param  ��
 * @retval ��
 */
void MotorCurrent_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ʹ��ADC1��GPIOA��GPIOCʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // ����GPIOA����Ϊģ�����루�������������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ����GPIOC����Ϊģ�����루�������������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ��λADC1����
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    // ADCͨ������
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ����ADC1����
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // ʹ��ADC1
    ADC_Cmd(ADC1, ENABLE);
}

/**
 * @brief  ͨ�ú�������ȡָ��ADC1ͨ����ԭʼADֵ
 * @param  channel: ADCͨ����
 * @retval 12λADת�����
 */
static uint16_t MotorCurrent_GetRawValue(uint8_t channel)
{
    // ����ָ����ADCͨ��
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);
    
    // ����ADCת��
    ADC_SoftwareStartConv(ADC1);
    
    // �ȴ�ת�����
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // ����ת�����
    return ADC_GetConversionValue(ADC1);
}

/**
 * @brief  ��ȡPA2ͨ���������ԭʼADֵ
 * @param  ��
 * @retval 12λADת�����
 */
uint16_t MotorCurrent_PA2_GetRawAD(void)
{
    return MotorCurrent_GetRawValue(ADC_Channel_2);
}

/**
 * @brief  ��ȡPA2ͨ���������ֵ���Ŵ�100����
 * @param  ��
 * @retval ����ֵ * 100 (��λ��A)
 */
uint16_t MotorCurrent_PA2_GetCurrent_X100(void)
{
    uint16_t adValue = MotorCurrent_PA2_GetRawAD();
    // ���ݵ�������������ת��ADֵΪ����ֵ
    return (uint16_t)((3.3f * adValue / 4095.0f) * ADC_REVISE);
}

/**
 * @brief  ��ȡPA3ͨ���������ԭʼADֵ
 * @param  ��
 * @retval 12λADת�����
 */
uint16_t MotorCurrent_PA3_GetRawAD(void)
{
    return MotorCurrent_GetRawValue(ADC_Channel_3);
}

/**
 * @brief  ��ȡPA3ͨ���������ֵ���Ŵ�100����
 * @param  ��
 * @retval ����ֵ * 100 (��λ��A)
 */
uint16_t MotorCurrent_PA3_GetCurrent_X100(void)
{
    uint16_t adValue = MotorCurrent_PA3_GetRawAD();
    return (uint16_t)((3.3f * adValue / 4095.0f) * ADC_REVISE);
}

/**
 * @brief  ��ȡPA4ͨ���������ԭʼADֵ
 * @param  ��
 * @retval 12λADת�����
 */
uint16_t MotorCurrent_PA4_GetRawAD(void)
{
    return MotorCurrent_GetRawValue(ADC_Channel_4);
}

/**
 * @brief  ��ȡPA4ͨ���������ֵ���Ŵ�100����
 * @param  ��
 * @retval ����ֵ * 100 (��λ��A)
 */
uint16_t MotorCurrent_PA4_GetCurrent_X100(void)
{
    uint16_t adValue = MotorCurrent_PA4_GetRawAD();
    return (uint16_t)((3.3f * adValue / 4095.0f) * ADC_REVISE);
}

/**
 * @brief  ��ȡPC1ͨ���������ԭʼADֵ
 * @param  ��
 * @retval 12λADת�����
 */
uint16_t MotorCurrent_PC1_GetRawAD(void)
{
    return MotorCurrent_GetRawValue(ADC_Channel_11);
}

/**
 * @brief  ��ȡPC1ͨ���������ֵ���Ŵ�100����
 * @param  ��
 * @retval ����ֵ * 100 (��λ��A)
 */
uint16_t MotorCurrent_PC1_GetCurrent_X100(void)
{
    uint16_t adValue = MotorCurrent_PC1_GetRawAD();
    return (uint16_t)((3.3f * adValue / 4095.0f) * ADC_REVISE);
}
