#include "ad.h"
#include "common.h"

int max1,min1;
int max2,min2;
int sum_ad1,sum_ad2;   //归一化后的值
int ad1;
int ad2;
int ad3;
int ad4;

int average1;
int average2;  //滤波后的值
//int sum1=0;
//int sum0=0;

char i=0;

int value_buf1[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int value_buf2[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int ad1_max;
int ad3_max;
int ad1_min=900;
int ad3_min=900;
float guiyi_ad1 = 0;
float guiyi_ad3 = 0;

//int ad3;
//int ad4;

void Init_AD(uint32_t instance)
{
	/* 初始化ADC模块 ADC0_SE19_BM0 */
    ADC_InitTypeDef ADC_InitStruct1;
    ADC_InitStruct1.instance = instance;
    ADC_InitStruct1.clockDiv = kADC_ClockDiv2; /* ADC采样时钟2分频 */
    ADC_InitStruct1.resolutionMode = kADC_SingleDiff12or13;
    ADC_InitStruct1.triggerMode = kADC_TriggerSoftware; /* 软件触发转换 */
    ADC_InitStruct1.singleOrDiffMode = kADC_Single; /*单端模式 */
    ADC_InitStruct1.continueMode = kADC_ContinueConversionEnable; /* 启动连续转换 转换一次后 自动开始下一次转换*/
    ADC_InitStruct1.hardwareAveMode = kADC_HardwareAverageDisable; /*禁止 硬件平均 功能 */
    ADC_InitStruct1.vref = kADC_VoltageVREF;                       /* 使用外部VERFH VREFL 作为模拟电压参考 */
    ADC_Init(&ADC_InitStruct1);
    
    /* 初始化对应引脚 */
    /* DM0引脚为专门的模拟引脚 ADC时 无需设置复用  DM0也无法当做普通的数字引脚 */
    
    /* 启动一次ADC转换 */
//    ADC_StartConversion(HW_ADC0, 13, kADC_MuxA);     //PB3
//	  ADC_StartConversion(HW_ADC1, 10, kADC_MuxA);     //PB4
//	  ADC_StartConversion(HW_ADC1, 11, kADC_MuxA);     //PB5
//	  ADC_StartConversion(HW_ADC1, 12, kADC_MuxA);     //PB6
}

//启动一次ad转换
int adc_once(uint32_t instance, uint32_t chl)
{
	ADC_StartConversion(instance, chl, kADC_MuxA);
  return ADC_ReadValue(instance, kADC_MuxA);
}

//int max_main(void)
//{
//	max
//}	


#define N 3
void filter(void)       //滤波
{  
	  ad();
}

void ad(void)
{
		ADC_StartConversion(HW_ADC1, 4, kADC_MuxA);
	ad1=ADC_QuickReadValue(ADC1_SE4_PE0);
////	ad1=2000;
	ADC_StartConversion(HW_ADC1, 5, kADC_MuxA);
	ad2=ADC_QuickReadValue(ADC1_SE5_PE1);
	
	ADC_StartConversion(HW_ADC1, 6, kADC_MuxA);
	ad3=ADC_QuickReadValue(ADC1_SE6_PE2);
	
	ADC_StartConversion(HW_ADC1, 7, kADC_MuxA);
	ad4=ADC_QuickReadValue(ADC1_SE7_PE3);
	
//	ad1=adc_once(HW_ADC1, 4);
//	ad2=adc_once(HW_ADC1, 13);
}

void guiyi(void)               //归一化
{
	
  guiyi_ad1 = ((float)ad1/((float)ad1_max - (float)ad1_min))*1000;
	guiyi_ad3 = ((float)ad3/((float)ad3_max - (float)ad3_min))*1000;
}

void max_min(void)
{
	if(ad1_max<=ad1)
	   ad1_max=ad1;
	
	if(ad3_max<=ad3)
	   ad3_max=ad3;
	
	if(ad1_min>=ad1)
	   ad1_min=ad1;
	
	if(ad3_min>=ad3)
	   ad3_min=ad3;
	
	
}