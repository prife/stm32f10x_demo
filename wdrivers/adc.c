/* AD_BAT : PF7 ADC3_IN5
 * AD_E   : PF6 ADC3_IN4
 **/

/**
  ******************************************************************************
  * @file    ADC/3ADCs_DMA/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_3ADCs_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static ADC_InitTypeDef ADC_InitStructure;
static DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t adc_value[2];

/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void GPIO_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int adc_init(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* DMA2 channel5 configuration ----------------------------------------------*/
  DMA_DeInit(DMA2_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC3_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel5, &DMA_InitStructure);  
  /* Enable DMA2 channel5 */
  DMA_Cmd(DMA2_Channel5, ENABLE);

  /* ADC3 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC3, &ADC_InitStructure);
  /* ADC3 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_28Cycles5);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 2, ADC_SampleTime_28Cycles5);
/* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

  /* Enable ADC3 reset calibration register */   
  ADC_ResetCalibration(ADC3);
  /* Check the end of ADC3 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC3));

  /* Start ADC3 calibration */
  ADC_StartCalibration(ADC3);
  /* Check the end of ADC3 calibration */
  while(ADC_GetCalibrationStatus(ADC3));

  /* Start ADC3 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
    
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 and DMA2 clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

  /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3 | RCC_APB2Periph_GPIOF, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure PC.02, PC.03 and PC.04 (ADC Channel12, ADC Channel13 and 
     ADC Channel14) as analog inputs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/* rtthread interface */
#include <rtthread.h>

#define VREF 3.3  //参考电压

/**
 * adc: 原始ADC数值
 * div: 分频系数
 **/
static rt_uint32_t adc_to_voltage(rt_uint32_t adc, float div)
{
	rt_uint32_t value;
	value = (rt_uint32_t)(adc * 1.0 / 4096 * VREF * 1000 / div); //结果放大了1000倍，即1250表示1.250
	return value;
}

void voltage_get(rt_uint32_t * bat_in, rt_uint32_t * bat_ext)
{
    if (bat_in != RT_NULL)
        *bat_in = adc_to_voltage(adc_value[0], 0.5);

    if (bat_ext != RT_NULL)
        *bat_ext = adc_to_voltage(adc_value[1], 0.12);
}

#ifdef RT_USING_FINSH

rt_uint32_t list_voltage(void)
{
    rt_uint32_t bat_in;
    rt_uint32_t bat_ext;
    voltage_get(&bat_in, &bat_ext);
    rt_kprintf("battery voltage: internal=%dmv, external=%dmv\n", bat_in, bat_ext);
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(list_voltage, adc)
#endif
