/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : mass_mal.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Medium Access Layer interface
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "sdcard.h"
#include "msd.h"
#include "nand_if.h"
#include "mass_mal.h"

#define SD_USING_SDIO  1
#define SD_USING_MSD   2

#define SD_INTERFACE SD_USING_MSD

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];
__IO uint32_t Status = 0;

extern SD_CardInfo SDCardInfo;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
  uint16_t status = MAL_OK;

  switch (lun)
  {
    case 0:
#if SD_INTERFACE == SD_USING_SDIO
      if (SD_Init() !=  SD_OK)
#else
      if (MSD_Init() !=  MSD_RESPONSE_NO_ERROR)
#endif
      {
          Status = MAL_FAIL;
          rt_kprintf("sd init failed\n");
      }
      else
          rt_kprintf("sd init ok\n");

      break;
    case 1:
      break;
    default:
      return MAL_FAIL;
  }
  return status;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:
#if SD_INTERFACE == SD_USING_SDIO
      Status = SD_WriteBlock(Memory_Offset, Writebuff, Transfer_Length);
      if ( Status != SD_OK )
      {
#else
      Status = MSD_WriteBlock((void *)Writebuff, Memory_Offset, Transfer_Length);
      if ( Status != MSD_RESPONSE_NO_ERROR )
      {
#endif
        return MAL_FAIL;
      }      
      break;
    case 1:
      break;
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:
#if SD_INTERFACE == SD_USING_SDIO
      Status = SD_ReadBlock(Memory_Offset, Readbuff, Transfer_Length);
      if ( Status != SD_OK )
#else
      Status = MSD_ReadBlock((void*)Readbuff, Memory_Offset, Transfer_Length);
      if ( Status != MSD_RESPONSE_NO_ERROR )
#endif
      {
        return MAL_FAIL;
      }
      break;
    case 1:
      ;
      break;
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
  SD_CSD SD_csd;
  uint32_t DeviceSizeMul = 0,NumberOfBlocks = 0;;

  if (lun == 0)
  {
#if SD_INTERFACE == SD_USING_SDIO
    if (SD_Init() == SD_OK)
    {
      DeviceSizeMul = (SDCardInfo.SD_csd.DeviceSizeMul + 2);

      if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
      {
        Mass_Block_Count[0] = (SDCardInfo.SD_csd.DeviceSize + 1) * 1024;
      }
      else
      {
        NumberOfBlocks  = ((1 << (SDCardInfo.SD_csd.RdBlockLen)) / 512);
        Mass_Block_Count[0] = ((SDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
      }
      Mass_Block_Size[0]  = 512;

      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      }

      if ( Status != SD_OK )
      {
        return MAL_FAIL;
      } 
     
      Mass_Memory_Size[0] = Mass_Block_Count[0] * Mass_Block_Size[0];

//      STM_EVAL_LEDOn(LED2);
      return MAL_OK;

    }
#else
//    if (MSD_Init() == MSD_RESPONSE_NO_ERROR)
    {
      extern unsigned int rt_hw_msd_get_size(void);
      Mass_Memory_Size[0] = rt_hw_msd_get_size();
      Mass_Block_Size[0]  = 512;
      Mass_Block_Count[0] = Mass_Memory_Size[0] / Mass_Block_Size[0];
      return MAL_OK;
    }
#endif
  }
//  STM_EVAL_LEDOn(LED2);
  return MAL_FAIL;
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
