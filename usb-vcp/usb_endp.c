/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "vcp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
uint8_t USB_Txing;
extern struct stm32_vcp_device vcp_data;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
	if(USB_Txing)
	{
	      USB_Txing = USART_To_USB_Send_Data(&vcp_device);
	}
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
  
  /* USB data will be send to rx buffer waiting to be processed */
  
  USB_To_USART_Send_Data(&vcp_device, USB_Rx_Buffer, USB_Rx_Cnt);
  
}

/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
  	extern void block_timer(void* para);
	block_timer(NULL);
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      if(vcp_data.tx.used)
	  {
	      USB_Txing = USART_To_USB_Send_Data(&vcp_device);
	  }
    }
  }  
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

