
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "usb_type.h"
#include "rtthread.h"

#define VCP_BUFFER_SIZE		(128)

struct stm32_vcp_buf
{
	uint8_t  buffer[VCP_BUFFER_SIZE];
	uint32_t read_index, save_index, used;
};

struct stm32_vcp_device
{
	/* rx structure */
	struct stm32_vcp_buf tx;
	struct stm32_vcp_buf rx;
	uint8_t	rx_blocked;

};

extern struct rt_device vcp_device;


void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
bool VCP_Linecoding_Config(void);
void USB_To_USART_Send_Data(rt_device_t device,uint8_t* data_buffer, uint8_t Nb_bytes);
rt_uint8_t USART_To_USB_Send_Data(rt_device_t device);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void); 	


#endif  /*__HW_CONFIG_H*/
