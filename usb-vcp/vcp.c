#include <rthw.h>
#include <rtthread.h>

#include "stm32f10x.h"
#include "board.h"
#include "vcp.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#define USB_DISCONNECT                      GPIOG
#define USB_DISCONNECT_PIN                  GPIO_Pin_6
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOG

static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);


struct stm32_vcp_device vcp_data;
struct rt_device vcp_device;

bool VCP_Linecoding_Config(void)
{
	return TRUE;
}

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */   

  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);


  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}

void block_timer(void* para)
{
	static int max_retry;
	if(vcp_data.rx_blocked)
	{
	    vcp_data.rx_blocked++;
		
		if(max_retry < vcp_data.rx_blocked)
			max_retry = vcp_data.rx_blocked;

		if(vcp_data.rx_blocked > 100)
		{
			/* usb can't wait anymore */
	 		SetEPRxValid(ENDP3);
			vcp_data.rx_blocked=0; 
		}
	}
}

/*******************************************************************************
* Function Name  : USB_To_USART_Send_Data.
* Description    : send the received data from USB to the UART 0.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_To_USART_Send_Data(rt_device_t device,uint8_t* data_buffer, uint8_t Nb_bytes)
{
	struct stm32_vcp_device* vcp = (struct stm32_vcp_device*) device->user_data;

	while(Nb_bytes)
	{
		rt_base_t level;
		/* disable interrupt */
		level = rt_hw_interrupt_disable();
		/* save character */
		vcp->rx.buffer[vcp->rx.save_index] = *data_buffer++;
		vcp->rx.save_index++;
		if (vcp->rx.save_index >= VCP_BUFFER_SIZE)
			vcp->rx.save_index = 0;
	   
		/* if the next position is read index, discard this 'read char' */
		if (vcp->rx.save_index == vcp->rx.read_index)
		{
			vcp->rx.read_index ++;
			if (vcp->rx.read_index >= VCP_BUFFER_SIZE)
				vcp->rx.read_index = 0;
		}else{
			vcp->rx.used++;	
		}
	
		/* enable interrupt */
		rt_hw_interrupt_enable(level);

		Nb_bytes--;
	}

	/* only take next trascation when we got enough mem */
	if(VCP_BUFFER_SIZE - vcp->rx.used >= VIRTUAL_COM_PORT_DATA_SIZE)
	{
		/* Enable the receive of data on EP3 */
	 	SetEPRxValid(ENDP3);
	}else{
		vcp->rx_blocked = 1;
	}

	/* invoke callback */
	if (device->rx_indicate != RT_NULL)
	{
		device->rx_indicate(device, vcp->rx.used);
	}
}
/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
rt_uint8_t USART_To_USB_Send_Data(rt_device_t device)
{
	struct stm32_vcp_device* vcp = (struct stm32_vcp_device*) device->user_data;

	if(vcp->tx.used)
	{
		uint8_t USB_Tx_length;
		uint8_t *USB_Tx_ptr;
		rt_base_t level;

	    /* disable interrupt */
		level = rt_hw_interrupt_disable();
		    
		USB_Tx_ptr = vcp->tx.buffer + vcp->tx.read_index;

		if(VCP_BUFFER_SIZE - vcp->tx.read_index >= VIRTUAL_COM_PORT_DATA_SIZE)
        	USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
		else
			USB_Tx_length = VCP_BUFFER_SIZE - vcp->tx.read_index;
		
		if(USB_Tx_length > vcp->tx.used)
			USB_Tx_length = vcp->tx.used;
					
        vcp->tx.read_index += USB_Tx_length;
		if(vcp->tx.read_index == VCP_BUFFER_SIZE)
			vcp->tx.read_index = 0; 
		 vcp->tx.used -= USB_Tx_length;

		/* enable interrupt */
		rt_hw_interrupt_enable(level);

		/* Check the data to be sent through IN pipe */
	    UserToPMABufferCopy((uint8_t*)USB_Tx_ptr, ENDP1_TXADDR, USB_Tx_length);
	    SetEPTxCount(ENDP1, USB_Tx_length);
	    SetEPTxValid(ENDP1);
		return 1;
	}
	return 0; 
}
/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);


  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/* RT-Thread Device Interface */
static rt_err_t rt_vcp_init (rt_device_t dev)
{
	struct stm32_vcp_device* vcp = (struct stm32_vcp_device*) dev->user_data;

	if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
	{
		rt_memset(vcp->rx.buffer, 0,
				sizeof(vcp->rx.buffer));
		vcp->rx.read_index = 0;
		vcp->rx.save_index = 0;  
		vcp->rx.used = 0; 
		vcp->tx.read_index = 0;
		vcp->tx.save_index = 0;  
		vcp->tx.used = 0;

		Set_System();
		Set_USBClock();
    	USB_Interrupts_Config();
   		USB_Init();

		dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
	}

	return RT_EOK;
}

static rt_err_t rt_vcp_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_vcp_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_vcp_read (rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_uint8_t* ptr;
	rt_err_t err_code;
	struct stm32_vcp_device* vcp;

	ptr = buffer;
	err_code = RT_EOK;
	vcp = (struct stm32_vcp_device*)dev->user_data;
   
	while(size)
	{
		rt_base_t level;
	   /* disable interrupt */
		level = rt_hw_interrupt_disable();
	
		if (vcp->rx.read_index != vcp->rx.save_index)
		{
			/* read a character */
			*ptr++ = vcp->rx.buffer[vcp->rx.read_index];
			size--;
		    vcp->rx.used--;
	
			/* move to next position */
			vcp->rx.read_index ++;
			if (vcp->rx.read_index >= VCP_BUFFER_SIZE)
				vcp->rx.read_index = 0;
		}
		else
		{
			/* set error code */
			err_code = -RT_EEMPTY;
			
			/* enable interrupt */
			rt_hw_interrupt_enable(level);

			break;
		}
		/* enable interrupt */
		rt_hw_interrupt_enable(level);
	}

	/* set error code */
	rt_set_errno(err_code);
	return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static void rt_vcp_write_bytes(struct stm32_vcp_device* vcp,rt_uint8_t ch)
{
    rt_base_t level;

	while(vcp->tx.used == VCP_BUFFER_SIZE)
		rt_thread_delay(0);
   
	/* write a character */
	vcp->tx.buffer[vcp->tx.save_index]=ch;
	
	/* disable interrupt */
	level = rt_hw_interrupt_disable();
	vcp->tx.used++;
	
	/* move to next position */
	vcp->tx.save_index ++;
	if (vcp->tx.save_index >= VCP_BUFFER_SIZE)
		vcp->tx.save_index = 0;
		
	/* enable interrupt */
	rt_hw_interrupt_enable(level);
}

static rt_size_t rt_vcp_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_uint8_t* ptr;
	rt_err_t err_code;
	struct stm32_vcp_device* vcp;

	err_code = RT_EOK;
	ptr = (rt_uint8_t*)buffer;
	vcp = (struct stm32_vcp_device*)dev->user_data;

	/* polling mode */
	if (dev->flag & RT_DEVICE_FLAG_STREAM)
	{
		/* stream mode */
		while (size)
		{
			if (*ptr == '\n')
			{
				rt_vcp_write_bytes(vcp,'\r');
			}
			
			rt_vcp_write_bytes(vcp,*ptr);
			++ptr; --size;
		}
	}
	else
	{
		/* write data directly */
		while (size)
		{
			rt_vcp_write_bytes(vcp,*ptr);
			++ptr; --size;
		}
	}

	/* set error code */
	rt_set_errno(err_code);

	return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_err_t rt_vcp_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
////	struct stm32_vcp_device* vcp;
//
//	RT_ASSERT(dev != RT_NULL);
//
//	vcp = (struct stm32_vcp_device*)dev->user_data;
//	switch (cmd)
//	{
//	case RT_DEVICE_CTRL_SUSPEND:
//		/* suspend device */
//		break;
//
//	case RT_DEVICE_CTRL_RESUME:
//		/* resume device */
//		break;
//	}

	return RT_EOK;
}



/*
 * vcp register for STM32
 * support STM32F103VB and STM32F103ZE
 */
rt_err_t rt_hw_vcp_register(rt_device_t device, const char* name, rt_uint32_t flag, struct stm32_vcp_device *vcp)
{
	RT_ASSERT(device != RT_NULL);

	device->type 		= RT_Device_Class_Char;
	device->rx_indicate = RT_NULL;
	device->tx_complete = RT_NULL;
	device->init 		= rt_vcp_init;
	device->open		= rt_vcp_open;
	device->close		= rt_vcp_close;
	device->read 		= rt_vcp_read;
	device->write 		= rt_vcp_write;
	device->control 	= rt_vcp_control;
	device->user_data	= vcp;

	/* register a character device */
	return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}
void rt_hw_vcp_init(void)
{
	rt_hw_vcp_register(&vcp_device, "usbvcp", RT_DEVICE_FLAG_STREAM, &vcp_data);
}

