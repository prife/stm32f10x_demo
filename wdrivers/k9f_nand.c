/*
 * File      : k9f2g08u0b.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author           Notes
 * 2012-11-24     heyuanjie87      first implementation
 * 2013-03-23     prife            add support for stm32f10x
 */
#include <rtdevice.h>
#include "stm32f10x.h"
#include "k9f_nand.h"
#ifdef RT_USING_DFS_UFFS
#include "uffs/uffs_flash.h"
#endif
#include <string.h>

//#define NAND_DEBUG    rt_kprintf
#define NAND_DEBUG(...)

/* nand bank configure */
#define FSMC_NAND_BANK  FSMC_Bank2_NAND
#define NAND_BANK     ((rt_uint32_t)0x70000000)
#define NAND_LARGE    0 /* k9f1g08 0;  k9f2g08: 1 */

static struct stm32_nand _device;
#if RT_CONFIG_UFFS_ECC_MODE == UFFS_ECC_SOFT
#define ECC_SIZE     0
#elif RT_CONFIG_UFFS_ECC_MODE == UFFS_ECC_HW_AUTO
#define ECC_SIZE     4
#else
#error "please set ECC_SIZE a valid value"
#endif

rt_inline void nand_cmd(rt_uint8_t cmd)
{
    /* write to CMD area */
    *(volatile rt_uint8_t*)(NAND_BANK | CMD_AREA) = cmd;
}

rt_inline void nand_addr(rt_uint8_t addr)
{
    /* write to address area */
    *(volatile rt_uint8_t*)(NAND_BANK | ADDR_AREA) = addr;
}

rt_inline rt_uint8_t nand_read8(void)
{
    /* read 1Byte */
    return (*(volatile rt_uint8_t*)(NAND_BANK | DATA_AREA));
}

rt_inline void nand_write8(rt_uint8_t data)
{
    /* write 1Byte */
    *(volatile rt_uint8_t*)(NAND_BANK | DATA_AREA) = data;
}

rt_inline void nand_waitready(void)
{
    while (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_6) == 0);
}

static rt_uint8_t nand_readstatus(void)
{
    nand_cmd(NAND_CMD_STATUS);
    return (nand_read8());
}

static rt_err_t nand_datacorrect(uint32_t generatedEcc, uint32_t readEcc, uint8_t *data)
{
#define ECC_MASK28    0x0FFFFFFF          /* 28 valid ECC parity bits. */
#define ECC_MASK      0x05555555          /* 14 ECC parity bits.       */

    rt_uint32_t count, bitNum, byteAddr;
    rt_uint32_t mask;
    rt_uint32_t syndrome;
    rt_uint32_t eccP;                            /* 14 even ECC parity bits. */
    rt_uint32_t eccPn;                           /* 14 odd ECC parity bits.  */

    syndrome = (generatedEcc ^ readEcc) & ECC_MASK28;

    if (syndrome == 0)
        return (RT_MTD_EOK);                  /* No errors in data. */

    eccPn = syndrome & ECC_MASK;              /* Get 14 odd parity bits.  */
    eccP  = (syndrome >> 1) & ECC_MASK;       /* Get 14 even parity bits. */

    if ((eccPn ^ eccP) == ECC_MASK)           /* 1-bit correctable error ? */
    {
        bitNum = (eccP & 0x01) |
                 ((eccP >> 1) & 0x02) |
                 ((eccP >> 2) & 0x04);
        NAND_DEBUG("ECC bit %d\n",bitNum);
        byteAddr = ((eccP >> 6) & 0x001) |
                   ((eccP >> 7) & 0x002) |
                   ((eccP >> 8) & 0x004) |
                   ((eccP >> 9) & 0x008) |
                   ((eccP >> 10) & 0x010) |
                   ((eccP >> 11) & 0x020) |
                   ((eccP >> 12) & 0x040) |
                   ((eccP >> 13) & 0x080) |
                   ((eccP >> 14) & 0x100) |
                   ((eccP >> 15) & 0x200) |
                   ((eccP >> 16) & 0x400) ;

        data[ byteAddr ] ^= 1 << bitNum;

        return RT_MTD_EOK;
    }

    /* Count number of one's in the syndrome. */
    count = 0;
    mask  = 0x00800000;
    while (mask)
    {
        if (syndrome & mask)
            count++;
        mask >>= 1;
    }

    if (count == 1)           /* Error in the ECC itself. */
        return RT_MTD_EECC;

    return -RT_MTD_EECC;       /* Unable to correct data. */

#undef ECC_MASK
#undef ECC_MASK24
}

static void gpio_nandflash_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#if defined(STM32_F4XX)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    /* FSMC GPIO config in board.c */

    /* NAND_R/B: PG6 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
#else

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
        RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO , ENABLE);

    /*-- GPIO Configuration ------------------------------------------------------*/
    /* CLE, ALE, D0->D3, NOE, NWE and NCE2  NAND pin configuration  */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15 |
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
        GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* D4->D7 NAND pin configuration  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* NWAIT NAND pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
#endif

    GPIO_Init(GPIOG, &GPIO_InitStructure);
}

static void fsmc_nandflash_init(void)
{
    FSMC_NANDInitTypeDef FSMC_NANDInitStructure;
    FSMC_NAND_PCCARDTimingInitTypeDef p;

#if defined(STM32_F4XX)
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
#else
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
#endif

    p.FSMC_SetupTime     = 0x1;
    p.FSMC_WaitSetupTime = 0x3;
    p.FSMC_HoldSetupTime = 0x2;
    p.FSMC_HiZSetupTime  = 0x1;

    FSMC_NANDInitStructure.FSMC_Bank = FSMC_NAND_BANK;
    FSMC_NANDInitStructure.FSMC_Waitfeature = FSMC_Waitfeature_Disable;
    FSMC_NANDInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
    FSMC_NANDInitStructure.FSMC_ECC = FSMC_ECC_Disable;/* 先不要开启ECC计算 */
    FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_2048Bytes;
    FSMC_NANDInitStructure.FSMC_TCLRSetupTime = 0x00;
    FSMC_NANDInitStructure.FSMC_TARSetupTime = 0x00;
    FSMC_NANDInitStructure.FSMC_CommonSpaceTimingStruct = &p;
    FSMC_NANDInitStructure.FSMC_AttributeSpaceTimingStruct = &p;

    FSMC_NANDInit(&FSMC_NANDInitStructure);

    /* FSMC NAND Bank Cmd Test */
    FSMC_NANDCmd(FSMC_NAND_BANK, ENABLE);
}

static struct nand_id nand_table[] =
{
  {{0xEC, 0xF1, 0x80, 0x15}, "K9F1G08U0A"},
  {{0xEC, 0xF1, 0x00, 0x95}, "K9F1G08U0B"},
  {{0xEC, 0xAA, 0x00, 0x15}, "K9F2G08R0A"},
  {{0xEC, 0xDA, 0x10, 0x95}, "K9F1G08U0A"},
  {{0xAD, 0xF1, 0x80, 0x1D}, "HY27UF081G2A"},

  {{0x0, 0x0, 0x0, 0x0}, "end"},
};

static rt_err_t nandflash_readid(struct rt_mtd_nand_device *mtd)
{
    int i;

    nand_cmd(NAND_CMD_READID);
    nand_addr(0);

    _device.id[0] = nand_read8();
    _device.id[1] = nand_read8();
    _device.id[2] = nand_read8();
    _device.id[3] = nand_read8();
    NAND_DEBUG("ID[%X,%X]\n",_device.id[0], _device.id[1]);
    for (i=0; nand_table->id[0] != 0; i++)
    {
        if (_device.id[0] == nand_table[i].id[0] &&
            _device.id[1] == nand_table[i].id[1])
            return (RT_EOK);
    }
    NAND_DEBUG("NAND device unsupported!\n");

    return (RT_ERROR);
}

static rt_err_t nandflash_readpage(struct rt_mtd_nand_device* device, rt_off_t page,
                                   rt_uint8_t *data, rt_uint32_t data_len,
                                   rt_uint8_t *spare, rt_uint32_t spare_len)
{
    rt_uint32_t index;
    rt_uint32_t gecc, recc;
    rt_uint8_t tmp[8];
    rt_err_t result;

    NAND_DEBUG("nand read[%d,%d,%d]\n",page,data_len,spare_len);
    result = -RT_MTD_EIO;
    rt_mutex_take(&_device.lock, RT_WAITING_FOREVER);

    if (data && data_len)
    {
        nand_cmd(NAND_CMD_READ_1);

        nand_addr(0);
        nand_addr(0);
        nand_addr(page);
        nand_addr(page >> 8);
#if NAND_LARGE
        nand_addr(page >> 16);
#endif
        nand_cmd(NAND_CMD_READ_TRUE);
        nand_waitready();
        FSMC_NANDECCCmd(FSMC_NAND_BANK,ENABLE);
        for (index = 0; index < data_len; index ++)
        {
            data[index] = nand_read8();
        }
        gecc = FSMC_GetECC(FSMC_NAND_BANK);
        FSMC_NANDECCCmd(FSMC_NAND_BANK,DISABLE);

#if RT_CONFIG_UFFS_ECC_MODE == UFFS_ECC_HW_AUTO
        if (data_len == 2048)
        {
            for (index = 0; index < ECC_SIZE; index ++)
                tmp[index] = nand_read8();
            if (spare && spare_len)
            {
                for (index = 0; index < spare_len-ECC_SIZE; index ++)
                    spare[ECC_SIZE + index] = nand_read8();

                spare_len = 0;

				memcpy(spare, tmp , ECC_SIZE);
            }

            recc   = (tmp[3] << 24) | (tmp[2] << 16) | (tmp[1] << 8) | tmp[0];

            NAND_DEBUG("<gecc %X,recc %X>",gecc,recc);
            if (nand_datacorrect(gecc, recc, data) != RT_EOK)
                result = -RT_MTD_EECC;
            else
                result = RT_MTD_EOK;

            goto _exit;
        }
#endif

        result = RT_MTD_EOK;
    }

    if (spare && spare_len)
    {
        nand_cmd(NAND_CMD_READ_1);

        nand_addr(0);
        nand_addr(8);
        nand_addr(page);
        nand_addr(page >> 8);
#if NAND_LARGE
        nand_addr(page >> 16);
#endif

        nand_cmd(NAND_CMD_READ_TRUE);
        nand_waitready();

        for (index = 0; index < spare_len; index ++)
        {
            spare[index] = nand_read8();
        }

        result = RT_MTD_EOK;
    }
_exit:
    rt_mutex_release(&_device.lock);

    return (result);
}

static rt_err_t nandflash_writepage(struct rt_mtd_nand_device* device, rt_off_t page,
                                    const rt_uint8_t *data, rt_uint32_t data_len,
                                    const rt_uint8_t *spare, rt_uint32_t spare_len)
{
    rt_uint32_t index;
    rt_err_t result;
    rt_uint32_t gecc;

    result = -RT_MTD_EIO;
    NAND_DEBUG("nand write[%d,%d]\n",page,data_len);
    rt_mutex_take(&_device.lock, RT_WAITING_FOREVER);

    if (data && data_len)
    {
        nand_cmd(NAND_CMD_PAGEPROGRAM);

        nand_addr(0);
        nand_addr(0);
        nand_addr(page);
        nand_addr(page >> 8);
#if NAND_LARGE
        nand_addr(page >> 16);
#endif

        FSMC_NANDECCCmd(FSMC_NAND_BANK,ENABLE);
        for (index = 0; index < data_len; index ++)
        {
            nand_write8(data[index]);
        }
        gecc = FSMC_GetECC(FSMC_NAND_BANK);
        FSMC_NANDECCCmd(FSMC_NAND_BANK,DISABLE);

#if RT_CONFIG_UFFS_ECC_MODE == UFFS_ECC_HW_AUTO
        NAND_DEBUG("<wecc %X>",gecc);
        if (data_len == 2048)
        {
            nand_write8((uint8_t)gecc);
            nand_write8((uint8_t)(gecc >> 8));
            nand_write8((uint8_t)(gecc >> 16));
            nand_write8((uint8_t)(gecc >> 24));
        }
#endif

        nand_cmd(NAND_CMD_PAGEPROGRAM_TRUE);
        nand_waitready();

        if (nand_readstatus() & 0x01 == 1)
        {
            result = -RT_MTD_EIO;
            goto _exit;
        }
        else
        {
            result = RT_MTD_EOK;
        }
    }

    if (spare && spare_len)
    {
        nand_cmd(NAND_CMD_PAGEPROGRAM);

        nand_addr(ECC_SIZE);
        nand_addr(0x08);
        nand_addr(page);
        nand_addr(page >> 8);
#if NAND_LARGE
        nand_addr(page >> 16);
#endif

        for (index = 0; index < spare_len-ECC_SIZE; index ++)
        {
            nand_write8(spare[ECC_SIZE+index]);
        }

        nand_cmd(NAND_CMD_PAGEPROGRAM_TRUE);
        nand_waitready();

        if (nand_readstatus() & 0x01 == 1)
            result = -RT_MTD_EIO;
        else
            result = RT_MTD_EOK;
    }

_exit:
    rt_mutex_release(&_device.lock);

    return (result);
}

rt_err_t nandflash_eraseblock(struct rt_mtd_nand_device* device, rt_uint32_t block)
{
    rt_uint32_t page;
    rt_err_t result;

    result = RT_EOK;
    page = block * 64;

    rt_mutex_take(&_device.lock, RT_WAITING_FOREVER);

    nand_cmd(NAND_CMD_ERASE0);

    nand_addr(page);
    nand_addr(page >> 8);
#if NAND_LARGE
    nand_addr(page >> 16);
#endif

    nand_cmd(NAND_CMD_ERASE1);

    nand_waitready();

    if (nand_readstatus() & 0x01 == 1)
        result = -RT_MTD_EIO;
    rt_mutex_release(&_device.lock);

    return (result);
}

static rt_err_t nandflash_pagecopy(struct rt_mtd_nand_device *device, rt_off_t src_page, rt_off_t dst_page)
{
    rt_err_t result;

    result = RT_MTD_EOK;
    rt_mutex_take(&_device.lock, RT_WAITING_FOREVER);

    nand_cmd(NAND_CMD_RDCOPYBACK);

    nand_addr(0);
    nand_addr(0);
    nand_addr(src_page);
    nand_addr(src_page >> 8);
#if NAND_LARGE
    nand_addr(src_page >> 16);
#endif

    nand_cmd(NAND_CMD_RDCOPYBACK_TRUE);

    nand_waitready();

    nand_cmd(NAND_CMD_COPYBACKPGM);

    nand_addr(0);
    nand_addr(0);
    nand_addr(dst_page);
    nand_addr(dst_page >> 8);
#if NAND_LARGE
    nand_addr(dst_page >> 16);
#endif

    nand_cmd(NAND_CMD_COPYBACKPGM_TRUE);

    nand_waitready();
    if ((nand_readstatus() & 0x01) == 0x01)
        result = -RT_MTD_EIO;

    rt_mutex_release(&_device.lock);

    return (result);
}

static rt_err_t nandflash_checkblock(struct rt_mtd_nand_device* device, rt_uint32_t block)
{
    return (RT_MTD_EOK);
}

static rt_err_t nandflash_markbad(struct rt_mtd_nand_device* device, rt_uint32_t block)
{
    return (RT_MTD_EOK);
}

static struct rt_mtd_nand_driver_ops ops =
{
    nandflash_readid,
    nandflash_readpage,
    nandflash_writepage,
    nandflash_pagecopy,
    nandflash_eraseblock,
#if defined(RT_USING_DFS_UFFS) && !defined(RT_UFFS_USE_CHECK_MARK_FUNCITON)
	RT_NULL,
	RT_NULL,
#else
    nandflash_checkblock,
    nandflash_markbad
#endif
};

static struct rt_mtd_nand_device _partition[2];

void rt_hw_mtd_nand_init(void)
{
    gpio_nandflash_init();
    fsmc_nandflash_init();

    rt_mutex_init(&_device.lock, "nand", RT_IPC_FLAG_FIFO);

    _partition[0].page_size   = 2048;
    _partition[0].pages_per_block = 64;
    _partition[0].block_total = 256;
    _partition[0].oob_size    = 64;
    _partition[0].oob_free    = 60;
    _partition[0].block_start = 0;
    _partition[0].block_end   = 255;
    _partition[0].ops         = &ops;

    rt_mtd_nand_register_device("nand0", &_partition[0]);

    _partition[1].page_size   = NAND_PAGE_SIZE;
    _partition[1].pages_per_block = 64;
    _partition[1].block_total = 2048 - _partition[0].block_total;
    _partition[1].oob_size    = 64;
    _partition[1].oob_free    = 60;
    _partition[1].block_start = _partition[0].block_end + 1;
    _partition[1].block_end   = 2047;
    _partition[1].ops         = &ops;

    rt_mtd_nand_register_device("nand1", &_partition[1]);
}
#include <math.h>
#include <finsh.h>
static uint8_t TxBuffer[NAND_PAGE_SIZE];
static uint8_t RxBuffer[NAND_PAGE_SIZE];
static rt_uint8_t Spare[64];
void ntest(void)
{
    int i,n;
    rt_uint8_t spare[64];

    nandflash_readid(0);

    /* Erase the Block */
    rt_kprintf("start erase test\n");
    for (i = 0; i < 256; i ++)
    {

        if (nandflash_eraseblock(RT_NULL, i) != RT_MTD_EOK)
        {
            RT_ASSERT(0);
        }
        else
            rt_kprintf(".");
    }
#if 1
    rt_kprintf("start read write test\n");
    /* Clear the buffer */
    for (i = 0; i < 2048; i++)
        TxBuffer[i] = i / 5 -i;
    memset(Spare, 0x33, 64);

    for (n = 0; n < 64 * 256; n++)
    {
        memset(RxBuffer, 0, sizeof(RxBuffer));
        memset(spare, 0x0, 64);

        if (nandflash_writepage(RT_NULL,n,TxBuffer,2048, Spare, 64) != RT_EOK)
        {
            RT_ASSERT(0);
        }
        /* Read back the written data */
        nandflash_readpage(RT_NULL,n,RxBuffer, 2048,spare,64);

        if( memcmp( (char*)TxBuffer, (char*)RxBuffer, NAND_PAGE_SIZE ) != 0 )
        {
            RT_ASSERT(0);
        }
        if( memcmp( (char*)Spare+4, (char*)spare+4, 60 ) != 0 )
        {
            RT_ASSERT(0);
        }

    }
    NAND_DEBUG("Nand Flash is OK \r\n");
#endif
}

void nread(int page)
{
    int index;

    rt_memset(RxBuffer, 0, 2048);
    rt_memset(Spare, 0, 64);

    if (nandflash_readpage(RT_NULL,page,RxBuffer, 2048,Spare,64) != RT_MTD_EOK)
    {
        rt_kprintf("read fail\n");
    }
	rt_kprintf("data:\n");
    for (index = 0; index < 2048; index ++)
    {
        rt_kprintf("%02X,",RxBuffer[index]);
		if ((index+1) % 16 == 0)
			rt_kprintf("\n");
    }
	rt_kprintf("\nspare:\n");
    for (index = 0; index < 64; index ++)
    {
        rt_kprintf("[%02X]", Spare[index]);
		if ((index+1) % 16 == 0)
			rt_kprintf("\n");
    }
    rt_kprintf("\n\n");
}

void nerase(int block)
{
    nandflash_eraseblock(RT_NULL,block);
}

void nwrite(int page)
{
    memset(TxBuffer, 0xAA, 2048);
    memset(Spare, 0x55, 60);
#if 1
    {
        int i;
        for (i = 0; i < 2048; i ++)
            TxBuffer[i] = i/5 - i;
    }
#endif
    nandflash_writepage(RT_NULL,page,TxBuffer,2048, Spare, 60);
}

void ncopy(int s, int d)
{

    if (nandflash_pagecopy(RT_NULL,s,d) != RT_MTD_EOK)
        rt_kprintf("copy fail\n");
}

void nand_eraseall()
{
	int index;
	for (index = 0; index < _partition[0].block_total; index ++)
	{
		nandflash_eraseblock(RT_NULL, index);
	}
}
FINSH_FUNCTION_EXPORT(nand_eraseall, erase all of block in the nand flash);
FINSH_FUNCTION_EXPORT(ncopy, nand copy);
FINSH_FUNCTION_EXPORT(nwrite, nand write);
FINSH_FUNCTION_EXPORT(nerase, nand erase);
FINSH_FUNCTION_EXPORT(nread, nand read);
FINSH_FUNCTION_EXPORT(ntest, nand test);
FINSH_FUNCTION_EXPORT_ALIAS(nandflash_readid, nid, read nand id)
