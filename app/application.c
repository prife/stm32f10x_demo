
/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#endif

extern unsigned int Image$$ER_IROM1$$Base;
extern unsigned int Image$$ER_IROM1$$Limit;
#define RODATA_START_ADDRESS        (&Image$$ER_IROM1$$Base)
#define RODATA_END_ADDRESS          (&Image$$ER_IROM1$$Limit)

void rt_init_thread_entry(void *parameter)
{
    /* Filesystem Initialization */
#ifdef RT_USING_DFS
    {
        /* init the device filesystem */
        dfs_init();

#ifdef RT_USING_DFS_ELMFAT
#ifdef RT_USING_DFS
        /* init sdcard driver */
#if STM32_USE_SDIO
        rt_hw_sdcard_init();
#else
        rt_hw_msd_init();
#endif
#endif
        /* init the elm chan FatFs filesystam*/
        elm_init();

        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("File System initialized!\n");
        }
        else
            rt_kprintf("File System initialzation failed!\n");
#endif
#ifdef RT_USING_MTD_NAND
        rt_hw_mtd_nand_init();
        dfs_uffs_init();

#ifdef RT_USING_DFS_UFFS
        /* mount nand flash partition 0 as root directory */
        if (dfs_mount("nand0", "/", "uffs", 0, 0) == 0)
            rt_kprintf("uffs initialized!\n");
        else
            rt_kprintf("uffs initialzation failed!\n");
#endif

#endif

    }
#endif

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

#ifdef STM32F10X_CL
        rt_hw_stm32_eth_init();
#else
        /* STM32F103 */
#if STM32_ETH_IF == 0
        rt_hw_enc28j60_init();
#elif STM32_ETH_IF == 1
        rt_hw_dm9000_init();
#endif
#endif

        /* re-init device driver */
        rt_device_init_all();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif

#if defined(RT_USING_DFS) && defined(RT_USING_LWIP) && defined(RT_USING_DFS_NFS)
    {
        /* NFSv3 Initialization */
        rt_kprintf("begin init NFSv3 File System ...\n");
        nfs_init();

        if (dfs_mount(RT_NULL, "/", "nfs", 0, RT_NFS_HOST_EXPORT) == 0)
            rt_kprintf("NFSv3 File System initialized!\n");
        else
            rt_kprintf("NFSv3 File System initialzation failed!\n");
    }
#endif

#if defined(RT_USING_RTGUI)
    {
        extern void rtgui_system_server_init(void);
        extern void rt_hw_lcd_init();
        extern void rtgui_touch_hw_init(void);

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* init touch panel */
        //rtgui_touch_hw_init();

        /* re-init device driver */
        rt_device_init_all();

        /* find lcd device */
        lcd = rt_device_find("lcd");

        /* set lcd device as rtgui graphic driver */
        rtgui_graphic_set_device(lcd);

        /* init rtgui system server */
        rtgui_system_server_init();
    }
#endif /* #ifdef RT_USING_RTGUI */
}

int rt_application_init()
{
    rt_thread_t init_thread;

    rt_err_t result;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
