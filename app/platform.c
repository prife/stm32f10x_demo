#include <rtthread.h>
#include "board.h"

void rt_platform_init(void)
{
#ifdef RT_USING_DFS
    /* initialize sd card */
#if RT_USING_DFS_ELMFAT
#if STM32_USE_SDIO
     rt_hw_sdcard_init();
#else
     rt_hw_msd_init();
#endif /* STM32_USE_SDIO */
#endif /* RT_USING_DFS_ELMFAT */

#if defined(RT_USING_MTD_NAND)
     rt_hw_mtd_nand_init();
#endif /* RT_USING_MTD_NAND */

#endif /* RT_USING_DFS */
}

