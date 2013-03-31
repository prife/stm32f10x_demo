#include "stm32f10x.h"

#define KR_KEY_Reload    ((uint16_t)0xAAAA)
#define KR_KEY_Enable    ((uint16_t)0xCCCC)

/*
 *  config iwdg with 5 second
 */
void iwdg_init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);     /* 写入0x5555,用于允许狗狗寄存器写入功能 */
    IWDG_SetPrescaler(IWDG_Prescaler_256);            /* 内部低速时钟256分频 40K/256=156HZ(6.4ms) */
    /* 看门狗定时器溢出时间 */
    IWDG_SetReload(781);                              /* 喂狗时间 5s/6.4MS=781 .注意不能大于0xfff*/
    IWDG_ReloadCounter();                             /* 喂狗*/
    IWDG_Enable();                                    /* 使能看门狗*/
}

int iwdg_reset_check()
{
    /* System Reset */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) /* 如果系统以为看门狗复位 */
    {
        RCC_ClearFlag();
        return 1;
    }

    return 0;
}

int iwdg_feed()
{
    IWDG->KR = KR_KEY_Reload;
}
