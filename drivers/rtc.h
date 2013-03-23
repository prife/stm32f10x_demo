/*
 * File      : rtc.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#ifndef __RTC_H__
#define __RTC_H__

void rt_hw_rtc_init(void);
void set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second);
void set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t day);
#endif
