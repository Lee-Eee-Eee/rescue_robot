#ifndef _CFF_UTILS_H_
#define _CFF_UTILS_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifndef millis
#define millis() (rt_tick_get_millisecond())
#endif

#ifndef delay
#define delay(ms) rt_thread_mdelay(ms)
#endif

#ifndef printf
#define printf rt_kprintf
#endif

#endif // _CFF_UTILS_H_