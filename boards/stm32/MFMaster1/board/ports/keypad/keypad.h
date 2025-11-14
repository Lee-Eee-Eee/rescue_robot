#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rtdef.h"

typedef enum
{
    KEY_NONE = 0,
    KEY1_PRESS_DOWN,
    KEY2_PRESS_DOWN,
    KEY3_PRESS_DOWN,
    KEY4_PRESS_DOWN,
    KEY5_PRESS_DOWN,
    // KEY_UP,
    // KEY_DOWN,
    // KEY_LEFT,
    // KEY_RIGHT,
    // KEY_ENTER,
    // KEY_NEXT,
    // KEY_PREV,
}keypad_t;

rt_uint32_t key_get(int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* _KEYPAD_H_ */
