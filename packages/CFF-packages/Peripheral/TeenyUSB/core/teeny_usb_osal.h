/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for STM32 micro controllers
 * 
 * Copyright (c) 2019 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __TUSBH_OS_H__
#define __TUSBH_OS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "teeny_usb_config.h"

////////////////////////////////////////////////
// OS related APIs
////////////////////////////////////////////////

typedef struct _tusb_msg tusb_msg_t;

struct _tusb_msg
{
    uint32_t     param;
    uint32_t     len;
    void*        data;
    void (*handler)(tusb_msg_t* msg);
};

#define POST_MESSAGE(mq, func, p, d, l)  \
do{                             \
    tusb_msg_t msg = {          \
    .param = p,                 \
    .len = l,                   \
    .data = d,                  \
    .handler = func};           \
    tusb_mq_post(mq, &msg);    \
}while(0)


/// API for message queue
#ifdef TUSB_MQ_T
typedef TUSB_MQ_T tusb_mq_t;
#else
typedef struct _tusb_mq tusb_mq_t;
#endif

tusb_mq_t* tusb_mq_create(void);

void tusb_mq_free(tusb_mq_t* mq);

int tusb_mq_init(tusb_mq_t* mq);

int tusb_mq_post(tusb_mq_t* mq, const tusb_msg_t* msg);

// return value: 1 - success, 0 
int tusb_mq_get(tusb_mq_t* mq, tusb_msg_t* msg);

/// API for event
typedef struct _tusb_evt tusb_evt_t;

tusb_evt_t* tusb_evt_create(void);

void tusb_evt_free(tusb_evt_t* evt);

int tusb_evt_init(tusb_evt_t* evt);

int tusb_evt_set(tusb_evt_t* evt);

int tusb_evt_clear(tusb_evt_t* evt);

// return 0 for success, otherwise fail
int tusb_evt_wait(tusb_evt_t* evt, uint32_t timeout_ms);

// return memory must 32 bit aligned
// real allocate memory size muse be muple of 4 (USB FIFO requirement)
void* tusb_malloc(uint32_t size);

void tusb_free(void* p);

#ifndef TUSB_NO_HOST
typedef struct _tusbh_device tusbh_device_t;

tusbh_device_t* tusbh_new_device(void);

void tusbh_free_device(tusbh_device_t* device);

void show_memory(void);
#endif /* TUSB_NO_HOST */

#ifdef __cplusplus
}
#endif

#endif

