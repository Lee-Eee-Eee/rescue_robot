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

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "teeny_usb_osal.h"
#include "teeny_usb_util.h"

#ifndef TUSB_NO_HOST
#include "tusbh.h"
#endif /* TUSB_NO_HOST */

#ifndef TUSB_HAS_OS

#define  TUSBH_MSG_Q_LENGTH        16
struct _tusb_mq{
    uint16_t  rd_index;
    uint16_t  wr_index;
    tusb_msg_t queue[TUSBH_MSG_Q_LENGTH];
};

static tusb_mq_t _default_mq;

#define MQ_SIZE    4
static tusb_mq_t mq_pool[MQ_SIZE];
static uint8_t mq_used[MQ_SIZE];



tusb_mq_t* tusb_mq_create()
{
    for(int i=0;i<MQ_SIZE;i++){
        if(!mq_used[i]){
            mq_used[i] = 1;
            return &mq_pool[i];
        }
    }
    TUSB_LOGE("no free mq space\n");
    return 0;
}

void tusb_mq_free(tusb_mq_t* mq)
{
    for(int i=0;i<MQ_SIZE;i++){
        if(mq == &mq_pool[i]){
            if(!mq_used[i]){
                TUSB_LOGE("mq not in use\n");            
            }
            mq_used[i] = 0;
        }
    }
    TUSB_LOGE("mq memory out bound\n");
}


int tusb_mq_init(tusb_mq_t* mq)
{
    memset(mq, 0, sizeof(tusb_mq_t));
    return 0;
}

#define MQ_MASK (TUSBH_MSG_Q_LENGTH-1)

static inline int tusb_mq_full(tusb_mq_t* mq){
    if(mq->wr_index == mq->rd_index)return 0;
    if( (mq->wr_index & MQ_MASK) == (mq->rd_index & MQ_MASK) ) return 1;
    return 0;
}

int tusb_mq_post(tusb_mq_t* mq, const tusb_msg_t* msg)
{
    if(!mq) mq = &_default_mq;
    if(! tusb_mq_full(mq) ){
        memcpy(&mq->queue[mq->wr_index & MQ_MASK ], msg, sizeof(tusb_msg_t));
        mq->wr_index++;
        return 1;
    }
    return 0;
}

int tusb_mq_get(tusb_mq_t* mq, tusb_msg_t* msg)
{
    if(!mq) mq = &_default_mq;
    if(mq->rd_index != mq->wr_index){
        memcpy(msg, &mq->queue[mq->rd_index & MQ_MASK ], sizeof(tusb_msg_t));
        mq->rd_index++;
        return 1;
    }
    return 0;
}

struct _tusb_evt
{
    uint8_t event;
};

static tusb_evt_t  event_pool[16];
static uint8_t event_used[16] = {0};

tusb_evt_t* tusb_evt_create()
{
    for(int i=0;i<sizeof(event_used);i++){
        if(!event_used[i]){
            event_used[i] = 1;
            tusb_evt_init(&event_pool[i]);
            return &event_pool[i];
        }
    }
    TUSB_LOGE("no free event space\n");
    return 0;
}

void tusb_evt_free(tusb_evt_t* evt)
{
    for(int i=0;i<sizeof(event_used);i++){
        if(&event_pool[i] == evt){
            if(!event_used[i]){
                TUSB_LOGE("event not in use\n");
            }
            event_used[i] = 0;
            return;
        }
    }
    TUSB_LOGE("event memory out bound\n");
}

int tusb_evt_init(tusb_evt_t* evt)
{
    evt->event = 0;
    return 0;
}

int tusb_evt_set(tusb_evt_t* evt)
{
    evt->event = 1;
    return 0;
}


int tusb_evt_clear(tusb_evt_t* evt)
{
    evt->event = 0;
    return 0;
}

int tusb_evt_wait(tusb_evt_t* evt, uint32_t timeout_ms)
{
    while(timeout_ms>0){
        if(evt->event){
            evt->event = 0;
            return 0;
        }
        if(timeout_ms != 0xffffffff){
            timeout_ms--;
        }
        tusb_delay_ms(1);
    }
    return -1;
}

static int mem_used;
static int mem_max;
void* tusb_malloc(uint32_t size)
{
    size = (size + 3) & (~3);
    mem_used+=size;
    if(mem_max < mem_used){
        mem_max = mem_used;
    }
    void* r = malloc(size+8);
    TUSB_ASSERT( (r != 0) && (((uint32_t)r) & 3) == 0 );
    uint32_t* p = (uint32_t*)r;
    *p = size;
    *(p + (size/4) + 1) = 0xdeadbeef;
    TUSB_LOGD("allocate %p %d\n", p, size);
    return (void*)(p+1);
}

void tusb_free(void* ptr)
{
    TUSB_ASSERT(ptr != 0);
    uint32_t* p = (uint32_t*)ptr;
    p = p - 1;
    uint32_t size = *p;
    mem_used -= size;
    TUSB_ASSERT(*(p+(size/4)+1) == 0xdeadbeef);
    TUSB_LOGD("free %p %d\n", p, size);
    free(p);
}

#ifndef TUSB_NO_HOST

#define MAX_DEVICE_COUNT   16
static tusbh_device_t device_pool[MAX_DEVICE_COUNT];
static uint8_t dev_used[MAX_DEVICE_COUNT] = {0};

tusbh_device_t* tusbh_new_device()
{
    for(int i=0;i<MAX_DEVICE_COUNT;i++){
        if(!dev_used[i]){
            dev_used[i] = 1;
            return &device_pool[i];
        }
    }
    TUSB_LOGE("no free device space\n");
    return 0;
}

void tusbh_free_device(tusbh_device_t* device)
{
    for(int i=0;i<MAX_DEVICE_COUNT;i++){
        if(device == &device_pool[i]){
            if(!dev_used[i]){
                TUSB_LOGW("device not in use\n");
            }
            dev_used[i] = 0;
            return;
        }
    }
    TUSB_LOGW("device memory out bound\n");
}

void show_memory(void)
{
    TUSB_LOGD("  Memory used %d, max %d\n", mem_used, mem_max);
    int used;
    used = 0;
    for(int i=0;i<sizeof(mq_used);i++){
        if(mq_used[i]){
            used++;
        }
    }
    TUSB_LOGD("  MQ used %d\n", used);
    used = 0;
    for(int i=0;i<sizeof(dev_used);i++){
        if(dev_used[i]){
            used++;
        }
    }
    TUSB_LOGD("  Device used %d\n", used);
    used = 0;
    for(int i=0;i<sizeof(event_used);i++){
        if(event_used[i]){
            used++;
        }
    }
    TUSB_LOGD("  Event used %d\n", used);
}
#endif /* TUSB_NO_HOST */

WEAK void tusb_delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; ++i)
        for (j = 0; j < 200; ++j)
            ;
}


#endif  // TUSB_HAS_OS



