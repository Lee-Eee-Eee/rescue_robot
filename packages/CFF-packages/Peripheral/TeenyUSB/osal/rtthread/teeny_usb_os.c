// OS port for rt-thread

#include <rtthread.h>
#include <string.h>
#include "tusbh.h"
#include "teeny_usb_osal.h"


void tusb_delay_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

tusb_mq_t* tusb_mq_create(void)
{
    rt_mq_t mq = rt_mq_create("tu_mq", sizeof(tusb_msg_t), 16, RT_IPC_FLAG_FIFO);
    return (tusb_mq_t*)mq;
}

void tusb_mq_free(tusb_mq_t* mq)
{
    rt_mq_delete((rt_mq_t)mq);
}

int tusb_mq_init(tusb_mq_t* mq)
{
    return 0;
}

int tusb_mq_post(tusb_mq_t* mq, const tusb_msg_t* msg)
{
    if(rt_mq_send((rt_mq_t)mq, (void*)msg, sizeof(tusb_msg_t)) == RT_EOK){
        return 0;
    }
    return -1;
}

// return value: 1 - success, 0 
int tusb_mq_get(tusb_mq_t* mq, tusb_msg_t* msg)
{
    if(rt_mq_recv((rt_mq_t)mq, msg, sizeof(tusb_msg_t), RT_WAITING_FOREVER) > 0){
        return 1;
    }
    return 0;
}

/// API for event

tusb_evt_t* tusb_evt_create(void)
{
    rt_event_t evt = rt_event_create("tu_ev", RT_IPC_FLAG_FIFO);
    return (tusb_evt_t*)evt;
}

void tusb_evt_free(tusb_evt_t* evt)
{
    rt_event_delete( (rt_event_t)evt );
}

int tusb_evt_init(tusb_evt_t* evt)
{
    return 0;
}

int tusb_evt_set(tusb_evt_t* evt)
{
    rt_event_send( (rt_event_t)evt, 1);
    return 0;
}

// return 0 for success, otherwise fail
int tusb_evt_wait(tusb_evt_t* evt, uint32_t timeout_ms)
{
    rt_uint32_t recv;
    rt_err_t r = rt_event_recv( (rt_event_t)evt,
                        0xffffffff,
                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                       rt_tick_from_millisecond(timeout_ms),
                       &recv);
    if( (r == RT_EOK) && (recv & 1) ){
        return 0;
    }
    return -1;
}

int tusb_evt_clear(tusb_evt_t* evt)
{
    rt_event_control((rt_event_t)evt, RT_IPC_CMD_RESET, 0);
    return 0;
}

// return memory must 32 bit aligned
// real allocate memory size muse be muple of 4 (USB FIFO requirement)
void* tusb_malloc(uint32_t size)
{
    return rt_malloc_align( ( (size+3) & ~3), 4);
}

void tusb_free(void* p)
{
    rt_free_align(p);
}


#ifndef TUSB_NO_HOST

tusbh_device_t* tusbh_new_device(void)
{
    tusbh_device_t* dev = (tusbh_device_t*)rt_malloc(sizeof(tusbh_device_t));
    if(dev){
        memset(dev, 0, sizeof(tusbh_device_t));
    }
    return dev;
}

void tusbh_free_device(tusbh_device_t* device)
{
    rt_free(device);
}

#endif /* TUSB_NO_HOST */
