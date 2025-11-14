#include "keypad.h"
#include <rtthread.h>
#include "pins_port.h"
#include "key_char.h"
#include "agile_button.h"

static rt_mailbox_t keypad_mb = RT_NULL;

static void btn_click_event_cb(agile_btn_t *btn)
{
    // rt_kprintf("[button click event] pin:%d   repeat:%d, hold_time:%d\r\n", btn->pin, btn->repeat_cnt, btn->hold_time);

    switch(btn->pin)
    {
    case KEY1_PIN:
        rt_mb_send(keypad_mb, KEY1_PRESS_DOWN);
        break;
    case KEY2_PIN:
        rt_mb_send(keypad_mb, KEY2_PRESS_DOWN);
        break;
    case KEY3_PIN:
        rt_mb_send(keypad_mb, KEY3_PRESS_DOWN);
        break;
    case KEY4_PIN:
        rt_mb_send(keypad_mb, KEY4_PRESS_DOWN);
        break;
    case KEY5_PIN:
        rt_mb_send(keypad_mb, KEY5_PRESS_DOWN);
        break;
    default:
        break;
    }
}

/**
 * @brief   Agile Button 内部线程初始化
 * @return  RT_EOK:成功
 */
static int keypad_thread_init(void)
{
    agile_btn_t *_btn;

    agile_btn_env_init();
    
    _btn = agile_btn_create(KEY1_PIN, PIN_LOW, PIN_MODE_INPUT_PULLUP);
    agile_btn_set_event_cb(_btn, BTN_PRESS_DOWN_EVENT, btn_click_event_cb);
    agile_btn_start(_btn);

    _btn = agile_btn_create(KEY2_PIN, PIN_LOW, PIN_MODE_INPUT_PULLUP);
    agile_btn_set_event_cb(_btn, BTN_PRESS_DOWN_EVENT, btn_click_event_cb);
    agile_btn_start(_btn);

    _btn = agile_btn_create(KEY3_PIN, PIN_LOW, PIN_MODE_INPUT_PULLUP);
    agile_btn_set_event_cb(_btn, BTN_PRESS_DOWN_EVENT, btn_click_event_cb);
    agile_btn_start(_btn);

    _btn = agile_btn_create(KEY4_PIN, PIN_LOW, PIN_MODE_INPUT_PULLUP);
    agile_btn_set_event_cb(_btn, BTN_PRESS_DOWN_EVENT, btn_click_event_cb);
    agile_btn_start(_btn);
    
    _btn = agile_btn_create(KEY5_PIN, PIN_LOW, PIN_MODE_INPUT_PULLUP);
    agile_btn_set_event_cb(_btn, BTN_PRESS_DOWN_EVENT, btn_click_event_cb);
    agile_btn_start(_btn);

    keypad_mb = rt_mb_create("keypad_mb", 8, RT_IPC_FLAG_FIFO);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(keypad_thread_init);

rt_uint32_t key_get(int timeout_ms)
{
    rt_ubase_t key;
    rt_err_t rst = rt_mb_recv(keypad_mb, &key, rt_tick_from_millisecond(timeout_ms));
    if (rst != RT_EOK)
        return(0);

    return(key);
}