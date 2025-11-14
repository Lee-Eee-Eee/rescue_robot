
#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "serial_bus.h"

#define DBG_TAG "SerialBus"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define SERIAL_BUS_EVT_RX_IND       (1<<0)
#define SERIAL_BUS_EVT_RX_BREAK     (1<<1)

static rt_err_t serial_bus_recv_ind_hook(rt_device_t dev, rt_size_t size)
{
    serial_bus_t bus = (serial_bus_t)(dev->user_data);
    if (bus->evt)
    {
        rt_event_send(bus->evt, SERIAL_BUS_EVT_RX_IND);
    }
    return(RT_EOK);
}

static int serial_bus_cal_byte_tmo(int baudrate)
{
    int tmo = (40 * 1000) / baudrate;
    if (tmo < SERIAL_BUS_BYTE_TMO_MIN)
    {
        tmo = SERIAL_BUS_BYTE_TMO_MIN;
    }
    else if (tmo > SERIAL_BUS_BYTE_TMO_MAX)
    {
        tmo = SERIAL_BUS_BYTE_TMO_MAX;
    }
    return (tmo);
}

static void serial_bus_mode_set(serial_bus_t bus, int mode) // mode : 0--receive mode, 1--send mode
{
    if (bus->pin < 0)
    {
        return;
    }

    if (mode)
    {
        rt_pin_write(bus->pin, bus->level);
#if (SERIAL_BUS_SW_DLY_US > 0)
        rt_hw_us_delay(SERIAL_BUS_SW_DLY_US);
#endif
    }
    else
    {
#if (SERIAL_BUS_SW_DLY_US > 0)
        rt_hw_us_delay(SERIAL_BUS_SW_DLY_US);
#endif
        rt_pin_write(bus->pin, !bus->level);
    }
}

serial_bus_t serial_bus_create(const char *name, int baudrate, int parity, int pin, int level)
{
    serial_bus_t bus;
    rt_device_t dev;

    dev = rt_device_find(name);
    if (dev == RT_NULL)
    {
        LOG_E("Create error, the serial device(%s) no found.", name);
        return(RT_NULL);
    }

    if (dev->type != RT_Device_Class_Char)
    {
        LOG_E("Create error, the serial device(%s) type is not char.", name);
        return(RT_NULL);
    }

    bus = rt_malloc(sizeof(struct serial_bus));
    if (bus == RT_NULL)
    {
        LOG_E("Create fail. no memory for create instance.");
        return(RT_NULL);
    }

    bus->lock = rt_mutex_create(name, RT_IPC_FLAG_FIFO);
    if (bus->lock == RT_NULL)
    {
        rt_free(bus);
        LOG_E("Create fail. no memory for create mutex.");
        return(RT_NULL);
    }

    bus->evt = rt_event_create(name, RT_IPC_FLAG_FIFO);
    if (bus->evt == RT_NULL)
    {
        rt_mutex_delete(bus->lock);
        rt_free(bus);
        LOG_E("Create fail. no memory for create event.");
        return(RT_NULL);
    }

    bus->serial = dev;
    bus->status = 0;
    bus->pin = pin;
    bus->level = (level != 0);
    bus->timeout = 10;
    bus->byte_tmo = serial_bus_cal_byte_tmo(baudrate);

    serial_bus_config(bus, baudrate, 8, parity, 0);

    LOG_D("Create success.");

    return(bus);
}

int serial_bus_destory(serial_bus_t bus)
{
    if (bus == RT_NULL)
    {
        LOG_E("Destory fail. bus is NULL.");
        return(-RT_ERROR);
    }

    serial_bus_disconn(bus);

    if (bus->lock)
    {
        rt_mutex_delete(bus->lock);
        bus->lock = RT_NULL;
    }

    if (bus->evt)
    {
        rt_event_delete(bus->evt);
        bus->evt = RT_NULL;
    }

    rt_free(bus);

    LOG_D("Destory success.");

    return(RT_EOK);
}

int serial_bus_config(serial_bus_t bus, int baudrate, int databits, int parity, int stopbits)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    if (bus == RT_NULL)
    {
        LOG_E("Config fail. bus is NULL.");
        return(-RT_ERROR);
    }

    bus->byte_tmo = serial_bus_cal_byte_tmo(baudrate);

    config.baud_rate = baudrate;
    config.data_bits = databits;
    config.parity = parity;
    config.stop_bits = stopbits;
    rt_device_control(bus->serial, RT_DEVICE_CTRL_CONFIG, &config);

    return(RT_EOK);
}


int serial_bus_set_recv_tmo(serial_bus_t bus, int tmo_ms)
{
    if (bus == RT_NULL)
    {
        LOG_E("Set recv timeout fail. bus is NULL.");
        return(-RT_ERROR);
    }

    bus->timeout = tmo_ms;

    LOG_D("Set recv timeout success. the value is %d.", tmo_ms);

    return(RT_EOK);
}


int serial_bus_set_byte_tmo(serial_bus_t bus, int tmo_ms)
{
    if (bus == RT_NULL)
    {
        LOG_E("Set byte timeout fail. bus is NULL.");
        return(-RT_ERROR);
    }

    if (tmo_ms < SERIAL_BUS_BYTE_TMO_MIN)
    {
        tmo_ms = SERIAL_BUS_BYTE_TMO_MIN;
    }
    else if (tmo_ms > SERIAL_BUS_BYTE_TMO_MAX)
    {
        tmo_ms = SERIAL_BUS_BYTE_TMO_MAX;
    }

    bus->byte_tmo = tmo_ms;

    LOG_D("Set byte timeout success. the value is %d.", tmo_ms);

    return(RT_EOK);
}

int serial_bus_connect(serial_bus_t bus, uint8_t open_mode)
{
    if (bus == RT_NULL)
    {
        LOG_E("Connect fail. bus is NULL.");
        return(-RT_ERROR);
    }

    if (bus->status == 1)//is connected
    {
        LOG_D("Is connected.");
        return(RT_EOK);
    }

#ifdef RT_USING_SERIAL_V2
    if ( rt_device_open(bus->serial, RT_SERIAL_RX_NON_BLOCKING | RT_SERIAL_TX_BLOCKING) != RT_EOK)
#else
    if ( rt_device_open(bus->serial, (open_mode == 0) ? RT_DEVICE_FLAG_INT_RX : RT_DEVICE_FLAG_DMA_RX) != RT_EOK)
#endif
    {
        LOG_E("Serial open fail.");
        return(-RT_ERROR);
    }

    if (bus->pin >= 0)
    {
        rt_pin_mode(bus->pin, PIN_MODE_OUTPUT);
        rt_pin_write(bus->pin, ! bus->level);
    }


    bus->serial->user_data = bus;
    bus->serial->rx_indicate = serial_bus_recv_ind_hook;
    bus->status = 1;

    LOG_D("Connect success.");

    return(RT_EOK);
}

int serial_bus_disconn(serial_bus_t bus)
{
    if (bus == RT_NULL)
    {
        LOG_E("Disconnect fail. bus is NULL.");
        return(-RT_ERROR);
    }

    if (bus->status == 0)//is not connected
    {
        LOG_D("Is not connected.");
        return(RT_EOK);
    }

    rt_mutex_take(bus->lock, RT_WAITING_FOREVER);

    if (bus->serial)
    {
        bus->serial->rx_indicate = RT_NULL;
        rt_device_close(bus->serial);
    }

    if (bus->pin >= 0)
    {
        rt_pin_mode(bus->pin, PIN_MODE_INPUT);
    }

    bus->status = 0;

    rt_mutex_release(bus->lock);

    LOG_D("Disconnect success.");

    return(RT_EOK);
}

int serial_bus_recv(serial_bus_t bus, void *buf, int size)
{
    int recv_len = 0;
    rt_uint32_t recved = 0;

    if (bus == RT_NULL || buf == RT_NULL || size == 0)
    {
        LOG_E("Receive fail. param error.");
        return(-RT_ERROR);
    }

    if (bus->status == 0)
    {
        LOG_E("Receive fail. it is not connected.");
        return(-RT_ERROR);
    }

    if (rt_mutex_take(bus->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Receive fail. it is destoried.");
        return(-RT_ERROR);
    }

    while(size)
    {
        int len = rt_device_read(bus->serial, 0, (char *)buf + recv_len, size);
        if (len)
        {
            recv_len += len;
            size -= len;
            continue;
        }
        rt_event_control(bus->evt, RT_IPC_CMD_RESET, RT_NULL);
        if (recv_len)
        {
            if (rt_event_recv(bus->evt, SERIAL_BUS_EVT_RX_IND,
                    (RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR), bus->byte_tmo, &recved) != RT_EOK)
            {
                break;
            }
        }
        else
        {
            if (rt_event_recv(bus->evt, (SERIAL_BUS_EVT_RX_IND | SERIAL_BUS_EVT_RX_BREAK),
                    (RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR), bus->timeout, &recved) != RT_EOK)
            {
                break;
            }
            if ((recved & SERIAL_BUS_EVT_RX_BREAK) != 0)
            {
                rt_mutex_release(bus->lock);
                rt_thread_delay(2);
                return(0);
            }
        }
    }

    rt_mutex_release(bus->lock);

    return(recv_len);
}

int serial_bus_send(serial_bus_t bus, void *buf, int size)
{
    int send_len = 0;

    if (bus == RT_NULL || buf == RT_NULL || size == 0)
    {
        LOG_E("Send fail. param is error.");
        return(-RT_ERROR);
    }

    if (bus->status == 0)
    {
        LOG_E("Send fail. it is not connected.");
        return(-RT_ERROR);
    }

    if (rt_mutex_take(bus->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("Send fail. it is destoried.");
        return(-RT_ERROR);
    }

    serial_bus_mode_set(bus, 1);//set to send mode

    send_len = rt_device_write(bus->serial, 0, buf, size);

    serial_bus_mode_set(bus, 0);//set to receive mode

    rt_mutex_release(bus->lock);

    return(send_len);
}

int serial_bus_break_recv(serial_bus_t bus)
{
    if ((bus == RT_NULL) || (bus->evt == RT_NULL))
    {
        return(-RT_ERROR);
    }

    rt_event_send(bus->evt, SERIAL_BUS_EVT_RX_BREAK);

    return (RT_EOK);
}

int serial_bus_send_then_recv(serial_bus_t bus, void *send_buf, int send_len, void *recv_buf, int recv_size)
{
    int recv_len = 0;
    rt_uint32_t recved = 0;

    if (bus == RT_NULL || send_buf == RT_NULL || send_len == 0 || recv_buf == RT_NULL || recv_size == 0)
    {
        LOG_E("send_then_recv fail. param is error.");
        return(-RT_ERROR);
    }

    if (bus->status == 0)
    {
        LOG_E("send_then_recv fail. it is not connected.");
        return(-RT_ERROR);
    }

    if (rt_mutex_take(bus->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        LOG_E("send_then_recv fail. it is destoried.");
        return(-RT_ERROR);
    }

    serial_bus_mode_set(bus, 1);//set to send mode
    send_len = rt_device_write(bus->serial, 0, send_buf, send_len);
    serial_bus_mode_set(bus, 0);//set to receive mode
    if (send_len < 0)
    {
        rt_mutex_release(bus->lock);
        LOG_E("send_then_recv fail. send datas error.");
        return(-RT_ERROR);
    }

    while(recv_size)
    {
        int len = rt_device_read(bus->serial, 0, (char *)recv_buf + recv_len, 1);
        if (len)
        {
            recv_len += len;
            recv_size -= len;
            continue;
        }
        rt_event_control(bus->evt, RT_IPC_CMD_RESET, RT_NULL);
        if (recv_len)
        {
            if (rt_event_recv(bus->evt, SERIAL_BUS_EVT_RX_IND,
                    (RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR), bus->byte_tmo, &recved) != RT_EOK)
            {
                break;
            }
        }
        else
        {
            if (rt_event_recv(bus->evt, SERIAL_BUS_EVT_RX_IND,
                    (RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR), bus->timeout, &recved) != RT_EOK)
            {
                break;
            }
        }
    }

    rt_mutex_release(bus->lock);
    // LOG_I("serial_bus_send_then_recv %d", recv_len);
    return(recv_len);
}

