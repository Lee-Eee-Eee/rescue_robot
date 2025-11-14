#ifndef __CFF_SERIAL_BUS_H__
#define __CFF_SERIAL_BUS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <rtconfig.h>

#define SERIAL_BUS_BYTE_TMO_MIN     2
#define SERIAL_BUS_BYTE_TMO_MAX     200
#define SERIAL_BUS_SW_DLY_US        10

struct serial_bus
{
    rt_device_t serial;     //serial device handle
    rt_mutex_t lock;        //mutex handle
    rt_event_t evt;         //event handle
    rt_uint8_t status;      //connect status
    rt_uint8_t level;       //control pin send mode level, 0--low, 1--high
    rt_int16_t pin;         //control pin number used, -1--no using
    rt_int32_t timeout;     //receive block timeout, ms
    rt_int32_t byte_tmo;    //receive byte interval timeout, ms
};

typedef struct serial_bus* serial_bus_t;

serial_bus_t serial_bus_create(const char *serial, int baudrate, int parity, int pin, int level);

int serial_bus_destory(serial_bus_t bus);

int serial_bus_config(serial_bus_t bus, int baudrate, int databits, int parity, int stopbits);

int serial_bus_set_recv_tmo(serial_bus_t bus, int tmo_ms);
 
int serial_bus_set_byte_tmo(serial_bus_t bus, int tmo_ms);

int serial_bus_connect(serial_bus_t bus, uint8_t open_mode);

int serial_bus_disconn(serial_bus_t bus);

int serial_bus_recv(serial_bus_t bus, void *buf, int size);

int serial_bus_send(serial_bus_t bus, void *buf, int size);

int serial_bus_break_recv(serial_bus_t bus);

int serial_bus_send_then_recv(serial_bus_t bus, void *send_buf, int send_len, void *recv_buf, int recv_size);

#ifdef __cplusplus
}
#endif
#endif // __CFF_SERIAL_BUS_H__

