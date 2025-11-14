#include "serial_bus.hpp"

#define DBG_TAG "SerialBus.cpp"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

namespace CFF
{

SerialBUS::SerialBUS(const char *port_name, int baudrate, int pin, int level) :
    prot_name_(port_name),
    baudrate_(baudrate),
    cs_pin_(pin),
    level_(level),
    bus_(NULL)
{ }

SerialBUS::~SerialBUS()
{
    serial_bus_destory(bus_);
}

bool SerialBUS::init()
{
    if (bus_ == NULL)
    {
        bus_ = serial_bus_create(prot_name_, baudrate_, PARITY_NONE, cs_pin_, level_);
        if (bus_)
            return true;
    }
    else
    {
        LOG_D("The bus has been created.");
    }
    return false;
}

bool SerialBUS::open()
{
    if (!serial_bus_connect(bus_))
    {
        return true;
    }
    return false;
}

bool SerialBUS::close()
{
    if (!serial_bus_disconn(bus_))
    {
        return true;
    }
    return false;
}

bool SerialBUS::config(int baudrate, int databits, int parity, int stopbits)
{
    if (!serial_bus_config(bus_, baudrate, databits, parity, stopbits))
    {
        return true;
    }
    return false;
}

bool SerialBUS::isOpened()
{
    if (bus_->status)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool SerialBUS::stopRead()
{
    if (!serial_bus_break_recv(bus_))
    {
        return true;
    }
    return false;
}

bool SerialBUS::setTimeout(int timeout_ms)
{
    if (!serial_bus_set_recv_tmo(bus_, timeout_ms))
    {
        return true;
    }
    return false;
}

bool SerialBUS::setByteTimeout(int timeout_ms)
{
    if (!serial_bus_set_byte_tmo(bus_, timeout_ms))
    {
        return true;
    }
    return false;
}

int SerialBUS::read(void *buf, int size)
{
    return serial_bus_recv(bus_, buf, size);
}

int SerialBUS::write(void *buf, int size)
{
    return serial_bus_send(bus_, buf, size);
}

int SerialBUS::sendThenReceive( void *send_buf, int send_len, void *recv_buf, int recv_size)
{
    return serial_bus_send_then_recv(bus_, send_buf, send_len, recv_buf, recv_size);
}

} // CFF

