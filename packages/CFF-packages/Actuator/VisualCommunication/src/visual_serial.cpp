#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "visual_serial.hpp"

#define DBG_TAG "VisualSerial"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


class VisualSerial
{
public:
    static const int DEFAULT_BAUDRATE_ = 1000000; // Default Baudrate

    VisualSerial(const char *port_name, int baudrate=DEFAULT_BAUDRATE_);
    ~VisualSerial();

    bool init();

    bool open();

    bool close();

    bool isOpened();

private:
    const char *prot_name_; //serial port name
    int32_t baudrate_;      //serial baudrate
};


static rt_err_t serial_bus_recv_ind_hook(rt_device_t dev, rt_size_t size)
{
    VisualSerial *ser = (VisualSerial *)(dev->user_data);
    
    return(RT_EOK);
}


VisualSerial::VisualSerial()
{
}

VisualSerial::~VisualSerial()
{
}
