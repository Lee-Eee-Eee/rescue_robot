#ifndef __CFF_SERIAL_BUS_HPP__
#define __CFF_SERIAL_BUS_HPP__

#include <stdio.h>
#include <vector>
#include <board.h>
#include "serial_bus.h"

namespace CFF
{




class SerialBUS
{
    
public:
    enum OpenMode
    {
        OPEN_MODE_INT_RX = 0,
        OPEN_MODE_DMA_RX
    };

    SerialBUS(const char *port_name, int baudrate, int pin, int level);

    ~SerialBUS();

    bool init();

    bool open(uint8_t open_mode = OPEN_MODE_INT_RX);

    bool close();

    bool config(int baudrate, int databits, int parity, int stopbits);

    bool isOpened();

    bool stopRead();

    bool setTimeout(int timeout_ms);

    bool setByteTimeout(int timeout_ms);

    int read(void *buf, int size);

    int write(void *buf, int size);

    int sendThenReceive( void *send_buf, int send_len, void *recv_buf, int recv_size);

protected:
    const char *prot_name_; //serial port name
    int32_t baudrate_;      //serial baudrate
    int16_t cs_pin_;        //control pin number used, -1--no using
    uint8_t level_;         //control pin send mode level, 0--low, 1--high
    serial_bus_t bus_;
};

} // CFF

#endif // __CFF_SERIAL_BUS_HPP__