#ifndef __CFF_VISUAL_SERIAL_H__
#define __CFF_VISUAL_SERIAL_H__

class VisualSerial
{
public:
    static const int DEFAULT_BAUDRATE_ = 1000000; // Default Baudrate

    VisualSerial(const char *port_name, int baudrate=DEFAULT_BAUDRATE_);
    ~VisualSerial();

private:
    const char *prot_name_; //serial port name
    int32_t baudrate_;      //serial baudrate
};



#endif // __CFF_VISUAL_SERIAL_H__
