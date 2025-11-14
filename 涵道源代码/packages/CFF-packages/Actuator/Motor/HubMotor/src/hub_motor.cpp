#include "hub_motor.hpp"

#define DBG_TAG "HubMotor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

namespace CFF
{

static const uint8_t auchCRCHi[] = { 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40} ; 
static const uint8_t auchCRCLo[] = { 
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,0x43, 0x83, 0x41, 0x81, 0x80, 0x40} ;

static uint16_t modbus_crc(uint8_t *puchMsg, uint16_t usDataLen) 
{ 
    uint8_t uchCRCHi = 0xFF ; 
    uint8_t uchCRCLo = 0xFF ; 
    uint32_t uIndex; 
    while (usDataLen--) 
    { 
        uIndex = uchCRCHi ^ *puchMsg++ ; 
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ; 
        uchCRCLo = auchCRCLo[uIndex] ; 
    } 
    return ((uchCRCHi<< 8)  | (uchCRCLo)) ; 
}

static uint8_t get_read_len(uint8_t cmd)
{
    uint8_t len = 0;
    switch (cmd)
    {
    case HM_CMD_GET_VERSION_INFO:
        len = 20;
        break;
    case HM_CMD_READ_REALTIME_DATA:
        len = 13;
        break;
    case HM_CMD_READ_SYS_PARAMETERS:
    case HM_CMD_WRITE_SYS_PARAMETERS:
    case HM_CMD_SAVE_SYS_PARAMETERS:
    case HM_CMD_FACTORY_RESET:
        len = 26;
        break;
    case HM_CMD_SET_CURRENT_POSITION_AS_ZERO:
        len = 3;
        break;
    case HM_CMD_READ_ENCODER_DATA:
    case HM_CMD_SHUTDOWN:
    case HM_CMD_RETURN_TO_ZERO_POS:
    case HM_CMD_SHORT_PATH_HOME:
    case HM_CMD_OPEN_LOOP_CONTROL:
    case HM_CMD_SPEED_CLOSED_LOOP:
    case HM_CMD_ABSOLUTE_POS_CLOSED_LOOP:
    case HM_CMD_RELATIVE_POS_CLOSED_LOOP:
        len = 8;
        break;
    case HM_CMD_READ_STATUS:
    case HM_CMD_CLEAR_FAULTS:
        len = 5;
        break;
    case HM_CMD_POS_CTRL_TARGET_SPEED:
        len = 2;
            break;
    default:
        len = 0;
        break;
    }
    return len;
}

HubMotorPort::HubMotorPort(const char *port_name, int baudrate, int pin, int level) :
    SerialBUS(port_name, baudrate, pin, level),
    is_using_(false),
    seq_num_(0)
{}


int HubMotorPort::writeData(uint8_t device_id, uint8_t cmd, const void *data, int16_t len)
{
    HM_FrameHeader_t *freame_header = (HM_FrameHeader_t *)send_buffer_;
    freame_header->header = HM_SEND_HEADER;
    freame_header->seq = seq_num_;
    freame_header->device_id = device_id;
    freame_header->cmd = cmd;
    freame_header->len = len;

    if (len != 0)
    {
        memcpy(HM_FRAME_BUFFER_ADDR(freame_header), data, len);
    }

    uint16_t crc = modbus_crc(send_buffer_, HM_MODBUS_CRC_LEN(send_buffer_)); 
    send_buffer_[HM_MODBUS_CRC_LEN(freame_header)] = (crc>>8)&0x00ff;
    send_buffer_[HM_MODBUS_CRC_LEN(freame_header)+1] = crc&0x00ff;

    if (write(send_buffer_, HM_BUFFER_LEN(send_buffer_)) > 0)
    {
        return (HM_SUCCESS);
    }
    else
    {
        return (-HM_TX_FAIL);
    }
}

int HubMotorPort::readData(uint8_t device_id, uint8_t cmd, void *data, int16_t len)
{
    int result, read_len;

    read_len = get_read_len(cmd);
    result = read(recv_buffer_, HM_BUFFER_MAX_LEN);
    if (result < 0)
    {
        return (-HM_RX_FAIL);
    }
    else if (result == 0)
    {
        return (-HM_RX_WAITING);
    }
    // else if (result != read_len)
    // {
    //     return (-HM_RX_TIMEOUT);
    // }
    
    HM_FrameHeader_t *freame_header = (HM_FrameHeader_t *)recv_buffer_;

    if (freame_header->header != HM_RECV_HEADER || freame_header->seq != seq_num_)
    {
        return (-HM_PROTOCOL_MISMATCH);
    }

    uint16_t crc = modbus_crc(recv_buffer_, HM_MODBUS_CRC_LEN(freame_header));
    uint8_t crc_l = (crc>>8)&0x00ff;
    uint8_t crc_h = crc&0x00ff;

    if (recv_buffer_[HM_MODBUS_CRC_LEN(freame_header)] != crc_l ||
         recv_buffer_[HM_MODBUS_CRC_LEN(freame_header)+1] != crc_h)
    {
        return (-HM_PROTOCOL_MISMATCH);
    }
    else
    {
        if (data != NULL)
        {
            memcpy(data, HM_FRAME_BUFFER_ADDR(freame_header), freame_header->len);
        }
        return (HM_SUCCESS);
    }
}

int HubMotorPort::execuit(uint8_t device_id, uint8_t cmd, void *write_data, int write_len, void *read_data, int read_len)
{
    int result = HM_FAILURE;
    if (is_using_)
        return HM_PORT_BUSY;
    is_using_ = true;

    result = writeData(device_id, cmd, write_data, write_len);
    if (result != HM_SUCCESS)
    {
        LOG_W("Writing data error, error code %d", result);
    }

    result = readData(device_id, cmd, read_data, read_len);
    if (result != HM_SUCCESS)
    {
        LOG_W("Reading data error, error code %d", result);
    }
    seq_num_++;
    is_using_ = false;

    return result;

}

HubMotor::HubMotor(HubMotorPort *port, uint8_t device_id) :
    port_(port),
    id_(device_id)
{ }

HM_Info_t* HubMotor::getVersionInfo()
{
    port_->execuit(id_, HM_CMD_GET_VERSION_INFO, NULL, 0, &info_, sizeof(HM_Info_t));
    return &info_;
}

HM_RealTimeData_t* HubMotor::readRealtimeData()
{
    port_->execuit(id_, HM_CMD_READ_REALTIME_DATA, NULL, 0, &real_data_, sizeof(HM_RealTimeData_t));
    return &real_data_;
}

HM_SystemParam_t* HubMotor::readSysParameters()
{
    port_->execuit(id_, HM_CMD_READ_SYS_PARAMETERS, NULL, 0, &sys_param_, sizeof(HM_SystemParam_t));
    return &sys_param_;
}

bool HubMotor::setId(uint8_t new_id)
{
    int result = HM_FAILURE;
    result = port_->execuit(id_, HM_CMD_READ_SYS_PARAMETERS, NULL, 0, &sys_param_, sizeof(HM_SystemParam_t));
    if (result != HM_SUCCESS)
        return false;
    sys_param_.device_address = new_id;
    result = port_->execuit(id_, HM_CMD_SAVE_SYS_PARAMETERS, &sys_param_, sizeof(HM_SystemParam_t), &sys_param_, sizeof(HM_SystemParam_t));
    if (result != HM_SUCCESS)
        return false;
    id_ = new_id;
    return true;
}

HM_SystemParam_t* HubMotor::factoryReset()
{
    port_->execuit(id_, HM_CMD_FACTORY_RESET, &sys_param_, sizeof(HM_SystemParam_t), &sys_param_, sizeof(HM_SystemParam_t));
    return &sys_param_;
}

bool HubMotor::calibration()
{
    int result = HM_FAILURE;
    result = port_->execuit(id_, HM_CMD_ENCODER_CALIBRATION, NULL, 0, NULL, 0);
    if (result != HM_SUCCESS)
        return false;
    return true;
}

bool HubMotor::setZero()
{
    int result = 0;
    uint8_t rxdata[3];
    result = port_->execuit(id_, HM_CMD_SET_CURRENT_POSITION_AS_ZERO, NULL, 0, rxdata, 3);
    if (result != HM_SUCCESS || rxdata[2] != 0x01)
        return false;
    return true;
}

HM_RealTimeData_t* HubMotor::readEncoder()
{
    port_->execuit(id_, HM_CMD_READ_ENCODER_DATA, NULL, 0, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::readStatus()
{
    port_->execuit(id_, HM_CMD_READ_STATUS, NULL, 0, HM_MEMBER_ADDRESS(&real_data_, HM_RealTimeData_t, power_voltage), 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::clearStatus()
{
    port_->execuit(id_, HM_CMD_CLEAR_FAULTS, NULL, 0, HM_MEMBER_ADDRESS(&real_data_, HM_RealTimeData_t, power_voltage), 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::shutdown()
{
    port_->execuit(id_, HM_CMD_SHUTDOWN, NULL, 0, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::goToZero()
{
    port_->execuit(id_, HM_CMD_RETURN_TO_ZERO_POS, NULL, 0, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::goToHome()
{
    port_->execuit(id_, HM_CMD_SHORT_PATH_HOME, NULL, 0, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::openLoopControl(int16_t power)
{
    port_->execuit(id_, HM_CMD_OPEN_LOOP_CONTROL, &power, 2, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::setSpeed(float speed_rpm)
{
    int16_t speed = speed_rpm * 10;
    port_->execuit(id_, HM_CMD_SPEED_CLOSED_LOOP, &speed, 2, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::setPosition(uint32_t pos)
{
    port_->execuit(id_, HM_CMD_ABSOLUTE_POS_CLOSED_LOOP, &pos, 4, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::setRelativePosition(uint32_t pos)
{
    port_->execuit(id_, HM_CMD_RELATIVE_POS_CLOSED_LOOP, &pos, 4, &real_data_, 8);
    return &real_data_;
}

HM_RealTimeData_t* HubMotor::setPosCtrlSpeed(float speed_rpm)
{
    int16_t speed = speed_rpm * 10;
    port_->execuit(id_, HM_CMD_POS_CTRL_TARGET_SPEED, &speed, 2, &real_data_, 8);
    return &real_data_;
}

bool HubMotor::setPid(float pos_kp, float speed_kp, float speed_ki, bool is_save)
{
    int result = HM_FAILURE;
    sys_param_.position_loop_proportion_kp = pos_kp;
    sys_param_.speed_loop_proportion_kp = speed_kp;
    sys_param_.speed_loop_integral_ki = speed_ki;
    if (is_save)
    {
        result = port_->execuit(id_, HM_CMD_SAVE_SYS_PARAMETERS, &sys_param_, sizeof(HM_SystemParam_t), &sys_param_, sizeof(HM_SystemParam_t));
    }
    else
    {
        result = port_->execuit(id_, HM_CMD_WRITE_SYS_PARAMETERS, &sys_param_, sizeof(HM_SystemParam_t), &sys_param_, sizeof(HM_SystemParam_t));
    }

    if (result != HM_SUCCESS)
        return false;
    return true;
}

HM_Info_t* HubMotor::getInfo()
{
    return &info_;
}

HM_SystemParam_t* HubMotor::getSysParam()
{
    return &sys_param_;
}

HM_RealTimeData_t* HubMotor::getRealData()
{
    return &real_data_;
}

float HubMotor::getSpeed()
{
    // readRealtimeData();
    return (float)real_data_.motor_speed / 10.0;
}

} // CFF