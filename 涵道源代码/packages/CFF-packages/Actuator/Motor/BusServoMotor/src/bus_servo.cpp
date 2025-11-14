#include "bus_servo.hpp"
#include "cff_utils.h"

#define DBG_TAG "Servo.motor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

namespace CFF
{

/**
 * @brief 用于构建舵机的发送数据包。函数设置设置数据包头、指令代码、数据（如提供）及计算校验和。
 * 
 * @param txpacket 指向用于构建数据包的传输缓冲区指针。
 * @param id 设备的ID。
 * @param func_code 指定操作的功能码。
 * @param address 操作（如读/写）的起始寄存器地址。
 * @param data 指向随数据包发送的数据数组的指针，可以为NULL，表示无需发送数据。
 * @param length 提供的数据数组的长度。
 * @return int 数据包总长度
 */
static int send_pack(uint8_t *txpacket, uint8_t id, uint8_t func_code, uint8_t address, const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    uint8_t total_packet_length = 0;
    txpacket[PKT_HEADER0] = 0xFF;
    txpacket[PKT_HEADER1] = 0xFF;
    txpacket[PKT_ID] = id;
    txpacket[PKT_INSTRUCTION] = func_code;
    
    if (data)
    {
        txpacket[PKT_LENGTH] = length + 3;
        txpacket[PKT_PARAMETER0] = address;

        /* 将数据复制到数据包中 */
        for (uint8_t i=0; i<length; i++)
        {
            txpacket[PKT_PARAMETER0+1+i] = data[i];
        }
    }
    else
    {
        /* 若无数据，设置数据包长度。*/
        txpacket[PKT_LENGTH] = 2;
    }
    total_packet_length = txpacket[PKT_LENGTH] + 4;

    // add a checksum to the packet
    for (uint16_t idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
        checksum += txpacket[idx];
    txpacket[total_packet_length - 1] = ~checksum;

    return total_packet_length;
}

/**
 * @brief 用于接收和解析数据包
 * 
 * @param rxpacket 指向接收到的数据包的缓冲区指针。
 * @param id 设备ID，用于验证数据包来源。
 * @param rx_len 接收到的数据包的总长度。
 * @param error 指向一个字节的指针，用于输出错误代码，如无错误则通常为0。
 * @param data 一个通用指针，用于输出解析出的数据，具体类型和长度需根据调用者的需求赋值。
 * @return true 
 * @return false 
 */
static bool recv_unpack(uint8_t *rxpacket, uint8_t id, uint8_t rx_len, uint8_t *error, void *data)
{
    uint8_t checksum = 0;
    uint8_t idx = 0;

    if (rx_len >= 6 && rxpacket[PKT_HEADER0] == 0xFF && rxpacket[PKT_HEADER1] == 0xFF)
    {
        if (rxpacket[PKT_ID] != id)
        {
            return(false);
        }
        // calculate checksum
        for (uint16_t i = 2; i < (rx_len - 1); i++)   // except header, checksum
            checksum += rxpacket[i];
        checksum = ~checksum;
        if (checksum == rxpacket[rx_len-1])
        {
            if (error != NULL)
            {
                *error = (uint8_t)rxpacket[PKT_ERROR];
            }
            if (data != NULL && rxpacket[PKT_LENGTH] > 2)
            {
                memcpy(data, &rxpacket[PKT_PARAMETER0], rxpacket[PKT_LENGTH]-2);
            }
        }
        else
        {
            return(false);
        }
    }
    else
    {
        return(false);
    }
    return(true);
}

ServoPort::ServoPort(const char *port_name, int baudrate) :
    SerialBUS(port_name, baudrate, -1, 0),
    is_using_(false)
{ }

bool ServoPort::ping(uint8_t id, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_ || id == SERVO_BROADCAST_ID)
        return false;
    is_using_ = true;

    tx_len = send_pack(send_buffer_, id, INST_PING, 0, NULL, 0);
    rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
    if (rx_len > 0)
    {
        recv_unpack(recv_buffer_, id, rx_len, error, NULL);
    }
    else
    {
        is_using_ = false;
        return false;
    }

    is_using_ = false;
    return true;
}

bool ServoPort::readData(uint8_t id, uint8_t address, uint8_t length, uint8_t *data, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_ || id == SERVO_BROADCAST_ID)
        return false;
    is_using_ = true;

    tx_len = send_pack(send_buffer_, id, INST_READ, address, &length, 1);

    rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
    if (rx_len > 0)
    {
        recv_unpack(recv_buffer_, id, rx_len, error, data);
    }
    else
    {
        is_using_ = false;
        return false;
    }
    
    is_using_ = false;
    return true;
}

bool ServoPort::writeData(uint8_t id, uint8_t address, uint8_t length, const uint8_t *data, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_)
        return false;
    is_using_ = true;

    tx_len = send_pack(send_buffer_, id, INST_WRITE, address, data, length);
    if (id == SERVO_BROADCAST_ID) // 广播消息
    {
        rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, NULL, 0);
    }
    else
    {
        rx_len = serial_bus_send_then_recv(bus_,send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
        if (rx_len > 0)
        {
            recv_unpack(recv_buffer_, id, rx_len, error, NULL);
        }
        else
        {
            is_using_ = false;
            return false;
        }
    }
    is_using_ = false;
    return true;
}

bool ServoPort::factoryReset(uint8_t id, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_ || id == SERVO_BROADCAST_ID)
        return false;
    is_using_ = true;

    tx_len = send_pack(send_buffer_, id, INST_FACTORY_RESET, 0, NULL, 0);

    rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
    if (rx_len > 0)
    {
        recv_unpack(recv_buffer_, id, rx_len, error, NULL);
    }
    else
    {
        is_using_ = false;
        return false;
    }
    
    is_using_ = false;
    return true;
}

bool ServoPort::regWrite(uint8_t id, uint8_t address, uint8_t length, const uint8_t *data, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_ || id == SERVO_BROADCAST_ID)
        return false;
    is_using_ = true;

    tx_len = send_pack(send_buffer_, id, INST_REG_WRITE, address, data, length);
    rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
    if (rx_len > 0)
    {
        recv_unpack(recv_buffer_, id, rx_len, error, NULL);
    }
    else
    {
        is_using_ = false;
        return false;
    }
    
    is_using_ = false;

    return true;
}

bool ServoPort::regAction(uint8_t id, uint8_t *error)
{
    uint8_t rx_len = 0;
    uint8_t tx_len = 0;

    if (is_using_)
        return false;
    is_using_ = true;
    
    tx_len = send_pack(send_buffer_, id, INST_REG_ACTION, 0, NULL, 0);
    if (id == SERVO_BROADCAST_ID) // 广播消息
    {
        rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, NULL, 0);
    }
    else
    {
        rx_len = serial_bus_send_then_recv(bus_, send_buffer_, tx_len, recv_buffer_, TXBUFFER_MAX_LEN);
        if (rx_len > 0)
        {
            recv_unpack(recv_buffer_, id, rx_len, error, NULL);
        }
        else
        {
            is_using_ = false;
            return false;
        }
    }
    is_using_ = false;
    return true;
}

bool ServoPort::syncRead(uint8_t start_address, uint8_t data_length, const uint8_t *param, uint8_t param_length)
{
    LOG_D("This function cannot be used.");
    return false;
}


bool ServoPort::syncWrite(uint8_t start_address, uint8_t data_length, const uint8_t *param, uint8_t param_length)
{
    uint8_t checksum = 0;
    uint8_t total_packet_length = 0;

    if (is_using_)
        return false;
    is_using_ = true;

    send_buffer_[PKT_HEADER0] = 0xFF;
    send_buffer_[PKT_HEADER1] = 0xFF;
    send_buffer_[PKT_ID] = SERVO_BROADCAST_ID;
    send_buffer_[PKT_INSTRUCTION] = INST_SYNC_WRITE;
    send_buffer_[PKT_LENGTH] = param_length + 4;
    send_buffer_[PKT_PARAMETER0+0] = start_address;
    send_buffer_[PKT_PARAMETER0+1] = data_length;

    for (uint16_t s = 0; s < param_length; s++)
        send_buffer_[PKT_PARAMETER0+2+s] = param[s];

    total_packet_length = send_buffer_[PKT_LENGTH] + 4;

    // add a checksum to the packet
    for (uint16_t idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
        checksum += send_buffer_[idx];
    send_buffer_[total_packet_length - 1] = ~checksum;
    
    serial_bus_send_then_recv(bus_, send_buffer_, total_packet_length, NULL, 0);

    is_using_ = false;

    return true;
}

const char *ServoPort::getRxPacketError(uint8_t error)
{
  if (error & ERRBIT_VOLTAGE)
    return "[RxError] Input voltage error!";

  if (error & ERRBIT_ANGLE)
    return "[RxError] Angle sen error!";

  if (error & ERRBIT_OVERHEAT)
    return "[RxError] Overheat error!";

  if (error & ERRBIT_OVERHEAT)
    return "[RxError] OverEle error!";

  if (error & ERRBIT_OVERLOAD)
    return "[RxError] Overload error!";

  return "";
}

Servo::Servo(ServoPort *port, uint8_t id, uint8_t type, float max_angle, float min_angle) : 
    port_(port),
    id_(id),
    type_(type),
    mode_(ServoMode::POSITION),
    max_angle_(max_angle), min_angle_(min_angle),
    encoder_(2048), min_encoder_(-1), max_encoder_(-1),
    error_(0), scs_end_(STS_END)
{
    if (port_ == NULL)
    {
        LOG_E("The port is not a valid object.");
    }

    if (type_ == ServoType::SCS)
    {
        scs_end_ = SCS_END;
    }

    if (max_angle_ == min_angle_)
    {
        LOG_E("Maximum Angle (%d) and minimum Angle (%d) cannot be set to the same value.", max_angle_, min_angle_);
    }
}

int Servo::getId()
{
    return id_;
}

bool Servo::setId(uint8_t new_id)
{
    if (!unlockEprom())
        return false;

    if (port_->writeData(id_, SERVO_ID, 1, &new_id, &error_))
    {
        id_ = new_id;
        return true;
    }
    else
    {
        return false;
    }
}

int Servo::getMode()
{
    if (type_ == ServoType::STS || type_ == ServoType::SMS)
    {
        port_->readData(id_, SERVO_MODE, 1, &mode_, &error_);
    }
    else
    {
        uint8_t data[4];
        int16_t max_encoder = 0;
        int16_t min_encoder = 0;
        if (port_->readData(id_, MIN_ANGLE_LIMIT_L, 4, data, &error_))
        {
            min_encoder = (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
            max_encoder = (scs_end_ == STS_END) ? makeWord(data[3], data[2]) : makeWord(data[2], data[3]);

            if (min_encoder == 0 && max_encoder == 0)
            {
                mode_ = ServoMode::PWM;
            }
            else
            {
                mode_ = ServoMode::POSITION;
            }
        }
    }
    return mode_;
}


bool Servo::setMode(uint8_t mode)
{
    if (type_ == ServoType::STS || type_ == ServoType::SMS)
    {
        if (port_->writeData(id_, SERVO_MODE, 1, &mode, &error_))
        {
            mode_ = mode;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (mode == ServoMode::PWM)
        {
            uint8_t data[4] = {0, 0, 0, 0};
            if (port_->writeData(id_, MIN_ANGLE_LIMIT_L, 4, data, &error_))
            {
                mode_ = ServoMode::PWM;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            uint8_t data[4] = {0};
            if (scs_end_ == STS_END)
            {
                data[0] = lowByte(min_encoder_);
                data[1] = highByte(min_encoder_);
                data[2] = lowByte(max_encoder_);
                data[3] = highByte(max_encoder_);
            }
            else
            {
                data[0] = highByte(min_encoder_);
                data[1] = lowByte(min_encoder_);
                data[2] = highByte(max_encoder_);
                data[3] = lowByte(max_encoder_);
            }
            if (port_->writeData(id_, MIN_ANGLE_LIMIT_L, 4, data, &error_))
            {
                mode_ = ServoMode::POSITION;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}

bool Servo::powerOn()
{
    uint8_t data[1] = {1};
    return port_->writeData(id_, TORQUE_ENABLE, 1, data, &error_);
}

bool Servo::powerOff()
{
    uint8_t data[1] = {0};
    return port_->writeData(id_, TORQUE_ENABLE, 1, data, &error_);
}


int Servo::getPosition()
{
    uint8_t data[2];
    if (port_->readData(id_, PRESENT_POSITION_L, 2, data, &error_))
    {
        encoder_ = (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
        return encoder_;
    }
    return -1;
}

bool Servo::setPosition(int16_t pos)
{
    uint8_t data[2];
    
    if (mode_ != ServoMode::POSITION)
    {
        LOG_W("Mode is not POSITION.");
        return false;
    }

    if (min_encoder_ == max_encoder_)
    {
        LOG_D("Update the encoder maximum and minimum values.");
        getMinPosition();
        getMaxPosition();
    }

    if (pos < min_encoder_ || pos > max_encoder_)
    {
        LOG_W("Goal position out of range.");
        return false;
    }

    if (scs_end_ == STS_END)
    {
        data[0] = lowByte(pos);
        data[1] = highByte(pos);
    }
    else
    {
        data[0] = highByte(pos);
        data[1] = lowByte(pos);
    }
    return port_->writeData(id_, GOAL_POSITION_L, 2, data, &error_);
}

bool Servo::setZeroPosition(int16_t pos)
{
    if (pos == 0)
    {
        uint8_t data[1] = {128};
        return port_->writeData(id_, TORQUE_ENABLE, 1, data, &error_);
    }
    else
    {
        uint8_t data[2];
        if(pos < 0)
        {
            pos = -pos;
            bitSet(pos, 15);
        }
        if (scs_end_ == STS_END)
        {
            data[0] = lowByte(pos);
            data[1] = highByte(pos);
        }
        else
        {
            data[0] = highByte(pos);
            data[1] = lowByte(pos);
        }
        return port_->writeData(id_, SERVO_OFS_L, 2, data, &error_);
    }
}

int Servo::getMinPosition()
{
    uint8_t data[2];
    if (port_->readData(id_, MIN_ANGLE_LIMIT_L, 2, data, &error_))
    {
        min_encoder_ = (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
        return min_encoder_;
    }
    return -1;
}

int Servo::getMaxPosition()
{
    uint8_t data[2];
    if (port_->readData(id_, MAX_ANGLE_LIMIT_L, 2, data, &error_))
    {
        max_encoder_ = (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
        return max_encoder_;
    }
    return -1; 
}

float Servo::getAngle()
{
    int16_t pos = getPosition();
    if (pos == -1)
        return 0;
    
    return fmap(pos, min_encoder_, max_encoder_, min_angle_, max_angle_);
}

bool Servo::setAngle(float angle)
{
    if (angle < min_angle_ || angle > max_angle_)
    {
        LOG_W("Goal angle out of range.");
        return false;
    }
    
    int16_t pos = fmap(angle, min_angle_, max_angle_, min_encoder_, max_encoder_);
    return setPosition(pos);
}

float Servo::getRadian()
{
    return radians(getAngle());
}

bool Servo::setRadian(float rad)
{
    if (rad < -PI || rad > PI)
    {
        LOG_W("Goal radian out of range.");
        return false;
    }

    return setAngle(degrees(rad));
}

int Servo::getVelocity()
{
    uint8_t data[2];
    int16_t vel;
    if (port_->readData(id_, PRESENT_SPEED_L, 2, (uint8_t*)data, &error_))
    {
        vel = (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
        
        if (bitRead(vel, 15)){
            vel = -bitClear(vel, 15);
        }
        return vel;
    }
    return -1;
}

bool Servo::setVelocity(int16_t vel)
{
    uint8_t data[2];
    if (mode_ != ServoMode::VELOCITY)
    {
        LOG_W("Mode is not VELOCITY.");
        return false;
    }

    if(vel < 0)
    {
        vel = -vel;
        bitSet(vel, 15);
    }
    if (scs_end_ == STS_END)
    {
        data[0] = lowByte(vel);
        data[1] = highByte(vel);
    }
    else
    {
        data[0] = highByte(vel);
        data[1] = lowByte(vel);
    }
    return port_->writeData(id_, GOAL_SPEED_L, 2, data, &error_);
}

int Servo::getAcceleration()
{
    int8_t acc = 0;
    if (type_ == ServoType::SCS)
    {
        LOG_W("Servo type does not support acceleration.");
        return 0;
    }
    if (port_->readData(id_, GOAL_ACC, 1, (uint8_t*)&acc, &error_))
        return acc;
    return 0;
}

bool Servo::setAcceleration(uint8_t acc)
{
    if (type_ == ServoType::SCS)
    {
        LOG_W("Servo type does not support acceleration.");
        return 0;
    }
    return port_->writeData(id_, GOAL_ACC, 1, (uint8_t*)&acc, &error_);
}

bool Servo::setPwm(uint16_t time)
{
    if(time<0)
    {
		time = -time;
        bitSet(time, 10);
	}
    if (type_ != ServoType::SCS)
    {
        LOG_W("Servo type does not support PWM.");
        return false;
    }

	int8_t data[2];
    return port_->writeData(id_, GOAL_TIME_L, 2, (uint8_t*)data, &error_);
}

int Servo::getLoad()
{
    int8_t data[2];
    if (port_->readData(id_, PRESENT_LOAD_L, 2, (uint8_t *)data, &error_))
    {
        return (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
    }
    return 0;
}

int Servo::getVoltage()
{
    int8_t vol;
    if (port_->readData(id_, PRESENT_VOLTAGE, 1, (uint8_t*)&vol, &error_))
        return vol;
    return 0;
}

int Servo::getTemperature()
{
    int8_t temp;
    if (port_->readData(id_, PRESENT_TEMPERATURE, 1, (uint8_t*)&temp, &error_))
        return temp;
    return 0;
}

int Servo::getElectricity()
{
    int8_t data[2];
    if (port_->readData(id_, PRESENT_LOAD_L, 2, (uint8_t*)data, &error_))
    {
        return (scs_end_ == STS_END) ? makeWord(data[1], data[0]) : makeWord(data[0], data[1]);
    }
    return 0;
}

uint8_t Servo::getKp()
{
    int8_t val;
    if (port_->readData(id_, SERVO_PID_KP, 1, (uint8_t*)&val, &error_))
        return val;
    return 0;
}

bool Servo::setKp(uint8_t val)
{
    return port_->writeData(id_, SERVO_PID_KP, 1, (uint8_t*)&val, &error_);
}

uint8_t Servo::getKd()
{
    int8_t val;
    if (port_->readData(id_, SERVO_PID_KD, 1, (uint8_t*)&val, &error_))
        return val;
    return 0;
}

bool Servo::setKd(uint8_t val)
{
    return port_->writeData(id_, SERVO_PID_KD, 1, (uint8_t*)&val, &error_);
}

uint8_t Servo::getKi()
{
    int8_t val;
    if (port_->readData(id_, SERVO_PID_KI, 1, (uint8_t*)&val, &error_))
        return val;
    return 0;
}

bool Servo::setKi(uint8_t val)
{
    return port_->writeData(id_, SERVO_PID_KI, 1, (uint8_t*)&val, &error_);
}

bool Servo::getPID(uint8_t *kp, uint8_t *ki, uint8_t *kd)
{
    uint8_t data[3];
    if (port_->readData(id_, SERVO_PID_KP, 3, (uint8_t*)data, &error_))
    {
        *kp = data[0];
        *kd = data[1];
        *ki = data[2];
        return true;
    }
    return false;
}

bool Servo::setPID(uint8_t kp, uint8_t ki, uint8_t kd)
{
    uint8_t data[3] = {kp, kd, ki};
    return port_->writeData(id_, SERVO_PID_KP, 3, (uint8_t*)data, &error_);
}

bool Servo::isMoving()
{
    int8_t move;
    if (port_->readData(id_, SERVO_MOVING, 1, (uint8_t*)&move, &error_))
        return move ? true : false;
    return false;
}

void Servo::waitForMove(uint16_t timeout_ms)
{
    bool move;
    int t1 = millis();
    while (true)
    {
        delay(10);
        move = isMoving();
        if((millis() - t1) >= timeout_ms || !move) 
        {
            return;
        }
    }
}

bool Servo::lockEprom()
{
    uint8_t lock = 1;
    if (type_ == ServoType::SCS)
    {
        return port_->writeData(id_, SCSCL_LOCK, 1, (uint8_t*)&lock, &error_);
    }
    else if (type_ == ServoType::STS || type_ == ServoType::SMS)
    {
        return port_->writeData(id_, SMS_STS_LOCK, 1, (uint8_t*)&lock, &error_);
    }
    else
    {
        LOG_W("Unsupported servo type encountered.");
        return false;
    }
}

bool Servo::unlockEprom()
{
    uint8_t lock = 0;
    if (type_ == ServoType::SCS)
    {
        return port_->writeData(id_, SCSCL_LOCK, 1, &lock, &error_);
    }
    else if (type_ == ServoType::STS || type_ == ServoType::SMS)
    {
        return port_->writeData(id_, SMS_STS_LOCK, 1, &lock, &error_);
    }
    else
    {
        LOG_W("Unsupported servo type encountered.");
        return false;
    }
}

ServoPort* Servo::getPort()
{
    return port_;
}

bool Servo::setPort(ServoPort *port)
{
    if (port == NULL)
    {
        LOG_E("The port is not a valid object.");
        return false;
    }
    port_ = port;
    return true;
}

int Servo::scan(ServoPort * port, uint8_t start_id, uint8_t end_id)
{
    for (uint8_t i=start_id; i<=end_id; i++)
    {
        if (port->ping(i))
            return i;
        delay(10);
    }
    return 0;
}


bool Servo::resetAllId(ServoPort * port, uint8_t new_id, uint8_t type)
{
    uint8_t lock = 0;
    if (type == ServoType::SCS)
    {
        port->writeData(SERVO_BROADCAST_ID, SCSCL_LOCK, 1, &lock, NULL);
    }
    else if (type == ServoType::STS || type == ServoType::SMS)
    {
        port->writeData(SERVO_BROADCAST_ID, SMS_STS_LOCK, 1, &lock, NULL);
    }
    else
    {
        LOG_W("Unsupported servo type encountered.");
        return false;
    }
    port->writeData(SERVO_BROADCAST_ID, SERVO_ID, 1, &new_id, NULL);
    return true;
}


} // CFF
