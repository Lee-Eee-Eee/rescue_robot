#ifndef __CFF_BUS_SERVO_H__
#define __CFF_BUS_SERVO_H__

#include <stdio.h>
#include <board.h>
#include "easy_math.h"
#include "serial_bus.hpp"
#include "bus_servo_protocol.h"

#define TXBUFFER_MAX_LEN        (250)
#define RXBUFFER_MAX_LEN        (250)

#define SERVO_BROADCAST_ID  0xFE // 广播ID 254
#define SERVO_MAX_ID        0xFC // 最大ID 252
#define SCS_END             1
#define STS_END             0

namespace CFF
{

class Servo;

/**
 * @class ServoPort
 * @brief 用于处理与总线舵机通信的类
 */

class ServoPort : public SerialBUS
{
public:
    /** 总线舵机默认波特率。 */
    static const int DEFAULT_BAUDRATE_ = 1000000; // Default Baudrate
    
    /**
     * @brief 初始化总线舵机，给定端口名和波特率。
     * 
     * @param port_name 通信端口的名称。
     * @param baudrate 通信的波特率。
     */
    ServoPort(const char *port_name, int baudrate=DEFAULT_BAUDRATE_);

    /**
     * @brief 向指定ID的舵机发送ping请求并接收响应，如果有响应总线上存在该ID舵机。
     * 
     * @param id 舵机ID。
     * @param error 返回的错误码。
     * @return bool
     */
    bool ping(uint8_t id, uint8_t *error = NULL);

    /**
     * @brief 向指定ID的舵机的地址读取数据。
     * 
     * @param id 舵机ID。
     * @param address 开始读取的地址。
     * @param length 读取的字节数。
     * @param data 用于存储读取数据的缓冲区。
     * @param error 返回的错误码。
     * @return bool
     */
    bool readData(uint8_t id, uint8_t address, uint8_t length, uint8_t *data, uint8_t *error = NULL);

    /**
     * @brief 向指定ID的舵机的地址写入数据。
     * 
     * @param id 舵机ID。
     * @param address 写入的地址。
     * @param length 写入的字节数。
     * @param data 用于存储写入数据的缓冲区。
     * @param error 返回的错误码。
     * @return bool
     */
    bool writeData(uint8_t id, uint8_t address, uint8_t length, const uint8_t *data, uint8_t *error = NULL);

    /**
     * @brief 恢复出厂设置。
     * 
     * @param id 舵机ID。
     * @param error 返回的错误码。
     * @return bool
     */
    bool factoryReset(uint8_t id, uint8_t *error = NULL);

    /**
     * @brief 异步写指令，向地址中写入数据, 在收到regAction指令后才会执行该动作。
     * 
     * @param id 舵机ID。
     * @param address 写入的地址。
     * @param length 写入的字节数。
     * @param data 用于存储写入数据的缓冲区。
     * @param error 返回的错误码。
     * @return bool
     */
    bool regWrite(uint8_t id, uint8_t address, uint8_t length, const uint8_t *data, uint8_t *error = NULL);

    /**
     * @brief 执行异步写入的动作，与regWrite同时使用。
     * 
     * @param id id 舵机ID。
     * @param error error 返回的错误码。
     * @return bool 
     */
    bool regAction(uint8_t id = SERVO_BROADCAST_ID, uint8_t *error = NULL);

    /**
     * @brief 同步读指令。 (该方法目前未支持)
     * 
     * @param start_address 读取数据的起始地址
     * @param data_length 读取的数据的长度
     * @param param 同步读取的数据
     * @param param_length 同步读取的数据长度
     * @return bool
     */
    bool syncRead(uint8_t start_address, uint8_t data_length, const uint8_t *param, uint8_t param_length);

    /**
     * @brief 同步写指令，同时向写入多个舵机写入数据并执行运动。
     * 
     * @param start_address 写入数据的起始地址。
     * @param data_length 写入数据的长度。
     * @param param 同步写入的数据。
     * @param param_length 同步写入的数据长度。
     * @return bool
     */
    bool syncWrite(uint8_t start_address, uint8_t data_length, const uint8_t *param, uint8_t param_length);
    
    /**
     * @brief Get the Rx Packet Error object
     * 
     * @param error 舵机返回的错误码。
     * @return const char* 
     */
    const char *getRxPacketError(uint8_t error);

private:
    bool is_using_;
    uint8_t send_buffer_[TXBUFFER_MAX_LEN]={0};
    uint8_t recv_buffer_[RXBUFFER_MAX_LEN]={0};
};

/**
 * @brief 舵机类型
 * 
 */
enum ServoType
{
    Unknown = -1,
    SCS = 0,
    STS = 1,
    SMS = 2,
};

/**
 * @brief 舵机运行模式。
 * 
 */
enum ServoMode
{
    POSITION = 0,   /*!< 位置模式 */ 
    VELOCITY = 1,   /*!< 速度模式 */ 
    PWM = VELOCITY, /*!< 脉宽调制模式，SCS舵机才能使用 */ 
};

class Servo
{
public:
    /**
     * @brief Construct a new Servo object
     * 
     * @param port 舵机使用的端口。
     * @param id 舵机ID号，设置为SERVO_BROADCAST_ID发出的消息为广播。
     * @param type 舵机种类。 如果外标签标注为STxxx就是STS舵机。
     * @param max_angle 舵机编码器对应的最大角度。 (角度控制时做转换用)
     * @param min_angle 舵机编码器对应的最小角度。 (角度控制时做转换用)
     */
    Servo(ServoPort *port, uint8_t id, uint8_t type = ServoType::STS, float max_angle = 0, float min_angle = 0);
    virtual ~Servo() { }

    /**
     * @brief 获取舵机ID。
     * 
     * @return int 返回舵机ID。
     */
    int getId();

    /**
     * @brief 修改舵机ID。
     * 
     * @param new_id 新ID。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setId(uint8_t new_id);

    /**
     * @brief 获取舵机类型。
     * 
     * @return ServoType::SCS
     * @return ServoType::STS
     * @return ServoType::SMS
     */
    uint8_t getType();

    /**
     * @brief 获取舵机运行模式。
     * 
     * @return ServoMode::POSITION 位置模式
     * @return ServoMode::VELOCITY 速度模式
     * @return ServoMode::PWM 脉宽调制模式
     */
    int getMode();

    /**
     * @brief 设置舵机模式。
     * 
     * @param mode 舵机工作模式，从ServoMode中选择。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setMode(uint8_t mode);

    /**
     * @brief 舵机启动。
     * 
     * @return true 执行成功。
     * @return false 执行失败。
     */
    bool powerOn();

    /**
     * @brief 舵机停止。
     * 
     * @return true 执行成功。
     * @return false 执行失败。
     */
    bool powerOff();

    /**
     * @brief 获取舵机当前位置。
     * 
     * @return int 舵机编码器值。
     */
    int getPosition();

    /**
     * @brief 设置舵机目标位置。
     * 
     * @param pos 写入编码器值。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setPosition(int16_t pos);

    /**
     * @brief 设置零点偏移位置
     * 
     * @param pos 偏移的编码器值，输入为0自动将当前位置校正为2048。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setZeroPosition(int16_t pos=0);

    /**
     * @brief 获取最小编码器值。
     * 
     * @return int 返回最小编码器值。
     */
    int getMinPosition();

    /**
     * @brief 获取最大编码器值。
     * 
     * @return int 返回最大编码器值。
     */
    int getMaxPosition();

    /**
     * @brief 获取当前角度值(单位：度)。
     * 
     * @return float 返回角度值。
     */
    float getAngle();

    /**
     * @brief 设置目标旋转角度值(单位：度)。
     * 
     * @param angle 目标角度值，指明希望舵机旋转至的角度。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setAngle(float angle);

    /**
     * @brief 获取当前角度值(单位：弧度)。
     * 
     * @return float 
     */
    float getRadian();

    /**
     * @brief 设置目标旋转角度值(单位：弧度)。
     * 
     * @param rad 目标旋转角度的弧度值。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setRadian(float rad);

    /**
     * @brief 获取当前速度，速度单位为50步/秒=0.732RPM。
     * 
     * @return int 当前速度值。
     */
    int getVelocity();

    /**
     * @brief 设置目标速度，单位与getVelocity相同。
     * 
     * @param vel 目标速度值。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setVelocity(int16_t vel);

    /**
     * @brief 获取加速度，单位100步/秒^2。
     * 
     * @return int 
     */
    int getAcceleration();

    /**
     * @brief 设置加速度。
     * 
     * @param acc 加速度，单位100步/秒^2。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setAcceleration(uint8_t acc);

    /**
     * @brief 设置PWM脉宽时间。
     * 
     * @param time 
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setPwm(int16_t time);

    /**
     * @brief 获取当前占空比，单位为0.1%。
     * 
     * @return int 
     */
    int getLoad();

    /**
     * @brief 获取当前舵机工作电压，单位为0.1V。
     * 
     * @return int 
     */
    int getVoltage();

    /**
     * @brief 获取当前舵机内部工作温度，单位为1摄氏度。
     * 
     * @return int 
     */
    int getTemperature();

    /**
     * @brief 获取当前工作电流值，单位为6.5mA。
     * 
     * @return int 
     */
    int getElectricity();

    /**
     * @brief 读取PID值。
     * 
     * @param kp 比例系数。
     * @param ki 积分系数。
     * @param kd 微分系数。
     * @return true 读取成功。
     * @return false 读取失败。
     */
    bool getPID(uint8_t *kp, uint8_t *ki, uint8_t *kd);

    uint8_t getKp();
    uint8_t getKd();
    uint8_t getKi();

    /**
     * @brief 设置PID控制器参数。
     * 
     * @param kp 比例系数，设大此值会增强静锁力但运动生硬;设小此值会减弱静锁力但运动平滑。
     * @param ki 积分系数，设置此值可减小静态误差，值设置越大积分速度越快，此值设置不合理会有抖动。
     * @param kd 微分系数，设大此值会增强刹车力度，设小此值会减弱刹车力度。
     * @return true 设置成功。
     * @return false 设置失败。
     */
    bool setPID(uint8_t kp, uint8_t ki, uint8_t kd);
    bool setKp(uint8_t val);
    bool setKd(uint8_t val);
    bool setKi(uint8_t val);

    /**
     * @brief 查询舵机是否正在运行。
     * 
     * @return true 如果舵机正在运动中，则返回true。
     * @return false 如果舵机静止，则返回false。
     */
    bool isMoving();

    /**
     * @brief 等待舵机移动结束。
     * 
     * @param timeout_ms 超时时间。
     */
    void waitForMove(uint16_t timeout_ms=5000);

    /**
     * @brief 锁住存储器，重新上电写入存储器的数据不保存。
     * 
     * @return true 成功。
     * @return false 失败。
     */
    bool lockEprom();

    /**
     * @brief 解锁存储器，写入存储器的数据不丢失。
     * 
     * @return true 成功
     * @return false 失败
     */
    bool unlockEprom();

    ServoPort* getPort();
    bool setPort(ServoPort *port);
    
    /**
     * @brief 扫描总线，获取到有返回的ID号。
     * 
     * @param port 总线端口。
     * @param start_id 扫描起始ID。
     * @param end_id 扫描结束ID。
     * @return int 找到的ID号，返回0说明没有找到舵机。
     */
    static int scan(ServoPort *port, uint8_t start_id=1, uint8_t end_id=SERVO_MAX_ID);

    /**
     * @brief 给总线上所以舵机设置新ID，当忘记舵机ID时可以使用此函数。
     * 
     * @param port 总线端口。
     * @param new_id 设置新的ID。
     * @param type 
     * @return true 
     * @return false 
     */
    static bool resetAllId(ServoPort *port, uint8_t new_id=1, uint8_t type = ServoType::STS);

private:
    ServoPort *port_;       /*!< 总线端口 */
    uint8_t id_;            /*!< 舵机ID */
    uint8_t type_;          /*!< 舵机类型 */
    uint8_t mode_;          /*!< 舵机运行的模式 */
    uint8_t error_;         /*!< 舵机状态 */
    uint8_t scs_end_;       /*!< 数据大小端结构 */
    int16_t encoder_;       /*!< 编码器值 */
    int16_t min_encoder_;   /*!< 最小编码器 */
    int16_t max_encoder_;   /*!< 最大编码器 */
    float max_angle_;       /*!< 最大角度值 */
    float min_angle_;       /*!< 最小角度值 */
};

} // CFF

#endif // __CFF_BUS_SERVO_H__