#ifndef __CFF_HUB_MOTOR_H__
#define __CFF_HUB_MOTOR_H__

#include <board.h>
#include "easy_math.h"
#include "serial_bus.hpp"

#define HM_SEND_HEADER              (0x3E)
#define HM_RECV_HEADER              (0x3C)

#define HM_FRAMEHEADER_LEN          sizeof(HM_FrameHeader_t)
#define HM_MODBUS_CRC_LEN(frame)    ((((HM_FrameHeader_t *)frame)->len) + HM_FRAMEHEADER_LEN)
#define HM_BUFFER_LEN(frame)        ((((HM_FrameHeader_t *)frame)->len) + HM_FRAMEHEADER_LEN+2)
#define HM_BUFFER_MAX_LEN           (68)
#define HM_FRAME_BUFFER_ADDR(frame) (((uint8_t *)frame) + HM_FRAMEHEADER_LEN)

#define HM_MEMBER_ADDRESS(struct_ptr, type, member_name) \
    ((char *)((type *)struct_ptr + offsetof(type, member_name) + sizeof(((type *)NULL)->member_name)))

#define HM_POS_TO_ANGLE(val)        ((val) * (360.0 / 16384.0))
#define HM_ANGLE_TO_VAL(angle)      ((angle) * (16384.0 / 360.0))

namespace CFF
{

/**
 * @brief HubMotorPort类，继承自SerialBUS，专门用于处理与轮毂电机相关的485总线通信。
 */
class HubMotorPort : public SerialBUS
{ 
public:
    /** 
     * @brief 默认485总线波特率设定。
     * 此值设为115200，适用于大多数情况下轮毂电机的通信需求。
     */
    static const int DEFAULT_BAUDRATE_ = 115200;

    /**
     * @brief 构造函数，初始化HubMotorPort实例。
     * 
     * @param port_name 串口设备名。
     * @param baudrate 串口波特率，默认使用DEFAULT_BAUDRATE_。
     * @param pin 可选，硬件流控制引脚号，-1表示不使用。
     * @param level 可选，流控制引脚的初始电平状态，默认PIN_HIGH。
     */
    HubMotorPort(const char *port_name, int baudrate=DEFAULT_BAUDRATE_, int pin = -1, int level = PIN_HIGH);
    
    /**
     * @brief 向指定设备写入数据。
     * 
     * @param device_id 目标设备ID。
     * @param cmd 控制命令。
     * @param data 待发送的数据指针。
     * @param len 数据长度。
     * @return 发送操作的状态码。
     */
    int writeData(uint8_t device_id, uint8_t cmd, const void *data, int16_t len);

    /**
     * @brief 从指定设备读取数据。
     * 
     * @param device_id 目标设备ID。
     * @param cmd 控制命令。
     * @param data 存储读取数据的缓冲区指针。
     * @param len 预期读取的数据长度。
     * @return 读取操作的状态码。
     */
    int readData(uint8_t device_id, uint8_t cmd, void *data, int16_t len);

    /**
     * @brief 执行命令并接收响应的综合方法。 包含写入命令、数据，然后读取响应的功能。
     * 
     * @param device_id 目标设备ID。
     * @param cmd 控制命令。
     * @param write_data 待发送的数据指针。
     * @param write_len 发送数据长度。
     * @param read_data 存储读取数据的缓冲区指针。
     * @param read_len 预期读取的数据长度。
     * @return 执行操作的状态码。
     */
    int execuit(uint8_t device_id, uint8_t cmd, void *write_data, int write_len, void *read_data, int read_len);
private:
    bool is_using_;                 /*!< 标记是否正在使用中，以避免并发访问冲突。*/
    uint8_t seq_num_;               /*!< 通讯序列号，用于保持消息顺序。*/
    uint8_t send_buffer_[HM_BUFFER_MAX_LEN]={0};    /*!< 发送数据缓冲区，大小由HM_BUFFER_MAX_LEN定义。*/
    uint8_t recv_buffer_[HM_BUFFER_MAX_LEN]={0};    /*!< 接收数据缓冲区，大小由HM_BUFFER_MAX_LEN定义。*/
};

/**
 * @brief  Hub Motor Control Command Enumerations
 */
enum HM_Command
{
    HM_CMD_GET_VERSION_INFO               = 0x0A,    /*!< 获取电机型号、软件版本、硬件版本等信息.         */
    HM_CMD_READ_REALTIME_DATA             = 0x0B,    /*!< 读取电机系统实时数据.                         */
    HM_CMD_READ_SYS_PARAMETERS            = 0x0C,    /*!< 读取电机保存的系统参数.                       */
    HM_CMD_WRITE_SYS_PARAMETERS           = 0x0D,    /*!< 写入系统参数到电机.                           */
    HM_CMD_SAVE_SYS_PARAMETERS            = 0x0E,    /*!< 保存系统参数到电机                            */
    HM_CMD_FACTORY_RESET                  = 0x0F,    /*!< 电机参数恢复出厂设置                          */
    HM_CMD_ENCODER_CALIBRATION            = 0x20,    /*!< 电机编码器校准                                */
    HM_CMD_SET_CURRENT_POSITION_AS_ZERO   = 0x21,    /*!< 设置电机当前位置为原点                         */
    HM_CMD_READ_ENCODER_DATA              = 0x2F,    /*!< 读取编码器单圈绝对值、多圈绝对值、速度实时数据   */
    HM_CMD_READ_STATUS                    = 0x40,    /*!< 读取电机状态信息（电压，电流，温度，故障码）     */
    HM_CMD_CLEAR_FAULTS                   = 0x41,    /*!< 清除电机故障码                                */
    HM_CMD_SHUTDOWN                       = 0x50,    /*!< 关闭电机，电机进入自由态不受控制                */
    HM_CMD_RETURN_TO_ZERO_POS             = 0x51,    /*!< 电机根据多圈绝对值角度，回到设定原点            */
    HM_CMD_SHORT_PATH_HOME                = 0x52,    /*!< 电机按照最短的距离回到设定的原点，旋转的角度不大于180度 */
    HM_CMD_OPEN_LOOP_CONTROL              = 0x53,    /*!< 电机开环控制                                  */
    HM_CMD_SPEED_CLOSED_LOOP              = 0x54,    /*!< 电机速度闭环控制                              */
    HM_CMD_ABSOLUTE_POS_CLOSED_LOOP       = 0x55,    /*!< 电机绝对值位置闭环控制                         */
    HM_CMD_RELATIVE_POS_CLOSED_LOOP       = 0x56,    /*!< 电机相对位置闭环控制                           */
    HM_CMD_POS_CTRL_TARGET_SPEED          = 0x57,    /*!< 位置闭环目标速度读取和配置                      */
    HM_CMD_LATEST
};

enum HM_Error
{
    HM_SUCCESS = 0,                 /*!< 成功，无错误 */
    HM_FAILURE,                     /*!< 失败 */
    HM_TX_FAIL,                     /*!< 发送错误 */
    HM_RX_FAIL,                     /*!< 读取错误 */
    HM_RX_WAITING,                  /*!< 读取异常 */
    HM_RX_TIMEOUT,                  /*!< 读取超时 */
    HM_PROTOCOL_MISMATCH,           /*!< 协议不匹配 */
    HM_INVALID_CMD,                 /*!< 无效命令码 */
    HM_PORT_BUSY,                   /*!< 端口阻塞 */
};

#pragma pack(push,1)

/**
 * @brief 定义通信帧头部结构体
 * 
 * 该结构体用于封装所有对外通信帧的固定头部信息。
 */
typedef struct
{
  rt_uint8_t header;
  rt_uint8_t seq;
  rt_uint8_t device_id;
  rt_uint8_t cmd;
  rt_uint8_t len;
} HM_FrameHeader_t;

/**
  * @brief  电机型号、软件版本、硬件版本等信息. 
  * 
  * 获取电机型号、电机软件版本号、硬件版本号等信息. 使用命令码：0x0A
  */
typedef struct
{
  uint16_t motor_model;              /*!< 电机型号标识. */
  uint8_t hardware_revision;         /*!< 硬件版本修订号. [Bit4:0]:硬件副版本号；[Bit7:5]:硬件主版本号. */
  uint8_t config_info;               /*!< 硬件配置详情.   [Bit0]: 0 设备地址不可软件配置; 1 设备地址可软件配置;
                                                         [Bit1]: 0 硬件没有CAN接口; 1 硬件具有CAN接口;
                                                         [Bit7:5]: 000常规版本; 001中孔版本; 010拓展版本 011H版本. */
  uint16_t software_revision;        /*!< 软件版本修订号. */
  uint8_t unique_mcu_id[12];         /*!< MCU唯一标识符. */
  uint16_t rs485_protocol_version;   /*!< RS485通信协议版本.  [Bit3:0]:RS485协议副版本号; [Bit7:4]:RS485协议主版本号. */
  uint16_t can_protocol_version;     /*!< CAN通信协议版本.  [Bit3:0]:CAN协议副版本号; [Bit7:4]:CAN协议主版本号. */
} HM_Info_t;

/**
  * @brief  电机系统实时数据. 
  * 
  * 读取电机系统实时数据（当前单圈绝对值角度、当前多圈绝对值角度、当前速度、当前
  * 电源电压、当前系统电流、当前系统温度、系统故障码） 
  * 使用命令码：0x0B，返回所有数据
  * 使用命令码：0x2F/0x50/0x51/0x52/0x53/0x54/0x55/0x56，返回单圈绝对值角度、当前多圈绝对值角度、当前速度
  * 使用命令码：0x40/0x41，返回当前电源电压、当前系统电流、当前系统温度、系统故障码
  */
typedef struct
{
  uint16_t single_turn_absolute;         /*!< 单圈绝对值. Angle°=val*(360/16384) */
  int32_t multi_turn_absolute;           /*!< 多圈绝对值. TotalAngle°=val*(360/16384) */
  int16_t motor_speed;                   /*!< 电机速度. 单位为0.1Rpm */
  uint8_t power_voltage;                 /*!< 电源电压. val*0.2(V) */
  uint8_t system_current;                /*!< 系统电流. val*0.03(A) */
  uint8_t system_temperature;            /*!< 系统温度. val*0.4(℃) */
  uint8_t system_fault_code;             /*!< 系统故障码.  [Bit1]:电压故障; [Bit2]:电流故障; [Bit3]:温度故障 */
  uint8_t motor_operation_status;        /*!< 电机运行状态.  0：关闭状态 1：开环模式 3：速度模式 5：位置模式 */
} HM_RealTimeData_t;

/**
  * @brief  电机系统参数. 
  * 
  * 使用命令码：0x0C，读取电机中系统参数
  * 使用命令码：0x0D，写入系统参数到电机，电机接收参数，但电机断电不保存
  * 使用命令码：0x0E，保存系统参数到电机，电机接收参数，并把参数永久保存到Flash中
  * 使用命令码：0x0F，电机参数恢复出厂设置；执行该命令，除设备地址及电机编码器校准数据不初始化外，其它参数均重置为系统默认值
  */
typedef struct
{
  uint8_t device_address;                  /*!< 设备地址 */
  uint8_t current_threshold;               /*!< 电流阈值，电机运行过程中电流大于电流阈值，系统将报电流故障 电流值/0.03; */
  uint8_t voltage_threshold;               /*!< 最大电压阈值，电机系统供电电压大于最大电压阈值，系统将报电压故障 电压值/0.2; */
  uint8_t communication_baud_rate;         /*!< 通信波特率， 字节的低4位表示RS485接口波特率，字节的高4位表示CAN接口波特率
                                                RS485接口波特率 0:表示波特率为115200; 
                                                               1:表示波特率为57600;
                                                               2:表示波特率为38400;
                                                               3:表示波特率为19200;
                                                               4:表示波特率为9600;
                                                CAN接口波特率   0:表示波特率为1MHz;
                                                               1:表示波特率为500KHz;
                                                               2:表示波特率为250KHz;
                                                               3:表示波特率为125KHz;
                                                               4:表示波特率为100KHz; */
  float position_loop_proportion_kp;       /*!< 位置环比例项Kp */
  float position_loop_target_speed;        /*!< 位置闭环目标速度 单位为0.1RPM */
  float speed_loop_proportion_kp;          /*!< 速度环比例项Kp */
  float speed_loop_integral_ki;            /*!< 速度环积分项Ki */
  float reserved;                          /*!< 预留参数 */
  uint8_t speed_filter_coefficient;        /*!< 速度滤波系数 */
  uint8_t motor_power_percentage;          /*!< 电机功率百分比 */

} HM_SystemParam_t;

#pragma pack(pop)

/**
 * @brief 轮毂电机控制类，封装了与轮毂电机通信及控制的相关方法。
 * 
 * 该类通过HubMotorPort接口与指定ID的电机进行交互，
 * 提供版本信息查询、实时数据读取、系统参数访问及多种控制命令的实现。
 */
class HubMotor
{
public:
    /**
     * @brief 构造函数，初始化轮毂电机对象。
     * 
     * @param port 指向轮毂电机通信端口的指针。
     * @param device_id 电机设备的ID。
     */
    HubMotor(HubMotorPort *port, uint8_t device_id);

    /**
     * @brief 获取电机版本信息。
     * 
     * @return HM_Info_t*
     */
    HM_Info_t *getVersionInfo();

    /**
     * @brief 读取电机实时运行数据。
     * 
     * @return HM_RealTimeData_t*
     */
    HM_RealTimeData_t *readRealtimeData();

    /**
     * @brief 读取电机系统参数。
     * 
     * @return HM_SystemParam_t*
     */
    HM_SystemParam_t *readSysParameters();

    /**
     * @brief 设置电机ID。
     * 
     * @param new_id 新的电机ID。
     * @return 设置成功返回true，否则false。
     */
    bool setId(uint8_t new_id);


    uint8_t getId();

    /**
     * @brief 执行出厂重置，恢复默认设置。
     * 
     * @return HM_SystemParam_t*
     */
    HM_SystemParam_t *factoryReset();

    /**
     * @brief 进行电机校准。
     * 
     * @return 成功返回true，否则false。
     */
    bool calibration();

    /**
     * @brief 将电机位置设定为零点。
     * 
     * @return true 
     * @return false 
     */
    bool setZero();

    /**
     * @brief 读取编码器数据，获取电机当前位置信息。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* readEncoder();

    /**
     * @brief 读取电机当前状态信息。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* readStatus();

    /**
     * @brief 清除电机错误或警告状态。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* clearStatus();

    /**
     * @brief 关闭电机电源，使其进入安全状态。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* shutdown();

    /**
     * @brief 使电机移动至零位。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* goToZero();

    /**
     * @brief 使电机移动至预设的绝对零位置。
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* goToHome();

    /**
     * @brief 开环控制模式下，设置电机输出功率
     * 
     * @param power 输出功率值。
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* openLoopControl(int16_t power);

    /**
     * @brief 设置电机的目标速度，闭环速度控制模式。
     * 
     * @param speed_rpm 目标速度，单位为RPM。
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* setSpeed(float speed_rpm);

    /**
     * @brief 设置电机目标绝对位置。
     * 
     * @param pos 目标位置值。
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* setPosition(uint32_t pos);

    /**
     * @brief 设置电机目标相对位置
     * 
     * @param pos 相对位置偏移值
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* setRelativePosition(uint32_t pos);

    /**
     * @brief 为位置控制器设置速度限制。
     * 
     * @param speed_rpm 速度限制值，单位为RPM。
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* setPosCtrlSpeed(float speed_rpm);

    /**
     * @brief 设置PID控制器参数。
     * 
     * @param pos_kp 位置环比例增益。
     * @param speed_kp 速度环比例增益。
     * @param speed_ki 速度环积分增益。
     * @param is_save 是否保存设置到非易失存储中，默认为false。
     * @return true 
     * @return false 
     */
    bool setPid(float pos_kp, float speed_kp, float speed_ki, bool is_save=false);

    /**
     * @brief 获取系统基本信息。（不进行通讯）
     * 
     * @return HM_Info_t* 
     */
    HM_Info_t * getInfo();

    /**
     * @brief 获取系统参数配置。（不进行通讯）
     * 
     * @return HM_SystemParam_t* 
     */
    HM_SystemParam_t* getSysParam();

    /**
     * @brief 获取电机的实时运行数据。（不进行通讯）
     * 
     * @return HM_RealTimeData_t* 
     */
    HM_RealTimeData_t* getRealData();

    /**
     * @brief 获取电机当前速度。（不进行通讯）
     * 
     * @return float 
     */
    float getSpeed();

    float getPowerVoltage();

    int comm_result_;               /*!< 保存通讯结果。*/

private:
    HubMotorPort *port_;            /*!< 指向轮毂电机通信端口的指针。*/
    uint8_t id_;                    /*!< 电机的ID。*/
    HM_Info_t info_;                /*!< 存储电机信息的结构体实例。*/
    HM_SystemParam_t sys_param_;    /*!< 存储电机系统参数的结构体实例。*/
    HM_RealTimeData_t real_data_;   /*!< 存储电机实时数据的结构体实例。*/
};


} // CFF

#endif // __CFF_HUB_MOTOR_H__

