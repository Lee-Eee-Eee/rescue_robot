#ifndef __CFF_BUS_SERVO_PROTOCOL_H__
#define __CFF_BUS_SERVO_PROTOCOL_H__

/* for Protocol 1.0 Packet */
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

/* Protocol 1.0 Error bit */
#define ERRBIT_VOLTAGE          0x01    // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            0x02    // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         0x04    // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_OVERELE          0x08    // Command(setting value) is out of the range for use.
#define ERRBIT_OVERLOAD         0x20    // The current load cannot be controlled by the set torque.

/* 总线舵机指令码 */
#define INST_PING               0x01
#define INST_READ               0x02
#define INST_WRITE              0x03
#define INST_REG_WRITE          0x04
#define INST_REG_ACTION         0x05
#define INST_FACTORY_RESET      0x06
#define INST_SYNC_READ          0x82
#define INST_SYNC_WRITE         0x83

/* 波特率定义 */
#define	_1M 0
#define	_0_5M 1
#define	_250K 2
#define	_128K 3
#define	_115200 4
#define	_76800 5
#define	_57600 6
#define	_38400 7
#define	_19200 8
#define	_14400 9
#define	_9600 10
#define	_4800 11

/* 内存表定义 */
//-------EPROM(只读)--------
#define VERSION_MAJOR       3   // 主版本号，可以用来区分舵机类型
#define VERSION_MINOR       4   // 子版本号

//-------EPROM(读写)--------
#define SERVO_ID            5   // ID
#define BAUDRATE            6   // 波特率
#define MIN_ANGLE_LIMIT_L   9   // 最小角度
#define MIN_ANGLE_LIMIT_H   10
#define MAX_ANGLE_LIMIT_L   11  // 最大角度
#define MAX_ANGLE_LIMIT_H   12
#define SERVO_PID_KP        21
#define SERVO_PID_KD        22
#define SERVO_PID_KI        23
#define CW_DEAD             26  // 顺时针死区
#define CCW_DEAD            27  // 逆时针死区
#define SERVO_OFS_L         31  // 位置补偿值，用于改变中位值
#define SERVO_OFS_H         32
#define SERVO_MODE          33  // 运行模式：0 为位置伺服模式；1 为电机恒速模式

//-------SRAM(读写)--------
#define TORQUE_ENABLE       40  // 扭矩开关写 0：关闭扭力输出;写 1：打开扭力输出;
                                // 写 128：当前位置(56)较正为 2048,同时扭矩开关自动置 0
#define GOAL_POSITION_L     42  // 目标位置
#define GOAL_POSITION_H     43
#define GOAL_TIME_L         44  // 暂不支持
#define GOAL_TIME_H         45
#define GOAL_SPEED_L        46  // 运行速度
#define GOAL_SPEED_H        47
#define SCSCL_LOCK          48  // 写入锁标志 写 0 关闭写入锁,写入 EPROM 地址的值掉电不丢失；
                                // 写 1 打开写入锁,写入 EPROM
#define GOAL_ACC            41  // 加速度
#define SMS_STS_LOCK        55

//-------SRAM(只读)--------
#define PRESENT_POSITION_L  56  // 当前位置
#define PRESENT_POSITION_H  57
#define PRESENT_SPEED_L     58  // 当前速度
#define PRESENT_SPEED_H     59
#define PRESENT_LOAD_L      60  // 当前负载
#define PRESENT_LOAD_H      61
#define PRESENT_VOLTAGE     62  // 当前电压
#define PRESENT_TEMPERATURE 63  // 当前温度
#define SERVO_MOVING        66  // 移动标志 舵机运动时标志为 1,舵机停止运动时为 0
#define PRESENT_CURRENT_L   69  // 当前电流
#define PRESENT_CURRENT_H   70

#endif // __CFF_BUS_SERVO_PROTOCOL_H__