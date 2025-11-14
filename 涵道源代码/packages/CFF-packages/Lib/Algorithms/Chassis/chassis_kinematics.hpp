#ifndef CHASSIS_KINEMATICS_H_
#define CHASSIS_KINEMATICS_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef __RTTHREAD__
#include <rtthread.h>
#define CHASSIS_ASSERT RT_ASSERT
#else
#define CHASSIS_ASSERT(EX) while(!(EX));
#endif

namespace CFF {

namespace Chassis {

/**
 * @brief 底盘类型
 * 
 */
enum ChassisTypes
{
    FOUR_WHEEL_OMNI = 0,            /*<! 四轮全向底盘 */
    THREE_WHEEL_OMNI,               /*<! 三轮全向底盘 */
    FOUR_WHEEL_DIFFERENTIAL,        /*<! 四轮差速底盘 */
    TOW_WHEEL_DIFFERENTIAL,         /*<! 两轮差速底盘 */
    ACKERMANN_STEERING,             /*<! 阿克曼底盘 */
    MECANUM_DRIVE = FOUR_WHEEL_OMNI /*<! 麦克纳姆轮底盘 */
};

enum ChassisWheel
{
    LEFT_FRONT_WHEEL = 0,
    RIGHT_FRONT_WHEEL = 1,
    LEFT_BACK_WHEEL = 2,
    RIGHT_BACK_WHEEL = 3,
    LEFT_WHEEL = LEFT_FRONT_WHEEL,
    RIGHT_WHEEL = RIGHT_FRONT_WHEEL,
    BACK_WHEEL = LEFT_BACK_WHEEL,
};
}

/**
 * @brief 底盘速度控制结构体
 */
struct ChassisVelocities
{
    volatile float linear_x;     /*<! 线性X轴速度(m/s) */
    volatile float linear_y;     /*<! 线性y轴速度(m/s) */
    volatile float angular_z;    /*<! 角速度(rad/s) */
};

/**
 * @brief 底盘里程计
 */
struct ChassisOdom
{
    volatile float x_pos;
    volatile float y_pos;
    volatile float heading;
};

struct ChassisParam
{
    volatile float wheels_diameter;          /*<! 单个轮子直径 */
    volatile float wheels_perimeter;         /*<! 轮子周长 */
    volatile float wheels_track;             /*<! 轮子间距离 */
    volatile float wheels_base;              /*<! 两个轴之间的距离 */
    volatile float wheels_center_distance;   /*<! 轮子到中心的距离 */
    volatile uint8_t total_wheels;           /*<! 车辆总轮子数 */
    volatile uint16_t wheels_speed_max;      /*<! 电机最大转速 */
};

/**
 * @class ChassisKinematics
 * @brief 用于实现底盘运动学计算，支持不同类型的底盘运动控制与状态检测。
 * 
 */
class ChassisKinematics
{
public:
    /**
     * @brief  构造函数，根据指定的底盘类型初始化。
     * 
     * @param type 底盘类型，定义于Chassis::ChassisTypes枚举中。
     */
    ChassisKinematics(Chassis::ChassisTypes type);

    /**
     * @brief 析构函数。
     */
    ~ChassisKinematics();

    /**
     * @brief 初始化底盘运动学参数。
     * 
     * @param wheels_diameter 轮子直径(m)。
     * @param wheels_track 轮距，即左右轮中心线间距离(m)。
     * @param wheels_base 底盘前后轮中心线间距离(m)。
     * @param wheels_speed_max 轮子最大速度(rpm)。
     */
    void init(float wheels_diameter, float wheels_track, float wheels_base, uint16_t wheels_speed_max);
    
    /**
     * @brief 初始化底盘运动学参数，适用于三轮底盘。
     * 
     * @param wheels_diameter 轮子直径(m)。
     * @param wheels_center_distance 轮子距离中心距离(半径)(m)。
     * @param wheels_speed_max 轮子最大速度(rpm)。
     */
    void init(float wheels_diameter, float wheels_center_distance, uint16_t wheels_speed_max);

    /**
     * @brief 应用运动指令，根据线性速度和角速度计算各轮速度。
     * 
     * @param linear_x 线性X轴速度(m/s)。
     * @param linear_y 线性Y轴速度(m/s)。全向底盘使用。
     * @param angular_z 角速度(rad/s)。
     * @return float* 
     */
    float *applyMotionCommand(float linear_x, float linear_y, float angular_z);

    /**
     * @brief 应用运动指令。
     * 
     * @param vel 包含线性与角速度的结构体指针。
     * @return float* 
     */
    float *applyMotionCommand(ChassisVelocities *vel);

    /**
     * @brief 根据轮速检测当前运动状态的速度。
     * 
     * @param wheels_speed 轮子实际转速数组[rpm]。
     * @return ChassisVelocities* 
     */
    ChassisVelocities *detectMovementSpeeds(float wheels_speed[]);

    /**
     * @brief 更新里程计信息。
     * 
     * @return struct ChassisOdom* 
     */
    struct ChassisOdom *updateOdom();

    /**
     * @brief 基于陀螺仪的航向角更新里程计信息。
     * 
     * @param gyro_yaw 陀螺仪测量的航向角(rad)。
     * @return struct ChassisOdom* 
     */
    struct ChassisOdom *updateOdom(float gyro_yaw);

    /**
     * @brief 重置里程计。
     */
    void resetOdom();

    /**
     * @brief 底盘类型。
     */
    Chassis::ChassisTypes type_;

    /**
     * @brief 各轮转速(rpm)。
     */
    float wheels_rpm_[4];

    /**
     * @brief 当前底盘速度。
     */
    ChassisVelocities current_vel_;

    /**
     * @brief 里程计数据。
     */
    ChassisOdom odom_;
    
private:
    /**
     * @brief 底盘参数集合。
     */
    ChassisParam param_;
    uint32_t now_time_;             /*<! 当前时间戳(ms) */
    uint32_t prev_time_;            /*<! 上次计算里程计时间(ms) */
};



} // namespace CFF

#endif /* CHASSIS_KINEMATICS_H_ */
