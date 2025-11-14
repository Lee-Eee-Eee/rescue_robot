#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdint.h>
#include "robot_driver.h"
#include "chassis_kinematics.hpp"

enum CtrlMode
{
    INIT_CTRL_MODE = 0,
    MANUAL_CTRL_MODE = 1,
    AUTO_CTRL_MODE = 2,
};

enum ChassisMode
{
    STOP_MODE = 0,
    SPEED_MODE = 1,
    POSTION_MODE = 2,
};

enum TeamColor
{
    RED,
    BLUE
};

struct position
{
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
};

typedef struct _robot_
{
    uint8_t ctrl_mode;
    uint8_t chassis_mode;
    uint8_t team_color;
    uint8_t auto_state;  // 切换不同的自动模式
    
    struct position current_pos;    // chassis_mode=POSTION_MODE
    struct position target_pos;

    CFF::ChassisVelocities set_vel; // chassis_mode=SPEED_MODE

    struct switchgear SGs;
    HubMotor *motor[4];
    Servo *servo[3];
} Robot_t;


#endif /* __ROBOT_H__ */
