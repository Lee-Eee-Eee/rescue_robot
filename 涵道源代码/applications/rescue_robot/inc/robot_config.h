#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#define MAX_ACC                    0.2 // 最大加速度 m/s

#define WHEELS_DIAMETER            0.08 //轮子直径 unit: m

#define OMNI4_WHEELS_TRACK         0.133 //4轮全向底盘轮距 unit: m
#define OMNI4_WHEELS_BASE          0.133 //4轮全向底盘轴距 unit: m
#define OMNI3_CENTER_DISTANCE      0.094 //3轮全向底盘轮中心距（半径） unit: m
#define DIFF_WHEELS_TRACK          0.157 //差速底盘轮距 unit: m
#define DIFF_WHEELS_BASE           0.157 //差速底盘轴距 unit: m

/* 代码段运行频率设置 */
#define DO_CHASSIS_CTRL_FREQUENCY               50  // hz

/*
    ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
#define LEFT_FRONT_MOTOR_ID     1
#define RIGHT_FRONT_MOTOR_ID    2
#define LEFT_BACK_MOTOR_ID      3
#define RIGHT_BACK_MOTOR_ID     4

#endif /* __ROBOT_CONFIG_H__ */