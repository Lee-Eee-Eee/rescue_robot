#include "robot.h"
#include "robot_driver.h"
#include "robot_function.h"
// #include "lcd.h"

using namespace CFF;

Robot_t robot;
Gamepad_t *gamepad;
HubMotorPort *motor_port;
ServoPort *servo_port;
static uint32_t tTime[10] = {0};

#if defined(ROBOT_CHASSIS_OMNI)
ChassisKinematics chassis(Chassis::FOUR_WHEEL_OMNI);
#elif defined(ROBOT_CHASSIS_THREE_WHEEL_OMNI)
ChassisKinematics chassis(Chassis::THREE_WHEEL_OMNI);
#elif defined(ROBOT_CHASSIS_TWO_DIFF)
ChassisKinematics chassis(Chassis::TOW_WHEEL_DIFFERENTIAL);
#elif defined(ROBOT_CHASSIS_FOUR_DIFF)
ChassisKinematics chassis(Chassis::FOUR_WHEEL_DIFFERENTIAL);
#endif

/**
 * @brief 初始化函数
 * 
 * Robot启动后只执行一次
 */
void setup()
{
    /* 获取Robot使用的控制对象指针 */
    motor_port = motor::PortPointer();
    servo_port = servo::PortPointer();
    gamepad = usb::GamepadPointer();
    
    /* 初始化Robot控制模式和底盘参数 */
    robot.ctrl_mode = INIT_CTRL_MODE;
#if defined(ROBOT_CHASSIS_OMNI)
    chassis.init(WHEELS_DIAMETER, OMNI4_WHEELS_TRACK, OMNI4_WHEELS_BASE, 300);
#elif defined(ROBOT_CHASSIS_THREE_WHEEL_OMNI)
    chassis.init(WHEELS_DIAMETER, OMNI3_CENTER_DISTANCE, 300);
#elif defined(ROBOT_CHASSIS_TWO_DIFF) || defined(ROBOT_CHASSIS_FOUR_DIFF)
    chassis.init(WHEELS_DIAMETER, DIFF_WHEELS_TRACK, DIFF_WHEELS_BASE, 300);
#endif

    /* 创建电机控制对象 */
    robot.motor[0] = new HubMotor(motor_port, LEFT_FRONT_MOTOR_ID);
    robot.motor[1] = new HubMotor(motor_port, RIGHT_FRONT_MOTOR_ID);
    robot.motor[2] = new HubMotor(motor_port, LEFT_BACK_MOTOR_ID);
    robot.motor[3] = new HubMotor(motor_port, RIGHT_BACK_MOTOR_ID);
    robot.motor[0]->setPid(1, 10, 0.5); // 默认PID pos_kp=1, speed_kp=5, speed_ki=1
    robot.motor[1]->setPid(1, 10, 0.5);
    robot.motor[2]->setPid(1, 10, 0.5);
    robot.motor[3]->setPid(1, 10, 0.5);
		
//		robot.motor[0]->setId(1);
		
    robot.servo[0] = new Servo(servo_port, 1);

    // robot.servo[0]->setZeroPosition();
    robot.servo[0]->setAngle(0);
    robot.servo[0]->setAcceleration(150);

    /* 机器人启动 */
    audio::playMusic("Startup", false);

    delay(1000); // 必要延时, 等待机器人其他设备稳定

    void auto_mode_path(void *parameter);
    auto_ctrl_init(&robot, auto_mode_path);
}

/**
 * @brief 循环函数
 * 
 * 会一直循环执行程序
 */
void loop()
{
    do_gamepad_connect();
    do_mode_choice(&robot, IS_BUTTON_PRESSED(gamepad->buttons, GAMEPAD_XBOX_BUTTON_HOME));

    switch (robot.ctrl_mode)
    {
    case INIT_CTRL_MODE:
        do_init_ctrl(&robot, gamepad);
        break;
    case MANUAL_CTRL_MODE:
        do_manual_ctrl(&robot, gamepad);
        break;
    case AUTO_CTRL_MODE:
        do_auto_ctrl(&robot, gamepad);
        break;
    default:
        break;
    }

    if ((millis()-tTime[0]) >= (1000 / DO_CHASSIS_CTRL_FREQUENCY))
    {
        tTime[0] = millis();
        do_chassis_ctrl(&robot, &chassis);
    }
    

    delay(5);
}

void auto_mode_path(void *parameter)
{
    while (1)
    {
        auto_wait_start(&robot);
        do_reset(&robot);
        if (robot.auto_state == 1)
        {
            auto_set_speed(&robot, 0.25, 0, 0, 2.8);
            robot.servo[0]->setAngle(-80);
            delay(500);
            auto_set_speed(&robot, 0, 0, -1, 1);
            robot.servo[0]->setAngle(-60);
            delay(300);
            io::setSG4(1, 150); //吹球
            delay(1000);
            io::setSG4(1, 0); //停
        }
        else if (robot.auto_state == 2)
        {
            /* code */
        }
        else if (robot.auto_state == 3)
        {
            /* code */
        }
        
    }
    
}