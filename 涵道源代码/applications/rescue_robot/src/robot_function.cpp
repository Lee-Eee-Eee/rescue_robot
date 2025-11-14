#include "robot_function.h"
#include "robot_driver.h"

/**
 * @brief 检测手柄是否链接, 手柄连接与拔出播放提示音
 * 
 */
void do_gamepad_connect()
{
    static bool conn_state = false;
    if (usb::isConnected() != conn_state)
    {
        if (conn_state == false)
        {
            audio::playMusic("Connect", false);
            conn_state = true;
        }
        else
        {
            audio::playMusic("Disconnect", false);
            conn_state = false;
        }
    }
}

/**
 * @brief 控制 Robot 模式的切换, 切换到不同模式都有不同提示音
 * 
 * @param robot 
 * @param toggle_state 
 */
void do_mode_choice(Robot_t *robot, bool toggle_state)
{
    static bool last_state = false;
    switch (robot->ctrl_mode)
    {
    case INIT_CTRL_MODE:
        io::setLedColor(LED_COLOR_WHITE);
        if (toggle_state && last_state != toggle_state) // 做一个手柄连接的保护
        {
            audio::playMusic("ManualCtrl", false);
            robot->ctrl_mode = MANUAL_CTRL_MODE;
        }
        break;
    case MANUAL_CTRL_MODE:
        io::setLedColor(LED_COLOR_RED);
        if (toggle_state && last_state != toggle_state)
        {
            audio::playMusic("AutoCtrl", false);
            robot->ctrl_mode = AUTO_CTRL_MODE;
        }
        break;
    case AUTO_CTRL_MODE:
        io::setLedColor(LED_COLOR_BLUE);
        if (toggle_state && last_state != toggle_state)
        {
            audio::playMusic("ManualCtrl", false);
            robot->ctrl_mode = MANUAL_CTRL_MODE;
        }
        break;
    default:
        break;
    }
    if (!usb::isConnected() && robot->ctrl_mode != INIT_CTRL_MODE)
    {
        robot->ctrl_mode = INIT_CTRL_MODE;
    }
    last_state = toggle_state;
}

/**
 * @brief 加速函数, 是控制更平滑 (测试中)
 * 
 * @param speed_x
 * @param speed_y 
 */
void do_acc(float *speed_x, float *speed_y)
{
    static float last_speed_x = 0;
    static float last_speed_y = 0;

    float acc_x = 0;
    float acc_y = 0;
    float acc_whole = 0;
    float sin_x = 0;
    float sin_y = 0;
    float tmp_float;
    float tmp_speed_x = *speed_x;
    float tmp_speed_y = *speed_y;

    acc_x = tmp_speed_x - last_speed_x;
    acc_y = tmp_speed_y - last_speed_y;
    acc_whole = acc_x * acc_x + acc_y * acc_y ;

    arm_sqrt_f32(acc_whole, &tmp_float);
    acc_whole = tmp_float + 0.001f;

    sin_x = acc_x / acc_whole;
    sin_y = acc_y / acc_whole;
    

    if(acc_whole > MAX_ACC)
    {
        acc_whole = MAX_ACC;
        acc_x = acc_whole * sin_x;
        acc_y = acc_whole * sin_y;
        *speed_x = last_speed_x + acc_x;
        *speed_y = last_speed_y + acc_y;
        
    }
//    printf("acc_whole %f\r\n", (acc_whole));
    
    last_speed_x = tmp_speed_x;
    last_speed_y = tmp_speed_y;
}

/**
 * @brief 控制底盘运动解算和电机速度
 * 
 * @param robot 
 * @param chassis 
 */
void do_chassis_stop_ctrl(Robot_t *robot, ChassisKinematics *chassis)
{
    float motor_speed[4];

    if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI)
    {
        robot->motor[0]->setSpeed(0);
        robot->motor[1]->setSpeed(0);
        robot->motor[2]->setSpeed(0);
        robot->motor[3]->setSpeed(0);
    }
    else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI)
    {
        robot->motor[0]->setSpeed(0);
        robot->motor[1]->setSpeed(0);
        robot->motor[2]->setSpeed(0);
    }
    else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL)
    {
        robot->motor[0]->setSpeed(0);
        robot->motor[1]->setSpeed(0);
    }
}

void do_chassis_speed_ctrl(Robot_t *robot, ChassisKinematics *chassis)
{
    chassis->applyMotionCommand(&robot->set_vel);
    if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI)
    {
        robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_FRONT_WHEEL]);
        robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_FRONT_WHEEL]);
        robot->motor[2]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_BACK_WHEEL]);
        robot->motor[3]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_BACK_WHEEL]);
    }
    else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI)
    {
        robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_WHEEL]);
        robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_WHEEL]);
        robot->motor[2]->setSpeed(chassis->wheels_rpm_[Chassis::BACK_WHEEL]);
    }
    else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL)
    {
        robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_WHEEL]);
        robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_WHEEL]);
    }
}

void do_chassis_position_ctrl(Robot_t *robot, ChassisKinematics *chassis)
{

}

void do_chassis_detect_speeds(Robot_t *robot, ChassisKinematics *chassis)
{
    float motor_speed[4];
    if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI)
    {
        motor_speed[Chassis::LEFT_FRONT_WHEEL] = robot->motor[0]->getSpeed();
        motor_speed[Chassis::RIGHT_FRONT_WHEEL] = robot->motor[1]->getSpeed();
        motor_speed[Chassis::LEFT_BACK_WHEEL] = robot->motor[2]->getSpeed();
        motor_speed[Chassis::RIGHT_BACK_WHEEL] = robot->motor[3]->getSpeed();
    }
    else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI)
    {
        motor_speed[Chassis::LEFT_WHEEL] = robot->motor[0]->getSpeed();
        motor_speed[Chassis::RIGHT_WHEEL] = robot->motor[1]->getSpeed();
        motor_speed[Chassis::BACK_WHEEL] = robot->motor[2]->getSpeed();
    }
    else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL)
    {
        motor_speed[Chassis::LEFT_WHEEL] = robot->motor[0]->getSpeed();
        motor_speed[Chassis::RIGHT_WHEEL] = robot->motor[1]->getSpeed();
    }

    chassis->detectMovementSpeeds(motor_speed);
    chassis->updateOdom();
}

void do_chassis_ctrl(Robot_t *robot, ChassisKinematics *chassis)
{
    // do_acc(&robot.set_vel.linear_x, &robot.set_vel.linear_y);
    switch (robot->chassis_mode)
    {
    case STOP_MODE:
        do_chassis_stop_ctrl(robot, chassis);
        break;
    case SPEED_MODE:
        do_chassis_speed_ctrl(robot, chassis);
        break;
    case POSTION_MODE:
        do_chassis_position_ctrl(robot, chassis);
        break;
    default:
        break;
    }
    do_chassis_detect_speeds(robot, chassis);
}

void do_reset(Robot_t *robot)
{
    robot->chassis_mode = STOP_MODE;
    robot->set_vel.linear_x = 0;
    robot->set_vel.linear_y = 0;
    robot->set_vel.angular_z = 0;

    robot->servo[0]->setAngle(0);
}

void do_init_ctrl(Robot_t *robot, Gamepad_t *gamepad)
{
    robot->chassis_mode = STOP_MODE;
    robot->set_vel.linear_x = 0;
    robot->set_vel.linear_y = 0;
    robot->set_vel.angular_z = 0;
}


/**
 * @brief 通过手柄按钮控制不同外设输出
 * 
 * @param robot 
 * @param gamepad 
 */
int a=0;
void do_manual_switchgear_action(Robot_t *robot, Gamepad_t *gamepad)
{
    static uint16_t last_buttons = 0;

    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_A))
    {
		  if (a==0){
				robot->servo[0]->setAngle(-80);
				a =1;
		}
			else if (a==1){
				robot->servo[0]->setAngle(-60);
				a =2;
		}
			else if (a==2){
				robot->servo[0]->setAngle(0);
				a =0;
		}
    }
    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_B))
    {
        robot->SGs.SG2_PWR = !robot->SGs.SG2_PWR;
        robot->SGs.SG2_IO = !robot->SGs.SG2_IO;
        io::setSG2(robot->SGs.SG2_PWR, robot->SGs.SG2_IO);
    }
    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_X))
    {
        robot->SGs.SG3_PWR = !robot->SGs.SG3_PWR;
        robot->SGs.SG3_IO = !robot->SGs.SG3_IO;
        io::setSG3(robot->SGs.SG3_PWR, robot->SGs.SG3_IO);
    }
#ifndef SWITCHGEAR_4_USE_PWM
    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_Y))
    {
        robot->SGs.SG4_PWR = !robot->SGs.SG4_PWR;
        robot->SGs.SG4_IO = !robot->SGs.SG4_IO;
        io::setSG4(robot->SGs.SG4_PWR, robot->SGs.SG4_IO);
    }
#else
    io::setSG4(1, gamepad->rt);
#endif
    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_LB))
    {
        robot->SGs.SG5_PWR = !robot->SGs.SG5_PWR;
        robot->SGs.SG5_IO = !robot->SGs.SG5_IO;
        io::setSG5(robot->SGs.SG5_PWR, robot->SGs.SG5_IO);
    }
    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_RB))
    {
        robot->SGs.SG6_PWR = !robot->SGs.SG6_PWR;
        robot->SGs.SG6_IO = !robot->SGs.SG6_IO;
        io::setSG6(robot->SGs.SG6_PWR, robot->SGs.SG6_IO);
    }

    last_buttons = gamepad->buttons;
}

void do__action(Robot_t *robot, Gamepad_t *gamepad)
{
    
}

void do_manual_ctrl(Robot_t *robot, Gamepad_t *gamepad)
{
    if (robot->chassis_mode != SPEED_MODE)
        robot->chassis_mode = SPEED_MODE;

    do_manual_switchgear_action(robot, gamepad);

    robot->set_vel.linear_x = fmap(gamepad->y, INT16_MIN, INT16_MAX, -0.5, 0.5); //m/s
    robot->set_vel.linear_y = -fmap(gamepad->x, INT16_MIN, INT16_MAX, -0.5, 0.5); //m/s
    robot->set_vel.angular_z = -fmap(gamepad->z, INT16_MIN, INT16_MAX, -PI/2, PI/2); //rad/s

    if(gamepad->hat & GAMEPAD_XBOX_HAT_UP)
    {
        robot->set_vel.linear_x = 0.5;
    }
    else if(gamepad->hat & GAMEPAD_XBOX_HAT_DOWN)
    {
        robot->set_vel.linear_x = -0.5;
    }
    if(gamepad->hat & GAMEPAD_XBOX_HAT_LEFT)
    {
        robot->set_vel.linear_y = 0.5;
    }
    if(gamepad->hat & GAMEPAD_XBOX_HAT_RIGHT)
    {
        robot->set_vel.linear_y = -0.5;
    }
}

#define AUTO_THREAD_PRIORITY    1
#define AUTO_THREAD_STACK_SIZE  2048
#define AUTO_THREAD_TIMESLICE   5
static rt_thread_t auto_thread = RT_NULL;
void auto_ctrl_init(Robot_t *robot, void (*entry)(void *parameter))
{
    auto_thread = rt_thread_create("auto",
                    entry, robot,
                    AUTO_THREAD_STACK_SIZE,
                    AUTO_THREAD_PRIORITY, 
                    AUTO_THREAD_TIMESLICE);
    if (auto_thread != RT_NULL)
        rt_thread_startup(auto_thread);
}

void auto_wait_start(Robot_t *robot)
{
    if (robot->auto_state != 0)
    {
        robot->auto_state = 0;
    }
    while (1)
    {
        if (robot->auto_state != 0)
        {
            return;
        }
        delay(100);
    }
}

void auto_set_speed(Robot_t *robot, float linear_x, float linear_y, float angular_z, float delay_s)
{
    robot->chassis_mode = SPEED_MODE;
    robot->set_vel.linear_x = linear_x;
    robot->set_vel.linear_y = linear_y;
    robot->set_vel.angular_z = angular_z;
    delay(delay_s* 1000);
    robot->chassis_mode = STOP_MODE;
}

void do_auto_ctrl(Robot_t *robot, Gamepad_t *gamepad)
{
    static uint16_t last_buttons = 0;

    if (IS_BUTTON_CLICKED(gamepad->buttons, last_buttons, GAMEPAD_XBOX_BUTTON_A))
    {
        if (robot->auto_state == 0)
        {
            robot->auto_state = 1;
        }
    }

    last_buttons = gamepad->buttons;
}

