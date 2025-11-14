#ifndef __ROBOT_FUNCTION_H__
#define __ROBOT_FUNCTION_H__

#include "robot.h"
#include "robot_driver.h"
#include "robot_config.h"
#include "chassis_kinematics.hpp"

#define IS_BUTTON_PRESSED(current_buttons, button_mask) (current_buttons & button_mask)
#define IS_BUTTON_CLICKED(current_buttons, last_buttons, button_mask) \
    ((current_buttons & button_mask) &&!(last_buttons & button_mask))

void do_gamepad_connect();
void do_mode_choice(Robot_t *robot, bool toggle_state);
void do_acc(float *speed_x, float *speed_y);
void do_chassis_stop_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_speed_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_position_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_detect_speeds(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_ctrl(Robot_t *robot, ChassisKinematics *chassis);

void do_reset(Robot_t *robot);
void do_init_ctrl(Robot_t *robot, Gamepad_t *gamepad);

void do_manual_ctrl(Robot_t *robot, Gamepad_t *gamepad);
void do_manual_switchgear_action(Robot_t *robot, Gamepad_t *gamepad);

void auto_ctrl_init(Robot_t *robot, void (*entry)(void *parameter));
void auto_wait_start(Robot_t *robot);
void auto_set_speed(Robot_t *robot, float linear_x, float linear_y, float angular_z, float delay_s);
void do_auto_ctrl(Robot_t *robot, Gamepad_t *gamepad);


#endif /* __ROBOT_FUNCTION_H__ */