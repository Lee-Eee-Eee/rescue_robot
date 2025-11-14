#include "chassis_kinematics.hpp"
#include "cff_utils.h"
#include "easy_math.h"

namespace CFF 
{

using namespace Chassis;

void calculateInverseKinematicsFor4WheelOmni(float wheels_rpm[], float linear_x, float linear_y, float angular_z, ChassisParam *param)
{
    float speed_temp[4];

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60;
    float linear_vel_y_mins = linear_y * 60;

    //convert rad/s to rad/min
    float angular_vel_z_mins = angular_z * 60;

    float tangential_vel = angular_vel_z_mins * ((param->wheels_track + param->wheels_base) / 2);

    float x_rpm = linear_vel_x_mins / param->wheels_perimeter;
    float y_rpm = linear_vel_y_mins / param->wheels_perimeter;
    float tan_rpm = tangential_vel / param->wheels_perimeter;

    speed_temp[0] =-x_rpm + y_rpm + tan_rpm;    // left_front 
    speed_temp[1] = x_rpm + y_rpm + tan_rpm;    // right_front
    speed_temp[2] =-x_rpm - y_rpm + tan_rpm;    // left_back
    speed_temp[3] = x_rpm - y_rpm + tan_rpm;    // right_back 

    wheels_rpm[LEFT_FRONT_WHEEL] = speed_temp[0];
    wheels_rpm[RIGHT_FRONT_WHEEL] = speed_temp[1];
    wheels_rpm[LEFT_BACK_WHEEL] = speed_temp[2];
    wheels_rpm[RIGHT_BACK_WHEEL] = speed_temp[3];
}

void calculateForwardKinematicsFor4WheelOmni(ChassisVelocities *vel, float lf_spd, float rf_spd, float lb_spd, float rb_spd, ChassisParam *param)
{
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    //convert average revolutions per minute in x axis to revolutions per second
    average_rps_x = ((float) (-lf_spd + rf_spd - lb_spd + rb_spd) / param->total_wheels) / 60;
    vel->linear_x = average_rps_x * param->wheels_perimeter; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float) (lf_spd + rf_spd - lb_spd - rb_spd) / param->total_wheels) / 60;
    vel->linear_y = average_rps_y * param->wheels_perimeter; // m/s

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float) (lf_spd + rf_spd + lb_spd + rb_spd) / param->total_wheels) / 60;
    vel->angular_z = (average_rps_a * param->wheels_perimeter) / ((param->wheels_track + param->wheels_base) / 2); //  rad/s
}

void calculateInverseKinematicsFor3WheelOmni(float wheels_rpm[], float linear_x, float linear_y, float angular_z, ChassisParam *param)
{
    float speed_temp[4];

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60;
    float linear_vel_y_mins = linear_y * 60;

    //convert rad/s to rad/min
    float angular_vel_z_mins = angular_z * 60;

    float tangential_vel = angular_vel_z_mins * param->wheels_center_distance;

    float x_rpm = linear_vel_x_mins / param->wheels_perimeter;
    float y_rpm = linear_vel_y_mins / param->wheels_perimeter;
    float tan_rpm = tangential_vel / param->wheels_perimeter;

    speed_temp[0] =-sqrt(3.0)/2.f*x_rpm + 0.5*y_rpm + tan_rpm;    // left_front 
    speed_temp[1] = sqrt(3.0)/2.f*x_rpm + 0.5*y_rpm + tan_rpm;    // right_front
    speed_temp[2] =-y_rpm + tan_rpm;    // back

    wheels_rpm[LEFT_FRONT_WHEEL] = speed_temp[0];
    wheels_rpm[RIGHT_FRONT_WHEEL] = speed_temp[1];
    wheels_rpm[BACK_WHEEL] = speed_temp[2];
}

void calculateForwardKinematicsFor3WheelOmni(ChassisVelocities *vel, float lf_spd, float rf_spd, float back_spd, ChassisParam *param)
{
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    average_rps_x = ((sqrt(3.0)*rf_spd) - (sqrt(3.0)*lf_spd)) / param->total_wheels / 60;
    vel->linear_x = average_rps_x * param->wheels_perimeter; // m/s

    average_rps_y = (-2.0L * back_spd + lf_spd + rf_spd) / param->total_wheels / 60;
    vel->linear_y = average_rps_y * param->wheels_perimeter; // m/s

    average_rps_a = (lf_spd + rf_spd + back_spd) / param->total_wheels / 60;
    vel->angular_z = (average_rps_a * param->wheels_perimeter) / param->wheels_center_distance; //  rad/s
}
void adjustSpeedsToMax(float in_speed[], float out_speed[], uint8_t total_wheels, float wheels_speed_max)
{
    // Find the maximum absolute value in the array
    float max = 0.0f;
    for (uint8_t i = 0; i < total_wheels; i++)
    {
        if (fabs(in_speed[i]) > max)
        {
            max = fabs(in_speed[i]);
        }
    }

    // Adjust speeds to ensure none exceed wheels_speed_max while maintaining proportions
    if (max > wheels_speed_max)
    {
        float rate = wheels_speed_max / max;
        for (uint8_t i = 0; i < total_wheels; i++)
        {
            in_speed[i] *= rate;
        }
    }
    for (uint8_t i = 0; i < total_wheels; i++)
    {
        out_speed[i] = in_speed[i];
    }
    
}


ChassisKinematics::ChassisKinematics(ChassisTypes type) :
    type_( type )
{ 
}

ChassisKinematics::~ChassisKinematics()
{
}

void ChassisKinematics::init(float wheels_diameter, float wheels_track, float wheels_base, uint16_t wheels_speed_max)
{
    switch (type_)
    {
        case FOUR_WHEEL_OMNI:
             param_.total_wheels = 4;
            break;
        case THREE_WHEEL_OMNI:
             param_.total_wheels = 3;
            break;
        case FOUR_WHEEL_DIFFERENTIAL:
             param_.total_wheels = 4;
            break;
        case TOW_WHEEL_DIFFERENTIAL:
             param_.total_wheels = 2;
            break;
        case ACKERMANN_STEERING:
             param_.total_wheels = 2;
            break;
        default:
            CHASSIS_ASSERT(0);
            break;
    }
    param_.wheels_diameter = wheels_diameter;
    param_.wheels_perimeter = wheels_diameter * PI;
    param_.wheels_track = wheels_track;
    param_.wheels_base = wheels_base;
    param_.wheels_speed_max = wheels_speed_max;
    memset(wheels_rpm_, 0, sizeof(float)*4);
    memset(&current_vel_, 0, sizeof(current_vel_));
    memset(&odom_, 0, sizeof(odom_));
    now_time_ = millis();
    prev_time_ = millis();
}

void ChassisKinematics::init(float wheels_diameter, float wheels_center_distance, uint16_t wheels_speed_max)
{
    param_.wheels_center_distance = wheels_center_distance;
    init(wheels_diameter, 0, 0, wheels_speed_max);
}

float *ChassisKinematics::applyMotionCommand(float linear_x, float linear_y, float angular_z)
{
    float wheels_temp[4];
    switch (type_)
    {
        case FOUR_WHEEL_OMNI:
            calculateInverseKinematicsFor4WheelOmni(wheels_temp, linear_x, linear_y, angular_z, &param_);
            break;
        case THREE_WHEEL_OMNI:
            calculateInverseKinematicsFor3WheelOmni(wheels_temp, linear_x, linear_y, angular_z, &param_);
            break;
        case FOUR_WHEEL_DIFFERENTIAL:
        case TOW_WHEEL_DIFFERENTIAL:
            calculateInverseKinematicsFor4WheelOmni(wheels_temp, linear_x, 0, angular_z, &param_);
            break;
        case ACKERMANN_STEERING:
            break;
    }
    adjustSpeedsToMax(wheels_temp, wheels_rpm_, param_.total_wheels, param_.wheels_speed_max);
    return wheels_rpm_;
}

float *ChassisKinematics::applyMotionCommand(ChassisVelocities *vel)
{
    return applyMotionCommand(vel->linear_x, vel->linear_y, vel->angular_z);
}

ChassisVelocities *ChassisKinematics::detectMovementSpeeds(float wheels_speed[])
{
    switch (type_)
    {
        case FOUR_WHEEL_OMNI:
        case FOUR_WHEEL_DIFFERENTIAL:
            calculateForwardKinematicsFor4WheelOmni(&current_vel_, 
                                                    wheels_speed[LEFT_FRONT_WHEEL], 
                                                    wheels_speed[RIGHT_FRONT_WHEEL], 
                                                    wheels_speed[LEFT_BACK_WHEEL], 
                                                    wheels_speed[RIGHT_BACK_WHEEL], 
                                                    &param_);
            break;
        case THREE_WHEEL_OMNI:
            calculateForwardKinematicsFor3WheelOmni(&current_vel_,
                                                    wheels_speed[LEFT_WHEEL], 
                                                    wheels_speed[RIGHT_WHEEL], 
                                                    wheels_speed[BACK_WHEEL], 
                                                    &param_);
            break;
        case TOW_WHEEL_DIFFERENTIAL:
            calculateForwardKinematicsFor4WheelOmni(&current_vel_, 
                                                    wheels_speed[LEFT_WHEEL], 
                                                    wheels_speed[RIGHT_WHEEL], 
                                                    0, 
                                                    0, 
                                                    &param_);
            current_vel_.linear_y = 0;
            break;
        case ACKERMANN_STEERING:
            break;
    }
    return &current_vel_;
}

struct ChassisOdom *ChassisKinematics::updateOdom()
{
    static bool is_init = true;
    float step_time;
    float linear_velocity_x = current_vel_.linear_x;
    float linear_velocity_y = current_vel_.linear_y;
    float angular_velocity_z = current_vel_.angular_z;
    if (is_init)
    {
        prev_time_ = now_time_ = millis();
        is_init = false;
    }
    
    now_time_ = millis();
    step_time = (float)(now_time_ - prev_time_) * 0.001;

    float delta_heading = angular_velocity_z * step_time; // radians
    float delta_x = (linear_velocity_x * cos(odom_.heading) - linear_velocity_y * sin(odom_.heading)) * step_time; //m
    float delta_y = (linear_velocity_x * sin(odom_.heading) + linear_velocity_y * cos(odom_.heading)) * step_time; //m
    
    /* calculate current position of the robot */
    odom_.x_pos += delta_x;
    odom_.y_pos += delta_y;
    odom_.heading += delta_heading;
    odom_.heading = LIMIT_ANGLE_RADIAN(odom_.heading);

    prev_time_ = now_time_;

    return &odom_;
}

struct ChassisOdom *ChassisKinematics::updateOdom(float gyro_yaw)
{
    static bool is_init = true;
    float step_time;
    float linear_velocity_x = current_vel_.linear_x;
    float linear_velocity_y = current_vel_.linear_y;
    float angular_velocity_z = current_vel_.angular_z;
    if (is_init)
    {
        prev_time_ = now_time_ = millis();
        is_init = false;
    }
    
    odom_.heading = gyro_yaw;
    now_time_ = millis();
    step_time = (float)(now_time_ - prev_time_) * 0.001;

    float delta_x = (linear_velocity_x * cos(odom_.heading) - linear_velocity_y * sin(odom_.heading)) * step_time; //m
    float delta_y = (linear_velocity_x * sin(odom_.heading) + linear_velocity_y * cos(odom_.heading)) * step_time; //m
    
    /* calculate current position of the robot */
    odom_.x_pos += delta_x;
    odom_.y_pos += delta_y;

    prev_time_ = now_time_;

    return &odom_;
}

void ChassisKinematics::resetOdom()
{
    odom_.x_pos = 0.0;
    odom_.y_pos = 0.0;
    odom_.heading = 0.0;
}

} // namespace CFF 
