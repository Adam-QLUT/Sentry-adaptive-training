#include "Chassis.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "pid.h"
#include "slope.h"
#include "User_math.h"

pid_t motor_speed_3508_FL, motor_speed_3508_FR, motor_speed_3508_BL, motor_speed_3508_BR;             // 行进电机速度环pid
pid_t motor_location_6020_FL, motor_location_6020_FR, motor_location_6020_BL, motor_location_6020_BR; // 舵向电机位置环pid
pid_t motor_speed_6020_FL, motor_speed_6020_FR, motor_speed_6020_BL, motor_speed_6020_BR;             // 舵向电机速度环pid
float set_angle_FL, set_angle_FR, set_angle_BL, set_angle_BR;                                                // 舵向电机目标角度
float set_mangle_FL, set_mangle_FR, set_mangle_BL, set_mangle_BR;                                            // 舵向电机目标编码器值
float X_AXIS_ECD_FL, X_AXIS_ECD_FR, X_AXIS_ECD_BL, X_AXIS_ECD_BR;                                            // 轮子向前时的零点
float tmp_delta_angle_FL, tmp_delta_angle_FR, tmp_delta_angle_BL, tmp_delta_angle_BR;                        // 目标角度
float target_velocity_FL, target_velocity_FR, target_velocity_BL, target_velocity_BR;                        // 行进电机目标速度
float dirt_FL = -1.0f, dirt_FR = 1.0f, dirt_BL = 1.0f, dirt_BR = -1.0f;                                      // 3508电机转向设置
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};                                                          // 计算转向时用到的角度
Chassis_t chassis;
pid_t chassis_speed_pid_FL, chassis_speed_pid_FR, chassis_speed_pid_BL, chassis_speed_pid_BR; // 轮子速度控制Pid
pid_t chassis_T_pid_x, chassis_T_pid_y, chassis_T_pid_w;
pid_t chassis_follow_pid; // 底盘跟随
pid_t chassis_power_pid;

float Chassis_pitch_angle;
extern pid_t yaw_auto_location_pid;

/**
 * @brief 底盘初始化
 *
 */
void Chassis_init()
{
    /*行进电机初始化*/
    CHASSISMotor_init(M3508_P, chassis_move_FL);
    CHASSISMotor_init(M3508_P, chassis_move_FR);
    CHASSISMotor_init(M3508_P, chassis_move_BL);
    CHASSISMotor_init(M3508_P, chassis_move_BR);
    /*航向电机初始化*/
    CHASSISMotor_init(GM6020, chassis_turn_FL);
    CHASSISMotor_init(GM6020, chassis_turn_FR);
    CHASSISMotor_init(GM6020, chassis_turn_BL);
    CHASSISMotor_init(GM6020, chassis_turn_BR);


#ifdef Speed_Control

    chassis.speed.max_x = 8.0f; // m/s
    chassis.speed.max_y = 8.0f; // m/s
    chassis.speed.max_r = 5.0f; // m/s

    chassis.acc.max_x = 2.5f; // 1m/^2
    chassis.acc.max_y = 2.5f; // m/^2
    chassis.acc.max_r = 2.5f; // m/^2
    /* 行进电机单速度环控制 */
    pid_set(&motor_speed_3508_FL, 1.0f, 0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_3508_FR, 1.0f, 0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_3508_BL, 1.0f, 0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_3508_BR, 1.0f, 0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    /* 舵向电机位置速度控制 */
    pid_set(&motor_location_6020_FL, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_location_6020_FR, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_location_6020_BL, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_location_6020_BR, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);

    pid_set(&motor_speed_6020_FL, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_6020_FR, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_6020_BL, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&motor_speed_6020_BR, 1.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 10000);
    /*底盘跟随PID*/
    pid_set(&chassis_follow_pid, 2.0f, 0.0f, 0.0f, 200, 40);
    /* 舵向电机参数初始化 */
    chassis.turn_FL.now = 0;
    chassis.turn_FL.set = TURN_FL_ECD;
    chassis.turn_FL.offset = 0;
    chassis.turn_FL.stable = 0;
    chassis.turn_FL.set_turn_FL_speed = 0;

    chassis.turn_FR.now = 0;
    chassis.turn_FR.set = TURN_FR_ECD;
    chassis.turn_FR.offset = 0;
    chassis.turn_FR.stable = 0;
    chassis.turn_FR.set_turn_FR_speed = 0;

    chassis.turn_BL.now = 0;
    chassis.turn_BL.set = TURN_BL_ECD;
    chassis.turn_BL.offset = 0;
    chassis.turn_BL.stable = 0;
    chassis.turn_BL.set_turn_BL_speed = 0;

    chassis.turn_BR.now = 0;
    chassis.turn_BR.set = TURN_BR_ECD;
    chassis.turn_BR.offset = 0;
    chassis.turn_BR.stable = 0;
    chassis.turn_BR.set_turn_BR_speed = 0;

    /* 旋转速度分解的角度deta */
    for (int i = 0; i < 4; i++)
    {
        deta[i] = deta[i] * PI / 180.0f; // 角度转弧度
    }


#endif
   
    /*默认地盘跟随模式*/
    Global.Chssis.mode = FLOW;
}

/**
 * @brief 舵向角度解算
 *
 * @param x
 * @param y
 * @param w
 */
void Rudder_Angle_calculation(float x, float y, float w)
{
    // 线速度
    w = w * Rotation_radius;
    int16_t angle_temp[4];

    // 旋转运动
    if (x == 0 && y == 0 && w == 0)
    {
        set_angle_FL = 0.0f;
        set_angle_FR = 0.0f;
        set_angle_BL = 0.0f;
        set_angle_BR = 0.0f;
    }
    else
    {
        set_angle_FL = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        set_angle_FR = atan2((y + w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
        set_angle_BL = atan2((y - w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        set_angle_BR = atan2((y - w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
    }

    set_mangle_FL = X_AXIS_ECD_FL + set_angle_FL * 8192.0 / 360.0;
    set_mangle_FR = X_AXIS_ECD_FR + set_angle_FR * 8192.0 / 360.0;
    set_mangle_BL = X_AXIS_ECD_BL + set_angle_BL * 8192.0 / 360.0;
    set_mangle_BR = X_AXIS_ECD_BR + set_angle_BR * 8192.0 / 360.0;

    // 角度赋值
    //--在这里设置的需要的角度对应在编码器上的位置
    chassis.turn_FL.set = (int32_t)set_mangle_FL;
    chassis.turn_FR.set = (int32_t)set_mangle_FR;
    chassis.turn_BL.set = (int32_t)set_mangle_BL;
    chassis.turn_BR.set = (int32_t)set_mangle_BR;
}

/**
 * @brief 舵向电机数据更新
 *
 */
void Chassis_course_updata()
{
    //--编码器赋值
    chassis.turn_FL.now = CHASSISMotor_get_data(chassis_turn_FL).ecd;
    chassis.turn_FR.now = CHASSISMotor_get_data(chassis_turn_FR).ecd;
    chassis.turn_BL.now = CHASSISMotor_get_data(chassis_turn_BL).ecd;
    chassis.turn_BR.now = CHASSISMotor_get_data(chassis_turn_BR).ecd;
}

/**
 * @brief 舵向电机PID计算
 *
 */
void Chassis_course_pid_cal()
{
    // 就近转位
    steer_transfer_nearby();

    chassis.turn_FL.turn_FL_speed = pid_cal(&motor_location_6020_FL, chassis.turn_FL.now, chassis.turn_FL.set);
    chassis.turn_FR.turn_FR_speed = pid_cal(&motor_location_6020_FR, chassis.turn_FR.now, chassis.turn_FR.set);
    chassis.turn_BL.turn_BL_speed = pid_cal(&motor_location_6020_BL, chassis.turn_BL.now, chassis.turn_BL.set);
    chassis.turn_BR.turn_BR_speed = pid_cal(&motor_location_6020_BR, chassis.turn_BR.now, chassis.turn_BR.set);

    //--电流计算
    CHASSISMotor_set(pid_cal(&motor_speed_6020_FL, CHASSISMotor_get_data(chassis_turn_FL).speed_rpm, chassis.turn_FL.turn_FL_speed), chassis_turn_FL);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_FR, CHASSISMotor_get_data(chassis_turn_FR).speed_rpm, chassis.turn_FR.turn_FR_speed), chassis_turn_FR);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_BL, CHASSISMotor_get_data(chassis_turn_BL).speed_rpm, chassis.turn_BL.turn_BL_speed), chassis_turn_BL);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_BR, CHASSISMotor_get_data(chassis_turn_BR).speed_rpm, chassis.turn_BR.turn_BR_speed), chassis_turn_BR);

}

/**
 * @brief 行进电机速度电流计算
 *
 * @param vx
 * @param vy
 * @param vw
 */
void Chassis_velocity_calc(float vx, float vy, float vw)
{
    // 计算和速度
    vw = vw * Rotation_radius;
    float V_FL, V_FR, V_BL, V_BR;
    // 分别计算四个轮子的速度大小
    V_FL = sqrt((-vx - vw * sinf(deta[1])) * (-vx - vw * sinf(deta[1])) + (-vy + vw * cosf(deta[1])) * (-vy + vw * cosf(deta[1])));
    V_FR = sqrt((vx + vw * sinf(deta[0])) * (vx + vw * sinf(deta[0])) + (vy + vw * cosf(deta[0])) * (vy + vw * cosf(deta[0])));
    V_BL = sqrt((vx - vw * sinf(deta[2])) * (vx - vw * sinf(deta[2])) + (vy - vw * cosf(deta[2])) * (vy - vw * cosf(deta[2])));
    V_BR = sqrt((vx + vw * sinf(deta[3])) * (vx + vw * sinf(deta[3])) + (vy - vw * cosf(deta[3])) * (vy - vw * cosf(deta[3])));

    // 最大速度限制
    val_limit(&vx, chassis.speed.max_x);
    val_limit(&vy, chassis.speed.max_y);
    val_limit(&vw, chassis.speed.max_r);

    //--确定目标速度的正负
    target_velocity_FL = V_FL * dirt_FL;
    target_velocity_FR = V_FR * dirt_FR;
    target_velocity_BL = V_BL * dirt_BL;
    target_velocity_BR = V_BR * dirt_BR;

    // 计算马达电流
    chassis.wheel_current_FL = pid_cal(&motor_speed_3508_FL, CHASSISMotor_get_data(chassis_move_FL).round_speed * WHEEL_RADIUS * PI, target_velocity_FL);
    chassis.wheel_current_FR = pid_cal(&motor_speed_3508_FR, CHASSISMotor_get_data(chassis_move_FR).round_speed * WHEEL_RADIUS * PI, target_velocity_FR);
    chassis.wheel_current_BL = pid_cal(&motor_speed_3508_BL, CHASSISMotor_get_data(chassis_move_BL).round_speed * WHEEL_RADIUS * PI, target_velocity_BL);
    chassis.wheel_current_BR = pid_cal(&motor_speed_3508_BR, CHASSISMotor_get_data(chassis_move_BR).round_speed * WHEEL_RADIUS * PI, target_velocity_BR);

    // 发送马达电流 （应在freeRTOS发送）
    CHASSISMotor_set(chassis.wheel_current_FR, chassis_move_FR);
    CHASSISMotor_set(chassis.wheel_current_FL, chassis_move_FL);
    CHASSISMotor_set(chassis.wheel_current_BL, chassis_move_BL);
    CHASSISMotor_set(chassis.wheel_current_BR, chassis_move_BR);

}

/**
 * @brief 就近转位
 *
 */
void steer_transfer_nearby()
{
    tmp_delta_angle_FL = obtain_modulus_normalization(chassis.turn_FL.set - chassis.turn_FL.now, 8192.0f);
    tmp_delta_angle_FR = obtain_modulus_normalization(chassis.turn_FR.set - chassis.turn_FR.now, 8192.0f);
    tmp_delta_angle_BL = obtain_modulus_normalization(chassis.turn_BL.set - chassis.turn_BL.now, 8192.0f);
    tmp_delta_angle_BR = obtain_modulus_normalization(chassis.turn_BR.set - chassis.turn_BR.now, 8192.0f);

    // 根据转动角度范围决定是否需要就近转位 FR
    if (-2048.0f <= tmp_delta_angle_FR && tmp_delta_angle_FR <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_FR.set = tmp_delta_angle_FR + chassis.turn_FR.now;
        dirt_FR = -1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_FR.set = obtain_modulus_normalization(tmp_delta_angle_FR + 4096.0f, 8192.0f) + chassis.turn_FR.now;
        dirt_FR = 1.0f;
    }
    // 根据转动角度范围决定是否需要就近转位 FL
    if (-2048.0f <= tmp_delta_angle_FL && tmp_delta_angle_FL <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_FL.set = tmp_delta_angle_FL + chassis.turn_FL.now;
        dirt_FL = 1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_FL.set = obtain_modulus_normalization(tmp_delta_angle_FL + 4096.0f, 8192.0f) + chassis.turn_FL.now;
        dirt_FL = -1.0f;
    }
    // 根据转动角度范围决定是否需要就近转位 BL
    if (-2048.0f <= tmp_delta_angle_BL && tmp_delta_angle_BL <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_BL.set = tmp_delta_angle_BL + chassis.turn_BL.now;
        dirt_BL = 1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_BL.set = obtain_modulus_normalization(tmp_delta_angle_BL + 4096.0f, 8192.0f) + chassis.turn_BL.now;
        dirt_BL = -1.0f;
    }
    // 根据转动角度范围决定是否需要就近转位 BR
    if (-2048.0f <= tmp_delta_angle_BR && tmp_delta_angle_BR <= 2048.0f)
    {
        // ±PI / 2之间无需反向就近转位
        chassis.turn_BR.set = tmp_delta_angle_BR + chassis.turn_BR.now;
        dirt_BR = -1.0f;
    }
    else
    {
        // 需要反转扣圈情况
        chassis.turn_BR.set = obtain_modulus_normalization(tmp_delta_angle_BR + 4096.0f, 8192.0f) + chassis.turn_BR.now;
        dirt_BR = 1.0f;
    }
}

/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 */
float obtain_modulus_normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}

/**
 * @brief 限制值
 *
 * @param val
 * @param MAX
 */
void val_limit(float *val, float MAX)
{
    if (fabs(*val) > MAX)
    {
        if (*val > 0)
            *val = MAX;
        else
            *val = -MAX;
    }
}

/**
 * @brief 获取底盘pitch角度数据，需pitch电机精确设置零点，仅能在云台底盘同方向的时候使用。
 *
 * @return float
 */
float Chassis_angle_getpitch()
{
    return GIMBALMotor_get_data(PITCHMotor).angle + IMU_data.AHRS.pitch * RAD_TO_DEG;
}

float power_limt(float FL_current, float FR_current, float BL_current, float BR_current,
                 float FL_speed, float FR_speed, float BL_speed, float BR_speed, float max_p)
{
    float current[4] = {FL_current, FR_current, BL_current, BR_current};
    float speed[4] = {FL_speed, FR_speed, BL_speed, BR_speed};
    float now_p = 0.0f;

    float a00 = 0.48872161;
    float a01 = -2.93589057e-04;
    float a10 = 5.3241338928e-05;
    float a02 = 2.70936086e-07;
    float a11 = 2.03985936e-06;
    float a20 = 2.17417767e-07;
    /*最大功率设置*/
    Supercap_set_power(max_p - 4.0f);
    if ((cap.remain_vol <= 9) || (Global.Cap.mode == Not_FULL))
    {
        max_p -= 2.0f; // 2w余量
        if (cap.remain_vol <= 9)
            max_p -= 3.0f;
    }

    else if (cap.remain_vol > 9)
    {
        max_p += cap.remain_vol * 12;
    }
    /*估算当前功率*/
    for (int i = 0; i < 4; i++)
    {
        now_p += fabs(a00 + a01 * speed[1] + a10 * current[i] +
                      a02 * speed[i] * speed[i] +
                      a11 * speed[i] * current[i] +
                      a20 * current[i] * current[i]);
    }
    float percentage = max_p / now_p;
    if (cap.Chassis_power >= (max_p / 2.0f) && (pid_cal(&chassis_power_pid, cap.Chassis_power, max_p) < 0)) // 底盘当前功率过小不使用闭环
        percentage += pid_cal(&chassis_power_pid, cap.Chassis_power, max_p);
    if (percentage > 1.0f)
        return 1.0f;
    return percentage;
}

void Chassis_set_x(float x)
{
    // RampGenerator_SetTarget(&Vx_ramp, x);
    Global.Chssis.input.x = x;
}

void Chassis_set_y(float y)
{
    // RampGenerator_SetTarget(&Vy_ramp, y);
    Global.Chssis.input.y = y;
}

void Chassis_set_r(float r)
{
    Global.Chssis.input.r = r;
}