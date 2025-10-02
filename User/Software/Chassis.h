#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "CAN_receive_send.h"

#include "User_math.h"

#include "motor.h"
/*底盘参数*/
// 行进电机
#define chassis_move_FL CAN_1_1
#define chassis_move_FR CAN_3_1
#define chassis_move_BL CAN_1_2
#define chassis_move_BR CAN_3_2
// 航向电机
#define chassis_turn_FL CAN_1_5
#define chassis_turn_FR CAN_3_5
#define chassis_turn_BL CAN_1_6
#define chassis_turn_BR CAN_3_6

#define WHEEL_RADIUS 0.26282f    // 旋转半径（单位：米，根据实际值修改）
#define Rotation_radius 0.37169f // 轮距（单位：米，根据实际值修改）


/*电机*/
#define USE_DJIMotor

/*电机配置*/
#ifdef USE_DJIMotor

#define CHASSISMotor_init DJIMotor_init
#define CHASSISMotor_set DJIMotor_set
#define CHASSISMotor_get_data DJIMotor_get_data
// 声明一下省的报错
void CHASSISMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void CHASSISMotor_set(int16_t val, DJIcan_id motor_id);
DJI_motor_data_s CHASSISMotor_get_data(DJIcan_id motor_id);
/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT DJIMOTOR_MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A
/* 舵向电机零点初始化 */
#define TURN_FR_ECD 7560
#define TURN_FL_ECD 4097
#define TURN_BL_ECD 5435
#define TURN_BR_ECD 6181

#endif // USE_DJIMotor

/*内部数据类型*/
typedef struct
{
    struct
    {
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_FR_speed;
        float set_turn_FR_speed;
    } turn_FR;
    struct
    {
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_FL_speed;
        float set_turn_FL_speed;
    } turn_FL;
    struct
    {
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_BL_speed;
        float set_turn_BL_speed;
    } turn_BL;
    struct
    {
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_BR_speed;
        float set_turn_BR_speed;
    } turn_BR;
    struct
    {
        float x, y, r, yaw;
        float now_x, now_y, now_r;
        float last_x, last_y, last_r;
        float max_x, max_y, max_r; // m/s
    } speed;

    struct
    {
        float x, y, r, big_yaw;
    } speed_RC;

    struct
    {
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s^2
    } acc;
    enum chassis_status_e
    {
        park = 0,
        move,
    } status;

    int16_t wheel_current_FL, wheel_current_FR, wheel_current_BL, wheel_current_BR; // PID输出的电调电流
    double wheel_speed[4];
    double angle_set[4];
    double angle_now[4];

    uint8_t is_open_cap;

} Chassis_t;
extern Chassis_t chassis;

/*外部调用*/
void Chassis_init();
void Rudder_Angle_calculation(float x, float y, float w);
void Chassis_course_updata();
void Chassis_course_pid_cal();
void Chassis_velocity_calc(float vx, float vy, float vw);
void steer_transfer_nearby();
float obtain_modulus_normalization(float x, float modulus);
void val_limit(float *val, float MAX);

void Chassis_move();
void Chassis_set_x(float x);
void Chassis_set_y(float y);
void Chassis_set_r(float r);
void Chassis_set_accel(float acc);

#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif // !__CHASSIS_H__
