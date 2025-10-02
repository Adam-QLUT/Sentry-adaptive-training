#include "Gimbal.h"
#include "Global_status.h"

#include "User_math.h"
#include "pid.h"
#include "slope.h"

#include "IMU_updata.h"

#include "dm_motor.h"
/*pid*/
pid_t   pitch_speed_pid,
        pitch_location_pid;
pid_t   yaw_speed_pid, 
        yaw_location_pid;
pid_t   pitch_auto_speed_pid, 
        pitch_auto_location_pid;
pid_t   yaw_auto_speed_pid, 
        yaw_auto_location_pid;
pid_t   imu_big_yaw_control_speed;
pid_t   imu_big_yaw_control_tor;
pid_t   J4310_speed_pid, 
        J4310_location_pid;
/*斜坡*/
Slope pitch_ramp, yaw_ramp;

big_yaw_status big_yaw;

extern float W_now;
/**
 * @brief 云台初始化
 *
 */
void Gimbal_init()
{
    /*电机初始化*/
    GIMBALMotor_init(GM6020, YAWMotor);
    GIMBALMotor_init(GM6020, PITCHMotor);
    // dm4310_init(); // 大yaw电机ID和模式初始化
/*PID速度环初始化*/


    // 遥控
    pid_set(&pitch_speed_pid, 80.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_speed_pid, 200.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    // 自瞄
    pid_set(&pitch_auto_speed_pid, 100.0f, 0, 0, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_auto_speed_pid, 200.0f, 0.0f, 5.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    pid_set(&pitch_location_pid, 10.0f, 0.0f, 0, GIMBALMOTOR_MAX_CURRENT, 100);
    pid_set(&yaw_location_pid, 14.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    pid_set(&pitch_auto_location_pid, 30.0f, 0.05f, 0, GIMBALMOTOR_MAX_CURRENT, 100);
    pid_set(&yaw_auto_location_pid, 15.0f, 0.0f, 10.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 大yaw
    pid_set(&imu_big_yaw_control_speed, 1.8f, 0.0f, 0.0f, 3.0f, 0.1f);
    pid_set(&imu_big_yaw_control_tor, 0.74f, 0.0f, 0.0f, 7.0f, 0.1f);

    pid_set(&J4310_speed_pid, 1.5f, 0.0f, 8.0f, 5, 5);
    pid_set(&J4310_location_pid, 0.5f, 0.05f, 5.0f, 30, 5);



    GIMBALMotor_setzero(YAW_ZERO, YAWMotor);
    GIMBALMotor_setzero(PITCH_ZERO, PITCHMotor);
}
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_set_pitch_angle(float angle)
{
    if (angle < PITCH_MIN_ANGLE)
        angle = PITCH_MIN_ANGLE;
    if (angle > PITCH_MAX_ANGLE)
        angle = PITCH_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台小YAW轴角度
 *
 * @param angle 云台小YAW轴角度
 */
void Gimbal_set_small_yaw_angle(float angle)
{
    if (angle < SMALL_YAW_MIN_ANGLE)
        angle = SMALL_YAW_MIN_ANGLE;
    if (angle > SMALL_YAW_MAX_ANGLE)
        angle = SMALL_YAW_MAX_ANGLE;
    Global.Gimbal.input.yaw = angle;
}

void Gimbal_set_big_yaw_angle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}


void Gimbal_control()
{
    float pitch_speed, yaw_speed;
    float big_yaw_speed;
    // 非自瞄
    switch(Global.Gimbal.mode)
    {
        case NORMAL:
        pitch_speed = pid_cal(&pitch_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        if (Global.Chssis.mode != FLOW)
        {
            yaw_speed = pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);//Global.Gimbal.input.yaw /* + W_now * K_YAW */; // pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
            big_yaw_speed = Global.Gimbal.input.yaw;
        }
        else
        {
            yaw_speed = Global.Gimbal.input.yaw;
            
        }
        if (Global.Chssis.mode == SPIN_P || Global.Chssis.mode == SPIN_N)
        {
            yaw_speed += 0.4 * fabsf(Global.Gimbal.input.yaw);
            big_yaw_speed += 0.4 * fabsf(Global.Gimbal.input.yaw);
        }
        GIMBALMotor_set(-pid_cal(&pitch_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed), PITCHMotor);
        GIMBALMotor_set(pid_cal(&yaw_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed), YAWMotor);
        DMMotor_set(pid_cal(&J4310_speed_pid, DMMotor_data[0][0].speed_rpm, pid_cal(&J4310_location_pid, DMMotor_data[0][0].angle, 0)), DM_1_1);
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
        Gimbal_set_small_yaw_angle(IMU_data.AHRS.yaw_rad_cnt * RAD_TO_DEG);
        Gimbal_set_big_yaw_angle(IMU_data.AHRS.yaw_rad_cnt * RAD_TO_DEG);
        break;
    
    /* if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    { // 速度控制
            pitch_speed = pid_cal(&pitch_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
            if (Global.Chssis.mode != FLOW)
        {
            yaw_speed = Global.Gimbal.input.yaw /* + W_now * K_YAW */ // pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
            /*big_yaw_speed = Global.Gimbal.input.yaw;
        }
        else
        {
            yaw_speed = Global.Gimbal.input.yaw;
            big_yaw_speed = Global.Gimbal.input.yaw;
        }
        if (Global.Chssis.mode == SPIN_P)
        {
            yaw_speed += 0.4 * fabsf(Global.Gimbal.input.yaw);
            big_yaw_speed += 0.4 * fabsf(Global.Gimbal.input.yaw);
        }
        GIMBALMotor_set(-pid_cal(&pitch_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed), PITCHMotor);
        GIMBALMotor_set(pid_cal(&yaw_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed), YAWMotor);
        DMMotor_set(pid_cal(&J4310_speed_pid, DMMotor_data[0][0].speed_rpm, pid_cal(&J4310_location_pid, DMMotor_data[0][0].angle, 0)), DM_1_1);
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
        Gimbal_set_small_yaw_angle(IMU_data.AHRS.yaw_rad_cnt * RAD_TO_DEG);
        Gimbal_set_big_yaw_angle(IMU_data.AHRS.yaw_rad_cnt * RAD_TO_DEG);
    } */ 
    case Aimbot:
     // 自瞄
        pitch_speed = pid_cal(&pitch_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        yaw_speed = pid_cal(&yaw_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        GIMBALMotor_set(-pid_cal(&pitch_auto_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed), PITCHMotor);
        GIMBALMotor_set(pid_cal(&yaw_auto_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed), YAWMotor);
        Global.Auto.input.Auto_control_online--;
        break;
    }

}

/**
 * @brief 大yaw电机控制，mit模式纯力矩输出
 *
 */
void big_yaw_cal()
{
    // 1拖4模式
    big_yaw.speed = cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0];
    big_yaw.set_speed = pid_cal(&imu_big_yaw_control_speed, IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.big_yaw);
    big_yaw.current = pid_cal(&imu_big_yaw_control_tor, big_yaw.speed, big_yaw.set_speed);
    // dm4310.ctrl.tor_set = big_yaw.current;
    // dm4310_ctrl_send(&hfdcan2, &dm4310);
}
