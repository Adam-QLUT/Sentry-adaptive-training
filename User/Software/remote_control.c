#include "remote_control.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "slope.h"

#include "DT7.h"
#include "VT13.h"
#include "Power_switch.h"
#include "IMU_updata.h"

#include "Stm32_time.h"
#include "Power_switch.h"

#include "cmsis_os2.h"

RC_ctrl_t RC_data;



#define MOVE_SENSITIVITY 10.0f   // 移动灵敏度，
#define PITCH_SENSITIVITY 0.008f // pitch轴灵敏度
#define YAW_SENSITIVITY 5.0f     // yaw轴灵敏度

/**
 * @brief 统一消抖
 *
 * @param key 按键宏
 * @return uint8_t 0未到时间，1到时间
 */

/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    RC_data.mouse.press_mid = 0;
    DT7_data.online--;
    RC_data.online = DT7_data.online;
}

/**
 * @brief 来自图传的遥控数据
 *
 */
void VT13toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // 挡位与拨杆映射
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
    {
        RC_data.rc.s[0] = RC_SW_UP;
        RC_data.rc.s[1] = RC_SW_UP;
    }
    // 滚轮与拨杆映射
    if (VT13_data.rc.wheel < -330)
        RC_data.rc.s[1] = RC_SW_DOWN;
    if ((VT13_data.rc.wheel > -330) && (VT13_data.rc.wheel < 330) && (VT13_data.rc.mode_sw != 2))
        RC_data.rc.s[1] = RC_SW_MID;
    if (VT13_data.rc.wheel >= 330)
        RC_data.rc.s[1] = RC_SW_UP;
    if (VT13_data.rc.left_button == 1)
        Power_Turn_off(power2);
    else
        Power_Turn_on(power2);
    if (VT13_data.rc.right_button == 1)
        GIMBALMotor_setzero(YAW_ZERO + 135.0f, YAWMotor);
    else
        GIMBALMotor_setzero(YAW_ZERO, YAWMotor);

    /*键鼠数据 */
    RC_data.key.v = VT13_data.key.v;
    RC_data.mouse.x = VT13_data.mouse.x;
    RC_data.mouse.y = VT13_data.mouse.y;
    RC_data.mouse.z = VT13_data.mouse.z;
    RC_data.mouse.press_l = VT13_data.mouse.press_l;
    RC_data.mouse.press_r = VT13_data.mouse.press_r;
    RC_data.mouse.press_mid = VT13_data.mouse.middle;
    VT13_data.online--;
    RC_data.online = VT13_data.online;
}

/**
 * @brief 遥控器控制
 *
 */
void RC_control()
{
    if (RC_data.online >= 0)
        RC_data.online--;
    /*控制模式选择*/
    if ((switch_is_down(RC_R_SW) && switch_is_down(RC_L_SW)) || (RC_data.online <= 0)) // 左下右下，锁死
        Global.Control.mode = LOCK;
    
    else
        Global.Control.mode = RC;
    if (Global.Control.mode != RC)
        return;
    /*底盘控制*/
    if (switch_is_up(RC_R_SW) && (switch_is_mid(RC_L_SW) || switch_is_up(RC_L_SW))) // 右上,左中||左上，正小陀螺
        Global.Chssis.mode = SPIN_P;
    else if (switch_is_down(RC_L_SW) && (switch_is_mid(RC_R_SW) || switch_is_up(RC_R_SW))) // 左下,右中||右上，負小陀螺
        Global.Chssis.mode = SPIN_N;
    else if (switch_is_up(RC_L_SW) && switch_is_down(RC_R_SW))
        Global.Chssis.mode = Navigation;
    else
        Global.Chssis.mode = FLOW;
    Chassis_set_x(RC_data.rc.ch[0] / 40.0f);
    Chassis_set_y(RC_data.rc.ch[1] / 40.0f);
    /*云台控制*/
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_set_small_yaw_angle(-RC_data.rc.ch[2]);
        Gimbal_set_big_yaw_angle(-RC_data.rc.ch[2]);
    }
    /*自瞄控制*/
    if (switch_is_down(RC_R_SW) && (switch_is_mid(RC_L_SW) || switch_is_up(RC_L_SW))) // 右下,左中||左上，自瞄,射击模式
    {
        Global.Auto.mode = CAR;
    }
    else
    {
        Global.Auto.mode = NONE;
    }
    /*发弹机构控制*/
    if (switch_is_up(RC_L_SW) && (switch_is_mid(RC_R_SW) || switch_is_down(RC_R_SW))) // 左上，右中||右下，开启摩擦轮
        Global.Shoot.shoot_mode = READY;
    else
        Global.Shoot.shoot_mode = CLOSE;
    if (RC_data.rc.ch[4] >= 300 &&
        RC_data.rc.ch[4] <= 660 &&
        Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||
         Global.Auto.input.fire == 1)) // 滚轮最下头，高速发弹，若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = HIGH;
    else if (RC_data.rc.ch[4] >= 50 &&
             RC_data.rc.ch[4] <= 300 &&
             Global.Shoot.shoot_mode != CLOSE &&
             (Global.Auto.mode == NONE ||
              Global.Auto.input.fire == 1)) // 滚轮中部，低速发弹,若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = LOW;
    else if (RC_data.rc.ch[4] > 660 &&
             Global.Shoot.shoot_mode != CLOSE)
    {
        Global.Shoot.shoot_mode = DEBUG_SHOOT;
        Global.Shoot.tigger_mode = DEBUG_SHOOT;
    }
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
}

