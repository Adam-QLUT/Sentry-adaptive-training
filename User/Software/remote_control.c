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



#define MOVE_SENSITIVITY 10.0f   // �ƶ������ȣ�
#define PITCH_SENSITIVITY 0.008f // pitch��������
#define YAW_SENSITIVITY 5.0f     // yaw��������

/**
 * @brief ͳһ����
 *
 * @param key ������
 * @return uint8_t 0δ��ʱ�䣬1��ʱ��
 */

/**
 * @brief ң��������Դ��DT7ң����
 *
 */
void DT7toRCdata()
{
    /*ң��������*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*�������� */
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
 * @brief ����ͼ����ң������
 *
 */
void VT13toRCdata()
{
    /*ң��������*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // ������뿪�����Ӧ
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // ��λ�벦��ӳ��
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
    {
        RC_data.rc.s[0] = RC_SW_UP;
        RC_data.rc.s[1] = RC_SW_UP;
    }
    // �����벦��ӳ��
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

    /*�������� */
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
 * @brief ң��������
 *
 */
void RC_control()
{
    if (RC_data.online >= 0)
        RC_data.online--;
    /*����ģʽѡ��*/
    if ((switch_is_down(RC_R_SW) && switch_is_down(RC_L_SW)) || (RC_data.online <= 0)) // �������£�����
        Global.Control.mode = LOCK;
    
    else
        Global.Control.mode = RC;
    if (Global.Control.mode != RC)
        return;
    /*���̿���*/
    if (switch_is_up(RC_R_SW) && (switch_is_mid(RC_L_SW) || switch_is_up(RC_L_SW))) // ����,����||���ϣ���С����
        Global.Chssis.mode = SPIN_P;
    else if (switch_is_down(RC_L_SW) && (switch_is_mid(RC_R_SW) || switch_is_up(RC_R_SW))) // ����,����||���ϣ�ؓС����
        Global.Chssis.mode = SPIN_N;
    else if (switch_is_up(RC_L_SW) && switch_is_down(RC_R_SW))
        Global.Chssis.mode = Navigation;
    else
        Global.Chssis.mode = FLOW;
    Chassis_set_x(RC_data.rc.ch[0] / 40.0f);
    Chassis_set_y(RC_data.rc.ch[1] / 40.0f);
    /*��̨����*/
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_set_small_yaw_angle(-RC_data.rc.ch[2]);
        Gimbal_set_big_yaw_angle(-RC_data.rc.ch[2]);
    }
    /*�������*/
    if (switch_is_down(RC_R_SW) && (switch_is_mid(RC_L_SW) || switch_is_up(RC_L_SW))) // ����,����||���ϣ�����,���ģʽ
    {
        Global.Auto.mode = CAR;
    }
    else
    {
        Global.Auto.mode = NONE;
    }
    /*������������*/
    if (switch_is_up(RC_L_SW) && (switch_is_mid(RC_R_SW) || switch_is_down(RC_R_SW))) // ���ϣ�����||���£�����Ħ����
        Global.Shoot.shoot_mode = READY;
    else
        Global.Shoot.shoot_mode = CLOSE;
    if (RC_data.rc.ch[4] >= 300 &&
        RC_data.rc.ch[4] <= 660 &&
        Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||
         Global.Auto.input.fire == 1)) // ��������ͷ�����ٷ�����������򿪣�������־λ��1������
        Global.Shoot.tigger_mode = HIGH;
    else if (RC_data.rc.ch[4] >= 50 &&
             RC_data.rc.ch[4] <= 300 &&
             Global.Shoot.shoot_mode != CLOSE &&
             (Global.Auto.mode == NONE ||
              Global.Auto.input.fire == 1)) // �����в������ٷ���,������򿪣�������־λ��1������
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

