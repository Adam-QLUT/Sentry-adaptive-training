#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "referee_system.h"
#include "UART_data_txrx.h"

#include "CRC8_CRC16.h"
#include "string.h"

STM32_data_t toNUC;
STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;

uint8_t data[128];
uint8_t rx_data[100];

void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
	memcpy(tx_buff, target, sizeof(STM32_data_t));
	return 0;
}

/**
 * @brief 向上位机发送自瞄相关数据
 *
 * @param yaw yaw轴当前角度（弧度）
 * @param pitch pitch轴当前角度（弧度）
 * @param omega yaw轴当前角速度（rad/s）
 */
void STM32_to_MINIPC()
{
    /* toMINIPC.FrameHeader.sof = 0xA5;
    toMINIPC.FrameHeader.crc8 = 0x00;
    toMINIPC.To_minipc_data.curr_pitch = pitch; // IMU_data.AHRS.pitch;
    toMINIPC.To_minipc_data.curr_yaw = yaw;     // IMU_data.AHRS.yaw;
    toMINIPC.To_minipc_data.curr_omega = omega; // cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[1];
    toMINIPC.To_minipc_data.autoaim = 1;
    if (Referee_data.robot_id >= 100)
        toMINIPC.To_minipc_data.enemy_color = 1;
    else
        toMINIPC.To_minipc_data.enemy_color = 0;
    toMINIPC.To_minipc_data.state = 0;
    toMINIPC.FrameTailer.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.FrameHeader.sof, 17, 0xffff);
    toMINIPC.enter = 0x0A;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    // VirCom_send(data, sizeof(STM32_data_t));
    UART_send_data(UART1_data, data, sizeof(STM32_data_t)); */
    toNUC.header = 0xff;
    toNUC.ender = 0x0d;
    toNUC.mode = 0;
    toNUC.roll = IMU_data.AHRS.roll;
    toNUC.pitch = IMU_data.AHRS.pitch;
	toNUC.yaw = IMU_data.AHRS.yaw ;
    encodeSTM32(&toNUC, data, sizeof(STM32_data_t));
    VirCom_send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    /* if (fabs(fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw) > PI / 2.0f) // 过零点处理
    {
        if (fromMINIPC.from_minipc_data.shoot_yaw > PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw -= 2 * PI;
        else if (fromMINIPC.from_minipc_data.shoot_yaw < -PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw += 2 * PI;
    }
    Global.Auto.input.shoot_pitch = fromMINIPC.from_minipc_data.shoot_pitch - IMU_data.AHRS.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw;
    Global.Auto.input.fire = fromMINIPC.from_minipc_data.fire;
    Global.Auto.input.target_id = fromMINIPC.from_minipc_data.target_id;
    if (fromMINIPC.from_minipc_data.shoot_pitch == 0 && fromMINIPC.from_minipc_data.shoot_yaw == 0)
        Global.Auto.input.fire = -1; */
    Global.Auto.input.shoot_pitch = fromMINIPC.pitch;
	Global.Auto.input.shoot_yaw = fromMINIPC.yaw;
	Global.Auto.input.fire = fromMINIPC.fire;
}

void Auto_control()
{

    Gimbal_set_small_yaw_angle((180.0 / 3.14159265358979323846) * IMU_data.AHRS.yaw_rad_cnt + (180.0 / 3.14159265358979323846) * (Global.Auto.input.shoot_yaw));
    Gimbal_set_pitch_angle((180.0 / 3.14159265358979323846) * IMU_data.AHRS.pitch + (180.0 / 3.14159265358979323846) * (Global.Auto.input.shoot_pitch));

}

/**
 * @brief 向导航发送裁判系统数据
 *
 */
void Navigation_send_message()
{
    stm32send_1.remain_hp = Referee_data.current_HP;                                // 机器人实时血量
    stm32send_1.max_hp = Referee_data.maximum_HP;                                   // 最大血量
    stm32send_1.game_type = 0;                                                      // 比赛类型
    stm32send_1.game_progress = Referee_data.game_progress;                          // 比赛进程
    stm32send_1.stage_remain_time = Referee_data.stage_remain_time;                  // 当前阶段剩余时间
    stm32send_1.bullet_remaining_num_17mm = Referee_data.projectile_allowance_17mm; // 允许发弹量
    stm32send_1.red_outpost_hp = Referee_data.red_outpost_HP;                      // 红方前哨站血量
    stm32send_1.red_base_hp = Referee_data.red_base_HP;                            // 红方基地血量
    stm32send_1.blue_outpost_hp = Referee_data.blue_outpost_HP;                    // 蓝方前哨站血量
    stm32send_1.blue_base_hp = Referee_data.blue_base_HP;                          // 蓝方基地血量
    stm32send_1.rfid_status = Referee_data.rfid_status;                              // RFID状态

    static char json[1024];
    int len = sprintf(json,
                      "{\"game_type\":%u,"
                      "\"game_progress\":%u,\"remain_hp\":%u,\"max_hp\":%u,\"stage_remain_time\":%u,\"bullet_remaining_num_17mm\":%u,"
                      "\"red_outpost_hp\":%u,\"red_base_hp\":%u,\"blue_outpost_hp\":%u,"
                      "\"blue_base_hp\":%u,\"rfid_status\":%u}\n",
                      /*stm32send_1.header,*/ stm32send_1.game_type,
                      stm32send_1.game_progress, stm32send_1.remain_hp, stm32send_1.max_hp, stm32send_1.stage_remain_time, stm32send_1.bullet_remaining_num_17mm,
                      stm32send_1.red_outpost_hp, stm32send_1.red_base_hp, stm32send_1.blue_outpost_hp,
                      stm32send_1.blue_base_hp, stm32send_1.rfid_status);
    HAL_UART_Transmit_DMA(UART1_data.huart, (uint8_t *)json, len);
}
