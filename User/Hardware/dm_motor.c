/**
 * @file DMmotor.c
 * @author Koala (2337248447@qq.com)
 * @brief 达妙电机相关代码
 * @version 0.1
 * @date 2025-7-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "dm_motor.h"
#include "math.h"

#ifdef USE_DAMIAO_MOTOR
#define abs(a) a > 0 ? a : -a
// 电机数据定义
DM_motor_data_s DMMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief达妙电机初始化
 *
 * @param motor_type 电机种类：J4310
 * @param motor_id 电机can通道及ID
 */
void DMMotor_init(DMMotor_Type_e motor_type, DMcan_id motor_id)
{
    uint8_t cantype = (motor_id + 1) / 8; // 获得电机所在can路
    uint8_t canid = motor_id % 8;         // 得到电机ID值；

    DMMotor_data[cantype][canid].Motor_type = motor_type; // 初始化相应电机
}


/**
 * @brief 设置DAMIAO电机电流
 *
 * @param val 电流值
 * @param motor_id 电机can通道及ID
 */
void DMMotor_set(int16_t val, DMcan_id motor_id)
{
    DMMotor_data[(motor_id + 1) / 8][motor_id % 8].set = val; // 设置电流
}

/**
 * @brief 获取DAMIAO电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DJI_motor_data_s 电机数据结构体。
 */
DM_motor_data_s DMMotor_get_data(DMcan_id motor_id) // 获取马达数据
{
    return DMMotor_data[(motor_id + 1) / 8][motor_id % 8];
}

/**
 * @brief DAMIAO电机CAN数据接受以及处理
 *
 * @param ptr 电机数据
 * @param data can数据
 */
void DMMotor_get_process_motor_data(DM_motor_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = 0.01f * (int16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->coil_temp = (data)[6];
    (ptr)->pcb_temp = (data)[7];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DM;
    // 根据不同电机进行不同处理
    if ((ptr)->Motor_type == J4310)
    {
        // 计算出轴转速
        (ptr)->round_speed = (ptr)->speed_rpm;

        // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else{}
   

}

/**
 * @brief DAMIAO电机can数据处理
 *
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DMMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id - DM_ID1 <= 8 && id - DM_ID1 >= 0) // 防止数组溢出
    {
        if (hfdcan == &hfdcan1)
        {
            DMMotor_get_process_motor_data(&DMMotor_data[0][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DJIMotor_get_process_motor_data(&DMMotor_data[1][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DJIMotor_get_process_motor_data(&DMMotor_data[2][id - DM_ID1], data);
        }
    }
}

/**
 * @brief DAMIAO电机电流值发送，建议放freertos里定期发送
 *
 */
void DMMotor_send_current(void)
{
    uint8_t can_send_data[3][8];
    uint8_t canid;
    static FDCAN_TxHeaderTypeDef tx_message;
    if (hfdcan1.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan1);
    if (hfdcan2.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan2);
    if (hfdcan3.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan3);
    // 发送J4310前4个
    tx_message.Identifier = DM_J1310_1_4_send_ID;
    tx_message.IdType = FDCAN_STANDARD_ID;              // 标准ID
    tx_message.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
    tx_message.DataLength = FDCAN_DLC_BYTES_8;          // 发送数据长度
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
    tx_message.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF
    // 三路can依次赋值
    for (int i = 0; i < 3; i++)
    {
        canid = 0;
        for (int k = 0; k < 8; k += 2)
        {
            can_send_data[i][k] = (DMMotor_data[i][canid].set >> 8);
            can_send_data[i][k + 1] = DMMotor_data[i][canid].set;
            canid++;
        }
    }
    // 三路can同时发送
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, can_send_data[0]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, can_send_data[1]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, can_send_data[2]);

    // 发送3508/2006后4个
    tx_message.Identifier = DM_J1310_5_8_send_ID;
    for (int i = 0; i < 3; i++)
    {
        canid = 4;
        for (int k = 0; k < 8; k += 2)
        {
            can_send_data[i][k] = (DMMotor_data[i][canid].set >> 8);
            can_send_data[i][k + 1] = DMMotor_data[i][canid].set;
            canid++;
        }
    }
    // 三路can同时发送
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, can_send_data[0]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, can_send_data[1]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, can_send_data[2]);

}

#endif // USE_DM_MOTOR