/**
 * @file DMmotor.c
 * @author Koala (2337248447@qq.com)
 * @brief ��������ش���
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
// ������ݶ���
DM_motor_data_s DMMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief��������ʼ��
 *
 * @param motor_type ������ࣺJ4310
 * @param motor_id ���canͨ����ID
 */
void DMMotor_init(DMMotor_Type_e motor_type, DMcan_id motor_id)
{
    uint8_t cantype = (motor_id + 1) / 8; // ��õ������can·
    uint8_t canid = motor_id % 8;         // �õ����IDֵ��

    DMMotor_data[cantype][canid].Motor_type = motor_type; // ��ʼ����Ӧ���
}


/**
 * @brief ����DAMIAO�������
 *
 * @param val ����ֵ
 * @param motor_id ���canͨ����ID
 */
void DMMotor_set(int16_t val, DMcan_id motor_id)
{
    DMMotor_data[(motor_id + 1) / 8][motor_id % 8].set = val; // ���õ���
}

/**
 * @brief ��ȡDAMIAO�������
 *
 * @param motor_id ���canͨ����ID
 * @return DJI_motor_data_s ������ݽṹ�塣
 */
DM_motor_data_s DMMotor_get_data(DMcan_id motor_id) // ��ȡ�������
{
    return DMMotor_data[(motor_id + 1) / 8][motor_id % 8];
}

/**
 * @brief DAMIAO���CAN���ݽ����Լ�����
 *
 * @param ptr �������
 * @param data can����
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
    // ���ݲ�ͬ������в�ͬ����
    if ((ptr)->Motor_type == J4310)
    {
        // �������ת��
        (ptr)->round_speed = (ptr)->speed_rpm;

        // ������ԽǶ� -180~180 �������ȶ�ʧ �ܽǶȹ���ʱ
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // ����-180��
                angle -= (mul + 1) * 180;
            else // ����180��
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // ����180��
                angle += (mul + 1) * 180;
            else // ����-180��
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else{}
   

}

/**
 * @brief DAMIAO���can���ݴ���
 *
 * @param hfdcan CANͨ��
 * @param id can��ʶ��
 * @param data can����
 */
void DMMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id - DM_ID1 <= 8 && id - DM_ID1 >= 0) // ��ֹ�������
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
 * @brief DAMIAO�������ֵ���ͣ������freertos�ﶨ�ڷ���
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
    // ����J4310ǰ4��
    tx_message.Identifier = DM_J1310_1_4_send_ID;
    tx_message.IdType = FDCAN_STANDARD_ID;              // ��׼ID
    tx_message.TxFrameType = FDCAN_DATA_FRAME;          // ����֡
    tx_message.DataLength = FDCAN_DLC_BYTES_8;          // �������ݳ���
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // ���ô���״ָ̬ʾ
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;           // �������ɱ䲨����
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;            // ��ͨCAN��ʽ
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // ���ڷ����¼�FIFO����, ���洢
    tx_message.MessageMarker = 0x00;                    // ���ڸ��Ƶ�TX EVENT FIFO����ϢMaker��ʶ����Ϣ״̬����Χ0��0xFF
    // ��·can���θ�ֵ
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
    // ��·canͬʱ����
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, can_send_data[0]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, can_send_data[1]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, can_send_data[2]);

    // ����3508/2006��4��
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
    // ��·canͬʱ����
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, can_send_data[0]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, can_send_data[1]);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, can_send_data[2]);

}

#endif // USE_DM_MOTOR