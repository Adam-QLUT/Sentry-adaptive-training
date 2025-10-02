/**
 * @file L1.c
 * @author Nas
 * @brief
 * @version 0.1
 * @date 2025-9-25
 *
 *
 */

#include "L1.h"
#include "string.h"
#include "stm32h7xx_hal.h"

#include "tim.h"
#include "Stm32_time.h"
imu_t imu;

static volatile IMU_Request_e current_imu_request = IMU_REQUEST_NONE;
static uint32_t tick_ms=0;
/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



void IMU_RequestData(FDCAN_HandleTypeDef* hfdcan,uint16_t can_id,uint8_t reg)
{
	static FDCAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
	tx_header.DataLength=FDCAN_DLC_BYTES_4;
	tx_header.IdType=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=0x6FF;
	tx_header.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,cmd);

	}
}


void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	imu.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(uint8_t* pData)
{

	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	
	}

}

void IMU_Decode()
{
    if(freq_1k)
		{
			tick_ms++;
			
			if(tick_ms%4==0)
			{
				IMU_RequestData(&hfdcan3,0x01,4);
			}
			else if(tick_ms%3==0)
			{
				IMU_RequestData(&hfdcan3,0x01,3);
			}
			else if(tick_ms%2==0)
			{
				IMU_RequestData(&hfdcan3,0x01,2);
			}
			else if(tick_ms%1==0)
			{
				IMU_RequestData(&hfdcan3,0x01,1);
			}
			
			if(tick_ms>1000)
				tick_ms=0;
				
			freq_1k=0;
		}
}

void DMIMU_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint32_t receive_id)
{
	
}