/**
 * @file L1.h
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __L1_H__
#define __L1_H__

#include "stdint.h"
#include "struct_typedef.h"
#include "CAN_receive_send.h"


#include "stm32h7xx_hal.h"

#define IMU_CAN_ID    0x01
#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

typedef enum {
    IMU_REQUEST_NONE,
    IMU_REQUEST_ACCEL,
    IMU_REQUEST_GYRO,
    IMU_REQUEST_EULER,
    IMU_REQUEST_QUATERNION
} IMU_Request_e;

typedef struct
{
	float roll;
	float pitch;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

}imu_t;

void IMU_UpdateData(uint8_t* pData);
void IMU_RequestData(FDCAN_HandleTypeDef* hfdcan,uint16_t can_id,uint8_t reg);

extern uint8_t freq_1k;
#endif
