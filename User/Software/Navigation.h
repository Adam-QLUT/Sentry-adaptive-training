/**
 * @file Navigation.h
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "struct_typedef.h"
#include "stdbool.h"
#include "remote_control.h"

#define Navigation_COMMUNICATE_SOF ((uint8_t)0xA5) // 数据帧起始字节，固定值为 0xA5
#define Navigation_COMMUNICATE_TIMEOUT  ((uint32_t)50)  // 超时等待时间

#define Navigation_Data_Test_ID              ((uint8_t)0x01)
#define Navigation_Data_Rc_ID                ((uint8_t)0x02)
#define Navigation_Data_Gimbal_ID            ((uint8_t)0x03)

#define Navigation_Data_Test_Duration        ((uint32_t)20) // ms
#define Navigation_Data_Rc_Duration          ((uint32_t)16) // ms
#define Navigation_Data_Gimbal_Duration      ((uint32_t)10) // ms

// UART2通信协议数据包长度定义
#define Navigation_FRAME_MAX_SIZE            ((uint8_t)250) // Byte

#define Navigation_FRAME_HEADER_SIZE         sizeof(FrameHeader_t)
#define Navigation_FRAME_TIMESTAMP_SIZE      ((uint8_t)sizeof(uint32_t))
#define Navigation_FRAME_CRC16_SIZE          ((uint8_t)sizeof(uint16_t))
#define Navigation_HEADER_CRC_LEN            (Navigation_FRAME_HEADER_SIZE + Navigation_FRAME_CRC16_SIZE)
#define Navigation_HEADER_CRC_TIMESTAMP_LEN  (Navigation_FRAME_HEADER_SIZE + Navigation_FRAME_CRC16_SIZE + Navigation_FRAME_TIMESTAMP_SIZE)
#define Navigation_HEADER_TIMESTAMP_LEN      (Navigation_FRAME_HEADER_SIZE + Navigation_FRAME_TIMESTAMP_SIZE)

typedef enum {
    STEP_HEADER_SOF = 0,
    STEP_LENGTH = 1,
    STEP_ID = 2,
    STEP_TYPE = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} UnpackStep_e;

typedef struct
{
    uint32_t Interrupt;
    uint32_t Data_Test;
    uint32_t Data_Rc;
    uint32_t Data_Gimbal;
} LastTime_t;

typedef struct
{
    uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
    uint8_t len;   // 数据段长度
    uint8_t id;    // 数据段id
    uint8_t type;  // 数据段类型
    uint8_t crc;   // 数据帧头的 CRC8
} __attribute__((packed)) FrameHeader_t;

typedef struct
{
    FrameHeader_t * p_header;
    uint8_t data_len;
    uint8_t protocol_packet[Navigation_FRAME_MAX_SIZE];
    UnpackStep_e unpack_step;
    uint16_t index;
} UnpackData_t;

//测试用数据包
typedef struct
{
    FrameHeader_t frame_header;

    uint32_t time_stamp;  //数据段时间戳

    struct
    {
        uint32_t index;
        float test_float;
        uint8_t reserved[42];
    } __attribute__((packed)) data;

    uint16_t crc16;  //crc16校验
} __attribute__((packed)) Data_Test_s;

//遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;

    uint32_t time_stamp;  //数据段时间戳

    struct
    {
        RC_ctrl_t rc_ctrl;
        bool rc_offline;
    } __attribute__((packed)) data;

    uint16_t crc16;  //crc16校验
} __attribute__((packed)) Data_Rc_s;

// 云台数据包
typedef struct
{
    FrameHeader_t frame_header;

    uint32_t time_stamp;  //数据段时间戳

    struct
    {
        float yaw_motor_pos;
        bool yaw_motor_offline;
        bool init_judge;
    } __attribute__((packed)) data;

    uint16_t crc16;  //crc16校验
} __attribute__((packed)) Data_Gimbal_s;


extern void Navigation_Usart1_init(void);

extern bool GetUartOffline(void);
extern bool GetUartRcOffline(void);
extern float GetUartGimbalYawMotorPos(void);
extern bool GetUartGimbalInitJudge(void);
extern uint32_t GetUartTimeStampForTest(void);


extern void Start_Navigation(void);

#endif
