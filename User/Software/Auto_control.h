#ifndef __AUTO_CONTROL_H__
#define __AUTO_CONTROL_H__

#include "stdint.h"

#pragma pack(1)


typedef struct
{
    uint8_t header;
    uint8_t fire;
    float pitch;
    float yaw;
    float distence;
    uint8_t ender;
} MINIPC_data_t;


typedef struct
{
    uint8_t header;//
    uint8_t mode; 
    float roll;
    float pitch;
    float yaw;
    uint8_t fill;
    uint8_t ender;
} STM32_data_t;

typedef struct
{
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t remain_hp;
    uint16_t max_hp;
    uint16_t stage_remain_time;
    uint16_t bullet_remaining_num_17mm;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
    uint32_t rfid_status;
} STM32ROS_data_t;
extern STM32ROS_data_t stm32send_1, stm32send_6, stm32receive_1, stm32receive_6;

#pragma pack(4)

void STM32_to_MINIPC();
void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len);
void Auto_control();
void MINIPC_to_STM32();
void Navigation_send_message();

extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;

#endif