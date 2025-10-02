#ifndef __DMMOTOR_H__
#define __DMMOTOR_H__

#include "stdint.h"

#include "User_math.h"
#include "CAN_receive_send.h"
// ???????
#define USE_DAMIAO_MOTOR
#define QUANTITY_OF_CAN 3       // ???????can????????????????????????????????3??????
#define QUANTITY_OF_DJIMOTOR 11 // ???????????¡¤can?????????

// ??????
#define PI 3.1415926f
#define ECD_TO_ANGEL_DM 0.043945f //(360/8192),??????????????????

#ifdef USE_DAMIAO_MOTOR
/* DJImotorCAN send and receive ID */
typedef enum
{
    DM_J1310_1_4_send_ID = 0x3FE,

    DM_J1310_5_8_send_ID = 0x4FE,


    DM_ID1 = 0x301,
} Mcan_send_id_e;



typedef enum
{
    DM_1_1 = 0,  // 0
    DM_1_2,      // 1
    DM_1_3,      // 2
    DM_1_4,      // 3
    DM_1_5,      // 4
    DM_1_6,      // 5
    DM_1_7,      // 6
    DM_1_8,      // 7

    DM_2_1,  // 8
    DM_2_2,     
    DM_2_3,      
    DM_2_4,     
    DM_2_5,    
    DM_2_6,     
    DM_2_7,     
    DM_2_8,
    
    DM_3_1,  // 16
    DM_3_2,      
    DM_3_3,      
    DM_3_4,      
    DM_3_5,      
    DM_3_6,     
    DM_3_7,      
    DM_3_8,      


} DMcan_id;

typedef enum
{
   J4310 = 0,
} DMMotor_Type_e;

typedef struct
{
    // ????????
    int16_t set; // ?Ú…????? / ???
    DMMotor_Type_e Motor_type;

    // ??????
    uint16_t ecd;          // ?????????
    uint16_t last_ecd;     // ????¦Á??????????
    int16_t speed_rpm;        // ???RPM
    int16_t given_current; // ????????? mA
    uint8_t coil_temp;     // ?????? ?????
    uint8_t pcb_temp;     // pcb???
    

    // ????????
    long long ecd_cnt;  // ????????????
    double angle_cnt;   // ????????? degree
    double angle_zero;  // ??????0???? degree
    double angle;       // -180~180 degree
    double round_speed; // ??????? rpm
} DM_motor_data_s;
extern DM_motor_data_s DMMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

void DMMotor_init(DMMotor_Type_e motor_type, DMcan_id motor_id);
void DMMotor_set(int16_t val, DMcan_id motor_id);
void DMMotor_get_process_motor_data(DM_motor_data_s *ptr, uint8_t data[]);
void DMMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DMMotor_send_current(void);
#endif // USE_DAMIAO_MOTOR
#endif