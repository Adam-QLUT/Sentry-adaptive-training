/**
 * @file bsp_uart.h
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef BSP_UART_H
#define BSP_UART_H
#include "stm32h7xx_hal.h"
#include "struct_typedef.h"
#include "usart.h"

typedef UART_HandleTypeDef huart_t;


typedef enum __UartSendState {
    UART_SEND_FAIL = 0,
    UART_SEND_OK,
} UartSendState_e;

extern UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout);

#endif  // BSP_UART_H
