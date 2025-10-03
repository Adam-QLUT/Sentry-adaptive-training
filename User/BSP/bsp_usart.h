/**
 * @file bsp_usart.h
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"


extern void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
