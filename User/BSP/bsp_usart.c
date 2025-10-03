/**
 * @file bsp_usart.c
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;


void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    ((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->PAR = (uint32_t) & (USART1->DR);

    usart1_tx_dma_init();
}

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    ((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->PAR = (uint32_t) & (USART1->DR);
    ((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->M0AR = (uint32_t)(NULL);
    ((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    ((DMA_Stream_TypeDef *)hdma_usart1_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

