/**
 * @file Nav_task.c
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "Nav_task.h"

#include "bsp_uart.h"
#include "cmsis_os.h"
#include "Navigation.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif

// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void Navigation_task(void const * pvParameters)
{
    Navigation_Usart1_Init();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1) {
        Start_Navigation();

        // 系统延时
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
