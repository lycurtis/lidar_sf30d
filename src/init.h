/**
 * @file init.h
 * @brief System initialization interface
 */

#ifndef INIT_H
#define INIT_H

#include <stdint.h>

/* USART instance aliases */
#define BSP_USART_DEBUG       1
#define BSP_USART_DEBUG_BAUD  115200U
#define BSP_USART_LIDAR       2
#define BSP_USART_LIDAR_BAUD  921600U

void BSP_Init(void);
uint32_t BSP_GetTick(void);

#endif /* INIT_H */
