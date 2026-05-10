/**
 * @file interrupts.c
 * @brief Interrupt handlers for STM32G431
 *
 * Default handlers are defined as weak in startup_stm32g431xx.s
 * Override them here as needed.
 */

#include "stm32g4xx.h"

/**
 * @brief Non-Maskable Interrupt handler
 */
void NMI_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief Hard Fault handler
 */
void HardFault_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief Memory Management Fault handler
 */
void MemManage_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief Bus Fault handler
 */
void BusFault_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief Usage Fault handler
 */
void UsageFault_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief System Service Call handler (for RTOS, not used in bare-metal)
 */
void SVC_Handler(void)
{
}

/**
 * @brief Debug Monitor handler
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief Pending Service Call handler (for RTOS, not used in bare-metal)
 */
void PendSV_Handler(void)
{
}

extern void BSP_TickIncrement(void);

/**
 * @brief SysTick handler — 1 ms tick
 */
void SysTick_Handler(void)
{
    BSP_TickIncrement();
}

/* Add peripheral interrupt handlers below as needed */
/* Example:
void USART1_IRQHandler(void)
{
}
*/