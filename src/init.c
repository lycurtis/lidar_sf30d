/**
 * @file init.c
 * @brief System initialization for STM32G431KB (NUCLEO-G431KB)
 *
 * HSE (24 MHz) + PLL → 170 MHz (Range 1 Boost, 4 WS)
 */

#include "stm32g4xx.h"
#include "init.h"

static void clock_init(void);
static void gpio_init(void);
static void systick_init(void);

static volatile uint32_t bsp_tick;

void BSP_Init(void)
{
    clock_init();
    gpio_init();
    systick_init();
}

uint32_t BSP_GetTick(void)
{
    return bsp_tick;
}

void BSP_TickIncrement(void)
{
    bsp_tick++;
}

static void clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        ;

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;

    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
    while (PWR->SR2 & PWR_SR2_VOSF)
        ;
    PWR->CR5 &= ~PWR_CR5_R1MODE;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY)
        ;

    /* HSE / M(6) * N(85) / R(2) = 170 MHz (HSE = 24 MHz) */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
                  | (5U  << RCC_PLLCFGR_PLLM_Pos)
                  | (85U << RCC_PLLCFGR_PLLN_Pos)
                  | (0U  << RCC_PLLCFGR_PLLR_Pos)
                  | RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS)
        ;

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    SystemCoreClock = 170000000U;
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    /* PA4 — LiDAR power-enable. Feeds the boost driver that supplies 5 V to
     * the SF30/D, so it must be HIGH whenever the firmware is running.
     * Drive the pin HIGH *before* switching MODER to output, otherwise the
     * default ODR value of 0 would cause a brief 0 V glitch on every reset
     * and momentarily kill the LiDAR's supply. */
    GPIOA->BSRR    = GPIO_BSRR_BS4;
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODE4) | (1U << GPIO_MODER_MODE4_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4;

    GPIOB->MODER   = (GPIOB->MODER & ~GPIO_MODER_MODE8) | (1U << GPIO_MODER_MODE8_Pos);
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT8;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8;

    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODE6) | (1U << GPIO_MODER_MODE6_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;

    /* PA7 — heartbeat indicator (toggled at 2 Hz from main). */
    GPIOA->MODER   = (GPIOA->MODER & ~GPIO_MODER_MODE7) | (1U << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7;
}

static void systick_init(void)
{
    SysTick_Config(SystemCoreClock / 1000U);  /* 1 ms tick */
}
