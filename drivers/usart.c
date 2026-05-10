#include "usart.h"

/*
 * APB1 and APB2 run at SystemCoreClock (prescalers are /1).
 * BRR is computed at runtime so baud rates stay correct whether
 * the clock is 170 MHz (HSE path) or 80 MHz (HSI fallback).
 */

static void usart1_hw_init(uint32_t baud, uint8_t stop_bits)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PB6 -> AF7 (USART1_TX) */
    GPIOB->MODER  = (GPIOB->MODER & ~GPIO_MODER_MODE6) | (2U << GPIO_MODER_MODE6_Pos);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL6) | (7U << GPIO_AFRL_AFSEL6_Pos);

    /* PB7 -> AF7 (USART1_RX) */
    GPIOB->MODER  = (GPIOB->MODER & ~GPIO_MODER_MODE7) | (2U << GPIO_MODER_MODE7_Pos);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL7) | (7U << GPIO_AFRL_AFSEL7_Pos);

    USART1->CR1 = 0;
    USART1->CR2 = (stop_bits == 2) ? USART_CR2_STOP_1 : 0;
    USART1->CR3 = 0;
    USART1->BRR = (SystemCoreClock + baud / 2U) / baud;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void usart2_hw_init(uint32_t baud, uint8_t stop_bits)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    /* PA2 -> AF7 (USART2_TX) */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODE2) | (2U << GPIO_MODER_MODE2_Pos);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (7U << GPIO_AFRL_AFSEL2_Pos);

    /* PA3 -> AF7 (USART2_RX) */
    GPIOA->MODER  = (GPIOA->MODER & ~GPIO_MODER_MODE3) | (2U << GPIO_MODER_MODE3_Pos);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL3) | (7U << GPIO_AFRL_AFSEL3_Pos);

    USART2->CR1 = 0;
    USART2->CR2 = (stop_bits == 2) ? USART_CR2_STOP_1 : 0;
    USART2->CR3 = USART_CR3_OVRDIS;   /* drop old data on overrun instead of stalling */
    USART2->BRR = (SystemCoreClock + baud / 2U) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* flush any garbage already in the RX register */
    (void)USART2->RDR;
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
}

/* --- Public generic API ------------------------------------------------- */

void usart_init(const usart_config_t *cfg)
{
    if (cfg->instance == 1)
        usart1_hw_init(cfg->baud_rate, cfg->stop_bits);
    else if (cfg->instance == 2)
        usart2_hw_init(cfg->baud_rate, cfg->stop_bits);
}

void usart1_putc(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE))
        ;
    USART1->TDR = (uint8_t)c;
}

void usart2_putc(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE))
        ;
    USART2->TDR = (uint8_t)c;
}

void usart_write(uint8_t instance, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (instance == 1)
            usart1_putc((char)data[i]);
        else
            usart2_putc((char)data[i]);
    }
}

bool usart_rx_ready(uint8_t instance)
{
    if (instance == 1)
        return (USART1->ISR & USART_ISR_RXNE) != 0;
    else
        return (USART2->ISR & USART_ISR_RXNE) != 0;
}

uint8_t usart_read_byte(uint8_t instance)
{
    if (instance == 1)
        return (uint8_t)USART1->RDR;
    else
        return (uint8_t)USART2->RDR;
}
