#ifndef USART_H
#define USART_H

#include "stm32g4xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t  instance;   /* 1 = USART1, 2 = USART2 */
    uint32_t baud_rate;
    uint8_t  stop_bits;  /* 1 or 2 */
} usart_config_t;

void usart_init(const usart_config_t *cfg);
void usart_write(uint8_t instance, const uint8_t *data, uint16_t len);
bool usart_rx_ready(uint8_t instance);
uint8_t usart_read_byte(uint8_t instance);

/* Low-level per-peripheral helpers (also used by syscall _write) */
void usart1_putc(char c);
void usart2_putc(char c);

#endif /* USART_H */
