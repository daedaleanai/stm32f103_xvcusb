
#include "usart.h"

extern inline void usart_wait(struct USART_Type* usart);

static inline int usart_index(struct USART_Type* usart) {
	if (usart == &USART1)
		return 0;
	if (usart == &USART2)
		return 1;
	if (usart == &USART3)
		return 2;

	for (;;)
		__NOP(); // hang

	return 0;
}

void usart_init(struct USART_Type* usart, int baud) {
	const int idx = usart_index(usart);

	usart->CR1 = 0;
	usart->CR2 = 0;
	usart->CR3 = 0;

	// PCLK2 for USART1, PCLK1 for USART 2, 3
	// this depends on SetSystemclockTo72MHz setting pclk1 to sysclk/2 and pclk1 to sysclc/2
	uint32_t clk = 72000000;
	if (usart != &USART1) {
		clk /= 2;
	}
	usart->BRR = clk / baud;

	static const enum IRQn_Type irqn[3] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
	NVIC_EnableIRQ(irqn[idx]);

	usart->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void usart_irq_handler(struct USART_Type* usart, struct Ringbuffer* rb) {
	if (!ringbuffer_empty(rb)) {
		if ((usart->SR & USART_SR_TXE) != 0) {
			usart->DR = ringbuffer_get_tail(rb);   
		}
	} else {
		usart->CR1 &= ~USART_CR1_TXEIE;
	}
	return;
}

size_t usart_puts(struct USART_Type* usart, struct Ringbuffer* rb, const char *buf, size_t len) {
    size_t r = ringbuffer_puts(rb, buf, len);
    // on overflow zap the buffer and leave a marker for the user that data was lost
    if (r < len) {
        ringbuffer_clear(rb);
        ringbuffer_puts(rb, "!OVFL!", 6);
    }
	usart->CR1 |= USART_CR1_TXEIE; // enable transmissions
    return r;
}
