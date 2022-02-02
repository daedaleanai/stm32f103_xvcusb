#include "gpio2.h"

void gpioConfig(enum GPIO_Pin pins, enum GPIO_Conf mode) {
	assert(!((pins >> 24) & ((pins >> 24) - 1)));
	while ((pins >> 24) & ((pins >> 24) - 1))
		; // hang if mixed gpios

	struct GPIO_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;
	uint32_t          pos;
	for (pos = 0; pos < 16; ++pos) {
		if ((pins & (0x1 << pos)) == 0)
			continue;

		__IO uint32_t* cr    = &gpio->CRL + (pos / 8);
		uint32_t       shift = (pos % 8) * 4;

		uint32_t val = mode;
		if (val == Mode_IPU)
			val = Mode_IPD;

		uint32_t tmp = *cr;
		tmp &= ~(0xf << shift);
		tmp |= (val & 0xf) << shift;
		*cr = tmp;

		if (mode == Mode_IPD) {
			gpio->ODR &= ~(1U << pos);
		} else if (mode == Mode_IPU) {
			gpio->ODR |= (1U << pos);
		}
	}
}

uint32_t gpioLock(enum GPIO_Pin pins) {
	assert(!((pins >> 24) & ((pins >> 24) - 1)));
	while ((pins >> 24) & ((pins >> 24) - 1))
		; // hang if mixed gpios

	struct GPIO_Type* gpio = &GPIO_ALL[(pins >> 16) & 7].gpio;

	pins &= Pin_All;
	gpio->LCKR = pins | (1 << 16);
	gpio->LCKR = pins;
	gpio->LCKR = pins | (1 << 16);
	return gpio->LCKR;
}
