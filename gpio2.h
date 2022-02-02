#pragma once

#include "stm32f103_md.h"
#include <assert.h>

/* clang-format off */

enum GPIO_Conf {
	Mode_INA	= 0x0, // 0000 // input analog
	Mode_IN 	= 0x4, // 0100 // input floating
	Mode_IPD	= 0x8, // 1000 // input pullup/pulldown, 
	Mode_IPU	= 0xC, // 1100 // reserved -> map to 0x8 and set ODR

	Mode_Out_PP_10MHz = 0x1, Mode_Out_PP_2MHz = 0x2, Mode_Out_PP_50MHz = 0x3, // 00xx  output push-pull
	Mode_Out_OD_10MHz = 0x5, Mode_Out_OD_2MHz = 0x6, Mode_Out_OD_50MHz = 0x7, // 01xx  output open drain
	Mode_AF_PP_10MHz  = 0x9, Mode_AF_PP_2MHz  = 0xa, Mode_AF_PP_50MHz  = 0xb, // 10xx  af output push-pull
	Mode_AF_OD_10MHz  = 0xd, Mode_AF_OD_2MHz  = 0xe, Mode_AF_OD_50MHz  = 0xf, // 11xx  af output open drains
};

// Pins of the same GPIO can be combined by OR-ing them together: PA3|PA5, but mixing GPIOS like PB3|PH3 makes
// the functions hang. The lower 16 bits represent the pin mask, bits 24:17 index the GPIOx as an array of 1k pages with the GPIOA..H
// and the upper 8 bits are a bitmask of the same.  the functions that use the values check that only 1 bit is set in
// these upper 8, and assert/hang if this is not the case, since it means that pins of different GPIOx ports have
// been combined. 
enum GPIO_Pin {
	Pin_0   = 0x0001,
	Pin_1   = 0x0002,
	Pin_2   = 0x0004,
	Pin_3   = 0x0008,
	Pin_4   = 0x0010,
	Pin_5   = 0x0020,
	Pin_6   = 0x0040,
	Pin_7   = 0x0080,
	Pin_8   = 0x0100,
	Pin_9   = 0x0200,
	Pin_10  = 0x0400,
	Pin_11  = 0x0800,
	Pin_12  = 0x1000,
	Pin_13  = 0x2000,
	Pin_14  = 0x4000,
	Pin_15  = 0x8000,
	Pin_All = 0xFFFF,

	GPIO_A = 0x01000000,
	GPIO_B = 0x02010000,
	GPIO_C = 0x04020000,
#if 0
	GPIO_D = 0x08030000,
	GPIO_E = 0x10040000,
	GPIO_F = 0x20050000,
	GPIO_G = 0x40060000,
	GPIO_H = 0x80070000,
#endif

	PA0   = GPIO_A|Pin_0  , PB0   = GPIO_B|Pin_0  , PC0   = GPIO_C|Pin_0  ,// PD0   = GPIO_D|Pin_0  , PE0   = GPIO_E|Pin_0  , PF0   = GPIO_F|Pin_0  , PG0   = GPIO_G|Pin_0  , PH0   = GPIO_H|Pin_0  ,
	PA1   = GPIO_A|Pin_1  , PB1   = GPIO_B|Pin_1  , PC1   = GPIO_C|Pin_1  ,// PD1   = GPIO_D|Pin_1  , PE1   = GPIO_E|Pin_1  , PF1   = GPIO_F|Pin_1  , PG1   = GPIO_G|Pin_1  , PH1   = GPIO_H|Pin_1  ,
	PA2   = GPIO_A|Pin_2  , PB2   = GPIO_B|Pin_2  , PC2   = GPIO_C|Pin_2  ,// PD2   = GPIO_D|Pin_2  , PE2   = GPIO_E|Pin_2  , PF2   = GPIO_F|Pin_2  , PG2   = GPIO_G|Pin_2  , PH2   = GPIO_H|Pin_2  ,
	PA3   = GPIO_A|Pin_3  , PB3   = GPIO_B|Pin_3  , PC3   = GPIO_C|Pin_3  ,// PD3   = GPIO_D|Pin_3  , PE3   = GPIO_E|Pin_3  , PF3   = GPIO_F|Pin_3  , PG3   = GPIO_G|Pin_3  , PH3   = GPIO_H|Pin_3  ,
	PA4   = GPIO_A|Pin_4  , PB4   = GPIO_B|Pin_4  , PC4   = GPIO_C|Pin_4  ,// PD4   = GPIO_D|Pin_4  , PE4   = GPIO_E|Pin_4  , PF4   = GPIO_F|Pin_4  , PG4   = GPIO_G|Pin_4  , PH4   = GPIO_H|Pin_4  ,
	PA5   = GPIO_A|Pin_5  , PB5   = GPIO_B|Pin_5  , PC5   = GPIO_C|Pin_5  ,// PD5   = GPIO_D|Pin_5  , PE5   = GPIO_E|Pin_5  , PF5   = GPIO_F|Pin_5  , PG5   = GPIO_G|Pin_5  , PH5   = GPIO_H|Pin_5  ,
	PA6   = GPIO_A|Pin_6  , PB6   = GPIO_B|Pin_6  , PC6   = GPIO_C|Pin_6  ,// PD6   = GPIO_D|Pin_6  , PE6   = GPIO_E|Pin_6  , PF6   = GPIO_F|Pin_6  , PG6   = GPIO_G|Pin_6  , PH6   = GPIO_H|Pin_6  ,
	PA7   = GPIO_A|Pin_7  , PB7   = GPIO_B|Pin_7  , PC7   = GPIO_C|Pin_7  ,// PD7   = GPIO_D|Pin_7  , PE7   = GPIO_E|Pin_7  , PF7   = GPIO_F|Pin_7  , PG7   = GPIO_G|Pin_7  , PH7   = GPIO_H|Pin_7  ,
	PA8   = GPIO_A|Pin_8  , PB8   = GPIO_B|Pin_8  , PC8   = GPIO_C|Pin_8  ,// PD8   = GPIO_D|Pin_8  , PE8   = GPIO_E|Pin_8  , PF8   = GPIO_F|Pin_8  , PG8   = GPIO_G|Pin_8  , PH8   = GPIO_H|Pin_8  ,
	PA9   = GPIO_A|Pin_9  , PB9   = GPIO_B|Pin_9  , PC9   = GPIO_C|Pin_9  ,// PD9   = GPIO_D|Pin_9  , PE9   = GPIO_E|Pin_9  , PF9   = GPIO_F|Pin_9  , PG9   = GPIO_G|Pin_9  , PH9   = GPIO_H|Pin_9  ,
	PA10  = GPIO_A|Pin_10 , PB10  = GPIO_B|Pin_10 , PC10  = GPIO_C|Pin_10 ,// PD10  = GPIO_D|Pin_10 , PE10  = GPIO_E|Pin_10 , PF10  = GPIO_F|Pin_10 , PG10  = GPIO_G|Pin_10 , PH10  = GPIO_H|Pin_10 ,
	PA11  = GPIO_A|Pin_11 , PB11  = GPIO_B|Pin_11 , PC11  = GPIO_C|Pin_11 ,// PD11  = GPIO_D|Pin_11 , PE11  = GPIO_E|Pin_11 , PF11  = GPIO_F|Pin_11 , PG11  = GPIO_G|Pin_11 , PH11  = GPIO_H|Pin_11 ,
	PA12  = GPIO_A|Pin_12 , PB12  = GPIO_B|Pin_12 , PC12  = GPIO_C|Pin_12 ,// PD12  = GPIO_D|Pin_12 , PE12  = GPIO_E|Pin_12 , PF12  = GPIO_F|Pin_12 , PG12  = GPIO_G|Pin_12 , PH12  = GPIO_H|Pin_12 ,
	PA13  = GPIO_A|Pin_13 , PB13  = GPIO_B|Pin_13 , PC13  = GPIO_C|Pin_13 ,// PD13  = GPIO_D|Pin_13 , PE13  = GPIO_E|Pin_13 , PF13  = GPIO_F|Pin_13 , PG13  = GPIO_G|Pin_13 , PH13  = GPIO_H|Pin_13 ,
	PA14  = GPIO_A|Pin_14 , PB14  = GPIO_B|Pin_14 , PC14  = GPIO_C|Pin_14 ,// PD14  = GPIO_D|Pin_14 , PE14  = GPIO_E|Pin_14 , PF14  = GPIO_F|Pin_14 , PG14  = GPIO_G|Pin_14 , PH14  = GPIO_H|Pin_14 ,
	PA15  = GPIO_A|Pin_15 , PB15  = GPIO_B|Pin_15 , PC15  = GPIO_C|Pin_15 ,// PD15  = GPIO_D|Pin_15 , PE15  = GPIO_E|Pin_15 , PF15  = GPIO_F|Pin_15 , PG15  = GPIO_G|Pin_15 , PH15  = GPIO_H|Pin_15 ,
	PAAll = GPIO_A|Pin_All, PBAll = GPIO_B|Pin_All, PCAll = GPIO_C|Pin_All,// PDAll = GPIO_D|Pin_All, PEAll = GPIO_E|Pin_All, PFAll = GPIO_F|Pin_All, PGAll = GPIO_G|Pin_All, PHAll = GPIO_H|Pin_All,
};

void gpioConfig(enum GPIO_Pin pins, enum GPIO_Conf mode);

uint32_t gpioLock(enum GPIO_Pin pins);

extern union GPIO_Page {
	struct GPIO_Type gpio;
	struct { uint8_t bytes[0x400]; } page;
} GPIO_ALL[8];

static inline int validGPIOPins(enum GPIO_Pin pins) { return !((pins>>24) & ((pins>>24)-1)); }

static inline void digitalHi(enum GPIO_Pin hi) {
	assert(!((hi>>24) & ((hi>>24)-1)));
	while   ((hi>>24) & ((hi>>24)-1)); // hang if mixed gpios
	GPIO_ALL[(hi>>16)&0x7].gpio.BSRR = hi&Pin_All; 
}

static inline void digitalLo(enum GPIO_Pin lo) { 
	assert(!((lo>>24) & ((lo>>24)-1))); 
	while   ((lo>>24) & ((lo>>24)-1)); // hang if mixed gpios
	GPIO_ALL[(lo>>16)&0x7].gpio.BRR = lo&Pin_All; 
}

static inline void digitalHiLo(enum GPIO_Pin hi, enum GPIO_Pin lo) { 
	assert(!(((hi|lo)>>24) & (((hi|lo)>>24)-1)));
	while   (((hi|lo)>>24) & (((hi|lo)>>24)-1)); // hang if mixed gpios	
	GPIO_ALL[(lo>>16)&0x7].gpio.BSRR = (hi&Pin_All) | (lo<<16);
}

static inline void digitalToggle(enum GPIO_Pin pins) {
	assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios
	GPIO_ALL[(pins>>16)&0x7].gpio.ODR ^= pins&Pin_All;
}

static inline enum GPIO_Pin digitalIn(enum GPIO_Pin pins) { 
	assert(!((pins>>24) & ((pins>>24)-1)));
	while   ((pins>>24) & ((pins>>24)-1)); // hang if mixed gpios
	return GPIO_ALL[(pins>>16)&0x7].gpio.IDR & (pins & Pin_All); 
}

/* clang-format on */
