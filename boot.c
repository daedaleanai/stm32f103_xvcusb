#include "stm32f103_md.h"

extern void main(void);                            // in main.c
extern void (*vector_table[])(void);               // in vector.c
extern char _sidata, _sdata, _edata, _sbss, _ebss; // provided by linker script

static inline void systemInit(void) {
	/* Reset the RCC clock configuration to the default reset state(for debug purpose) */
	RCC.CR |= RCC_CR_HSION;
	RCC.CFGR &= ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_MCO);
	RCC.CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_HSEBYP);
	RCC.CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE);

	/* Disable all interrupts and clear pending bits  */
	RCC.CIR = RCC_CIR_LSIRDYC | RCC_CIR_LSERDYC | RCC_CIR_HSIRDYC | RCC_CIR_HSERDYC | RCC_CIR_PLLRDYC | RCC_CIR_CSSC;
}

enum { HSE_RDY_TIMEOUT = 1500 };

// set system clock to HSE*pll9 = 72.00MHz (+/- .1%)
// SYSCLK, HCLK, PCLK2 and PCLK1 configuration
static int setSysClockTo72MHz(void) {
	// Enable HSE
	RCC.CR |= RCC_CR_HSEON;

	for (int i = 0; i < HSE_RDY_TIMEOUT; i++) {
		if (RCC.CR & RCC_CR_HSERDY)
			break;
	}

	if ((RCC.CR & RCC_CR_HSERDY) == 0) {
		// clock failed to become ready
		// TODO notify and exit
		return 0;
	}

	// Enable Prefetch Buffer
	FLASH_R.ACR |= FLASH_ACR_PRFTBE;

	// Flash 2 wait state
	FLASH_R.ACR &= ~FLASH_ACR_LATENCY;
	FLASH_R.ACR |= FLASH_ACR_LATENCY_2;

	// HCLK = SYSCLK
	RCC.CFGR |= RCC_CFGR_HPRE_DIV1;

	// PCLK2 = HCLK
	RCC.CFGR |= RCC_CFGR_PPRE2_DIV1;

	// PCLK1 = HCLK/2
	RCC.CFGR |= RCC_CFGR_PPRE1_DIV2;

	//  PLL configuration: PLLCLK = HSE * 9 = 72 MHz
	RCC.CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
	RCC.CFGR |= (RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);

	// Enable PLL
	RCC.CR |= RCC_CR_PLLON;

	// Wait till PLL is ready
	while ((RCC.CR & RCC_CR_PLLRDY) == 0)
		__NOP();

	/* Select PLL as system clock source */
	RCC.CFGR &= ~RCC_CFGR_SW;
	RCC.CFGR |= RCC_CFGR_SW_PLL;

	// Wait till PLL is used as system clock source
	while ((RCC.CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		__NOP();

	return 1;
}

void Reset_Handler(void) {
	char* src = &_sidata;
	char* dst = &_sdata;

	while (dst < &_edata)
		*dst++ = *src++;

	for (dst = &_sbss; dst < &_ebss; dst++)
		*dst = 0;

	SCB.VTOR = (uintptr_t)&vector_table; /* Vector Table Relocation in Internal FLASH. */

	systemInit();

	// just keep trying
	while (!setSysClockTo72MHz())
		__NOP();

	main();

	for (;;)
		__NOP(); // hang
}
