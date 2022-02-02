#include "stm32f103_md.h"

extern void _estack(void); // fake definition, will be filled in by linker script.

// Hang, let the watchdog reboot us.
void default_IRQ_Handler(void) {
	// todo: reset usart0 and report unexpected irq
	for (;;) {
		__WFE();
	}
}

// CM3 core fault handlers
void Reset_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void NonMaskableInt_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void MemoryManagement_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_8_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_9_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SVCall_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DebugMonitor_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_13_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));

// stm32f103_xx (medium density) IRQ handlers
void WWDG_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PVD_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TAMPER_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void FLASH_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RCC_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI0_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI1_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI3_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI4_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel1_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel3_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel4_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel5_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel6_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_Channel7_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void ADC1_2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USB_HP_CAN1_TX_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USB_LP_CAN1_RX0_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_RX1_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_SCE_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI9_5_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_BRK_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_UP_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_TRG_COM_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_CC_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM3_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM4_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_EV_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_ER_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_EV_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_ER_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI1_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART1_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART2_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART3_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI15_10_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTCAlarm_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USBWakeUp_IRQ_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));

__attribute__((section(".isr_vector"))) void (*vector_table[])(void) = {
    _estack,
    Reset_Handler,
    NonMaskableInt_Handler,
    Reserved_3_Handler,
    MemoryManagement_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    Reserved_7_Handler,
    Reserved_8_Handler,
    Reserved_9_Handler,
    Reserved_10_Handler,
    SVCall_Handler,
    DebugMonitor_Handler,
    Reserved_13_Handler,
    PendSV_Handler,
    SysTick_Handler,
    // IRQ's
    WWDG_IRQ_Handler,
    PVD_IRQ_Handler,
    TAMPER_IRQ_Handler,
    RTC_IRQ_Handler,
    FLASH_IRQ_Handler,
    RCC_IRQ_Handler,
    EXTI0_IRQ_Handler,
    EXTI1_IRQ_Handler,
    EXTI2_IRQ_Handler,
    EXTI3_IRQ_Handler,
    EXTI4_IRQ_Handler,
    DMA1_Channel1_IRQ_Handler,
    DMA1_Channel2_IRQ_Handler,
    DMA1_Channel3_IRQ_Handler,
    DMA1_Channel4_IRQ_Handler,
    DMA1_Channel5_IRQ_Handler,
    DMA1_Channel6_IRQ_Handler,
    DMA1_Channel7_IRQ_Handler,
    ADC1_2_IRQ_Handler,
    USB_HP_CAN1_TX_IRQ_Handler,
    USB_LP_CAN1_RX0_IRQ_Handler,
    CAN1_RX1_IRQ_Handler,
    CAN1_SCE_IRQ_Handler,
    EXTI9_5_IRQ_Handler,
    TIM1_BRK_IRQ_Handler,
    TIM1_UP_IRQ_Handler,
    TIM1_TRG_COM_IRQ_Handler,
    TIM1_CC_IRQ_Handler,
    TIM2_IRQ_Handler,
    TIM3_IRQ_Handler,
    TIM4_IRQ_Handler,
    I2C1_EV_IRQ_Handler,
    I2C1_ER_IRQ_Handler,
    I2C2_EV_IRQ_Handler,
    I2C2_ER_IRQ_Handler,
    SPI1_IRQ_Handler,
    SPI2_IRQ_Handler,
    USART1_IRQ_Handler,
    USART2_IRQ_Handler,
    USART3_IRQ_Handler,
    EXTI15_10_IRQ_Handler,
    RTCAlarm_IRQ_Handler,
    USBWakeUp_IRQ_Handler,
};
