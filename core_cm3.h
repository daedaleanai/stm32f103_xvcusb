#pragma once

/*
  based on core_cm3.h
  CMSIS Cortex-M3 Core Peripheral Access Layer Header File V1.30,
  30. October 2009 Copyright (C) 2009 ARM Limited.

  GCC version, no MPU

 TODO(lvd) unfortunately, use of unsigned long constants in enums triggers a -Wpedantic warning

*/

/**
 * @brief Configuration of the Cortex-M3 Processor and Core Peripherals
 */

enum { __NVIC_PRIO_BITS = 4 }; /*!< standard definition for NVIC Priority Bits */

/**
 * @brief STM32F103 Medium Density Interrupt Number Definition.
 */
enum IRQn_Type {
	None_IRQn  = -16, /*!< 0 position of estack reset pointer  */
	Reset_IRQn = -15, /*!< 1 Reset, not a real IRQ             */

	/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
	NonMaskableInt_IRQn   = -14, /*!< 2 Non Maskable Interrupt                             */
	Reserved_3_IRQn       = -13,
	MemoryManagement_IRQn = -12, /*!< 4 Cortex-M3 Memory Management Interrupt              */
	BusFault_IRQn         = -11, /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
	UsageFault_IRQn       = -10, /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
	Reserved_7_IRQn       = -9,
	Reserved_8_IRQn       = -8,
	Reserved_9_IRQn       = -7,
	Reserved_10_IRQn      = -6,
	SVCall_IRQn           = -5, /*!< 11 Cortex-M3 SV Call Interrupt                       */
	DebugMonitor_IRQn     = -4, /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
	Reserved_13_IRQn      = -3,
	PendSV_IRQn           = -2, /*!< 14 Cortex-M3 Pend SV Interrupt                       */
	SysTick_IRQn          = -1, /*!< 15 Cortex-M3 System Tick Interrupt                   */

	/******  STM32F103 Medium Density specific Interrupt Numbers ******************************************/
	WWDG_IRQn            = 0,  /*!< Window WatchDog Interrupt                            */
	PVD_IRQn             = 1,  /*!< PVD through EXTI Line detection Interrupt            */
	TAMPER_IRQn          = 2,  /*!< Tamper Interrupt                                     */
	RTC_IRQn             = 3,  /*!< RTC global Interrupt                                 */
	FLASH_IRQn           = 4,  /*!< FLASH global Interrupt                               */
	RCC_IRQn             = 5,  /*!< RCC global Interrupt                                 */
	EXTI0_IRQn           = 6,  /*!< EXTI Line0 Interrupt                                 */
	EXTI1_IRQn           = 7,  /*!< EXTI Line1 Interrupt                                 */
	EXTI2_IRQn           = 8,  /*!< EXTI Line2 Interrupt                                 */
	EXTI3_IRQn           = 9,  /*!< EXTI Line3 Interrupt                                 */
	EXTI4_IRQn           = 10, /*!< EXTI Line4 Interrupt                                 */
	DMA1_Channel1_IRQn   = 11, /*!< DMA1 Channel 1 global Interrupt                      */
	DMA1_Channel2_IRQn   = 12, /*!< DMA1 Channel 2 global Interrupt                      */
	DMA1_Channel3_IRQn   = 13, /*!< DMA1 Channel 3 global Interrupt                      */
	DMA1_Channel4_IRQn   = 14, /*!< DMA1 Channel 4 global Interrupt                      */
	DMA1_Channel5_IRQn   = 15, /*!< DMA1 Channel 5 global Interrupt                      */
	DMA1_Channel6_IRQn   = 16, /*!< DMA1 Channel 6 global Interrupt                      */
	DMA1_Channel7_IRQn   = 17, /*!< DMA1 Channel 7 global Interrupt                      */
	ADC1_2_IRQn          = 18, /*!< ADC1 and ADC2 global Interrupt                       */
	USB_HP_CAN1_TX_IRQn  = 19, /*!< USB Device High Priority or CAN1 TX Interrupts       */
	USB_LP_CAN1_RX0_IRQn = 20, /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
	CAN1_RX1_IRQn        = 21, /*!< CAN1 RX1 Interrupt                                   */
	CAN1_SCE_IRQn        = 22, /*!< CAN1 SCE Interrupt                                   */
	EXTI9_5_IRQn         = 23, /*!< External Line[9:5] Interrupts                        */
	TIM1_BRK_IRQn        = 24, /*!< TIM1 Break Interrupt                                 */
	TIM1_UP_IRQn         = 25, /*!< TIM1 Update Interrupt                                */
	TIM1_TRG_COM_IRQn    = 26, /*!< TIM1 Trigger and Commutation Interrupt               */
	TIM1_CC_IRQn         = 27, /*!< TIM1 Capture Compare Interrupt                       */
	TIM2_IRQn            = 28, /*!< TIM2 global Interrupt                                */
	TIM3_IRQn            = 29, /*!< TIM3 global Interrupt                                */
	TIM4_IRQn            = 30, /*!< TIM4 global Interrupt                                */
	I2C1_EV_IRQn         = 31, /*!< I2C1 Event Interrupt                                 */
	I2C1_ER_IRQn         = 32, /*!< I2C1 Error Interrupt                                 */
	I2C2_EV_IRQn         = 33, /*!< I2C2 Event Interrupt                                 */
	I2C2_ER_IRQn         = 34, /*!< I2C2 Error Interrupt                                 */
	SPI1_IRQn            = 35, /*!< SPI1 global Interrupt                                */
	SPI2_IRQn            = 36, /*!< SPI2 global Interrupt                                */
	USART1_IRQn          = 37, /*!< USART1 global Interrupt                              */
	USART2_IRQn          = 38, /*!< USART2 global Interrupt                              */
	USART3_IRQn          = 39, /*!< USART3 global Interrupt                              */
	EXTI15_10_IRQn       = 40, /*!< External Line[15:10] Interrupts                      */
	RTCAlarm_IRQn        = 41, /*!< RTC Alarm through EXTI Line Interrupt                */
	USBWakeUp_IRQn       = 42  /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
};

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#define __I volatile const /*!< defines 'read only' permissions      */
#define __O volatile       /*!< defines 'write only' permissions     */
#define __IO volatile      /*!< defines 'read / write' permissions   */

/*******************************************************************************
 *                 Register Abstraction
 ******************************************************************************/

/**
 * memory mapped structure for Nested Vectored Interrupt Controller (NVIC)
 */
struct NVIC_Type {
	__IO uint32_t ISER[8]; /*!< Offset: 0x000  Interrupt Set Enable Register           */
	uint32_t      RESERVED0[24];
	__IO uint32_t ICER[8]; /*!< Offset: 0x080  Interrupt Clear Enable Register         */
	uint32_t      RSERVED1[24];
	__IO uint32_t ISPR[8]; /*!< Offset: 0x100  Interrupt Set Pending Register          */
	uint32_t      RESERVED2[24];
	__IO uint32_t ICPR[8]; /*!< Offset: 0x180  Interrupt Clear Pending Register        */
	uint32_t      RESERVED3[24];
	__IO uint32_t IABR[8]; /*!< Offset: 0x200  Interrupt Active bit Register           */
	uint32_t      RESERVED4[56];
	__IO uint8_t IP[240]; /*!< Offset: 0x300  Interrupt Priority Register (8Bit wide) */
	uint32_t     RESERVED5[644];
	__O uint32_t STIR; /*!< Offset: 0xE00  Software Trigger Interrupt Register     */
};

/**
 * memory mapped structure for System Control Block (SCB)
 */
struct SCB_Type {
	__I uint32_t CPUID;   /*!< Offset: 0x00  CPU ID Base Register                                  */
	__IO uint32_t ICSR;   /*!< Offset: 0x04  Interrupt Control State Register                      */
	__IO uint32_t VTOR;   /*!< Offset: 0x08  Vector Table Offset Register                          */
	__IO uint32_t AIRCR;  /*!< Offset: 0x0C  Application Interrupt / Reset Control Register        */
	__IO uint32_t SCR;    /*!< Offset: 0x10  System Control Register                               */
	__IO uint32_t CCR;    /*!< Offset: 0x14  Configuration Control Register                        */
	__IO uint8_t SHP[12]; /*!< Offset: 0x18  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	__IO uint32_t SHCSR;  /*!< Offset: 0x24  System Handler Control and State Register             */
	__IO uint32_t CFSR;   /*!< Offset: 0x28  Configurable Fault Status Register                    */
	__IO uint32_t HFSR;   /*!< Offset: 0x2C  Hard Fault Status Register                            */
	__IO uint32_t DFSR;   /*!< Offset: 0x30  Debug Fault Status Register                           */
	__IO uint32_t MMFAR;  /*!< Offset: 0x34  Mem Manage Address Register                           */
	__IO uint32_t BFAR;   /*!< Offset: 0x38  Bus Fault Address Register                            */
	__IO uint32_t AFSR;   /*!< Offset: 0x3C  Auxiliary Fault Status Register                       */
	__I uint32_t PFR[2];  /*!< Offset: 0x40  Processor Feature Register                            */
	__I uint32_t DFR;     /*!< Offset: 0x48  Debug Feature Register                                */
	__I uint32_t ADR;     /*!< Offset: 0x4C  Auxiliary Feature Register                            */
	__I uint32_t MMFR[4]; /*!< Offset: 0x50  Memory Model Feature Register                         */
	__I uint32_t ISAR[5]; /*!< Offset: 0x60  ISA Feature Register                                  */
};

enum {
	/* SCB CPUID Register Definitions */
	SCB_CPUID_IMPLEMENTER = (0xFFul << 24), /*!< SCB CPUID: IMPLEMENTER Mask */
	SCB_CPUID_VARIANT     = (0xFul << 20),  /*!< SCB CPUID: VARIANT Mask */
	SCB_CPUID_PARTNO      = (0xFFFul << 4), /*!< SCB CPUID: PARTNO Mask */
	SCB_CPUID_REVISION    = (0xFul << 0),   /*!< SCB CPUID: REVISION Mask */

	/* SCB Interrupt Control State Register Definitions */
	SCB_ICSR_NMIPENDSET  = (1ul << 31),     /*!< SCB ICSR: NMIPENDSET Mask */
	SCB_ICSR_PENDSVSET   = (1ul << 28),     /*!< SCB ICSR: PENDSVSET Mask */
	SCB_ICSR_PENDSVCLR   = (1ul << 27),     /*!< SCB ICSR: PENDSVCLR Mask */
	SCB_ICSR_PENDSTSET   = (1ul << 26),     /*!< SCB ICSR: PENDSTSET Mask */
	SCB_ICSR_PENDSTCLR   = (1ul << 25),     /*!< SCB ICSR: PENDSTCLR Mask */
	SCB_ICSR_ISRPREEMPT  = (1ul << 23),     /*!< SCB ICSR: ISRPREEMPT Mask */
	SCB_ICSR_ISRPENDING  = (1ul << 22),     /*!< SCB ICSR: ISRPENDING Mask */
	SCB_ICSR_VECTPENDING = (0x1FFul << 12), /*!< SCB ICSR: VECTPENDING Mask */
	SCB_ICSR_RETTOBASE   = (1ul << 11),     /*!< SCB ICSR: RETTOBASE Mask */
	SCB_ICSR_VECTACTIVE  = (0x1FFul << 0),  /*!< SCB ICSR: VECTACTIVE Mask */

	/* SCB Interrupt Control State Register Definitions */
	SCB_VTOR_TBLBASE = (0x1FFul << 29),   /*!< SCB VTOR: TBLBASE Mask */
	SCB_VTOR_TBLOFF  = (0x3FFFFFul << 7), /*!< SCB VTOR: TBLOFF Mask */

	/* SCB Application Interrupt and Reset Control Register Definitions */
	SCB_AIRCR_VECTKEY       = (0xFFFFul << 16), /*!< SCB AIRCR: VECTKEY Mask */
	SCB_AIRCR_VECTKEYSTAT   = (0xFFFFul << 16), /*!< SCB AIRCR: VECTKEYSTAT Mask */
	SCB_AIRCR_ENDIANESS     = (1ul << 15),      /*!< SCB AIRCR: ENDIANESS Mask */
	SCB_AIRCR_PRIGROUP      = (7ul << 8),       /*!< SCB AIRCR: PRIGROUP Mask */
	SCB_AIRCR_SYSRESETREQ   = (1ul << 2),       /*!< SCB AIRCR: SYSRESETREQ Mask */
	SCB_AIRCR_VECTCLRACTIVE = (1ul << 1),       /*!< SCB AIRCR: VECTCLRACTIVE Mask */
	SCB_AIRCR_VECTRESET     = (1ul << 0),       /*!< SCB AIRCR: VECTRESET Mask */

	/* SCB System Control Register Definitions */
	SCB_SCR_SEVONPEND   = (1ul << 4), /*!< SCB SCR: SEVONPEND Mask */
	SCB_SCR_SLEEPDEEP   = (1ul << 2), /*!< SCB SCR: SLEEPDEEP Mask */
	SCB_SCR_SLEEPONEXIT = (1ul << 1), /*!< SCB SCR: SLEEPONEXIT Mask */

	/* SCB Configuration Control Register Definitions */
	SCB_CCR_STKALIGN       = (1ul << 9), /*!< SCB CCR: STKALIGN Mask */
	SCB_CCR_BFHFNMIGN      = (1ul << 8), /*!< SCB CCR: BFHFNMIGN Mask */
	SCB_CCR_DIV_0_TRP      = (1ul << 4), /*!< SCB CCR: DIV_0_TRP Mask */
	SCB_CCR_UNALIGN_TRP    = (1ul << 3), /*!< SCB CCR: UNALIGN_TRP Mask */
	SCB_CCR_USERSETMPEND   = (1ul << 1), /*!< SCB CCR: USERSETMPEND Mask */
	SCB_CCR_NONBASETHRDENA = (1ul << 0), /*!< SCB CCR: NONBASETHRDENA Mask */

	/* SCB System Handler Control and State Register Definitions */
	SCB_SHCSR_USGFAULTENA    = (1ul << 18), /*!< SCB SHCSR: USGFAULTENA Mask */
	SCB_SHCSR_BUSFAULTENA    = (1ul << 17), /*!< SCB SHCSR: BUSFAULTENA Mask */
	SCB_SHCSR_MEMFAULTENA    = (1ul << 16), /*!< SCB SHCSR: MEMFAULTENA Mask */
	SCB_SHCSR_SVCALLPENDED   = (1ul << 15), /*!< SCB SHCSR: SVCALLPENDED Mask */
	SCB_SHCSR_BUSFAULTPENDED = (1ul << 14), /*!< SCB SHCSR: BUSFAULTPENDED Mask */
	SCB_SHCSR_MEMFAULTPENDED = (1ul << 13), /*!< SCB SHCSR: MEMFAULTPENDED Mask */
	SCB_SHCSR_USGFAULTPENDED = (1ul << 12), /*!< SCB SHCSR: USGFAULTPENDED Mask */
	SCB_SHCSR_SYSTICKACT     = (1ul << 11), /*!< SCB SHCSR: SYSTICKACT Mask */
	SCB_SHCSR_PENDSVACT      = (1ul << 10), /*!< SCB SHCSR: PENDSVACT Mask */
	SCB_SHCSR_MONITORACT     = (1ul << 8),  /*!< SCB SHCSR: MONITORACT Mask */
	SCB_SHCSR_SVCALLACT      = (1ul << 7),  /*!< SCB SHCSR: SVCALLACT Mask */
	SCB_SHCSR_USGFAULTACT    = (1ul << 3),  /*!< SCB SHCSR: USGFAULTACT Mask */
	SCB_SHCSR_BUSFAULTACT    = (1ul << 1),  /*!< SCB SHCSR: BUSFAULTACT Mask */
	SCB_SHCSR_MEMFAULTACT    = (1ul << 0),  /*!< SCB SHCSR: MEMFAULTACT Mask */

	/* SCB Configurable Fault Status Registers Definitions */
	SCB_CFSR_USGFAULTSR = (0xFFFFul << 16), /*!< SCB CFSR: Usage Fault Status Register Mask  */
	SCB_CFSR_BUSFAULTSR = (0xFFul << 8),    /*!< SCB CFSR: Bus Fault Status Register Mask */
	SCB_CFSR_MEMFAULTSR = (0xFFul << 0),    /*!< SCB CFSR: Memory Manage Fault Status Register Mask */

	/* SCB Hard Fault Status Registers Definitions */
	SCB_HFSR_DEBUGEVT = (1ul << 31), /*!< SCB HFSR: DEBUGEVT Mask */
	SCB_HFSR_FORCED   = (1ul << 30), /*!< SCB HFSR: FORCED Mask */
	SCB_HFSR_VECTTBL  = (1ul << 1),  /*!< SCB HFSR: VECTTBL Mask */

	/* SCB Debug Fault Status Register Definitions */
	SCB_DFSR_EXTERNAL = (1ul << 4), /*!< SCB DFSR: EXTERNAL Mask */
	SCB_DFSR_VCATCH   = (1ul << 3), /*!< SCB DFSR: VCATCH Mask */
	SCB_DFSR_DWTTRAP  = (1ul << 2), /*!< SCB DFSR: DWTTRAP Mask */
	SCB_DFSR_BKPT     = (1ul << 1), /*!< SCB DFSR: BKPT Mask */
	SCB_DFSR_HALTED   = (1ul << 0), /*!< SCB DFSR: HALTED Mask */
};

/**
 * memory mapped structure for SysTick
 */
struct SysTick_Type {
	__IO uint32_t CTRL; /*!< Offset: 0x00  SysTick Control and Status Register */
	__IO uint32_t LOAD; /*!< Offset: 0x04  SysTick Reload Value Register       */
	__IO uint32_t VAL;  /*!< Offset: 0x08  SysTick Current Value Register      */
	__I uint32_t CALIB; /*!< Offset: 0x0C  SysTick Calibration Register        */
};

enum {
	/* SysTick Control / Status Register Definitions */
	SysTick_CTRL_COUNTFLAG = (1ul << 16), /*!< SysTick CTRL: COUNTFLAG Mask */
	SysTick_CTRL_CLKSOURCE = (1ul << 2),  /*!< SysTick CTRL: CLKSOURCE Mask */
	SysTick_CTRL_TICKINT   = (1ul << 1),  /*!< SysTick CTRL: TICKINT Mask */
	SysTick_CTRL_ENABLE    = (1ul << 0),  /*!< SysTick CTRL: ENABLE Mask */

	/* SysTick Reload Register Definitions */
	SysTick_LOAD_RELOAD = (0xFFFFFFul << 0), /*!< SysTick LOAD: RELOAD Mask */

	/* SysTick Current Register Definitions */
	SysTick_VAL_CURRENT = (0xFFFFFFul << 0), /*!< SysTick VAL: CURRENT Mask */

	/* SysTick Calibration Register Definitions */
	SysTick_CALIB_NOREF = (1ul << 31),       /*!< SysTick CALIB: NOREF Mask */
	SysTick_CALIB_SKEW  = (1ul << 20),       /*!< SysTick CALIB: SKEW Mask */
	SysTick_CALIB_TENMS = (0xFFFFFFul << 0), /*!< SysTick CALIB: TENMS Mask */
};

/**
 * memory mapped structure for Instrumentation Trace Macrocell (ITM)
 */
struct ITM_Type {
	__O union {
		__O uint8_t u8;   /*!< Offset:       ITM Stimulus Port 8-bit                   */
		__O uint16_t u16; /*!< Offset:       ITM Stimulus Port 16-bit                  */
		__O uint32_t u32; /*!< Offset:       ITM Stimulus Port 32-bit                  */
	} PORT[32];           /*!< Offset: 0x00  ITM Stimulus Port Registers               */
	uint32_t RESERVED0[864];
	__IO uint32_t TER; /*!< Offset:       ITM Trace Enable Register                 */
	uint32_t      RESERVED1[15];
	__IO uint32_t TPR; /*!< Offset:       ITM Trace Privilege Register              */
	uint32_t      RESERVED2[15];
	__IO uint32_t TCR; /*!< Offset:       ITM Trace Control Register                */
	uint32_t      RESERVED3[29];
	__IO uint32_t IWR;  /*!< Offset:       ITM Integration Write Register            */
	__IO uint32_t IRR;  /*!< Offset:       ITM Integration Read Register             */
	__IO uint32_t IMCR; /*!< Offset:      ITM Integration Mode Control Register     */
	uint32_t      RESERVED4[43];
	__IO uint32_t LAR; /*!< Offset:       ITM Lock Access Register                  */
	__IO uint32_t LSR; /*!< Offset:       ITM Lock Status Register                  */
	uint32_t      RESERVED5[6];
	__I uint32_t PID4; /*!< Offset:       ITM Peripheral Identification Register #4 */
	__I uint32_t PID5; /*!< Offset:       ITM Peripheral Identification Register #5 */
	__I uint32_t PID6; /*!< Offset:       ITM Peripheral Identification Register #6 */
	__I uint32_t PID7; /*!< Offset:       ITM Peripheral Identification Register #7 */
	__I uint32_t PID0; /*!< Offset:       ITM Peripheral Identification Register #0 */
	__I uint32_t PID1; /*!< Offset:       ITM Peripheral Identification Register #1 */
	__I uint32_t PID2; /*!< Offset:       ITM Peripheral Identification Register #2 */
	__I uint32_t PID3; /*!< Offset:       ITM Peripheral Identification Register #3 */
	__I uint32_t CID0; /*!< Offset:       ITM Component  Identification Register #0 */
	__I uint32_t CID1; /*!< Offset:       ITM Component  Identification Register #1 */
	__I uint32_t CID2; /*!< Offset:       ITM Component  Identification Register #2 */
	__I uint32_t CID3; /*!< Offset:       ITM Component  Identification Register #3 */
};

enum {
	/* ITM Trace Privilege Register Definitions */
	ITM_TPR_PRIVMASK = (0xFul << 0), /*!< ITM TPR: PRIVMASK Mask */

	/* ITM Trace Control Register Definitions */
	ITM_TCR_BUSY       = (1ul << 23),    /*!< ITM TCR: BUSY Mask */
	ITM_TCR_ATBID      = (0x7Ful << 16), /*!< ITM TCR: ATBID Mask */
	ITM_TCR_TSPrescale = (3ul << 8),     /*!< ITM TCR: TSPrescale Mask */
	ITM_TCR_SWOENA     = (1ul << 4),     /*!< ITM TCR: SWOENA Mask */
	ITM_TCR_DWTENA     = (1ul << 3),     /*!< ITM TCR: DWTENA Mask */
	ITM_TCR_SYNCENA    = (1ul << 2),     /*!< ITM TCR: SYNCENA Mask */
	ITM_TCR_TSENA      = (1ul << 1),     /*!< ITM TCR: TSENA Mask */
	ITM_TCR_ITMENA     = (1ul << 0),     /*!< ITM TCR: ITM Enable bit Mask */

	/* ITM Integration Write Register Definitions */
	ITM_IWR_ATVALIDM = (1ul << 0), /*!< ITM IWR: ATVALIDM Mask */

	/* ITM Integration Read Register Definitions */
	ITM_IRR_ATREADYM = (1ul << 0), /*!< ITM IRR: ATREADYM Mask */

	/* ITM Integration Mode Control Register Definitions */
	ITM_IMCR_INTEGRATION = (1ul << 0), /*!< ITM IMCR: INTEGRATION Mask */

	/* ITM Lock Status Register Definitions */
	ITM_LSR_ByteAcc = (1ul << 2), /*!< ITM LSR: ByteAcc Mask */
	ITM_LSR_Access  = (1ul << 1), /*!< ITM LSR: Access Mask */
	ITM_LSR_Present = (1ul << 0), /*!< ITM LSR: Present Mask */
};
/**
 * memory mapped structure for Interrupt Type
 */
struct InterruptType_Type {
	uint32_t RESERVED0;
	__I uint32_t ICTR;   /*!< Offset: 0x04  Interrupt Control Type Register */
	__IO uint32_t ACTLR; /*!< Offset: 0x08  Auxiliary Control Register __CM3_REV >= 0x200))  */
};

enum {
	/* Interrupt Controller Type Register Definitions */
	InterruptType_ICTR_INTLINESNUM = (0x1Ful << 0), /*!< InterruptType ICTR: INTLINESNUM Mask */

	/* Auxiliary Control Register Definitions */
	InterruptType_ACTLR_DISFOLD    = (1ul << 2), /*!< InterruptType ACTLR: DISFOLD Mask */
	InterruptType_ACTLR_DISDEFWBUF = (1ul << 1), /*!< InterruptType ACTLR: DISDEFWBUF Mask */
	InterruptType_ACTLR_DISMCYCINT = (1ul << 0), /*!< InterruptType ACTLR: DISMCYCINT Mask */
};

/**
 * memory mapped structure for Core Debug Register
 */
struct CoreDebug_Type {
	__IO uint32_t DHCSR; /*!< Offset: 0x00  Debug Halting Control and Status Register    */
	__O uint32_t DCRSR;  /*!< Offset: 0x04  Debug Core Register Selector Register        */
	__IO uint32_t DCRDR; /*!< Offset: 0x08  Debug Core Register Data Register            */
	__IO uint32_t DEMCR; /*!< Offset: 0x0C  Debug Exception and Monitor Control Register */
};

enum {
	/* Debug Halting Control and Status Register */
	CoreDebug_DHCSR_DBGKEY      = (0xFFFFul << 16), /*!< CoreDebug DHCSR: DBGKEY Mask */
	CoreDebug_DHCSR_S_RESET_ST  = (1ul << 25),      /*!< CoreDebug DHCSR: S_RESET_ST Mask     */
	CoreDebug_DHCSR_S_RETIRE_ST = (1ul << 24),      /*!< CoreDebug DHCSR: S_RETIRE_ST Mask */
	CoreDebug_DHCSR_S_LOCKUP    = (1ul << 19),      /*!< CoreDebug DHCSR: S_LOCKUP Mask */
	CoreDebug_DHCSR_S_SLEEP     = (1ul << 18),      /*!< CoreDebug DHCSR: S_SLEEP Mask */
	CoreDebug_DHCSR_S_HALT      = (1ul << 17),      /*!< CoreDebug DHCSR: S_HALT Mask */
	CoreDebug_DHCSR_S_REGRDY    = (1ul << 16),      /*!< CoreDebug DHCSR: S_REGRDY Mask */
	CoreDebug_DHCSR_C_SNAPSTALL = (1ul << 5),       /*!< CoreDebug DHCSR: C_SNAPSTALL Mask */
	CoreDebug_DHCSR_C_MASKINTS  = (1ul << 3),       /*!< CoreDebug DHCSR: C_MASKINTS Mask */
	CoreDebug_DHCSR_C_STEP      = (1ul << 2),       /*!< CoreDebug DHCSR: C_STEP Mask */
	CoreDebug_DHCSR_C_HALT      = (1ul << 1),       /*!< CoreDebug DHCSR: C_HALT Mask */
	CoreDebug_DHCSR_C_DEBUGEN   = (1ul << 0),       /*!< CoreDebug DHCSR: C_DEBUGEN Mask */

	/* Debug Core Register Selector Register */
	CoreDebug_DCRSR_REGWnR = (1ul << 16),   /*!< CoreDebug DCRSR: REGWnR Mask */
	CoreDebug_DCRSR_REGSEL = (0x1Ful << 0), /*!< CoreDebug DCRSR: REGSEL Mask */

	/* Debug Exception and Monitor Control Register */
	CoreDebug_DEMCR_TRCENA       = (1ul << 24), /*!< CoreDebug DEMCR: TRCENA Mask */
	CoreDebug_DEMCR_MON_REQ      = (1ul << 19), /*!< CoreDebug DEMCR: MON_REQ Mask */
	CoreDebug_DEMCR_MON_STEP     = (1ul << 18), /*!< CoreDebug DEMCR: MON_STEP Mask */
	CoreDebug_DEMCR_MON_PEND     = (1ul << 17), /*!< CoreDebug DEMCR: MON_PEND Mask */
	CoreDebug_DEMCR_MON_EN       = (1ul << 16), /*!< CoreDebug DEMCR: MON_EN Mask */
	CoreDebug_DEMCR_VC_HARDERR   = (1ul << 10), /*!< CoreDebug DEMCR: VC_HARDERR Mask */
	CoreDebug_DEMCR_VC_INTERR    = (1ul << 9),  /*!< CoreDebug DEMCR: VC_INTERR Mask */
	CoreDebug_DEMCR_VC_BUSERR    = (1ul << 8),  /*!< CoreDebug DEMCR: VC_BUSERR Mask */
	CoreDebug_DEMCR_VC_STATERR   = (1ul << 7),  /*!< CoreDebug DEMCR: VC_STATERR Mask*/
	CoreDebug_DEMCR_VC_CHKERR    = (1ul << 6),  /*!< CoreDebug DEMCR: VC_CHKERR Mask */
	CoreDebug_DEMCR_VC_NOCPERR   = (1ul << 5),  /*!< CoreDebug DEMCR: VC_NOCPERR Mask */
	CoreDebug_DEMCR_VC_MMERR     = (1ul << 4),  /*!< CoreDebug DEMCR: VC_MMERR Mask */
	CoreDebug_DEMCR_VC_CORERESET = (1ul << 0),  /*!< CoreDebug DEMCR: VC_CORERESET Mask */
};

extern struct ITM_Type           ITM;           /*!< ITM configuration struct          */
extern struct InterruptType_Type InterruptType; /*!< Interrupt Type Register           */
extern struct SysTick_Type       SysTick;       /*!< SysTick configuration struct      */
extern struct NVIC_Type          NVIC;          /*!< NVIC configuration struct         */
extern struct SCB_Type           SCB;           /*!< SCB configuration struct          */
extern struct CoreDebug_Type     CoreDebug;     /*!< Core Debug configuration struct   */

#undef __I
#undef __O
#undef __IO

/*******************************************************************************
 *                Hardware Abstraction Layer
 ******************************************************************************/

/* ###################  Compiler specific Intrinsics  ########################### */

/* GNU gcc specific functions */

static inline void __enable_irq() { __asm volatile("cpsie i"); }
static inline void __disable_irq() { __asm volatile("cpsid i"); }

static inline void __enable_fault_irq() { __asm volatile("cpsie f"); }
static inline void __disable_fault_irq() { __asm volatile("cpsid f"); }

static inline void __NOP() { __asm volatile("nop"); }
static inline void __WFI() { __asm volatile("wfi"); }
static inline void __WFE() { __asm volatile("wfe"); }
static inline void __SEV() { __asm volatile("sev"); }
static inline void __ISB() { __asm volatile("isb"); }
static inline void __DSB() { __asm volatile("dsb"); }
static inline void __DMB() { __asm volatile("dmb"); }
static inline void __CLREX() { __asm volatile("clrex"); }

/**
 * @brief  Return the Process Stack Pointer
 *
 * @return ProcessStackPointer
 *
 * Return the actual process stack pointer
 */
extern uint32_t __get_PSP(void);

/**
 * @brief  Set the Process Stack Pointer
 *
 * @param  topOfProcStack  Process Stack Pointer
 *
 * Assign the value ProcessStackPointer to the MSP
 * (process stack pointer) Cortex processor register
 */
extern void __set_PSP(uint32_t topOfProcStack);

/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
extern uint32_t __get_MSP(void);

/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP
 * (main stack pointer) Cortex processor register
 */
extern void __set_MSP(uint32_t topOfMainStack);

/**
 * @brief  Return the Base Priority value
 *
 * @return BasePriority
 *
 * Return the content of the base priority register
 */
extern uint32_t __get_BASEPRI(void);

/**
 * @brief  Set the Base Priority value
 *
 * @param  basePri  BasePriority
 *
 * Set the base priority register
 */
extern void __set_BASEPRI(uint32_t basePri);

/**
 * @brief  Return the Priority Mask value
 *
 * @return PriMask
 *
 * Return state of the priority mask bit from the priority mask register
 */
extern uint32_t __get_PRIMASK(void);

/**
 * @brief  Set the Priority Mask value
 *
 * @param  priMask  PriMask
 *
 * Set the priority mask bit in the priority mask register
 */
extern void __set_PRIMASK(uint32_t priMask);

/**
 * @brief  Return the Fault Mask value
 *
 * @return FaultMask
 *
 * Return the content of the fault mask register
 */
extern uint32_t __get_FAULTMASK(void);

/**
 * @brief  Set the Fault Mask value
 *
 * @param  faultMask  faultMask value
 *
 * Set the fault mask register
 */
extern void __set_FAULTMASK(uint32_t faultMask);

/**
 * @brief  Return the Control Register value
 *
 *  @return Control value
 *
 * Return the content of the control register
 */
extern uint32_t __get_CONTROL(void);

/**
 * @brief  Set the Control Register value
 *
 * @param  control  Control value
 *
 * Set the control register
 */
extern void __set_CONTROL(uint32_t control);

/**
 * @brief  Reverse byte order in integer value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in integer value
 */
extern uint32_t __REV(uint32_t value);

/**
 * @brief  Reverse byte order in unsigned short value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in unsigned short value
 */
extern uint32_t __REV16(uint16_t value);

/**
 * @brief  Reverse byte order in signed short value with sign extension to integer
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in signed short value with sign extension to integer
 */
extern int32_t __REVSH(int16_t value);

/**
 * @brief  Reverse bit order of value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse bit order of value
 */
extern uint32_t __RBIT(uint32_t value);

/**
 * @brief  LDR Exclusive (8 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 8 bit value
 */
extern uint8_t __LDREXB(uint8_t* addr);

/**
 * @brief  LDR Exclusive (16 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 16 bit values
 */
extern uint16_t __LDREXH(uint16_t* addr);

/**
 * @brief  LDR Exclusive (32 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 32 bit values
 */
extern uint32_t __LDREXW(uint32_t* addr);

/**
 * @brief  STR Exclusive (8 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 8 bit values
 */
extern uint32_t __STREXB(uint8_t value, uint8_t* addr);

/**
 * @brief  STR Exclusive (16 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 16 bit values
 */
extern uint32_t __STREXH(uint16_t value, uint16_t* addr);

/**
 * @brief  STR Exclusive (32 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 32 bit values
 */
extern uint32_t __STREXW(uint32_t value, uint32_t* addr);

/**
  Core  Function Interface containing:
  - Core NVIC Functions
  - Core SysTick Functions
  - Core Reset Functions
*/

/* ##########################   NVIC functions  #################################### */

/**
 * @brief  Set the Priority Grouping in NVIC Interrupt Controller
 *
 * @param  PriorityGroup is priority grouping field
 *
 * Set the priority grouping field using the required unlock sequence.
 * The parameter priority_grouping is assigned to the field
 * SCB.AIRCR [10:8] PRIGROUP field. Only values from 0..7 are used.
 * In case of a conflict between priority grouping and available
 * priority bits (__NVIC_PRIO_BITS) the smallest possible priority group is set.
 */
static inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup) {
	uint32_t reg_value;
	uint32_t PriorityGroupTmp = (PriorityGroup & 0x07); /* only values 0..7 are used          */

	reg_value = SCB.AIRCR;                                             /* read old register configuration    */
	reg_value &= ~(SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP);    /* clear bits to change               */
	reg_value = (reg_value | (0x5FA << 16) | (PriorityGroupTmp << 8)); /* Insert write key and priorty group */
	SCB.AIRCR = reg_value;
}

/**
 * @brief  Get the Priority Grouping from NVIC Interrupt Controller
 *
 * @return priority grouping field
 *
 * Get the priority grouping from NVIC Interrupt Controller.
 * priority grouping is SCB.AIRCR [10:8] PRIGROUP field.
 */
static inline uint32_t NVIC_GetPriorityGrouping(void) {
	return ((SCB.AIRCR & SCB_AIRCR_PRIGROUP) >> 8); /* read priority grouping field */
}

/**
 * @brief  Enable Interrupt in NVIC Interrupt Controller
 *
 * @param  IRQn   The positive number of the external interrupt to enable
 *
 * Enable a device specific interupt in the NVIC interrupt controller.
 * The interrupt number cannot be a negative value.
 */
static inline void NVIC_EnableIRQ(enum IRQn_Type IRQn) {
	NVIC.ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn)&0x1F)); /* enable interrupt */
}

/**
 * @brief  Disable the interrupt line for external interrupt specified
 *
 * @param  IRQn   The positive number of the external interrupt to disable
 *
 * Disable a device specific interupt in the NVIC interrupt controller.
 * The interrupt number cannot be a negative value.
 */
static inline void NVIC_DisableIRQ(enum IRQn_Type IRQn) {
	NVIC.ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn)&0x1F)); /* disable interrupt */
}

/**
 * @brief  Read the interrupt pending bit for a device specific interrupt source
 *
 * @param  IRQn    The number of the device specifc interrupt
 * @return         1 = interrupt pending, 0 = interrupt not pending
 *
 * Read the pending register in NVIC and return 1 if its status is pending,
 * otherwise it returns 0
 */
static inline uint32_t NVIC_GetPendingIRQ(enum IRQn_Type IRQn) {
	return ((uint32_t)((NVIC.ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn)&0x1F))) ? 1 : 0)); /* Return 1 if pending else 0 */
}

/**
 * @brief  Set the pending bit for an external interrupt
 *
 * @param  IRQn    The number of the interrupt for set pending
 *
 * Set the pending bit for the specified interrupt.
 * The interrupt number cannot be a negative value.
 */
static inline void NVIC_SetPendingIRQ(enum IRQn_Type IRQn) {
	NVIC.ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn)&0x1F)); /* set interrupt pending */
}

/**
 * @brief  Clear the pending bit for an external interrupt
 *
 * @param  IRQn    The number of the interrupt for clear pending
 *
 * Clear the pending bit for the specified interrupt.
 * The interrupt number cannot be a negative value.
 */
static inline void NVIC_ClearPendingIRQ(enum IRQn_Type IRQn) {
	NVIC.ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn)&0x1F)); /* Clear pending interrupt */
}

/**
 * @brief  Read the active bit for an external interrupt
 *
 * @param  IRQn    The number of the interrupt for read active bit
 * @return         1 = interrupt active, 0 = interrupt not active
 *
 * Read the active register in NVIC and returns 1 if its status is active,
 * otherwise it returns 0.
 */
static inline uint32_t NVIC_GetActive(enum IRQn_Type IRQn) {
	return ((uint32_t)((NVIC.IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn)&0x1F))) ? 1 : 0)); /* Return 1 if active else 0 */
}

/**
 * @brief  Set the priority for an interrupt
 *
 * @param  IRQn      The number of the interrupt for set priority
 * @param  priority  The priority to set
 *
 * Set the priority for the specified interrupt. The interrupt
 * number can be positive to specify an external (device specific)
 * interrupt, or negative to specify an internal (core) interrupt.
 *
 * Note: The priority cannot be set for every core interrupt.
 */
static inline void NVIC_SetPriority(enum IRQn_Type IRQn, uint32_t priority) {
	if (IRQn < 0) {
		SCB.SHP[((uint32_t)(IRQn)&0xF) - 4] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);
	} /* set Priority for Cortex-M3 System Interrupts */
	else {
		NVIC.IP[(uint32_t)(IRQn)] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);
	} /* set Priority for device specific Interrupts  */
}

/**
 * @brief  Read the priority for an interrupt
 *
 * @param  IRQn      The number of the interrupt for get priority
 * @return           The priority for the interrupt
 *
 * Read the priority for the specified interrupt. The interrupt
 * number can be positive to specify an external (device specific)
 * interrupt, or negative to specify an internal (core) interrupt.
 *
 * The returned priority value is automatically aligned to the implemented
 * priority bits of the microcontroller.
 *
 * Note: The priority cannot be set for every core interrupt.
 */
static inline uint32_t NVIC_GetPriority(enum IRQn_Type IRQn) {
	if (IRQn < 0) {
		return ((uint32_t)(SCB.SHP[((uint32_t)(IRQn)&0xF) - 4] >> (8 - __NVIC_PRIO_BITS)));
	} /* get priority for Cortex-M3 system interrupts */
	else {
		return ((uint32_t)(NVIC.IP[(uint32_t)(IRQn)] >> (8 - __NVIC_PRIO_BITS)));
	} /* get priority for device specific interrupts  */
}

/**
 * @brief  Encode the priority for an interrupt
 *
 * @param  PriorityGroup    The used priority group
 * @param  PreemptPriority  The preemptive priority value (starting from 0)
 * @param  SubPriority      The sub priority value (starting from 0)
 * @return                  The encoded priority for the interrupt
 *
 * Encode the priority for an interrupt with the given priority group,
 * preemptive priority value and sub priority value.
 * In case of a conflict between priority grouping and available
 * priority bits (__NVIC_PRIO_BITS) the samllest possible priority group is set.
 *
 * The returned priority value can be used for NVIC_SetPriority(...) function
 */
static inline uint32_t NVIC_EncodePriority(uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority) {
	uint32_t PriorityGroupTmp = (PriorityGroup & 0x07); /* only values 0..7 are used          */
	uint32_t PreemptPriorityBits;
	uint32_t SubPriorityBits;

	PreemptPriorityBits = ((7 - PriorityGroupTmp) > __NVIC_PRIO_BITS) ? __NVIC_PRIO_BITS : 7 - PriorityGroupTmp;
	SubPriorityBits     = ((PriorityGroupTmp + __NVIC_PRIO_BITS) < 7) ? 0 : PriorityGroupTmp - 7 + __NVIC_PRIO_BITS;

	return (((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) | ((SubPriority & ((1 << (SubPriorityBits)) - 1))));
}

/**
 * @brief  Decode the priority of an interrupt
 *
 * @param  Priority           The priority for the interrupt
 * @param  PriorityGroup      The used priority group
 * @param  pPreemptPriority   The preemptive priority value (starting from 0)
 * @param  pSubPriority       The sub priority value (starting from 0)
 *
 * Decode an interrupt priority value with the given priority group to
 * preemptive priority value and sub priority value.
 * In case of a conflict between priority grouping and available
 * priority bits (__NVIC_PRIO_BITS) the samllest possible priority group is set.
 *
 * The priority value can be retrieved with NVIC_GetPriority(...) function
 */
static inline void NVIC_DecodePriority(uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority) {
	uint32_t PriorityGroupTmp = (PriorityGroup & 0x07); /* only values 0..7 are used          */
	uint32_t PreemptPriorityBits;
	uint32_t SubPriorityBits;

	PreemptPriorityBits = ((7 - PriorityGroupTmp) > __NVIC_PRIO_BITS) ? __NVIC_PRIO_BITS : 7 - PriorityGroupTmp;
	SubPriorityBits     = ((PriorityGroupTmp + __NVIC_PRIO_BITS) < 7) ? 0 : PriorityGroupTmp - 7 + __NVIC_PRIO_BITS;

	*pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
	*pSubPriority     = (Priority) & ((1 << (SubPriorityBits)) - 1);
}

/* ##################################    SysTick function  ############################################ */

/**
 * @brief  Initialize and start the SysTick counter and its interrupt.
 *
 * @param   ticks   number of ticks between two interrupts
 * @return  1 = failed, 0 = successful
 *
 * Initialise the system tick timer and its interrupt and start the
 * system tick timer / counter in free running mode to generate
 * periodical interrupts.
 */
static inline uint32_t SysTick_Config(uint32_t ticks) {
	if (ticks > SysTick_LOAD_RELOAD + 1)
		return (1); /* Reload value impossible */

	SysTick.LOAD = (ticks - 1) & SysTick_LOAD_RELOAD;        /* set reload register */
	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* set Priority for Cortex-M0 System Interrupts */
	SysTick.VAL = 0;                                             /* Load the SysTick Counter Value */
	SysTick.CTRL =
	    SysTick_CTRL_CLKSOURCE | SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE; /* Enable SysTick IRQ and SysTick Timer */
	return (0);                                                                          /* Function successful */
}

/* ##################################    Reset function  ############################################ */

/**
 * @brief  Initiate a system reset request.
 *
 * Initiate a system reset request to reset the MCU
 */
static inline void NVIC_SystemReset(void) {
	SCB.AIRCR = ((0x5FA << 16) | (SCB.AIRCR & SCB_AIRCR_PRIGROUP) | SCB_AIRCR_SYSRESETREQ); /* Keep priority group unchanged */
	__DSB();                                                                                        /* Ensure completion of memory access */
	while (1)
		; /* wait until reset */
}

/* ##################################### Debug In/Output function ########################################### */

/**
  Core Debug Interface containing:
  - Core Debug Receive / Transmit Functions
  - Core Debug Defines
  - Core Debug Variables
*/

extern volatile int ITM_RxBuffer;         /*!< variable to receive characters                             */
enum { ITM_RXBUFFER_EMPTY = 0x5AA55AA5 }; /*!< value identifying ITM_RxBuffer is ready for next character */

/**
 * @brief  Outputs a character via the ITM channel 0
 *
 * @param  ch   character to output
 * @return      character to output
 *
 * The function outputs a character via the ITM channel 0.
 * The function returns when no debugger is connected that has booked the output.
 * It is blocking when a debugger is connected, but the previous character send is not transmitted.
 */
static inline uint32_t ITM_SendChar(uint32_t ch) {
	if ((CoreDebug.DEMCR & CoreDebug_DEMCR_TRCENA) && /* Trace enabled */
	    (ITM.TCR & ITM_TCR_ITMENA) &&                 /* ITM enabled */
	    (ITM.TER & (1ul << 0)))                           /* ITM Port #0 enabled */
	{
		while (ITM.PORT[0].u32 == 0)
			;
		ITM.PORT[0].u8 = (uint8_t)ch;
	}
	return (ch);
}

/**
 * @brief  Inputs a character via variable ITM_RxBuffer
 *
 * @return      received character, -1 = no character received
 *
 * The function inputs a character via variable ITM_RxBuffer.
 * The function returns when no debugger is connected that has booked the output.
 * It is blocking when a debugger is connected, but the previous character send is not transmitted.
 */
static inline int ITM_ReceiveChar(void) {
	int ch = -1; /* no character available */

	if (ITM_RxBuffer != ITM_RXBUFFER_EMPTY) {
		ch           = ITM_RxBuffer;
		ITM_RxBuffer = ITM_RXBUFFER_EMPTY; /* ready for next character */
	}

	return (ch);
}

/**
 * @brief  Check if a character via variable ITM_RxBuffer is available
 *
 * @return      1 = character available, 0 = no character available
 *
 * The function checks  variable ITM_RxBuffer whether a character is available or not.
 * The function returns '1' if a character is available and '0' if no character is available.
 */
static inline int ITM_CheckChar(void) {
	if (ITM_RxBuffer == ITM_RXBUFFER_EMPTY) {
		return (0); /* no character available */
	} else {
		return (1); /*    character available */
	}
}
