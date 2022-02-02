#pragma once

/**

 Based on  stm32f10x.h  V3.5.0  11-March-2011
 COPYRIGHT 2011 STMicroelectronics</center>

  #define STM32F10X_MD */ /*!< STM32F10X_MD: STM32 Medium density devices 
  - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers where the Flash memory density ranges between 64 and 128 Kbytes.

  */

#include <stdint.h>

#include "core_cm3.h"

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#define __I volatile const /*!< defines 'read only' permissions      */
#define __O volatile       /*!< defines 'write only' permissions     */
#define __IO volatile      /*!< defines 'read / write' permissions   */

/**
 * @brief Analog to Digital Converter
 */

struct ADC_Type {
	__IO uint32_t SR;
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t SMPR1;
	__IO uint32_t SMPR2;
	__IO uint32_t JOFR1;
	__IO uint32_t JOFR2;
	__IO uint32_t JOFR3;
	__IO uint32_t JOFR4;
	__IO uint32_t HTR;
	__IO uint32_t LTR;
	__IO uint32_t SQR1;
	__IO uint32_t SQR2;
	__IO uint32_t SQR3;
	__IO uint32_t JSQR;
	__IO uint32_t JDR1;
	__IO uint32_t JDR2;
	__IO uint32_t JDR3;
	__IO uint32_t JDR4;
	__IO uint32_t DR;
};

/**
 * @brief Backup Registers
 */

struct BKP_Type {
	uint32_t RESERVED0;
	__IO uint16_t DR1;
	uint16_t      RESERVED1;
	__IO uint16_t DR2;
	uint16_t      RESERVED2;
	__IO uint16_t DR3;
	uint16_t      RESERVED3;
	__IO uint16_t DR4;
	uint16_t      RESERVED4;
	__IO uint16_t DR5;
	uint16_t      RESERVED5;
	__IO uint16_t DR6;
	uint16_t      RESERVED6;
	__IO uint16_t DR7;
	uint16_t      RESERVED7;
	__IO uint16_t DR8;
	uint16_t      RESERVED8;
	__IO uint16_t DR9;
	uint16_t      RESERVED9;
	__IO uint16_t DR10;
	uint16_t      RESERVED10;
	__IO uint16_t RTCCR;
	uint16_t      RESERVED11;
	__IO uint16_t CR;
	uint16_t      RESERVED12;
	__IO uint16_t CSR;
	uint16_t      RESERVED13[5];
	__IO uint16_t DR11;
	uint16_t      RESERVED14;
	__IO uint16_t DR12;
	uint16_t      RESERVED15;
	__IO uint16_t DR13;
	uint16_t      RESERVED16;
	__IO uint16_t DR14;
	uint16_t      RESERVED17;
	__IO uint16_t DR15;
	uint16_t      RESERVED18;
	__IO uint16_t DR16;
	uint16_t      RESERVED19;
	__IO uint16_t DR17;
	uint16_t      RESERVED20;
	__IO uint16_t DR18;
	uint16_t      RESERVED21;
	__IO uint16_t DR19;
	uint16_t      RESERVED22;
	__IO uint16_t DR20;
	uint16_t      RESERVED23;
	__IO uint16_t DR21;
	uint16_t      RESERVED24;
	__IO uint16_t DR22;
	uint16_t      RESERVED25;
	__IO uint16_t DR23;
	uint16_t      RESERVED26;
	__IO uint16_t DR24;
	uint16_t      RESERVED27;
	__IO uint16_t DR25;
	uint16_t      RESERVED28;
	__IO uint16_t DR26;
	uint16_t      RESERVED29;
	__IO uint16_t DR27;
	uint16_t      RESERVED30;
	__IO uint16_t DR28;
	uint16_t      RESERVED31;
	__IO uint16_t DR29;
	uint16_t      RESERVED32;
	__IO uint16_t DR30;
	uint16_t      RESERVED33;
	__IO uint16_t DR31;
	uint16_t      RESERVED34;
	__IO uint16_t DR32;
	uint16_t      RESERVED35;
	__IO uint16_t DR33;
	uint16_t      RESERVED36;
	__IO uint16_t DR34;
	uint16_t      RESERVED37;
	__IO uint16_t DR35;
	uint16_t      RESERVED38;
	__IO uint16_t DR36;
	uint16_t      RESERVED39;
	__IO uint16_t DR37;
	uint16_t      RESERVED40;
	__IO uint16_t DR38;
	uint16_t      RESERVED41;
	__IO uint16_t DR39;
	uint16_t      RESERVED42;
	__IO uint16_t DR40;
	uint16_t      RESERVED43;
	__IO uint16_t DR41;
	uint16_t      RESERVED44;
	__IO uint16_t DR42;
	uint16_t      RESERVED45;
};

/**
 * @brief Controller Area Network TxMailBox
 */

struct CAN_TxMailBox_Type {
	__IO uint32_t TIR;
	__IO uint32_t TDTR;
	__IO uint32_t TDLR;
	__IO uint32_t TDHR;
};

/**
 * @brief Controller Area Network FIFOMailBox
 */

struct CAN_FIFOMailBox_Type {
	__IO uint32_t RIR;
	__IO uint32_t RDTR;
	__IO uint32_t RDLR;
	__IO uint32_t RDHR;
};

/**
 * @brief Controller Area Network FilterRegister
 */

struct CAN_FilterRegister_Type {
	__IO uint32_t FR1;
	__IO uint32_t FR2;
};

/**
 * @brief Controller Area Network
 */

struct CAN_Type {
	__IO uint32_t MCR;
	__IO uint32_t MSR;
	__IO uint32_t TSR;
	__IO uint32_t RF0R;
	__IO uint32_t RF1R;
	__IO uint32_t IER;
	__IO uint32_t ESR;
	__IO uint32_t               BTR;
	uint32_t                    RESERVED0[88];
	struct CAN_TxMailBox_Type   sTxMailBox[3];
	struct CAN_FIFOMailBox_Type sFIFOMailBox[2];
	uint32_t                    RESERVED1[12];
	__IO uint32_t FMR;
	__IO uint32_t FM1R;
	uint32_t      RESERVED2;
	__IO uint32_t FS1R;
	uint32_t      RESERVED3;
	__IO uint32_t FFA1R;
	uint32_t      RESERVED4;
	__IO uint32_t                  FA1R;
	uint32_t                       RESERVED5[8];
	struct CAN_FilterRegister_Type sFilterRegister[14];
};

/**
 * @brief CRC calculation unit
 */

struct CRC_Type {
	__IO uint32_t DR;
	__IO uint8_t IDR;
	uint8_t      RESERVED0;
	uint16_t     RESERVED1;
	__IO uint32_t CR;
};

/**
 * @brief Debug MCU
 */

struct DBGMCU_Type {
	__IO uint32_t IDCODE;
	__IO uint32_t CR;
};

/**
 * @brief DMA Controller
 */

struct DMA_Channel_Type {
	__IO uint32_t CCR;
	__IO uint32_t CNDTR;
	__IO uint32_t CPAR;
	__IO uint32_t CMAR;
};

struct DMA_Type {
	__IO uint32_t ISR;
	__IO uint32_t IFCR;
};

/**
 * @brief External Interrupt/Event Controller
 */

struct EXTI_Type {
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;
};

/**
 * @brief FLASH Registers
 */

struct FLASH_Type {
	__IO uint32_t ACR;
	__IO uint32_t KEYR;
	__IO uint32_t OPTKEYR;
	__IO uint32_t SR;
	__IO uint32_t CR;
	__IO uint32_t AR;
	__IO uint32_t RESERVED;
	__IO uint32_t OBR;
	__IO uint32_t WRPR;
};

/**
 * @brief Option Bytes Registers
 */

struct OB_Type {
	__IO uint16_t RDP;
	__IO uint16_t USER;
	__IO uint16_t Data0;
	__IO uint16_t Data1;
	__IO uint16_t WRP0;
	__IO uint16_t WRP1;
	__IO uint16_t WRP2;
	__IO uint16_t WRP3;
};

/**
 * @brief General Purpose I/O
 */

struct GPIO_Type {
	__IO uint32_t CRL;
	__IO uint32_t CRH;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t BRR;
	__IO uint32_t LCKR;
};

/**
 * @brief Alternate Function I/O
 */

struct AFIO_Type {
	__IO uint32_t EVCR;
	__IO uint32_t MAPR;
	__IO uint32_t EXTICR[4];
	uint32_t      RESERVED0;
	__IO uint32_t MAPR2;
};
/**
 * @brief Inter Integrated Circuit Interface
 */

struct I2C_Type {
	__IO uint16_t CR1;
	uint16_t      RESERVED0;
	__IO uint16_t CR2;
	uint16_t      RESERVED1;
	__IO uint16_t OAR1;
	uint16_t      RESERVED2;
	__IO uint16_t OAR2;
	uint16_t      RESERVED3;
	__IO uint16_t DR;
	uint16_t      RESERVED4;
	__IO uint16_t SR1;
	uint16_t      RESERVED5;
	__IO uint16_t SR2;
	uint16_t      RESERVED6;
	__IO uint16_t CCR;
	uint16_t      RESERVED7;
	__IO uint16_t TRISE;
	uint16_t      RESERVED8;
};

/**
 * @brief Independent WATCHDOG
 */

struct IWDG_Type {
	__IO uint32_t KR;
	__IO uint32_t PR;
	__IO uint32_t RLR;
	__IO uint32_t SR;
};

/**
 * @brief Power Control
 */

struct PWR_Type {
	__IO uint32_t CR;
	__IO uint32_t CSR;
};

/**
 * @brief Reset and Clock Control
 */

struct RCC_Type {
	__IO uint32_t CR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t APB2RSTR;
	__IO uint32_t APB1RSTR;
	__IO uint32_t AHBENR;
	__IO uint32_t APB2ENR;
	__IO uint32_t APB1ENR;
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
};

/**
 * @brief Real-Time Clock
 */

struct RTC_Type {
	__IO uint16_t CRH;
	uint16_t      RESERVED0;
	__IO uint16_t CRL;
	uint16_t      RESERVED1;
	__IO uint16_t PRLH;
	uint16_t      RESERVED2;
	__IO uint16_t PRLL;
	uint16_t      RESERVED3;
	__IO uint16_t DIVH;
	uint16_t      RESERVED4;
	__IO uint16_t DIVL;
	uint16_t      RESERVED5;
	__IO uint16_t CNTH;
	uint16_t      RESERVED6;
	__IO uint16_t CNTL;
	uint16_t      RESERVED7;
	__IO uint16_t ALRH;
	uint16_t      RESERVED8;
	__IO uint16_t ALRL;
	uint16_t      RESERVED9;
};

/**
 * @brief Serial Peripheral Interface
 */

struct SPI_Type {
	__IO uint16_t CR1;
	uint16_t      RESERVED0;
	__IO uint16_t CR2;
	uint16_t      RESERVED1;
	__IO uint16_t SR;
	uint16_t      RESERVED2;
	__IO uint16_t DR;
	uint16_t      RESERVED3;
	__IO uint16_t CRCPR;
	uint16_t      RESERVED4;
	__IO uint16_t RXCRCR;
	uint16_t      RESERVED5;
	__IO uint16_t TXCRCR;
	uint16_t      RESERVED6;
	__IO uint16_t I2SCFGR;
	uint16_t      RESERVED7;
	__IO uint16_t I2SPR;
	uint16_t      RESERVED8;
};

/**
 * @brief TIM
 */

struct TIM_Type {
	__IO uint16_t CR1;
	uint16_t      RESERVED0;
	__IO uint16_t CR2;
	uint16_t      RESERVED1;
	__IO uint16_t SMCR;
	uint16_t      RESERVED2;
	__IO uint16_t DIER;
	uint16_t      RESERVED3;
	__IO uint16_t SR;
	uint16_t      RESERVED4;
	__IO uint16_t EGR;
	uint16_t      RESERVED5;
	__IO uint16_t CCMR1;
	uint16_t      RESERVED6;
	__IO uint16_t CCMR2;
	uint16_t      RESERVED7;
	__IO uint16_t CCER;
	uint16_t      RESERVED8;
	__IO uint16_t CNT;
	uint16_t      RESERVED9;
	__IO uint16_t PSC;
	uint16_t      RESERVED10;
	__IO uint16_t ARR;
	uint16_t      RESERVED11;
	__IO uint16_t RCR;
	uint16_t      RESERVED12;
	__IO uint16_t CCR1;
	uint16_t      RESERVED13;
	__IO uint16_t CCR2;
	uint16_t      RESERVED14;
	__IO uint16_t CCR3;
	uint16_t      RESERVED15;
	__IO uint16_t CCR4;
	uint16_t      RESERVED16;
	__IO uint16_t BDTR;
	uint16_t      RESERVED17;
	__IO uint16_t DCR;
	uint16_t      RESERVED18;
	__IO uint16_t DMAR;
	uint16_t      RESERVED19;
};

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

struct USART_Type {
	__IO uint16_t SR;
	uint16_t      RESERVED0;
	__IO uint16_t DR;
	uint16_t      RESERVED1;
	__IO uint16_t BRR;
	uint16_t      RESERVED2;
	__IO uint16_t CR1;
	uint16_t      RESERVED3;
	__IO uint16_t CR2;
	uint16_t      RESERVED4;
	__IO uint16_t CR3;
	uint16_t      RESERVED5;
	__IO uint16_t GTPR;
	uint16_t      RESERVED6;
};

/**
 * @brief Window WATCHDOG
 */

struct WWDG_Type {
	__IO uint32_t CR;
	__IO uint32_t CFR;
	__IO uint32_t SR;
};

/**
 * @}
 */

extern struct ADC_Type         ADC1;
extern struct ADC_Type         ADC2;
extern struct AFIO_Type        AFIO;
extern struct BKP_Type         BKP;
extern struct CAN_Type         CAN1;
extern struct CRC_Type         CRC;
extern struct DMA_Channel_Type DMA1_Channel1;
extern struct DMA_Channel_Type DMA1_Channel2;
extern struct DMA_Channel_Type DMA1_Channel3;
extern struct DMA_Channel_Type DMA1_Channel4;
extern struct DMA_Channel_Type DMA1_Channel5;
extern struct DMA_Channel_Type DMA1_Channel6;
extern struct DMA_Channel_Type DMA1_Channel7;
extern struct DMA_Type         DMA1;
extern struct EXTI_Type        EXTI;
extern struct FLASH_Type       FLASH_R;
extern struct GPIO_Type        GPIOA;
extern struct GPIO_Type        GPIOB;
extern struct GPIO_Type        GPIOC;
extern struct GPIO_Type        GPIOD;
extern struct I2C_Type         I2C1;
extern struct I2C_Type         I2C2;
extern struct IWDG_Type        IWDG;
extern struct OB_Type          OPTION_BYTES;
extern struct PWR_Type         PWR;
extern struct RCC_Type         RCC;
extern struct RTC_Type         RTC;
extern struct SPI_Type         SPI1;
extern struct SPI_Type         SPI2;
extern struct TIM_Type         TIM1;
extern struct TIM_Type         TIM2;
extern struct TIM_Type         TIM3;
extern struct TIM_Type         TIM4;

extern struct USART_Type  USART1;
extern struct USART_Type  USART2;
extern struct USART_Type  USART3;
extern struct WWDG_Type   WWDG;
extern struct DBGMCU_Type DBGMCU;

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 30.2

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR ((uint32_t)0xFFFFFFFF) /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR ((uint8_t)0xFF) /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET ((uint8_t)0x01) /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS ((uint16_t)0x0001) /*!< Low-Power Deepsleep */
#define PWR_CR_PDDS ((uint16_t)0x0002) /*!< Power Down Deepsleep */
#define PWR_CR_CWUF ((uint16_t)0x0004) /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF ((uint16_t)0x0008) /*!< Clear Standby Flag */
#define PWR_CR_PVDE ((uint16_t)0x0010) /*!< Power Voltage Detector Enable */

#define PWR_CR_PLS ((uint16_t)0x00E0)   /*!< PLS[2:0] bits (PVD Level Selection) */

/*!< PVD level configuration */
#define PWR_CR_PLS_2V2 ((uint16_t)0x0000) /*!< PVD level 2.2V */
#define PWR_CR_PLS_2V3 ((uint16_t)0x0020) /*!< PVD level 2.3V */
#define PWR_CR_PLS_2V4 ((uint16_t)0x0040) /*!< PVD level 2.4V */
#define PWR_CR_PLS_2V5 ((uint16_t)0x0060) /*!< PVD level 2.5V */
#define PWR_CR_PLS_2V6 ((uint16_t)0x0080) /*!< PVD level 2.6V */
#define PWR_CR_PLS_2V7 ((uint16_t)0x00A0) /*!< PVD level 2.7V */
#define PWR_CR_PLS_2V8 ((uint16_t)0x00C0) /*!< PVD level 2.8V */
#define PWR_CR_PLS_2V9 ((uint16_t)0x00E0) /*!< PVD level 2.9V */

#define PWR_CR_DBP ((uint16_t)0x0100) /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF ((uint16_t)0x0001)  /*!< Wakeup Flag */
#define PWR_CSR_SBF ((uint16_t)0x0002)  /*!< Standby Flag */
#define PWR_CSR_PVDO ((uint16_t)0x0004) /*!< PVD Output */
#define PWR_CSR_EWUP ((uint16_t)0x0100) /*!< Enable WKUP pin */

/******************  Bit definition for BKP_RTCCR register  *******************/
#define BKP_RTCCR_CAL ((uint16_t)0x007F)  /*!< Calibration value */
#define BKP_RTCCR_CCO ((uint16_t)0x0080)  /*!< Calibration Clock Output */
#define BKP_RTCCR_ASOE ((uint16_t)0x0100) /*!< Alarm or Second Output Enable */
#define BKP_RTCCR_ASOS ((uint16_t)0x0200) /*!< Alarm or Second Output Selection */

/********************  Bit definition for BKP_CR register  ********************/
#define BKP_CR_TPE ((uint8_t)0x01)  /*!< TAMPER pin enable */
#define BKP_CR_TPAL ((uint8_t)0x02) /*!< TAMPER pin active level */

/*******************  Bit definition for BKP_CSR register  ********************/
#define BKP_CSR_CTE ((uint16_t)0x0001)  /*!< Clear Tamper event */
#define BKP_CSR_CTI ((uint16_t)0x0002)  /*!< Clear Tamper Interrupt */
#define BKP_CSR_TPIE ((uint16_t)0x0004) /*!< TAMPER Pin interrupt enable */
#define BKP_CSR_TEF ((uint16_t)0x0100)  /*!< Tamper Event Flag */
#define BKP_CSR_TIF ((uint16_t)0x0200)  /*!< Tamper Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION ((uint32_t)0x00000001)   /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY ((uint32_t)0x00000002)  /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM ((uint32_t)0x000000F8) /*!< Internal High Speed clock trimming */
#define RCC_CR_HSICAL ((uint32_t)0x0000FF00)  /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSEON ((uint32_t)0x00010000)   /*!< External High Speed clock enable */
#define RCC_CR_HSERDY ((uint32_t)0x00020000)  /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP ((uint32_t)0x00040000)  /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON ((uint32_t)0x00080000)   /*!< Clock Security System enable */
#define RCC_CR_PLLON ((uint32_t)0x01000000)   /*!< PLL enable */
#define RCC_CR_PLLRDY ((uint32_t)0x02000000)  /*!< PLL clock ready flag */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define RCC_CFGR_SW ((uint32_t)0x00000003)   /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_HSI ((uint32_t)0x00000000) /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE ((uint32_t)0x00000001) /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL ((uint32_t)0x00000002) /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS ((uint32_t)0x0000000C)   /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI ((uint32_t)0x00000000) /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE ((uint32_t)0x00000004) /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL ((uint32_t)0x00000008) /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE ((uint32_t)0x000000F0)   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_DIV1 ((uint32_t)0x00000000)   /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2 ((uint32_t)0x00000080)   /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4 ((uint32_t)0x00000090)   /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8 ((uint32_t)0x000000A0)   /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16 ((uint32_t)0x000000B0)  /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64 ((uint32_t)0x000000C0)  /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128 ((uint32_t)0x000000D0) /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256 ((uint32_t)0x000000E0) /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512 ((uint32_t)0x000000F0) /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1 ((uint32_t)0x00000700)   /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_DIV1 ((uint32_t)0x00000000)  /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2 ((uint32_t)0x00000400)  /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4 ((uint32_t)0x00000500)  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8 ((uint32_t)0x00000600)  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16 ((uint32_t)0x00000700) /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2 ((uint32_t)0x00003800)   /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_DIV1 ((uint32_t)0x00000000)  /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2 ((uint32_t)0x00002000)  /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4 ((uint32_t)0x00002800)  /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8 ((uint32_t)0x00003000)  /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16 ((uint32_t)0x00003800) /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define RCC_CFGR_ADCPRE ((uint32_t)0x0000C000)   /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_ADCPRE_DIV2 ((uint32_t)0x00000000) /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4 ((uint32_t)0x00004000) /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6 ((uint32_t)0x00008000) /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8 ((uint32_t)0x0000C000) /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC ((uint32_t)0x00010000) /*!< PLL entry clock source */

#define RCC_CFGR_PLLXTPRE ((uint32_t)0x00020000) /*!< HSE divider for PLL entry */
#define RCC_CFGR_PLLSRC_HSI_Div2 ((uint32_t)0x00000000) /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSE ((uint32_t)0x00010000)      /*!< HSE clock selected as PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_HSE ((uint32_t)0x00000000)      /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_Div2 ((uint32_t)0x00020000) /*!< HSE clock divided by 2 for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL ((uint32_t)0x003C0000)   /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMULL2 ((uint32_t)0x00000000)  /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3 ((uint32_t)0x00040000)  /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4 ((uint32_t)0x00080000)  /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5 ((uint32_t)0x000C0000)  /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6 ((uint32_t)0x00100000)  /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7 ((uint32_t)0x00140000)  /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8 ((uint32_t)0x00180000)  /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9 ((uint32_t)0x001C0000)  /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10 ((uint32_t)0x00200000) /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11 ((uint32_t)0x00240000) /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12 ((uint32_t)0x00280000) /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13 ((uint32_t)0x002C0000) /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14 ((uint32_t)0x00300000) /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15 ((uint32_t)0x00340000) /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16 ((uint32_t)0x00380000) /*!< PLL input clock*16 */

#define RCC_CFGR_USBPRE ((uint32_t)0x00400000)    /*!< USB Device prescaler */

/*!< MCO configuration */
#define RCC_CFGR_MCO ((uint32_t)0x07000000)   /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_NOCLOCK ((uint32_t)0x00000000) /*!< No clock */
#define RCC_CFGR_MCO_SYSCLK ((uint32_t)0x04000000)  /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI ((uint32_t)0x05000000)     /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE ((uint32_t)0x06000000)     /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLL ((uint32_t)0x07000000)     /*!< PLL clock divided by 2 selected as MCO source */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define RCC_CIR_LSIRDYF ((uint32_t)0x00000001)  /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF ((uint32_t)0x00000002)  /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF ((uint32_t)0x00000004)  /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF ((uint32_t)0x00000008)  /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF ((uint32_t)0x00000010)  /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF ((uint32_t)0x00000080)     /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE ((uint32_t)0x00000100) /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE ((uint32_t)0x00000200) /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE ((uint32_t)0x00000400) /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE ((uint32_t)0x00000800) /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE ((uint32_t)0x00001000) /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC ((uint32_t)0x00010000)  /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC ((uint32_t)0x00020000)  /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC ((uint32_t)0x00040000)  /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC ((uint32_t)0x00080000)  /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC ((uint32_t)0x00100000)  /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC ((uint32_t)0x00800000)     /*!< Clock Security System Interrupt Clear */

/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define RCC_APB2RSTR_AFIORST ((uint32_t)0x00000001)   /*!< Alternate Function I/O reset */
#define RCC_APB2RSTR_IOPARST ((uint32_t)0x00000004)   /*!< I/O port A reset */
#define RCC_APB2RSTR_IOPBRST ((uint32_t)0x00000008)   /*!< I/O port B reset */
#define RCC_APB2RSTR_IOPCRST ((uint32_t)0x00000010)   /*!< I/O port C reset */
#define RCC_APB2RSTR_IOPDRST ((uint32_t)0x00000020)   /*!< I/O port D reset */
#define RCC_APB2RSTR_ADC1RST ((uint32_t)0x00000200)   /*!< ADC 1 interface reset */
#define RCC_APB2RSTR_ADC2RST ((uint32_t)0x00000400)   /*!< ADC 2 interface reset */
#define RCC_APB2RSTR_TIM1RST ((uint32_t)0x00000800)   /*!< TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST ((uint32_t)0x00001000)   /*!< SPI 1 reset */
#define RCC_APB2RSTR_USART1RST ((uint32_t)0x00004000) /*!< USART1 reset */
#define RCC_APB2RSTR_IOPERST ((uint32_t)0x00000040)   /*!< I/O port E reset */

/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define RCC_APB1RSTR_TIM2RST ((uint32_t)0x00000001)   /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST ((uint32_t)0x00000002)   /*!< Timer 3 reset */
#define RCC_APB1RSTR_WWDGRST ((uint32_t)0x00000800)   /*!< Window Watchdog reset */
#define RCC_APB1RSTR_USART2RST ((uint32_t)0x00020000) /*!< USART 2 reset */
#define RCC_APB1RSTR_I2C1RST ((uint32_t)0x00200000)   /*!< I2C 1 reset */
#define RCC_APB1RSTR_CAN1RST ((uint32_t)0x02000000)   /*!< CAN1 reset */
#define RCC_APB1RSTR_BKPRST ((uint32_t)0x08000000)    /*!< Backup interface reset */
#define RCC_APB1RSTR_PWRRST ((uint32_t)0x10000000)    /*!< Power interface reset */
#define RCC_APB1RSTR_TIM4RST ((uint32_t)0x00000004)   /*!< Timer 4 reset */
#define RCC_APB1RSTR_SPI2RST ((uint32_t)0x00004000)   /*!< SPI 2 reset */
#define RCC_APB1RSTR_USART3RST ((uint32_t)0x00040000) /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C2RST ((uint32_t)0x00400000)   /*!< I2C 2 reset */
#define RCC_APB1RSTR_USBRST ((uint32_t)0x00800000)    /*!< USB Device reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define RCC_AHBENR_DMA1EN ((uint16_t)0x0001)  /*!< DMA1 clock enable */
#define RCC_AHBENR_SRAMEN ((uint16_t)0x0004)  /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN ((uint16_t)0x0010) /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN ((uint16_t)0x0040)   /*!< CRC clock enable */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_AFIOEN ((uint32_t)0x00000001)   /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN ((uint32_t)0x00000004)   /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN ((uint32_t)0x00000008)   /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN ((uint32_t)0x00000010)   /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN ((uint32_t)0x00000020)   /*!< I/O port D clock enable */
#define RCC_APB2ENR_ADC1EN ((uint32_t)0x00000200)   /*!< ADC 1 interface clock enable */
#define RCC_APB2ENR_ADC2EN ((uint32_t)0x00000400)   /*!< ADC 2 interface clock enable */
#define RCC_APB2ENR_TIM1EN ((uint32_t)0x00000800)   /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN ((uint32_t)0x00001000)   /*!< SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN ((uint32_t)0x00004000) /*!< USART1 clock enable */
#define RCC_APB2ENR_IOPEEN ((uint32_t)0x00000040)   /*!< I/O port E clock enable */

/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define RCC_APB1ENR_TIM2EN ((uint32_t)0x00000001)   /*!< Timer 2 clock enabled*/
#define RCC_APB1ENR_TIM3EN ((uint32_t)0x00000002)   /*!< Timer 3 clock enable */
#define RCC_APB1ENR_WWDGEN ((uint32_t)0x00000800)   /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_USART2EN ((uint32_t)0x00020000) /*!< USART 2 clock enable */
#define RCC_APB1ENR_I2C1EN ((uint32_t)0x00200000)   /*!< I2C 1 clock enable */
#define RCC_APB1ENR_CAN1EN ((uint32_t)0x02000000)   /*!< CAN1 clock enable */
#define RCC_APB1ENR_BKPEN ((uint32_t)0x08000000)    /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN ((uint32_t)0x10000000)    /*!< Power interface clock enable */
#define RCC_APB1ENR_TIM4EN ((uint32_t)0x00000004)   /*!< Timer 4 clock enable */
#define RCC_APB1ENR_SPI2EN ((uint32_t)0x00004000)   /*!< SPI 2 clock enable */
#define RCC_APB1ENR_USART3EN ((uint32_t)0x00040000) /*!< USART 3 clock enable */
#define RCC_APB1ENR_I2C2EN ((uint32_t)0x00400000)   /*!< I2C 2 clock enable */
#define RCC_APB1ENR_USBEN ((uint32_t)0x00800000)    /*!< USB Device clock enable */

/*******************  Bit definition for RCC_BDCR register  *******************/
#define RCC_BDCR_LSEON ((uint32_t)0x00000001)    /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY ((uint32_t)0x00000002)   /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP ((uint32_t)0x00000004)   /*!< External Low Speed oscillator Bypass */
#define RCC_BDCR_RTCSEL ((uint32_t)0x00000300)   /*!< RTCSEL[1:0] bits (RTC clock source selection) */
/*!< RTC congiguration */
#define RCC_BDCR_RTCSEL_NOCLOCK ((uint32_t)0x00000000) /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE ((uint32_t)0x00000100)     /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI ((uint32_t)0x00000200)     /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE ((uint32_t)0x00000300)     /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN ((uint32_t)0x00008000)          /*!< RTC clock enable */
#define RCC_BDCR_BDRST ((uint32_t)0x00010000)          /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/
#define RCC_CSR_LSION ((uint32_t)0x00000001)    /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY ((uint32_t)0x00000002)   /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF ((uint32_t)0x01000000)     /*!< Remove reset flag */
#define RCC_CSR_PINRSTF ((uint32_t)0x04000000)  /*!< PIN reset flag */
#define RCC_CSR_PORRSTF ((uint32_t)0x08000000)  /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF ((uint32_t)0x10000000)  /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF ((uint32_t)0x20000000) /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF ((uint32_t)0x40000000) /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF ((uint32_t)0x80000000) /*!< Low-Power reset flag */

/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/


/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN ((uint8_t)0x0F)   /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_PX0 ((uint8_t)0x00)  /*!< Pin 0 selected */
#define AFIO_EVCR_PIN_PX1 ((uint8_t)0x01)  /*!< Pin 1 selected */
#define AFIO_EVCR_PIN_PX2 ((uint8_t)0x02)  /*!< Pin 2 selected */
#define AFIO_EVCR_PIN_PX3 ((uint8_t)0x03)  /*!< Pin 3 selected */
#define AFIO_EVCR_PIN_PX4 ((uint8_t)0x04)  /*!< Pin 4 selected */
#define AFIO_EVCR_PIN_PX5 ((uint8_t)0x05)  /*!< Pin 5 selected */
#define AFIO_EVCR_PIN_PX6 ((uint8_t)0x06)  /*!< Pin 6 selected */
#define AFIO_EVCR_PIN_PX7 ((uint8_t)0x07)  /*!< Pin 7 selected */
#define AFIO_EVCR_PIN_PX8 ((uint8_t)0x08)  /*!< Pin 8 selected */
#define AFIO_EVCR_PIN_PX9 ((uint8_t)0x09)  /*!< Pin 9 selected */
#define AFIO_EVCR_PIN_PX10 ((uint8_t)0x0A) /*!< Pin 10 selected */
#define AFIO_EVCR_PIN_PX11 ((uint8_t)0x0B) /*!< Pin 11 selected */
#define AFIO_EVCR_PIN_PX12 ((uint8_t)0x0C) /*!< Pin 12 selected */
#define AFIO_EVCR_PIN_PX13 ((uint8_t)0x0D) /*!< Pin 13 selected */
#define AFIO_EVCR_PIN_PX14 ((uint8_t)0x0E) /*!< Pin 14 selected */
#define AFIO_EVCR_PIN_PX15 ((uint8_t)0x0F) /*!< Pin 15 selected */

#define AFIO_EVCR_PORT ((uint8_t)0x70)   /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_PA ((uint8_t)0x00) /*!< Port A selected */
#define AFIO_EVCR_PORT_PB ((uint8_t)0x10) /*!< Port B selected */
#define AFIO_EVCR_PORT_PC ((uint8_t)0x20) /*!< Port C selected */
#define AFIO_EVCR_PORT_PD ((uint8_t)0x30) /*!< Port D selected */
#define AFIO_EVCR_PORT_PE ((uint8_t)0x40) /*!< Port E selected */

#define AFIO_EVCR_EVOE ((uint8_t)0x80) /*!< Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP ((uint32_t)0x00000001)   /*!< SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP ((uint32_t)0x00000002)   /*!< I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP ((uint32_t)0x00000004) /*!< USART1 remapping */
#define AFIO_MAPR_USART2_REMAP ((uint32_t)0x00000008) /*!< USART2 remapping */

#define AFIO_MAPR_USART3_REMAP ((uint32_t)0x00000030)   /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_NOREMAP ((uint32_t)0x00000000)      /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP ((uint32_t)0x00000010) /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP ((uint32_t)0x00000030)    /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP ((uint32_t)0x000000C0)   /*!< TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP ((uint32_t)0x00000000) /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13,CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP ((uint32_t)0x00000040) /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP ((uint32_t)0x000000C0) /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8,CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP ((uint32_t)0x00000300)   /*!< TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP ((uint32_t)0x00000000)       /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 ((uint32_t)0x00000100) /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 ((uint32_t)0x00000200) /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP ((uint32_t)0x00000300)     /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP ((uint32_t)0x00000C00)   /*!< TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP ((uint32_t)0x00000000)      /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP ((uint32_t)0x00000800) /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP ((uint32_t)0x00000C00)    /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP ((uint32_t)0x00001000) /*!< TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP ((uint32_t)0x00006000)   /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_REMAP1 ((uint32_t)0x00000000) /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2 ((uint32_t)0x00004000) /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3 ((uint32_t)0x00006000) /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP ((uint32_t)0x00008000)     /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP ((uint32_t)0x00010000) /*!< TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP ((uint32_t)0x00020000) /*!< ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP ((uint32_t)0x00040000) /*!< ADC 1 External Trigger Regular Conversion remapping  */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP ((uint32_t)0x00080000) /*!< ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP ((uint32_t)0x00100000) /*!< ADC 2 External Trigger Regular Conversion remapping */

/*!< SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG ((uint32_t)0x07000000)   /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_RESET ((uint32_t)0x00000000)       /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST ((uint32_t)0x01000000)    /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE ((uint32_t)0x02000000) /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE ((uint32_t)0x04000000)     /*!< JTAG-DP Disabled and SW-DP Disabled */


/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define DMA_ISR_GIF1  ((uint32_t)0x00000001)  /*!< Channel 1 Global interrupt flag */
#define DMA_ISR_TCIF1 ((uint32_t)0x00000002) /*!< Channel 1 Transfer Complete flag */
#define DMA_ISR_HTIF1 ((uint32_t)0x00000004) /*!< Channel 1 Half Transfer flag */
#define DMA_ISR_TEIF1 ((uint32_t)0x00000008) /*!< Channel 1 Transfer Error flag */
#define DMA_ISR_GIF2  ((uint32_t)0x00000010)  /*!< Channel 2 Global interrupt flag */
#define DMA_ISR_TCIF2 ((uint32_t)0x00000020) /*!< Channel 2 Transfer Complete flag */
#define DMA_ISR_HTIF2 ((uint32_t)0x00000040) /*!< Channel 2 Half Transfer flag */
#define DMA_ISR_TEIF2 ((uint32_t)0x00000080) /*!< Channel 2 Transfer Error flag */
#define DMA_ISR_GIF3  ((uint32_t)0x00000100)  /*!< Channel 3 Global interrupt flag */
#define DMA_ISR_TCIF3 ((uint32_t)0x00000200) /*!< Channel 3 Transfer Complete flag */
#define DMA_ISR_HTIF3 ((uint32_t)0x00000400) /*!< Channel 3 Half Transfer flag */
#define DMA_ISR_TEIF3 ((uint32_t)0x00000800) /*!< Channel 3 Transfer Error flag */
#define DMA_ISR_GIF4  ((uint32_t)0x00001000)  /*!< Channel 4 Global interrupt flag */
#define DMA_ISR_TCIF4 ((uint32_t)0x00002000) /*!< Channel 4 Transfer Complete flag */
#define DMA_ISR_HTIF4 ((uint32_t)0x00004000) /*!< Channel 4 Half Transfer flag */
#define DMA_ISR_TEIF4 ((uint32_t)0x00008000) /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF5  ((uint32_t)0x00010000)  /*!< Channel 5 Global interrupt flag */
#define DMA_ISR_TCIF5 ((uint32_t)0x00020000) /*!< Channel 5 Transfer Complete flag */
#define DMA_ISR_HTIF5 ((uint32_t)0x00040000) /*!< Channel 5 Half Transfer flag */
#define DMA_ISR_TEIF5 ((uint32_t)0x00080000) /*!< Channel 5 Transfer Error flag */
#define DMA_ISR_GIF6  ((uint32_t)0x00100000)  /*!< Channel 6 Global interrupt flag */
#define DMA_ISR_TCIF6 ((uint32_t)0x00200000) /*!< Channel 6 Transfer Complete flag */
#define DMA_ISR_HTIF6 ((uint32_t)0x00400000) /*!< Channel 6 Half Transfer flag */
#define DMA_ISR_TEIF6 ((uint32_t)0x00800000) /*!< Channel 6 Transfer Error flag */
#define DMA_ISR_GIF7  ((uint32_t)0x01000000)  /*!< Channel 7 Global interrupt flag */
#define DMA_ISR_TCIF7 ((uint32_t)0x02000000) /*!< Channel 7 Transfer Complete flag */
#define DMA_ISR_HTIF7 ((uint32_t)0x04000000) /*!< Channel 7 Half Transfer flag */
#define DMA_ISR_TEIF7 ((uint32_t)0x08000000) /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1 ((uint32_t)0x00000001)  /*!< Channel 1 Global interrupt clear */
#define DMA_IFCR_CTCIF1 ((uint32_t)0x00000002) /*!< Channel 1 Transfer Complete clear */
#define DMA_IFCR_CHTIF1 ((uint32_t)0x00000004) /*!< Channel 1 Half Transfer clear */
#define DMA_IFCR_CTEIF1 ((uint32_t)0x00000008) /*!< Channel 1 Transfer Error clear */
#define DMA_IFCR_CGIF2 ((uint32_t)0x00000010)  /*!< Channel 2 Global interrupt clear */
#define DMA_IFCR_CTCIF2 ((uint32_t)0x00000020) /*!< Channel 2 Transfer Complete clear */
#define DMA_IFCR_CHTIF2 ((uint32_t)0x00000040) /*!< Channel 2 Half Transfer clear */
#define DMA_IFCR_CTEIF2 ((uint32_t)0x00000080) /*!< Channel 2 Transfer Error clear */
#define DMA_IFCR_CGIF3 ((uint32_t)0x00000100)  /*!< Channel 3 Global interrupt clear */
#define DMA_IFCR_CTCIF3 ((uint32_t)0x00000200) /*!< Channel 3 Transfer Complete clear */
#define DMA_IFCR_CHTIF3 ((uint32_t)0x00000400) /*!< Channel 3 Half Transfer clear */
#define DMA_IFCR_CTEIF3 ((uint32_t)0x00000800) /*!< Channel 3 Transfer Error clear */
#define DMA_IFCR_CGIF4 ((uint32_t)0x00001000)  /*!< Channel 4 Global interrupt clear */
#define DMA_IFCR_CTCIF4 ((uint32_t)0x00002000) /*!< Channel 4 Transfer Complete clear */
#define DMA_IFCR_CHTIF4 ((uint32_t)0x00004000) /*!< Channel 4 Half Transfer clear */
#define DMA_IFCR_CTEIF4 ((uint32_t)0x00008000) /*!< Channel 4 Transfer Error clear */
#define DMA_IFCR_CGIF5 ((uint32_t)0x00010000)  /*!< Channel 5 Global interrupt clear */
#define DMA_IFCR_CTCIF5 ((uint32_t)0x00020000) /*!< Channel 5 Transfer Complete clear */
#define DMA_IFCR_CHTIF5 ((uint32_t)0x00040000) /*!< Channel 5 Half Transfer clear */
#define DMA_IFCR_CTEIF5 ((uint32_t)0x00080000) /*!< Channel 5 Transfer Error clear */
#define DMA_IFCR_CGIF6 ((uint32_t)0x00100000)  /*!< Channel 6 Global interrupt clear */
#define DMA_IFCR_CTCIF6 ((uint32_t)0x00200000) /*!< Channel 6 Transfer Complete clear */
#define DMA_IFCR_CHTIF6 ((uint32_t)0x00400000) /*!< Channel 6 Half Transfer clear */
#define DMA_IFCR_CTEIF6 ((uint32_t)0x00800000) /*!< Channel 6 Transfer Error clear */
#define DMA_IFCR_CGIF7 ((uint32_t)0x01000000)  /*!< Channel 7 Global interrupt clear */
#define DMA_IFCR_CTCIF7 ((uint32_t)0x02000000) /*!< Channel 7 Transfer Complete clear */
#define DMA_IFCR_CHTIF7 ((uint32_t)0x04000000) /*!< Channel 7 Half Transfer clear */
#define DMA_IFCR_CTEIF7 ((uint32_t)0x08000000) /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR1 register  *******************/
#define DMA_CCR1_EN ((uint16_t)0x0001)   /*!< Channel enable*/
#define DMA_CCR1_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR1_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR1_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR1_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR1_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR1_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR1_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR1_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR1_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR1_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR1_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR1_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR1_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR1_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits(Channel Priority level) */
#define DMA_CCR1_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR1_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR1_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR2 register  *******************/
#define DMA_CCR2_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR2_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR2_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR2_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR2_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR2_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR2_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR2_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR2_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR2_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR2_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR2_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR2_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR2_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR2_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR2_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR2_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR2_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR3 register  *******************/
#define DMA_CCR3_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR3_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR3_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR3_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR3_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR3_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR3_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR3_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR3_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR3_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR3_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR3_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR3_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR3_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR3_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR3_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR3_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR3_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*!<******************  Bit definition for DMA_CCR4 register  *******************/
#define DMA_CCR4_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR4_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR4_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR4_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR4_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR4_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR4_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR4_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR4_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR4_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR4_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR4_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR4_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR4_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR4_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR4_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR4_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR4_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/******************  Bit definition for DMA_CCR5 register  *******************/
#define DMA_CCR5_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR5_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR5_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR5_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR5_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR5_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR5_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR5_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR5_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR5_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR5_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR5_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR5_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR5_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR5_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR5_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR5_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR5_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode enable */

/*******************  Bit definition for DMA_CCR6 register  *******************/
#define DMA_CCR6_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR6_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR6_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR6_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR6_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR6_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR6_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR6_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR6_PSIZE ((uint16_t)0x0300)   /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR6_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR6_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR6_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR6_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR6_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR6_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR6_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR6_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR6_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR7 register  *******************/
#define DMA_CCR7_EN ((uint16_t)0x0001)   /*!< Channel enable */
#define DMA_CCR7_TCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CCR7_HTIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CCR7_TEIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CCR7_DIR ((uint16_t)0x0010)  /*!< Data transfer direction */
#define DMA_CCR7_CIRC ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CCR7_PINC ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CCR7_MINC ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CCR7_PSIZE , ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR7_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CCR7_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CCR7_MSIZE ((uint16_t)0x0C00)   /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR7_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CCR7_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CCR7_PL ((uint16_t)0x3000)   /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CCR7_PL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CCR7_PL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CCR7_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode enable */

/******************  Bit definition for DMA_CNDTR1 register  ******************/
#define DMA_CNDTR1_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR2 register  ******************/
#define DMA_CNDTR2_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR3 register  ******************/
#define DMA_CNDTR3_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR4 register  ******************/
#define DMA_CNDTR4_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR5 register  ******************/
#define DMA_CNDTR5_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR6 register  ******************/
#define DMA_CNDTR6_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR7 register  ******************/
#define DMA_CNDTR7_NDT ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CPAR1 register  *******************/
#define DMA_CPAR1_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR2 register  *******************/
#define DMA_CPAR2_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR3 register  *******************/
#define DMA_CPAR3_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR4 register  *******************/
#define DMA_CPAR4_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR5 register  *******************/
#define DMA_CPAR5_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR6 register  *******************/
#define DMA_CPAR6_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR7 register  *******************/
#define DMA_CPAR7_PA ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_CMAR1 register  *******************/
#define DMA_CMAR1_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR2 register  *******************/
#define DMA_CMAR2_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR3 register  *******************/
#define DMA_CMAR3_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR4 register  *******************/
#define DMA_CMAR4_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR5 register  *******************/
#define DMA_CMAR5_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR6 register  *******************/
#define DMA_CMAR6_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_CMAR7 register  *******************/
#define DMA_CMAR7_MA ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_AWD ((uint8_t)0x01)   /*!< Analog watchdog flag */
#define ADC_SR_EOC ((uint8_t)0x02)   /*!< End of conversion */
#define ADC_SR_JEOC ((uint8_t)0x04)  /*!< Injected channel end of conversion */
#define ADC_SR_JSTRT ((uint8_t)0x08) /*!< Injected channel Start flag */
#define ADC_SR_STRT ((uint8_t)0x10)  /*!< Regular channel Start flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define ADC_CR1_AWDCH ((uint32_t)0x0000001F)   /*!< AWDCH[4:0] bits (Analog watchdog channel select bits) */

#define ADC_CR1_EOCIE ((uint32_t)0x00000020)   /*!< Interrupt enable for EOC */
#define ADC_CR1_AWDIE ((uint32_t)0x00000040)   /*!< Analog Watchdog interrupt enable */
#define ADC_CR1_JEOCIE ((uint32_t)0x00000080)  /*!< Interrupt enable for injected channels */
#define ADC_CR1_SCAN ((uint32_t)0x00000100)    /*!< Scan mode */
#define ADC_CR1_AWDSGL ((uint32_t)0x00000200)  /*!< Enable the watchdog on a single channel in scan mode */
#define ADC_CR1_JAUTO ((uint32_t)0x00000400)   /*!< Automatic injected group conversion */
#define ADC_CR1_DISCEN ((uint32_t)0x00000800)  /*!< Discontinuous mode on regular channels */
#define ADC_CR1_JDISCEN ((uint32_t)0x00001000) /*!< Discontinuous mode on injected channels */

#define ADC_CR1_DISCNUM ((uint32_t)0x0000E000)   /*!< DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define ADC_CR1_DUALMOD ((uint32_t)0x000F0000)   /*!< DUALMOD[3:0] bits (Dual mode selection) */
#define ADC_CR1_JAWDEN ((uint32_t)0x00400000) /*!< Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN ((uint32_t)0x00800000)  /*!< Analog watchdog enable on regular channels */

/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON ((uint32_t)0x00000001)   /*!< A/D Converter ON / OFF */
#define ADC_CR2_CONT ((uint32_t)0x00000002)   /*!< Continuous Conversion */
#define ADC_CR2_CAL ((uint32_t)0x00000004)    /*!< A/D Calibration */
#define ADC_CR2_RSTCAL ((uint32_t)0x00000008) /*!< Reset Calibration */
#define ADC_CR2_DMA ((uint32_t)0x00000100)    /*!< Direct Memory access mode */
#define ADC_CR2_ALIGN ((uint32_t)0x00000800)  /*!< Data Alignment */

#define ADC_CR2_JEXTSEL ((uint32_t)0x00007000)   /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define ADC_CR2_JEXTTRIG ((uint32_t)0x00008000) /*!< External Trigger Conversion mode for injected channels */

#define ADC_CR2_EXTSEL ((uint32_t)0x000E0000)   /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define ADC_CR2_EXTTRIG ((uint32_t)0x00100000)  /*!< External Trigger Conversion mode for regular channels */
#define ADC_CR2_JSWSTART ((uint32_t)0x00200000) /*!< Start Conversion of injected channels */
#define ADC_CR2_SWSTART ((uint32_t)0x00400000)  /*!< Start Conversion of regular channels */
#define ADC_CR2_TSVREFE ((uint32_t)0x00800000)  /*!< Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define ADC_SMPR1_SMP10 ((uint32_t)0x00000007)   /*!< SMP10[2:0] bits (Channel 10 Sample time selection) */
#define ADC_SMPR1_SMP11 ((uint32_t)0x00000038)   /*!< SMP11[2:0] bits (Channel 11 Sample time selection) */
#define ADC_SMPR1_SMP12 ((uint32_t)0x000001C0)   /*!< SMP12[2:0] bits (Channel 12 Sample time selection) */
#define ADC_SMPR1_SMP13 ((uint32_t)0x00000E00)   /*!< SMP13[2:0] bits (Channel 13 Sample time selection) */
#define ADC_SMPR1_SMP14 ((uint32_t)0x00007000)   /*!< SMP14[2:0] bits (Channel 14 Sample time selection) */
#define ADC_SMPR1_SMP15 ((uint32_t)0x00038000)   /*!< SMP15[2:0] bits (Channel 15 Sample time selection) */
#define ADC_SMPR1_SMP16 ((uint32_t)0x001C0000)   /*!< SMP16[2:0] bits (Channel 16 Sample time selection) */
#define ADC_SMPR1_SMP17 ((uint32_t)0x00E00000)   /*!< SMP17[2:0] bits (Channel 17 Sample time selection) */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0 ((uint32_t)0x00000007)   /*!< SMP0[2:0] bits (Channel 0 Sample time selection) */
#define ADC_SMPR2_SMP1 ((uint32_t)0x00000038)   /*!< SMP1[2:0] bits (Channel 1 Sample time selection) */
#define ADC_SMPR2_SMP2 ((uint32_t)0x000001C0)   /*!< SMP2[2:0] bits (Channel 2 Sample time selection) */
#define ADC_SMPR2_SMP3 ((uint32_t)0x00000E00)   /*!< SMP3[2:0] bits (Channel 3 Sample time selection) */
#define ADC_SMPR2_SMP4 ((uint32_t)0x00007000)   /*!< SMP4[2:0] bits (Channel 4 Sample time selection) */
#define ADC_SMPR2_SMP5 ((uint32_t)0x00038000)   /*!< SMP5[2:0] bits (Channel 5 Sample time selection) */
#define ADC_SMPR2_SMP6 ((uint32_t)0x001C0000)   /*!< SMP6[2:0] bits (Channel 6 Sample time selection) */
#define ADC_SMPR2_SMP7 ((uint32_t)0x00E00000)   /*!< SMP7[2:0] bits (Channel 7 Sample time selection) */
#define ADC_SMPR2_SMP8 ((uint32_t)0x07000000)   /*!< SMP8[2:0] bits (Channel 8 Sample time selection) */
#define ADC_SMPR2_SMP9 ((uint32_t)0x38000000)   /*!< SMP9[2:0] bits (Channel 9 Sample time selection) */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define ADC_JOFR1_JOFFSET1 ((uint16_t)0x0FFF) /*!< Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define ADC_JOFR2_JOFFSET2 ((uint16_t)0x0FFF) /*!< Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define ADC_JOFR3_JOFFSET3 ((uint16_t)0x0FFF) /*!< Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define ADC_JOFR4_JOFFSET4 ((uint16_t)0x0FFF) /*!< Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define ADC_HTR_HT ((uint16_t)0x0FFF) /*!< Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define ADC_LTR_LT ((uint16_t)0x0FFF) /*!< Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define ADC_SQR1_SQ13 ((uint32_t)0x0000001F)   /*!< SQ13[4:0] bits (13th conversion in regular sequence) */
#define ADC_SQR1_SQ14 ((uint32_t)0x000003E0)   /*!< SQ14[4:0] bits (14th conversion in regular sequence) */
#define ADC_SQR1_SQ15 ((uint32_t)0x00007C00)   /*!< SQ15[4:0] bits (15th conversion in regular sequence) */
#define ADC_SQR1_SQ16 ((uint32_t)0x000F8000)   /*!< SQ16[4:0] bits (16th conversion in regular sequence) */
#define ADC_SQR1_L ((uint32_t)0x00F00000)   /*!< L[3:0] bits (Regular channel sequence length) */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define ADC_SQR2_SQ7 ((uint32_t)0x0000001F)   /*!< SQ7[4:0] bits (7th conversion in regular sequence) */
#define ADC_SQR2_SQ8 ((uint32_t)0x000003E0)   /*!< SQ8[4:0] bits (8th conversion in regular sequence) */
#define ADC_SQR2_SQ9 ((uint32_t)0x00007C00)   /*!< SQ9[4:0] bits (9th conversion in regular sequence) */

#define ADC_SQR2_SQ10 ((uint32_t)0x000F8000)   /*!< SQ10[4:0] bits (10th conversion in regular sequence) */
#define ADC_SQR2_SQ11 ((uint32_t)0x01F00000)   /*!< SQ11[4:0] bits (11th conversion in regular sequence) */
#define ADC_SQR2_SQ12 ((uint32_t)0x3E000000)   /*!< SQ12[4:0] bits (12th conversion in regular sequence) */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define ADC_SQR3_SQ1 ((uint32_t)0x0000001F)   /*!< SQ1[4:0] bits (1st conversion in regular sequence) */
#define ADC_SQR3_SQ2 ((uint32_t)0x000003E0)   /*!< SQ2[4:0] bits (2nd conversion in regular sequence) */
#define ADC_SQR3_SQ3 ((uint32_t)0x00007C00)   /*!< SQ3[4:0] bits (3rd conversion in regular sequence) */
#define ADC_SQR3_SQ4 ((uint32_t)0x000F8000)   /*!< SQ4[4:0] bits (4th conversion in regular sequence) */
#define ADC_SQR3_SQ5 ((uint32_t)0x01F00000)   /*!< SQ5[4:0] bits (5th conversion in regular sequence) */
#define ADC_SQR3_SQ6 ((uint32_t)0x3E000000)   /*!< SQ6[4:0] bits (6th conversion in regular sequence) */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define ADC_JSQR_JSQ1 ((uint32_t)0x0000001F)   /*!< JSQ1[4:0] bits (1st conversion in injected sequence) */
#define ADC_JSQR_JSQ2 ((uint32_t)0x000003E0)   /*!< JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define ADC_JSQR_JSQ3 ((uint32_t)0x00007C00)   /*!< JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define ADC_JSQR_JSQ4 ((uint32_t)0x000F8000)   /*!< JSQ4[4:0] bits (4th conversion in injected sequence) */
#define ADC_JSQR_JL ((uint32_t)0x00300000)   /*!< JL[1:0] bits (Injected Sequence length) */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define ADC_JDR1_JDATA ((uint16_t)0xFFFF) /*!< Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define ADC_JDR2_JDATA ((uint16_t)0xFFFF) /*!< Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define ADC_JDR3_JDATA ((uint16_t)0xFFFF) /*!< Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define ADC_JDR4_JDATA ((uint16_t)0xFFFF) /*!< Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA ((uint32_t)0x0000FFFF)     /*!< Regular data */
#define ADC_DR_ADC2DATA ((uint32_t)0xFFFF0000) /*!< ADC2 data */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CEN ((uint16_t)0x0001)  /*!< Counter enable */
#define TIM_CR1_UDIS ((uint16_t)0x0002) /*!< Update disable */
#define TIM_CR1_URS ((uint16_t)0x0004)  /*!< Update request source */
#define TIM_CR1_OPM ((uint16_t)0x0008)  /*!< One pulse mode */
#define TIM_CR1_DIR ((uint16_t)0x0010)  /*!< Direction */

#define TIM_CR1_CMS ((uint16_t)0x0060)   /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_ARPE ((uint16_t)0x0080) /*!< Auto-reload preload enable */

#define TIM_CR1_CKD ((uint16_t)0x0300)   /*!< CKD[1:0] bits (clock division) */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define TIM_CR2_CCPC ((uint16_t)0x0001) /*!< Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS ((uint16_t)0x0004) /*!< Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS ((uint16_t)0x0008) /*!< Capture/Compare DMA Selection */

#define TIM_CR2_MMS ((uint16_t)0x0070)   /*!< MMS[2:0] bits (Master Mode Selection) */

#define TIM_CR2_TI1S ((uint16_t)0x0080)  /*!< TI1 Selection */
#define TIM_CR2_OIS1 ((uint16_t)0x0100)  /*!< Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N ((uint16_t)0x0200) /*!< Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2 ((uint16_t)0x0400)  /*!< Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N ((uint16_t)0x0800) /*!< Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3 ((uint16_t)0x1000)  /*!< Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N ((uint16_t)0x2000) /*!< Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4 ((uint16_t)0x4000)  /*!< Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define TIM_SMCR_SMS ((uint16_t)0x0007)   /*!< SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_TS ((uint16_t)0x0070)   /*!< TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_MSM ((uint16_t)0x0080) /*!< Master/slave mode */

#define TIM_SMCR_ETF ((uint16_t)0x0F00)   /*!< ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETPS ((uint16_t)0x3000)   /*!< ETPS[1:0] bits (External trigger prescaler) */

#define TIM_SMCR_ECE ((uint16_t)0x4000) /*!< External clock enable */
#define TIM_SMCR_ETP ((uint16_t)0x8000) /*!< External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE ((uint16_t)0x0001)   /*!< Update interrupt enable */
#define TIM_DIER_CC1IE ((uint16_t)0x0002) /*!< Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE ((uint16_t)0x0004) /*!< Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE ((uint16_t)0x0008) /*!< Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE ((uint16_t)0x0010) /*!< Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE ((uint16_t)0x0020) /*!< COM interrupt enable */
#define TIM_DIER_TIE ((uint16_t)0x0040)   /*!< Trigger interrupt enable */
#define TIM_DIER_BIE ((uint16_t)0x0080)   /*!< Break interrupt enable */
#define TIM_DIER_UDE ((uint16_t)0x0100)   /*!< Update DMA request enable */
#define TIM_DIER_CC1DE ((uint16_t)0x0200) /*!< Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE ((uint16_t)0x0400) /*!< Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE ((uint16_t)0x0800) /*!< Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE ((uint16_t)0x1000) /*!< Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE ((uint16_t)0x2000) /*!< COM DMA request enable */
#define TIM_DIER_TDE ((uint16_t)0x4000)   /*!< Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF ((uint16_t)0x0001)   /*!< Update interrupt Flag */
#define TIM_SR_CC1IF ((uint16_t)0x0002) /*!< Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF ((uint16_t)0x0004) /*!< Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF ((uint16_t)0x0008) /*!< Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF ((uint16_t)0x0010) /*!< Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF ((uint16_t)0x0020) /*!< COM interrupt Flag */
#define TIM_SR_TIF ((uint16_t)0x0040)   /*!< Trigger interrupt Flag */
#define TIM_SR_BIF ((uint16_t)0x0080)   /*!< Break interrupt Flag */
#define TIM_SR_CC1OF ((uint16_t)0x0200) /*!< Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF ((uint16_t)0x0400) /*!< Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF ((uint16_t)0x0800) /*!< Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF ((uint16_t)0x1000) /*!< Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define TIM_EGR_UG ((uint8_t)0x01)   /*!< Update Generation */
#define TIM_EGR_CC1G ((uint8_t)0x02) /*!< Capture/Compare 1 Generation */
#define TIM_EGR_CC2G ((uint8_t)0x04) /*!< Capture/Compare 2 Generation */
#define TIM_EGR_CC3G ((uint8_t)0x08) /*!< Capture/Compare 3 Generation */
#define TIM_EGR_CC4G ((uint8_t)0x10) /*!< Capture/Compare 4 Generation */
#define TIM_EGR_COMG ((uint8_t)0x20) /*!< Capture/Compare Control Update Generation */
#define TIM_EGR_TG ((uint8_t)0x40)   /*!< Trigger Generation */
#define TIM_EGR_BG ((uint8_t)0x80)   /*!< Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define TIM_CCMR1_CC1S ((uint16_t)0x0003)   /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_OC1FE ((uint16_t)0x0004) /*!< Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE ((uint16_t)0x0008) /*!< Output Compare 1 Preload enable */
#define TIM_CCMR1_OC1M ((uint16_t)0x0070)   /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1CE ((uint16_t)0x0080) /*!< Output Compare 1Clear Enable */
#define TIM_CCMR1_CC2S ((uint16_t)0x0300)   /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_OC2FE ((uint16_t)0x0400) /*!< Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE ((uint16_t)0x0800) /*!< Output Compare 2 Preload enable */
#define TIM_CCMR1_OC2M ((uint16_t)0x7000)   /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2CE ((uint16_t)0x8000) /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC ((uint16_t)0x000C)   /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1F ((uint16_t)0x00F0)   /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC2PSC ((uint16_t)0x0C00)   /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2F ((uint16_t)0xF000)   /*!< IC2F[3:0] bits (Input Capture 2 Filter) */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define TIM_CCMR2_CC3S ((uint16_t)0x0003)   /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_OC3FE ((uint16_t)0x0004) /*!< Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE ((uint16_t)0x0008) /*!< Output Compare 3 Preload enable */
#define TIM_CCMR2_OC3M ((uint16_t)0x0070)   /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3CE ((uint16_t)0x0080) /*!< Output Compare 3 Clear Enable */
#define TIM_CCMR2_CC4S ((uint16_t)0x0300)   /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_OC4FE ((uint16_t)0x0400) /*!< Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE ((uint16_t)0x0800) /*!< Output Compare 4 Preload enable */
#define TIM_CCMR2_OC4M ((uint16_t)0x7000)   /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4CE ((uint16_t)0x8000) /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC ((uint16_t)0x000C)   /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3F ((uint16_t)0x00F0)   /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC4PSC ((uint16_t)0x0C00)   /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4F ((uint16_t)0xF000)   /*!< IC4F[3:0] bits (Input Capture 4 Filter) */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E ((uint16_t)0x0001)  /*!< Capture/Compare 1 output enable */
#define TIM_CCER_CC1P ((uint16_t)0x0002)  /*!< Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE ((uint16_t)0x0004) /*!< Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP ((uint16_t)0x0008) /*!< Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E ((uint16_t)0x0010)  /*!< Capture/Compare 2 output enable */
#define TIM_CCER_CC2P ((uint16_t)0x0020)  /*!< Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE ((uint16_t)0x0040) /*!< Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP ((uint16_t)0x0080) /*!< Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E ((uint16_t)0x0100)  /*!< Capture/Compare 3 output enable */
#define TIM_CCER_CC3P ((uint16_t)0x0200)  /*!< Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE ((uint16_t)0x0400) /*!< Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP ((uint16_t)0x0800) /*!< Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E ((uint16_t)0x1000)  /*!< Capture/Compare 4 output enable */
#define TIM_CCER_CC4P ((uint16_t)0x2000)  /*!< Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP ((uint16_t)0x8000) /*!< Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT ((uint16_t)0xFFFF) /*!< Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC ((uint16_t)0xFFFF) /*!< Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define TIM_ARR_ARR ((uint16_t)0xFFFF) /*!< actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define TIM_RCR_REP ((uint8_t)0xFF) /*!< Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define TIM_CCR1_CCR1 ((uint16_t)0xFFFF) /*!< Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define TIM_CCR2_CCR2 ((uint16_t)0xFFFF) /*!< Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define TIM_CCR3_CCR3 ((uint16_t)0xFFFF) /*!< Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define TIM_CCR4_CCR4 ((uint16_t)0xFFFF) /*!< Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_DTG ((uint16_t)0x00FF)   /*!< DTG[0:7] bits (Dead-Time Generator set-up) */

#define TIM_BDTR_LOCK ((uint16_t)0x0300)   /*!< LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_OSSI ((uint16_t)0x0400) /*!< Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR ((uint16_t)0x0800) /*!< Off-State Selection for Run mode */
#define TIM_BDTR_BKE ((uint16_t)0x1000)  /*!< Break enable */
#define TIM_BDTR_BKP ((uint16_t)0x2000)  /*!< Break Polarity */
#define TIM_BDTR_AOE ((uint16_t)0x4000)  /*!< Automatic Output enable */
#define TIM_BDTR_MOE ((uint16_t)0x8000)  /*!< Main Output enable */

/*******************  Bit definition for TIM_DCR register  ********************/
#define TIM_DCR_DBA ((uint16_t)0x001F)   /*!< DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBL ((uint16_t)0x1F00)   /*!< DBL[4:0] bits (DMA Burst Length) */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define TIM_DMAR_DMAB ((uint16_t)0xFFFF) /*!< DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for RTC_CRH register  ********************/
#define RTC_CRH_SECIE ((uint8_t)0x01) /*!< Second Interrupt Enable */
#define RTC_CRH_ALRIE ((uint8_t)0x02) /*!< Alarm Interrupt Enable */
#define RTC_CRH_OWIE ((uint8_t)0x04)  /*!< OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CRL register  ********************/
#define RTC_CRL_SECF ((uint8_t)0x01)  /*!< Second Flag */
#define RTC_CRL_ALRF ((uint8_t)0x02)  /*!< Alarm Flag */
#define RTC_CRL_OWF ((uint8_t)0x04)   /*!< OverfloW Flag */
#define RTC_CRL_RSF ((uint8_t)0x08)   /*!< Registers Synchronized Flag */
#define RTC_CRL_CNF ((uint8_t)0x10)   /*!< Configuration Flag */
#define RTC_CRL_RTOFF ((uint8_t)0x20) /*!< RTC operation OFF */

/*******************  Bit definition for RTC_PRLH register  *******************/
#define RTC_PRLH_PRL ((uint16_t)0x000F) /*!< RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PRLL register  *******************/
#define RTC_PRLL_PRL ((uint16_t)0xFFFF) /*!< RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define RTC_DIVH_RTC_DIV ((uint16_t)0x000F) /*!< RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define RTC_DIVL_RTC_DIV ((uint16_t)0xFFFF) /*!< RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define RTC_CNTH_RTC_CNT ((uint16_t)0xFFFF) /*!< RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define RTC_CNTL_RTC_CNT ((uint16_t)0xFFFF) /*!< RTC Counter Low */

/*******************  Bit definition for RTC_ALRH register  *******************/
#define RTC_ALRH_RTC_ALR ((uint16_t)0xFFFF) /*!< RTC Alarm High */

/*******************  Bit definition for RTC_ALRL register  *******************/
#define RTC_ALRL_RTC_ALR ((uint16_t)0xFFFF) /*!< RTC Alarm Low */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY ((uint16_t)0xFFFF) /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR ((uint8_t)0x07)   /*!< PR[2:0] (Prescaler divider) */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL ((uint16_t)0x0FFF) /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU ((uint8_t)0x01) /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU ((uint8_t)0x02) /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T ((uint8_t)0x7F)  /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_WDGA ((uint8_t)0x80) /*!< Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W ((uint16_t)0x007F)  /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFR_WDGTB ((uint16_t)0x0180)  /*!< WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_EWI ((uint16_t)0x0200) /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF ((uint8_t)0x01) /*!< Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/

/*!< CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define CAN_MCR_INRQ ((uint16_t)0x0001)  /*!< Initialization Request */
#define CAN_MCR_SLEEP ((uint16_t)0x0002) /*!< Sleep Mode Request */
#define CAN_MCR_TXFP ((uint16_t)0x0004)  /*!< Transmit FIFO Priority */
#define CAN_MCR_RFLM ((uint16_t)0x0008)  /*!< Receive FIFO Locked Mode */
#define CAN_MCR_NART ((uint16_t)0x0010)  /*!< No Automatic Retransmission */
#define CAN_MCR_AWUM ((uint16_t)0x0020)  /*!< Automatic Wakeup Mode */
#define CAN_MCR_ABOM ((uint16_t)0x0040)  /*!< Automatic Bus-Off Management */
#define CAN_MCR_TTCM ((uint16_t)0x0080)  /*!< Time Triggered Communication Mode */
#define CAN_MCR_RESET ((uint16_t)0x8000) /*!< CAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define CAN_MSR_INAK ((uint16_t)0x0001)  /*!< Initialization Acknowledge */
#define CAN_MSR_SLAK ((uint16_t)0x0002)  /*!< Sleep Acknowledge */
#define CAN_MSR_ERRI ((uint16_t)0x0004)  /*!< Error Interrupt */
#define CAN_MSR_WKUI ((uint16_t)0x0008)  /*!< Wakeup Interrupt */
#define CAN_MSR_SLAKI ((uint16_t)0x0010) /*!< Sleep Acknowledge Interrupt */
#define CAN_MSR_TXM ((uint16_t)0x0100)   /*!< Transmit Mode */
#define CAN_MSR_RXM ((uint16_t)0x0200)   /*!< Receive Mode */
#define CAN_MSR_SAMP ((uint16_t)0x0400)  /*!< Last Sample Point */
#define CAN_MSR_RX ((uint16_t)0x0800)    /*!< CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define CAN_TSR_RQCP0 ((uint32_t)0x00000001) /*!< Request Completed Mailbox0 */
#define CAN_TSR_TXOK0 ((uint32_t)0x00000002) /*!< Transmission OK of Mailbox0 */
#define CAN_TSR_ALST0 ((uint32_t)0x00000004) /*!< Arbitration Lost for Mailbox0 */
#define CAN_TSR_TERR0 ((uint32_t)0x00000008) /*!< Transmission Error of Mailbox0 */
#define CAN_TSR_ABRQ0 ((uint32_t)0x00000080) /*!< Abort Request for Mailbox0 */
#define CAN_TSR_RQCP1 ((uint32_t)0x00000100) /*!< Request Completed Mailbox1 */
#define CAN_TSR_TXOK1 ((uint32_t)0x00000200) /*!< Transmission OK of Mailbox1 */
#define CAN_TSR_ALST1 ((uint32_t)0x00000400) /*!< Arbitration Lost for Mailbox1 */
#define CAN_TSR_TERR1 ((uint32_t)0x00000800) /*!< Transmission Error of Mailbox1 */
#define CAN_TSR_ABRQ1 ((uint32_t)0x00008000) /*!< Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2 ((uint32_t)0x00010000) /*!< Request Completed Mailbox2 */
#define CAN_TSR_TXOK2 ((uint32_t)0x00020000) /*!< Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2 ((uint32_t)0x00040000) /*!< Arbitration Lost for mailbox 2 */
#define CAN_TSR_TERR2 ((uint32_t)0x00080000) /*!< Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2 ((uint32_t)0x00800000) /*!< Abort Request for Mailbox 2 */
#define CAN_TSR_CODE ((uint32_t)0x03000000)  /*!< Mailbox Code */

#define CAN_TSR_TME ((uint32_t)0x1C000000)  /*!< TME[2:0] bits */
#define CAN_TSR_TME0 ((uint32_t)0x04000000) /*!< Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1 ((uint32_t)0x08000000) /*!< Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2 ((uint32_t)0x10000000) /*!< Transmit Mailbox 2 Empty */

#define CAN_TSR_LOW ((uint32_t)0xE0000000)  /*!< LOW[2:0] bits */
#define CAN_TSR_LOW0 ((uint32_t)0x20000000) /*!< Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1 ((uint32_t)0x40000000) /*!< Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2 ((uint32_t)0x80000000) /*!< Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define CAN_RF0R_FMP0 ((uint8_t)0x03)  /*!< FIFO 0 Message Pending */
#define CAN_RF0R_FULL0 ((uint8_t)0x08) /*!< FIFO 0 Full */
#define CAN_RF0R_FOVR0 ((uint8_t)0x10) /*!< FIFO 0 Overrun */
#define CAN_RF0R_RFOM0 ((uint8_t)0x20) /*!< Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define CAN_RF1R_FMP1 ((uint8_t)0x03)  /*!< FIFO 1 Message Pending */
#define CAN_RF1R_FULL1 ((uint8_t)0x08) /*!< FIFO 1 Full */
#define CAN_RF1R_FOVR1 ((uint8_t)0x10) /*!< FIFO 1 Overrun */
#define CAN_RF1R_RFOM1 ((uint8_t)0x20) /*!< Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define CAN_IER_TMEIE ((uint32_t)0x00000001)  /*!< Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0 ((uint32_t)0x00000002) /*!< FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0 ((uint32_t)0x00000004)  /*!< FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0 ((uint32_t)0x00000008) /*!< FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1 ((uint32_t)0x00000010) /*!< FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1 ((uint32_t)0x00000020)  /*!< FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1 ((uint32_t)0x00000040) /*!< FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE ((uint32_t)0x00000100)  /*!< Error Warning Interrupt Enable */
#define CAN_IER_EPVIE ((uint32_t)0x00000200)  /*!< Error Passive Interrupt Enable */
#define CAN_IER_BOFIE ((uint32_t)0x00000400)  /*!< Bus-Off Interrupt Enable */
#define CAN_IER_LECIE ((uint32_t)0x00000800)  /*!< Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE ((uint32_t)0x00008000)  /*!< Error Interrupt Enable */
#define CAN_IER_WKUIE ((uint32_t)0x00010000)  /*!< Wakeup Interrupt Enable */
#define CAN_IER_SLKIE ((uint32_t)0x00020000)  /*!< Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define CAN_ESR_EWGF ((uint32_t)0x00000001) /*!< Error Warning Flag */
#define CAN_ESR_EPVF ((uint32_t)0x00000002) /*!< Error Passive Flag */
#define CAN_ESR_BOFF ((uint32_t)0x00000004) /*!< Bus-Off Flag */

#define CAN_ESR_LEC ((uint32_t)0x00000070)   /*!< LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0 ((uint32_t)0x00000010) /*!< Bit 0 */
#define CAN_ESR_LEC_1 ((uint32_t)0x00000020) /*!< Bit 1 */
#define CAN_ESR_LEC_2 ((uint32_t)0x00000040) /*!< Bit 2 */

#define CAN_ESR_TEC ((uint32_t)0x00FF0000) /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC ((uint32_t)0xFF000000) /*!< Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define CAN_BTR_BRP ((uint32_t)0x000003FF)  /*!< Baud Rate Prescaler */
#define CAN_BTR_TS1 ((uint32_t)0x000F0000)  /*!< Time Segment 1 */
#define CAN_BTR_TS2 ((uint32_t)0x00700000)  /*!< Time Segment 2 */
#define CAN_BTR_SJW ((uint32_t)0x03000000)  /*!< Resynchronization Jump Width */
#define CAN_BTR_LBKM ((uint32_t)0x40000000) /*!< Loop Back Mode (Debug) */
#define CAN_BTR_SILM ((uint32_t)0x80000000) /*!< Silent Mode */

/*!< Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define CAN_TI0R_TXRQ ((uint32_t)0x00000001) /*!< Transmit Mailbox Request */
#define CAN_TI0R_RTR ((uint32_t)0x00000002)  /*!< Remote Transmission Request */
#define CAN_TI0R_IDE ((uint32_t)0x00000004)  /*!< Identifier Extension */
#define CAN_TI0R_EXID ((uint32_t)0x001FFFF8) /*!< Extended Identifier */
#define CAN_TI0R_STID ((uint32_t)0xFFE00000) /*!< Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define CAN_TDT0R_DLC ((uint32_t)0x0000000F)  /*!< Data Length Code */
#define CAN_TDT0R_TGT ((uint32_t)0x00000100)  /*!< Transmit Global Time */
#define CAN_TDT0R_TIME ((uint32_t)0xFFFF0000) /*!< Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define CAN_TDL0R_DATA0 ((uint32_t)0x000000FF) /*!< Data byte 0 */
#define CAN_TDL0R_DATA1 ((uint32_t)0x0000FF00) /*!< Data byte 1 */
#define CAN_TDL0R_DATA2 ((uint32_t)0x00FF0000) /*!< Data byte 2 */
#define CAN_TDL0R_DATA3 ((uint32_t)0xFF000000) /*!< Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define CAN_TDH0R_DATA4 ((uint32_t)0x000000FF) /*!< Data byte 4 */
#define CAN_TDH0R_DATA5 ((uint32_t)0x0000FF00) /*!< Data byte 5 */
#define CAN_TDH0R_DATA6 ((uint32_t)0x00FF0000) /*!< Data byte 6 */
#define CAN_TDH0R_DATA7 ((uint32_t)0xFF000000) /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define CAN_TI1R_TXRQ ((uint32_t)0x00000001) /*!< Transmit Mailbox Request */
#define CAN_TI1R_RTR ((uint32_t)0x00000002)  /*!< Remote Transmission Request */
#define CAN_TI1R_IDE ((uint32_t)0x00000004)  /*!< Identifier Extension */
#define CAN_TI1R_EXID ((uint32_t)0x001FFFF8) /*!< Extended Identifier */
#define CAN_TI1R_STID ((uint32_t)0xFFE00000) /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define CAN_TDT1R_DLC ((uint32_t)0x0000000F)  /*!< Data Length Code */
#define CAN_TDT1R_TGT ((uint32_t)0x00000100)  /*!< Transmit Global Time */
#define CAN_TDT1R_TIME ((uint32_t)0xFFFF0000) /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define CAN_TDL1R_DATA0 ((uint32_t)0x000000FF) /*!< Data byte 0 */
#define CAN_TDL1R_DATA1 ((uint32_t)0x0000FF00) /*!< Data byte 1 */
#define CAN_TDL1R_DATA2 ((uint32_t)0x00FF0000) /*!< Data byte 2 */
#define CAN_TDL1R_DATA3 ((uint32_t)0xFF000000) /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define CAN_TDH1R_DATA4 ((uint32_t)0x000000FF) /*!< Data byte 4 */
#define CAN_TDH1R_DATA5 ((uint32_t)0x0000FF00) /*!< Data byte 5 */
#define CAN_TDH1R_DATA6 ((uint32_t)0x00FF0000) /*!< Data byte 6 */
#define CAN_TDH1R_DATA7 ((uint32_t)0xFF000000) /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define CAN_TI2R_TXRQ ((uint32_t)0x00000001) /*!< Transmit Mailbox Request */
#define CAN_TI2R_RTR ((uint32_t)0x00000002)  /*!< Remote Transmission Request */
#define CAN_TI2R_IDE ((uint32_t)0x00000004)  /*!< Identifier Extension */
#define CAN_TI2R_EXID ((uint32_t)0x001FFFF8) /*!< Extended identifier */
#define CAN_TI2R_STID ((uint32_t)0xFFE00000) /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define CAN_TDT2R_DLC ((uint32_t)0x0000000F)  /*!< Data Length Code */
#define CAN_TDT2R_TGT ((uint32_t)0x00000100)  /*!< Transmit Global Time */
#define CAN_TDT2R_TIME ((uint32_t)0xFFFF0000) /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define CAN_TDL2R_DATA0 ((uint32_t)0x000000FF) /*!< Data byte 0 */
#define CAN_TDL2R_DATA1 ((uint32_t)0x0000FF00) /*!< Data byte 1 */
#define CAN_TDL2R_DATA2 ((uint32_t)0x00FF0000) /*!< Data byte 2 */
#define CAN_TDL2R_DATA3 ((uint32_t)0xFF000000) /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define CAN_TDH2R_DATA4 ((uint32_t)0x000000FF) /*!< Data byte 4 */
#define CAN_TDH2R_DATA5 ((uint32_t)0x0000FF00) /*!< Data byte 5 */
#define CAN_TDH2R_DATA6 ((uint32_t)0x00FF0000) /*!< Data byte 6 */
#define CAN_TDH2R_DATA7 ((uint32_t)0xFF000000) /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define CAN_RI0R_RTR ((uint32_t)0x00000002)  /*!< Remote Transmission Request */
#define CAN_RI0R_IDE ((uint32_t)0x00000004)  /*!< Identifier Extension */
#define CAN_RI0R_EXID ((uint32_t)0x001FFFF8) /*!< Extended Identifier */
#define CAN_RI0R_STID ((uint32_t)0xFFE00000) /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define CAN_RDT0R_DLC ((uint32_t)0x0000000F)  /*!< Data Length Code */
#define CAN_RDT0R_FMI ((uint32_t)0x0000FF00)  /*!< Filter Match Index */
#define CAN_RDT0R_TIME ((uint32_t)0xFFFF0000) /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define CAN_RDL0R_DATA0 ((uint32_t)0x000000FF) /*!< Data byte 0 */
#define CAN_RDL0R_DATA1 ((uint32_t)0x0000FF00) /*!< Data byte 1 */
#define CAN_RDL0R_DATA2 ((uint32_t)0x00FF0000) /*!< Data byte 2 */
#define CAN_RDL0R_DATA3 ((uint32_t)0xFF000000) /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define CAN_RDH0R_DATA4 ((uint32_t)0x000000FF) /*!< Data byte 4 */
#define CAN_RDH0R_DATA5 ((uint32_t)0x0000FF00) /*!< Data byte 5 */
#define CAN_RDH0R_DATA6 ((uint32_t)0x00FF0000) /*!< Data byte 6 */
#define CAN_RDH0R_DATA7 ((uint32_t)0xFF000000) /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define CAN_RI1R_RTR ((uint32_t)0x00000002)  /*!< Remote Transmission Request */
#define CAN_RI1R_IDE ((uint32_t)0x00000004)  /*!< Identifier Extension */
#define CAN_RI1R_EXID ((uint32_t)0x001FFFF8) /*!< Extended identifier */
#define CAN_RI1R_STID ((uint32_t)0xFFE00000) /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define CAN_RDT1R_DLC ((uint32_t)0x0000000F)  /*!< Data Length Code */
#define CAN_RDT1R_FMI ((uint32_t)0x0000FF00)  /*!< Filter Match Index */
#define CAN_RDT1R_TIME ((uint32_t)0xFFFF0000) /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define CAN_RDL1R_DATA0 ((uint32_t)0x000000FF) /*!< Data byte 0 */
#define CAN_RDL1R_DATA1 ((uint32_t)0x0000FF00) /*!< Data byte 1 */
#define CAN_RDL1R_DATA2 ((uint32_t)0x00FF0000) /*!< Data byte 2 */
#define CAN_RDL1R_DATA3 ((uint32_t)0xFF000000) /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define CAN_RDH1R_DATA4 ((uint32_t)0x000000FF) /*!< Data byte 4 */
#define CAN_RDH1R_DATA5 ((uint32_t)0x0000FF00) /*!< Data byte 5 */
#define CAN_RDH1R_DATA6 ((uint32_t)0x00FF0000) /*!< Data byte 6 */
#define CAN_RDH1R_DATA7 ((uint32_t)0xFF000000) /*!< Data byte 7 */

/*!< CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define CAN_FMR_FINIT ((uint8_t)0x01) /*!< Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define CAN_FM1R_FBM ((uint16_t)0x3FFF)   /*!< Filter Mode */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define CAN_FS1R_FSC ((uint16_t)0x3FFF)   /*!< Filter Scale Configuration */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define CAN_FFA1R_FFA ((uint16_t)0x3FFF)   /*!< Filter FIFO Assignment */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define CAN_FA1R_FACT ((uint16_t)0x3FFF)   /*!< Filter Active */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA ((uint16_t)0x0001) /*!< Clock Phase */
#define SPI_CR1_CPOL ((uint16_t)0x0002) /*!< Clock Polarity */
#define SPI_CR1_MSTR ((uint16_t)0x0004) /*!< Master Selection */

#define SPI_CR1_BR ((uint16_t)0x0038)   /*!< BR[2:0] bits (Baud Rate Control) */

#define SPI_CR1_SPE ((uint16_t)0x0040)      /*!< SPI Enable */
#define SPI_CR1_LSBFIRST ((uint16_t)0x0080) /*!< Frame Format */
#define SPI_CR1_SSI ((uint16_t)0x0100)      /*!< Internal slave select */
#define SPI_CR1_SSM ((uint16_t)0x0200)      /*!< Software slave management */
#define SPI_CR1_RXONLY ((uint16_t)0x0400)   /*!< Receive only */
#define SPI_CR1_DFF ((uint16_t)0x0800)      /*!< Data Frame Format */
#define SPI_CR1_CRCNEXT ((uint16_t)0x1000)  /*!< Transmit CRC next */
#define SPI_CR1_CRCEN ((uint16_t)0x2000)    /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE ((uint16_t)0x4000)   /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE ((uint16_t)0x8000) /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN ((uint8_t)0x01) /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN ((uint8_t)0x02) /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE ((uint8_t)0x04)    /*!< SS Output Enable */
#define SPI_CR2_ERRIE ((uint8_t)0x20)   /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE ((uint8_t)0x40)  /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE ((uint8_t)0x80)   /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE ((uint8_t)0x01)   /*!< Receive buffer Not Empty */
#define SPI_SR_TXE ((uint8_t)0x02)    /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE ((uint8_t)0x04) /*!< Channel side */
#define SPI_SR_UDR ((uint8_t)0x08)    /*!< Underrun flag */
#define SPI_SR_CRCERR ((uint8_t)0x10) /*!< CRC Error flag */
#define SPI_SR_MODF ((uint8_t)0x20)   /*!< Mode fault */
#define SPI_SR_OVR ((uint8_t)0x40)    /*!< Overrun flag */
#define SPI_SR_BSY ((uint8_t)0x80)    /*!< Busy flag */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR ((uint16_t)0xFFFF) /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY ((uint16_t)0xFFFF) /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC ((uint16_t)0xFFFF) /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC ((uint16_t)0xFFFF) /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN ((uint16_t)0x0001) /*!< Channel length (number of bits per audio channel) */
#define SPI_I2SCFGR_DATLEN ((uint16_t)0x0006)   /*!< DATLEN[1:0] bits (Data length to be transferred) */
#define SPI_I2SCFGR_CKPOL ((uint16_t)0x0008) /*!< steady state clock polarity */
#define SPI_I2SCFGR_I2SSTD ((uint16_t)0x0030)   /*!< I2SSTD[1:0] bits (I2S standard selection) */
#define SPI_I2SCFGR_PCMSYNC ((uint16_t)0x0080) /*!< PCM frame synchronization */
#define SPI_I2SCFGR_I2SCFG ((uint16_t)0x0300)   /*!< I2SCFG[1:0] bits (I2S configuration mode) */
#define SPI_I2SCFGR_I2SE ((uint16_t)0x0400)   /*!< I2S Enable */
#define SPI_I2SCFGR_I2SMOD ((uint16_t)0x0800) /*!< I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_I2SDIV ((uint16_t)0x00FF) /*!< I2S Linear prescaler */
#define SPI_I2SPR_ODD ((uint16_t)0x0100)    /*!< Odd factor for the prescaler */
#define SPI_I2SPR_MCKOE ((uint16_t)0x0200)  /*!< Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE ((uint16_t)0x0001)        /*!< Peripheral Enable */
#define I2C_CR1_SMBUS ((uint16_t)0x0002)     /*!< SMBus Mode */
#define I2C_CR1_SMBTYPE ((uint16_t)0x0008)   /*!< SMBus Type */
#define I2C_CR1_ENARP ((uint16_t)0x0010)     /*!< ARP Enable */
#define I2C_CR1_ENPEC ((uint16_t)0x0020)     /*!< PEC Enable */
#define I2C_CR1_ENGC ((uint16_t)0x0040)      /*!< General Call Enable */
#define I2C_CR1_NOSTRETCH ((uint16_t)0x0080) /*!< Clock Stretching Disable (Slave mode) */
#define I2C_CR1_START ((uint16_t)0x0100)     /*!< Start Generation */
#define I2C_CR1_STOP ((uint16_t)0x0200)      /*!< Stop Generation */
#define I2C_CR1_ACK ((uint16_t)0x0400)       /*!< Acknowledge Enable */
#define I2C_CR1_POS ((uint16_t)0x0800)       /*!< Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC ((uint16_t)0x1000)       /*!< Packet Error Checking */
#define I2C_CR1_ALERT ((uint16_t)0x2000)     /*!< SMBus Alert */
#define I2C_CR1_SWRST ((uint16_t)0x8000)     /*!< Software Reset */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ ((uint16_t)0x003F)   /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define I2C_CR2_ITERREN ((uint16_t)0x0100) /*!< Error Interrupt Enable */
#define I2C_CR2_ITEVTEN ((uint16_t)0x0200) /*!< Event Interrupt Enable */
#define I2C_CR2_ITBUFEN ((uint16_t)0x0400) /*!< Buffer Interrupt Enable */
#define I2C_CR2_DMAEN ((uint16_t)0x0800)   /*!< DMA Requests Enable */
#define I2C_CR2_LAST ((uint16_t)0x1000)    /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7 ((uint16_t)0x00FE) /*!< Interface Address */
#define I2C_OAR1_ADD8_9 ((uint16_t)0x0300) /*!< Interface Address */
#define I2C_OAR1_ADDMODE ((uint16_t)0x8000) /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL ((uint8_t)0x01) /*!< Dual addressing mode enable */
#define I2C_OAR2_ADD2 ((uint8_t)0xFE)   /*!< Interface address */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR ((uint8_t)0xFF) /*!< 8-bit Data Register */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB ((uint16_t)0x0001)       /*!< Start Bit (Master mode) */
#define I2C_SR1_ADDR ((uint16_t)0x0002)     /*!< Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF ((uint16_t)0x0004)      /*!< Byte Transfer Finished */
#define I2C_SR1_ADD10 ((uint16_t)0x0008)    /*!< 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF ((uint16_t)0x0010)    /*!< Stop detection (Slave mode) */
#define I2C_SR1_RXNE ((uint16_t)0x0040)     /*!< Data Register not Empty (receivers) */
#define I2C_SR1_TXE ((uint16_t)0x0080)      /*!< Data Register Empty (transmitters) */
#define I2C_SR1_BERR ((uint16_t)0x0100)     /*!< Bus Error */
#define I2C_SR1_ARLO ((uint16_t)0x0200)     /*!< Arbitration Lost (master mode) */
#define I2C_SR1_AF ((uint16_t)0x0400)       /*!< Acknowledge Failure */
#define I2C_SR1_OVR ((uint16_t)0x0800)      /*!< Overrun/Underrun */
#define I2C_SR1_PECERR ((uint16_t)0x1000)   /*!< PEC Error in reception */
#define I2C_SR1_TIMEOUT ((uint16_t)0x4000)  /*!< Timeout or Tlow Error */
#define I2C_SR1_SMBALERT ((uint16_t)0x8000) /*!< SMBus Alert */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL ((uint16_t)0x0001)        /*!< Master/Slave */
#define I2C_SR2_BUSY ((uint16_t)0x0002)       /*!< Bus Busy */
#define I2C_SR2_TRA ((uint16_t)0x0004)        /*!< Transmitter/Receiver */
#define I2C_SR2_GENCALL ((uint16_t)0x0010)    /*!< General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT ((uint16_t)0x0020) /*!< SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST ((uint16_t)0x0040)    /*!< SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF ((uint16_t)0x0080)      /*!< Dual Flag (Slave mode) */
#define I2C_SR2_PEC ((uint16_t)0xFF00)        /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR ((uint16_t)0x0FFF)  /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_DUTY ((uint16_t)0x4000) /*!< Fast Mode Duty Cycle */
#define I2C_CCR_FS ((uint16_t)0x8000)   /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE ((uint8_t)0x3F) /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE ((uint16_t)0x0001)   /*!< Parity Error */
#define USART_SR_FE ((uint16_t)0x0002)   /*!< Framing Error */
#define USART_SR_NE ((uint16_t)0x0004)   /*!< Noise Error Flag */
#define USART_SR_ORE ((uint16_t)0x0008)  /*!< OverRun Error */
#define USART_SR_IDLE ((uint16_t)0x0010) /*!< IDLE line detected */
#define USART_SR_RXNE ((uint16_t)0x0020) /*!< Read Data Register Not Empty */
#define USART_SR_TC ((uint16_t)0x0040)   /*!< Transmission Complete */
#define USART_SR_TXE ((uint16_t)0x0080)  /*!< Transmit Data Register Empty */
#define USART_SR_LBD ((uint16_t)0x0100)  /*!< LIN Break Detection Flag */
#define USART_SR_CTS ((uint16_t)0x0200)  /*!< CTS Flag */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR ((uint16_t)0x01FF) /*!< Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction ((uint16_t)0x000F) /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa ((uint16_t)0xFFF0) /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK ((uint16_t)0x0001)    /*!< Send Break */
#define USART_CR1_RWU ((uint16_t)0x0002)    /*!< Receiver wakeup */
#define USART_CR1_RE ((uint16_t)0x0004)     /*!< Receiver Enable */
#define USART_CR1_TE ((uint16_t)0x0008)     /*!< Transmitter Enable */
#define USART_CR1_IDLEIE ((uint16_t)0x0010) /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE ((uint16_t)0x0020) /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE ((uint16_t)0x0040)   /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE ((uint16_t)0x0080)  /*!< PE Interrupt Enable */
#define USART_CR1_PEIE ((uint16_t)0x0100)   /*!< PE Interrupt Enable */
#define USART_CR1_PS ((uint16_t)0x0200)     /*!< Parity Selection */
#define USART_CR1_PCE ((uint16_t)0x0400)    /*!< Parity Control Enable */
#define USART_CR1_WAKE ((uint16_t)0x0800)   /*!< Wakeup method */
#define USART_CR1_M ((uint16_t)0x1000)      /*!< Word length */
#define USART_CR1_UE ((uint16_t)0x2000)     /*!< USART Enable */
#define USART_CR1_OVER8 ((uint16_t)0x8000)  /*!< USART Oversmapling 8-bits */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD ((uint16_t)0x000F)   /*!< Address of the USART node */
#define USART_CR2_LBDL ((uint16_t)0x0020)  /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE ((uint16_t)0x0040) /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL ((uint16_t)0x0100)  /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA ((uint16_t)0x0200)  /*!< Clock Phase */
#define USART_CR2_CPOL ((uint16_t)0x0400)  /*!< Clock Polarity */
#define USART_CR2_CLKEN ((uint16_t)0x0800) /*!< Clock Enable */

#define USART_CR2_STOP ((uint16_t)0x3000)   /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_LINEN ((uint16_t)0x4000) /*!< LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE ((uint16_t)0x0001)    /*!< Error Interrupt Enable */
#define USART_CR3_IREN ((uint16_t)0x0002)   /*!< IrDA mode Enable */
#define USART_CR3_IRLP ((uint16_t)0x0004)   /*!< IrDA Low-Power */
#define USART_CR3_HDSEL ((uint16_t)0x0008)  /*!< Half-Duplex Selection */
#define USART_CR3_NACK ((uint16_t)0x0010)   /*!< Smartcard NACK enable */
#define USART_CR3_SCEN ((uint16_t)0x0020)   /*!< Smartcard mode enable */
#define USART_CR3_DMAR ((uint16_t)0x0040)   /*!< DMA Enable Receiver */
#define USART_CR3_DMAT ((uint16_t)0x0080)   /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE ((uint16_t)0x0100)   /*!< RTS Enable */
#define USART_CR3_CTSE ((uint16_t)0x0200)   /*!< CTS Enable */
#define USART_CR3_CTSIE ((uint16_t)0x0400)  /*!< CTS Interrupt Enable */
#define USART_CR3_ONEBIT ((uint16_t)0x0800) /*!< One Bit method */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC ((uint16_t)0x00FF)   /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT ((uint16_t)0xFF00) /*!< Guard time value */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define DBGMCU_IDCODE_DEV_ID ((uint32_t)0x00000FFF) /*!< Device Identifier */

#define DBGMCU_IDCODE_REV_ID ((uint32_t)0xFFFF0000)    /*!< REV_ID[15:0] bits (Revision Identifier) */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define DBGMCU_CR_DBG_SLEEP ((uint32_t)0x00000001)   /*!< Debug Sleep Mode */
#define DBGMCU_CR_DBG_STOP ((uint32_t)0x00000002)    /*!< Debug Stop Mode */
#define DBGMCU_CR_DBG_STANDBY ((uint32_t)0x00000004) /*!< Debug Standby mode */
#define DBGMCU_CR_TRACE_IOEN ((uint32_t)0x00000020)  /*!< Trace Pin Assignment Control */

#define DBGMCU_CR_TRACE_MODE ((uint32_t)0x000000C0)   /*!< TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define DBGMCU_CR_TRACE_MODE_0 ((uint32_t)0x00000040) /*!< Bit 0 */
#define DBGMCU_CR_TRACE_MODE_1 ((uint32_t)0x00000080) /*!< Bit 1 */

#define DBGMCU_CR_DBG_IWDG_STOP ((uint32_t)0x00000100)          /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_WWDG_STOP ((uint32_t)0x00000200)          /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM1_STOP ((uint32_t)0x00000400)          /*!< TIM1 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM2_STOP ((uint32_t)0x00000800)          /*!< TIM2 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM3_STOP ((uint32_t)0x00001000)          /*!< TIM3 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM4_STOP ((uint32_t)0x00002000)          /*!< TIM4 counter stopped when core is halted */
#define DBGMCU_CR_DBG_CAN1_STOP ((uint32_t)0x00004000)          /*!< Debug CAN1 stopped when Core is halted */
#define DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT ((uint32_t)0x00008000) /*!< SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT ((uint32_t)0x00010000) /*!< SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM8_STOP ((uint32_t)0x00020000)          /*!< TIM8 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM5_STOP ((uint32_t)0x00040000)          /*!< TIM5 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM6_STOP ((uint32_t)0x00080000)          /*!< TIM6 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM7_STOP ((uint32_t)0x00100000)          /*!< TIM7 counter stopped when core is halted */
#define DBGMCU_CR_DBG_CAN2_STOP ((uint32_t)0x00200000)          /*!< Debug CAN2 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM15_STOP ((uint32_t)0x00400000)         /*!< Debug TIM15 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM16_STOP ((uint32_t)0x00800000)         /*!< Debug TIM16 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM17_STOP ((uint32_t)0x01000000)         /*!< Debug TIM17 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM12_STOP ((uint32_t)0x02000000)         /*!< Debug TIM12 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM13_STOP ((uint32_t)0x04000000)         /*!< Debug TIM13 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM14_STOP ((uint32_t)0x08000000)         /*!< Debug TIM14 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM9_STOP ((uint32_t)0x10000000)          /*!< Debug TIM9 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM10_STOP ((uint32_t)0x20000000)         /*!< Debug TIM10 stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM11_STOP ((uint32_t)0x40000000)         /*!< Debug TIM11 stopped when Core is halted */

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACR register  ******************/
#define FLASH_ACR_LATENCY ((uint8_t)0x03)   /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_0 ((uint8_t)0x00) /*!< Bit 0 */
#define FLASH_ACR_LATENCY_1 ((uint8_t)0x01) /*!< Bit 0 */
#define FLASH_ACR_LATENCY_2 ((uint8_t)0x02) /*!< Bit 1 */

#define FLASH_ACR_HLFCYA ((uint8_t)0x08) /*!< Flash Half Cycle Access Enable */
#define FLASH_ACR_PRFTBE ((uint8_t)0x10) /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS ((uint8_t)0x20) /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_FKEYR ((uint32_t)0xFFFFFFFF) /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define FLASH_OPTKEYR_OPTKEYR ((uint32_t)0xFFFFFFFF) /*!< Option Byte Key */

/******************  Bit definition for FLASH_SR register  *******************/
#define FLASH_SR_BSY ((uint8_t)0x01)      /*!< Busy */
#define FLASH_SR_PGERR ((uint8_t)0x04)    /*!< Programming Error */
#define FLASH_SR_WRPRTERR ((uint8_t)0x10) /*!< Write Protection Error */
#define FLASH_SR_EOP ((uint8_t)0x20)      /*!< End of operation */

/*******************  Bit definition for FLASH_CR register  *******************/
#define FLASH_CR_PG ((uint16_t)0x0001)     /*!< Programming */
#define FLASH_CR_PER ((uint16_t)0x0002)    /*!< Page Erase */
#define FLASH_CR_MER ((uint16_t)0x0004)    /*!< Mass Erase */
#define FLASH_CR_OPTPG ((uint16_t)0x0010)  /*!< Option Byte Programming */
#define FLASH_CR_OPTER ((uint16_t)0x0020)  /*!< Option Byte Erase */
#define FLASH_CR_STRT ((uint16_t)0x0040)   /*!< Start */
#define FLASH_CR_LOCK ((uint16_t)0x0080)   /*!< Lock */
#define FLASH_CR_OPTWRE ((uint16_t)0x0200) /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE ((uint16_t)0x0400)  /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE ((uint16_t)0x1000)  /*!< End of operation interrupt enable */

/*******************  Bit definition for FLASH_AR register  *******************/
#define FLASH_AR_FAR ((uint32_t)0xFFFFFFFF) /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define FLASH_OBR_OPTERR ((uint16_t)0x0001) /*!< Option Byte Error */
#define FLASH_OBR_RDPRT ((uint16_t)0x0002)  /*!< Read protection */

#define FLASH_OBR_USER ((uint16_t)0x03FC)       /*!< User Option Bytes */
#define FLASH_OBR_WDG_SW ((uint16_t)0x0004)     /*!< WDG_SW */
#define FLASH_OBR_nRST_STOP ((uint16_t)0x0008)  /*!< nRST_STOP */
#define FLASH_OBR_nRST_STDBY ((uint16_t)0x0010) /*!< nRST_STDBY */
#define FLASH_OBR_BFB2 ((uint16_t)0x0020)       /*!< BFB2 */

/******************  Bit definition for FLASH_WRPR register  ******************/
#define FLASH_WRPR_WRP ((uint32_t)0xFFFFFFFF) /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for FLASH_RDP register  *******************/
#define FLASH_RDP_RDP ((uint32_t)0x000000FF)  /*!< Read protection option byte */
#define FLASH_RDP_nRDP ((uint32_t)0x0000FF00) /*!< Read protection complemented option byte */

/******************  Bit definition for FLASH_USER register  ******************/
#define FLASH_USER_USER ((uint32_t)0x00FF0000)  /*!< User option byte */
#define FLASH_USER_nUSER ((uint32_t)0xFF000000) /*!< User complemented option byte */

/******************  Bit definition for FLASH_Data0 register  *****************/
#define FLASH_Data0_Data0 ((uint32_t)0x000000FF)  /*!< User data storage option byte */
#define FLASH_Data0_nData0 ((uint32_t)0x0000FF00) /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_Data1 register  *****************/
#define FLASH_Data1_Data1 ((uint32_t)0x00FF0000)  /*!< User data storage option byte */
#define FLASH_Data1_nData1 ((uint32_t)0xFF000000) /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_WRP0 register  ******************/
#define FLASH_WRP0_WRP0 ((uint32_t)0x000000FF)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP0_nWRP0 ((uint32_t)0x0000FF00) /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP1 register  ******************/
#define FLASH_WRP1_WRP1 ((uint32_t)0x00FF0000)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP1_nWRP1 ((uint32_t)0xFF000000) /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP2 register  ******************/
#define FLASH_WRP2_WRP2 ((uint32_t)0x000000FF)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP2_nWRP2 ((uint32_t)0x0000FF00) /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP3 register  ******************/
#define FLASH_WRP3_WRP3 ((uint32_t)0x00FF0000)  /*!< Flash memory write protection option bytes */
#define FLASH_WRP3_nWRP3 ((uint32_t)0xFF000000) /*!< Flash memory write protection complemented option bytes */
