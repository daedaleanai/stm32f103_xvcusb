/* Linker script for STM32F103CB Device with 128KByte FLASH, 20KByte RAM */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH =  20K
}

ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Core CM3 Peripherals */

ITM           = 0xE0000000;          /*!< ITM Base Address                  */
SCS_BASE      = 0xE000E000;          /*!< System Control Space Base Address */
InterruptType = SCS_BASE;        
SysTick       = SCS_BASE + 0x0010;   /*!< SysTick Base Address              */
NVIC          = SCS_BASE + 0x0100;   /*!< NVIC Base Address                 */
SCB           = SCS_BASE + 0x0D00;   /*!< System Control Block Base Address */
CoreDebug     = 0xE000EDF0;          /*!< Core Debug Base Address           */

/* STM32F103CB Peripherals */

PERIPH_BASE = 0x40000000; /*!< Peripheral base address in the alias region */

APB1PERIPH = PERIPH_BASE;
APB2PERIPH = PERIPH_BASE + 0x10000;
AHBPERIPH  = PERIPH_BASE + 0x20000;

ADC1   = APB2PERIPH + 0x2400;
ADC2   = APB2PERIPH + 0x2800;
AFIO   = APB2PERIPH + 0x0000;
BKP    = APB1PERIPH + 0x6C00;
CAN1   = APB1PERIPH + 0x6400;
CRC    = AHBPERIPH + 0x3000;
DMA1          = AHBPERIPH + 0x0000;
DMA1_Channel1 = AHBPERIPH + 0x0008;
DMA1_Channel2 = AHBPERIPH + 0x001C;
DMA1_Channel3 = AHBPERIPH + 0x0030;
DMA1_Channel4 = AHBPERIPH + 0x0044;
DMA1_Channel5 = AHBPERIPH + 0x0058;
DMA1_Channel6 = AHBPERIPH + 0x006C;
DMA1_Channel7 = AHBPERIPH + 0x0080;
EXTI   = APB2PERIPH + 0x0400;
FLASH_R = AHBPERIPH + 0x2000; /*!< Flash registers base address */
GPIOA  = APB2PERIPH + 0x0800;
GPIO_ALL = GPIOA;
GPIOB  = APB2PERIPH + 0x0C00;
GPIOC  = APB2PERIPH + 0x1000;
I2C1   = APB1PERIPH + 0x5400;
I2C2   = APB1PERIPH + 0x5800;
IWDG   = APB1PERIPH + 0x3000;
PWR    = APB1PERIPH + 0x7000;
RCC    = AHBPERIPH + 0x1000;
RTC    = APB1PERIPH + 0x2800;
SPI1   = APB2PERIPH + 0x3000;
SPI2   = APB1PERIPH + 0x3800;
TIM1   = APB2PERIPH + 0x2C00;
TIM2   = APB1PERIPH + 0x0000;
TIM3   = APB1PERIPH + 0x0400;
TIM4   = APB1PERIPH + 0x0800;
USART1 = APB2PERIPH + 0x3800;
USART2 = APB1PERIPH + 0x4400;
USART3 = APB1PERIPH + 0x4800;
USB    = APB1PERIPH + 0x5C00;
USB_PMA = APB1PERIPH + 0x6000;
WWDG   = APB1PERIPH + 0x2C00;


DBGMCU = 0xE0042000; /*!< Debug MCU registers base address */

OPTION_BYTES  = 0x1FFFF800; /*!< Flash Option Bytes base address */
UNIQUE_DEVICE_ID = 0x1FFFF7E8;

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH


   .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
    __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
    } >FLASH


  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH
  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >FLASH
  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = .;

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(SORT_BY_ALIGNMENT(.bss*))
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

