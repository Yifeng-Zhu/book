;******************** (C) Yifeng ZHU ******************************************************************
; @file    stm32f401xc_constant.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    May-17-2015
; @note    Modifed from stm32f407xx.h (C) 2010 STMicroelectronics
; @brief   Assembly version of Cortex M3 core
; @note
;          This code is for the book "Embedded Systems with ARM Cortex-M3 
;          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;          ISBN-10: 0982692625.
; @attension
;          This code is provided for education purpose. The author shall not be 
;          held liable for any direct, indirect or consequential damages, for any 
;          reason whatever. More information can be found from book website: 
;          http://www.eece.maine.edu/~zhu/book
;******************************************************************************************************

; This following is added to remove the compiler warning.
    AREA    __DEFINES_STM32F4_xx_DUMMY, CODE, READONLY
		
;******************************************************************************************************
; Configuration of the Cortex-M4 Processor and Core Peripherals
;******************************************************************************************************

;__CM4_REV                 0x0001  ;  Core revision r0p1
;__MPU_PRESENT             1       ;  STM32F4XX provides an MPU
;__NVIC_PRIO_BITS          4       ;  STM32F4XX uses 4 Bits for the Priority Levels
;__Vendor_SysTickConfig    0       ;  Set to 1 if different SysTick Config is used
;__FPU_PRESENT             1       ;  FPU present

;******************************************************************************************************
; Peripheral_interrupt_number_definition
;******************************************************************************************************

; STM32F4XX Interrupt Number Definition, according to the selected device

; Cortex-M4 Processor Exceptions Numbers **************************************************************
NonMaskableInt_IRQn         EQU -14     ;  2 Non Maskable Interrupt
MemoryManagement_IRQn       EQU -12     ;  4 Cortex-M4 Memory Management Interrupt
BusFault_IRQn               EQU -11     ;  5 Cortex-M4 Bus Fault Interrupt
UsageFault_IRQn             EQU -10     ;  6 Cortex-M4 Usage Fault Interrupt
SVCall_IRQn                 EQU -5      ;  11 Cortex-M4 SV Call Interrupt
DebugMonitor_IRQn           EQU -4      ;  12 Cortex-M4 Debug Monitor Interrupt
PendSV_IRQn                 EQU -2      ;  14 Cortex-M4 Pend SV Interrupt
SysTick_IRQn                EQU -1      ;  15 Cortex-M4 System Tick Interrupt
; STM32 specific Interrupt Numbers *********************************************************************
WWDG_IRQn                   EQU 0       ;  Window WatchDog Interrupt
PVD_IRQn                    EQU 1       ;  PVD through EXTI Line detection Interrupt
TAMP_STAMP_IRQn             EQU 2       ;  Tamper and TimeStamp interrupts through the EXTI line
RTC_WKUP_IRQn               EQU 3       ;  RTC Wakeup interrupt through the EXTI line
FLASH_IRQn                  EQU 4       ;  FLASH global Interrupt
RCC_IRQn                    EQU 5       ;  RCC global Interrupt
EXTI0_IRQn                  EQU 6       ;  EXTI Line0 Interrupt
EXTI1_IRQn                  EQU 7       ;  EXTI Line1 Interrupt
EXTI2_IRQn                  EQU 8       ;  EXTI Line2 Interrupt
EXTI3_IRQn                  EQU 9       ;  EXTI Line3 Interrupt
EXTI4_IRQn                  EQU 10      ;  EXTI Line4 Interrupt
DMA1_Stream0_IRQn           EQU 11      ;  DMA1 Stream 0 global Interrupt
DMA1_Stream1_IRQn           EQU 12      ;  DMA1 Stream 1 global Interrupt
DMA1_Stream2_IRQn           EQU 13      ;  DMA1 Stream 2 global Interrupt
DMA1_Stream3_IRQn           EQU 14      ;  DMA1 Stream 3 global Interrupt
DMA1_Stream4_IRQn           EQU 15      ;  DMA1 Stream 4 global Interrupt
DMA1_Stream5_IRQn           EQU 16      ;  DMA1 Stream 5 global Interrupt
DMA1_Stream6_IRQn           EQU 17      ;  DMA1 Stream 6 global Interrupt
ADC_IRQn                    EQU 18      ;  ADC1, ADC2 and ADC3 global Interrupts
EXTI9_5_IRQn                EQU 23      ;  External Line[9:5] Interrupts
TIM1_BRK_TIM9_IRQn          EQU 24      ;  TIM1 Break interrupt and TIM9 global interrupt
TIM1_UP_TIM10_IRQn          EQU 25      ;  TIM1 Update Interrupt and TIM10 global interrupt
TIM1_TRG_COM_TIM11_IRQn     EQU 26      ;  TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
TIM1_CC_IRQn                EQU 27      ;  TIM1 Capture Compare Interrupt
TIM2_IRQn                   EQU 28      ;  TIM2 global Interrupt
TIM3_IRQn                   EQU 29      ;  TIM3 global Interrupt
TIM4_IRQn                   EQU 30      ;  TIM4 global Interrupt
I2C1_EV_IRQn                EQU 31      ;  I2C1 Event Interrupt
I2C1_ER_IRQn                EQU 32      ;  I2C1 Error Interrupt
I2C2_EV_IRQn                EQU 33      ;  I2C2 Event Interrupt
I2C2_ER_IRQn                EQU 34      ;  I2C2 Error Interrupt
SPI1_IRQn                   EQU 35      ;  SPI1 global Interrupt
SPI2_IRQn                   EQU 36      ;  SPI2 global Interrupt
USART1_IRQn                 EQU 37      ;  USART1 global Interrupt
USART2_IRQn                 EQU 38      ;  USART2 global Interrupt
EXTI15_10_IRQn              EQU 40      ;  External Line[15:10] Interrupts
RTC_Alarm_IRQn              EQU 41      ;  RTC Alarm (A and B) through EXTI Line Interrupt
OTG_FS_WKUP_IRQn            EQU 42      ;  USB OTG FS Wakeup through EXTI line interrupt
DMA1_Stream7_IRQn           EQU 47      ;  DMA1 Stream7 Interrupt
SDIO_IRQn                   EQU 49      ;  SDIO global Interrupt
TIM5_IRQn                   EQU 50      ;  TIM5 global Interrupt
SPI3_IRQn                   EQU 51      ;  SPI3 global Interrupt
DMA2_Stream0_IRQn           EQU 56      ;  DMA2 Stream 0 global Interrupt
DMA2_Stream1_IRQn           EQU 57      ;  DMA2 Stream 1 global Interrupt
DMA2_Stream2_IRQn           EQU 58      ;  DMA2 Stream 2 global Interrupt
DMA2_Stream3_IRQn           EQU 59      ;  DMA2 Stream 3 global Interrupt
DMA2_Stream4_IRQn           EQU 60      ;  DMA2 Stream 4 global Interrupt
OTG_FS_IRQn                 EQU 67      ;  USB OTG FS global Interrupt
DMA2_Stream5_IRQn           EQU 68      ;  DMA2 Stream 5 global interrupt
DMA2_Stream6_IRQn           EQU 69      ;  DMA2 Stream 6 global interrupt
DMA2_Stream7_IRQn           EQU 70      ;  DMA2 Stream 7 global interrupt
USART6_IRQn                 EQU 71      ;  USART6 global interrupt
I2C3_EV_IRQn                EQU 72      ;  I2C3 event interrupt
I2C3_ER_IRQn                EQU 73      ;  I2C3 error interrupt
FPU_IRQn                    EQU 81      ;  FPU global interrupt
SPI4_IRQn                   EQU 84      ;  SPI4 global Interrupt



;*******************************************************************************************************************
; Peripheral_registers_structures
;*******************************************************************************************************************

; Analog to Digital Converter

ADC_ISR    EQU    0x00     ;  ADC status register,                         Address offset: 0x00
ADC_CR1    EQU    0x04     ;  ADC control register 1,                      Address offset: 0x04
ADC_CR2    EQU    0x08     ;  ADC control register 2,                      Address offset: 0x08
ADC_SMPR1  EQU    0x0C     ;  ADC sample time register 1,                  Address offset: 0x0C
ADC_SMPR2  EQU    0x10     ;  ADC sample time register 2,                  Address offset: 0x10
ADC_JOFR1  EQU    0x14     ;  ADC injected channel data offset register 1, Address offset: 0x14
ADC_JOFR2  EQU    0x18     ;  ADC injected channel data offset register 2, Address offset: 0x18
ADC_JOFR3  EQU    0x1C     ;  ADC injected channel data offset register 3, Address offset: 0x1C
ADC_JOFR4  EQU    0x20     ;  ADC injected channel data offset register 4, Address offset: 0x20
ADC_HTR    EQU    0x24     ;  ADC watchdog higher threshold register,      Address offset: 0x24
ADC_LTR    EQU    0x28     ;  ADC watchdog lower threshold register,       Address offset: 0x28
ADC_SQR1   EQU    0x2C     ;  ADC regular sequence register 1,             Address offset: 0x2C
ADC_SQR2   EQU    0x30     ;  ADC regular sequence register 2,             Address offset: 0x30
ADC_SQR3   EQU    0x34     ;  ADC regular sequence register 3,             Address offset: 0x34
ADC_JSQR   EQU    0x38     ;  ADC injected sequence register,              Address offset: 0x38
ADC_JDR1   EQU    0x3C     ;  ADC injected data register 1,                Address offset: 0x3C
ADC_JDR2   EQU    0x40     ;  ADC injected data register 2,                Address offset: 0x40
ADC_JDR3   EQU    0x44     ;  ADC injected data register 3,                Address offset: 0x44
ADC_JDR4   EQU    0x48     ;  ADC injected data register 4,                Address offset: 0x48
ADC_DR     EQU    0x4C     ;  ADC regular data register,                   Address offset: 0x4C

; ADC Common
ADC_CSR    EQU    0x300    ;  ADC Common status register,                  Address offset: ADC1 base address + 0x300
ADC_CCR    EQU    0x304    ;  ADC common control register,                 Address offset: ADC1 base address + 0x304
ADC_CDR    EQU    0x308    ;  ADC common regular data register for dual AND triple modes, Address offset: ADC1 base address + 0x308


; CRC calculation unit

CRC_DR         EQU    0x00    ;  CRC Data register,             Address offset: 0x00
CRC_IDR        EQU    0x04    ;  CRC Independent data register, Address offset: 0x04
CRC_RESERVED0  EQU    0x05    ;  Reserved, 0x05
CRC_RESERVED1  EQU    0x06    ;  Reserved, 0x06
CRC_CR         EQU    0x08    ;  CRC Control register,          Address offset: 0x08


; Debug MCU

DBGMCU_IDCODE  EQU    0x00    ;  MCU device ID code,               Address offset: 0x00
DBGMCU_CR      EQU    0x04    ;  Debug MCU configuration register, Address offset: 0x04
DBGMCU_APB1FZ  EQU    0x08    ;  Debug MCU APB1 freeze register,   Address offset: 0x08
DBGMCU_APB2FZ  EQU    0x0C    ;  Debug MCU APB2 freeze register,   Address offset: 0x0C


; DMA Controller

DMA_Stream_CR     EQU  0x00   ;  DMA stream x configuration register,      Address offset: 0x00
DMA_Stream_NDTR   EQU  0x04   ;  DMA stream x number of data register,     Address offset: 0x04
DMA_Stream_PAR    EQU  0x08   ;  DMA stream x peripheral address register, Address offset: 0x08
DMA_Stream_M0AR   EQU  0x0C   ;  DMA stream x memory 0 address register,   Address offset: 0x0C
DMA_Stream_M1AR   EQU  0x10   ;  DMA stream x memory 1 address register,   Address offset: 0x10
DMA_Stream_FCR    EQU  0x14   ;  DMA stream x FIFO control register,       Address offset: 0x14

DMA_LISR   EQU  0x00   ;  DMA low interrupt status register,      Address offset: 0x00
DMA_HISR   EQU  0x04   ;  DMA high interrupt status register,     Address offset: 0x04
DMA_LIFCR  EQU  0x08   ;  DMA low interrupt flag clear register,  Address offset: 0x08
DMA_HIFCR  EQU  0x0C   ;  DMA high interrupt flag clear register, Address offset: 0x0C



; External Interrupt/Event Controller

EXTI_IMR   EQU  0x00   ;  EXTI Interrupt mask register,            Address offset: 0x00
EXTI_EMR   EQU  0x04   ;  EXTI Event mask register,                Address offset: 0x04
EXTI_RTSR  EQU  0x08   ;  EXTI Rising trigger selection register,  Address offset: 0x08
EXTI_FTSR  EQU  0x0C   ;  EXTI Falling trigger selection register, Address offset: 0x0C
EXTI_SWIER EQU  0x10   ;  EXTI Software interrupt event register,  Address offset: 0x10
EXTI_PR    EQU  0x14   ;  EXTI Pending register,                   Address offset: 0x14


; FLASH Registers

FLASH_ACR      EQU  0x00   ;  FLASH access control register,   Address offset: 0x00
FLASH_KEYR     EQU  0x04   ;  FLASH key register,              Address offset: 0x04
FLASH_OPTKEYR  EQU  0x08   ;  FLASH option key register,       Address offset: 0x08
FLASH_SR       EQU  0x0C   ;  FLASH status register,           Address offset: 0x0C
FLASH_CR       EQU  0x10   ;  FLASH control register,          Address offset: 0x10
FLASH_OPTCR    EQU  0x14   ;  FLASH option control register ,  Address offset: 0x14
FLASH_OPTCR1   EQU  0x18   ;  FLASH option control register 1, Address offset: 0x18


; General Purpose I/O

GPIO_MODER     EQU  0x00   ;  GPIO port mode register,               Address offset: 0x00
GPIO_OTYPER    EQU  0x04   ;  GPIO port output type register,        Address offset: 0x04
GPIO_OSPEEDR   EQU  0x08   ;  GPIO port output speed register,       Address offset: 0x08
GPIO_PUPDR     EQU  0x0C   ;  GPIO port pull-up/pull-down register,  Address offset: 0x0C
GPIO_IDR       EQU  0x10   ;  GPIO port input data register,         Address offset: 0x10
GPIO_ODR       EQU  0x14   ;  GPIO port output data register,        Address offset: 0x14
GPIO_BSRRL     EQU  0x18   ;  GPIO port bit set/reset low register,  Address offset: 0x18
GPIO_BSRRH     EQU  0x1A   ;  GPIO port bit set/reset high register, Address offset: 0x1A
GPIO_LCKR      EQU  0x1C   ;  GPIO port configuration lock register, Address offset: 0x1C
GPIO_AFR0      EQU  0x20   ;  GPIO alternate function registers,     Address offset: 0x20-0x24
GPIO_AFR1      EQU  0x24   ;  GPIO alternate function registers,     Address offset: 0x20-0x24
GPIO_AFRL      EQU  0x20   ;  GPIO alternate function registers,     Address offset: 0x20-0x24
GPIO_AFRH      EQU  0x24   ;  GPIO alternate function registers,     Address offset: 0x20-0x24


; System configuration controller

SYSCFG_MEMRMP       EQU  0x00   ;  SYSCFG memory remap register,                      Address offset: 0x00
SYSCFG_PMC          EQU  0x04   ;  SYSCFG peripheral mode configuration register,     Address offset: 0x04
SYSCFG_EXTICR0      EQU  0x08   ;  SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR1      EQU  0x0C   ;  SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR2      EQU  0x10   ;  SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_EXTICR3      EQU  0x14   ;  SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
SYSCFG_RESERVED0    EQU  0x18   ;  Reserved, 0x18-0x1C
SYSCFG_RESERVED1    EQU  0x1C   ;  Reserved, 0x18-0x1C
SYSCFG_CMPCR        EQU  0x20   ;  SYSCFG Compensation cell control register,         Address offset: 0x20


; Inter-integrated Circuit Interface

I2C_CR1        EQU  0x00   ;  I2C Control register 1,     Address offset: 0x00
I2C_CR2        EQU  0x04   ;  I2C Control register 2,     Address offset: 0x04
I2C_OAR1       EQU  0x08   ;  I2C Own address register 1, Address offset: 0x08
I2C_OAR2       EQU  0x0C   ;  I2C Own address register 2, Address offset: 0x0C
I2C_DR         EQU  0x10   ;  I2C Data register,          Address offset: 0x10
I2C_SR1        EQU  0x14   ;  I2C Status register 1,      Address offset: 0x14
I2C_SR2        EQU  0x18   ;  I2C Status register 2,      Address offset: 0x18
I2C_CCR        EQU  0x1C   ;  I2C Clock control register, Address offset: 0x1C
I2C_TRISE      EQU  0x20   ;  I2C TRISE register,         Address offset: 0x20
I2C_FLTR       EQU  0x24   ;  I2C FLTR register,          Address offset: 0x24


; Independent WATCHDOG

IWDG_KR        EQU  0x00   ;  IWDG Key register,       Address offset: 0x00
IWDG_PR        EQU  0x04   ;  IWDG Prescaler register, Address offset: 0x04
IWDG_RLR       EQU  0x08   ;  IWDG Reload register,    Address offset: 0x08
IWDG_SR        EQU  0x0C   ;  IWDG Status register,    Address offset: 0x0C


; Power Control

PWR_CR         EQU  0x00   ;  PWR power control register,        Address offset: 0x00
PWR_CSR        EQU  0x04   ;  PWR power control/status register, Address offset: 0x04


; Reset and Clock Control

RCC_CR            EQU  0x00   ;  RCC clock control register,                                  Address offset: 0x00
RCC_PLLCFGR       EQU  0x04   ;  RCC PLL configuration register,                              Address offset: 0x04
RCC_CFGR          EQU  0x08   ;  RCC clock configuration register,                            Address offset: 0x08
RCC_CIR           EQU  0x0C   ;  RCC clock interrupt register,                                Address offset: 0x0C
RCC_AHB1RSTR      EQU  0x10   ;  RCC AHB1 peripheral reset register,                          Address offset: 0x10
RCC_AHB2RSTR      EQU  0x14   ;  RCC AHB2 peripheral reset register,                          Address offset: 0x14
RCC_AHB3RSTR      EQU  0x18   ;  RCC AHB3 peripheral reset register,                          Address offset: 0x18
RCC_RESERVED0     EQU  0x1C   ;  Reserved, 0x1C
RCC_APB1RSTR      EQU  0x20   ;  RCC APB1 peripheral reset register,                          Address offset: 0x20
RCC_APB2RSTR      EQU  0x24   ;  RCC APB2 peripheral reset register,                          Address offset: 0x24
RCC_RESERVED1_0   EQU  0x28   ;  Reserved, 0x28-0x2C
RCC_RESERVED1_1   EQU  0x2C   ;  Reserved, 0x28-0x2C
RCC_AHB1ENR       EQU  0x30   ;  RCC AHB1 peripheral clock register,                          Address offset: 0x30
RCC_AHB2ENR       EQU  0x34   ;  RCC AHB2 peripheral clock register,                          Address offset: 0x34
RCC_AHB3ENR       EQU  0x38   ;  RCC AHB3 peripheral clock register,                          Address offset: 0x38
RCC_RESERVED2     EQU  0x3C   ;  Reserved, 0x3C
RCC_APB1ENR       EQU  0x40   ;  RCC APB1 peripheral clock enable register,                   Address offset: 0x40
RCC_APB2ENR       EQU  0x44   ;  RCC APB2 peripheral clock enable register,                   Address offset: 0x44
RCC_RESERVED3_0   EQU  0x48   ;  Reserved, 0x48-0x4C
RCC_RESERVED3_1   EQU  0x4C   ;  Reserved, 0x48-0x4C
RCC_AHB1LPENR     EQU  0x50   ;  RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
RCC_AHB2LPENR     EQU  0x54   ;  RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
RCC_AHB3LPENR     EQU  0x58   ;  RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
RCC_RESERVED4     EQU  0x5C   ;  Reserved, 0x5C
RCC_APB1LPENR     EQU  0x60   ;  RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
RCC_APB2LPENR     EQU  0x64   ;  RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
RCC_RESERVED5_1   EQU  0x68   ;  Reserved, 0x68-0x6C
RCC_RESERVED5_2   EQU  0x6C   ;  Reserved, 0x68-0x6C
RCC_BDCR          EQU  0x70   ;  RCC Backup domain control register,                          Address offset: 0x70
RCC_CSR           EQU  0x74   ;  RCC clock control & status register,                         Address offset: 0x74
RCC_RESERVED6_0   EQU  0x78   ;  Reserved, 0x78-0x7C
RCC_RESERVED6_1   EQU  0x7C   ;  Reserved, 0x78-0x7C
RCC_SSCGR         EQU  0x80   ;  RCC spread spectrum clock generation register,               Address offset: 0x80
RCC_PLLI2SCFGR    EQU  0x84   ;  RCC PLLI2S configuration register,                           Address offset: 0x84


; Real-Time Clock

RTC_TR        EQU  0x00   ;  RTC time register,                                        Address offset: 0x00
RTC_DR        EQU  0x04   ;  RTC date register,                                        Address offset: 0x04
RTC_CR        EQU  0x08   ;  RTC control register,                                     Address offset: 0x08
RTC_ISR       EQU  0x0C   ;  RTC initialization and status register,                   Address offset: 0x0C
RTC_PRER      EQU  0x10   ;  RTC prescaler register,                                   Address offset: 0x10
RTC_WUTR      EQU  0x14   ;  RTC wakeup timer register,                                Address offset: 0x14
RTC_CALIBR    EQU  0x18   ;  RTC calibration register,                                 Address offset: 0x18
RTC_ALRMAR    EQU  0x1C   ;  RTC alarm A register,                                     Address offset: 0x1C
RTC_ALRMBR    EQU  0x20   ;  RTC alarm B register,                                     Address offset: 0x20
RTC_WPR       EQU  0x24   ;  RTC write protection register,                            Address offset: 0x24
RTC_SSR       EQU  0x28   ;  RTC sub second register,                                  Address offset: 0x28
RTC_SHIFTR    EQU  0x2C   ;  RTC shift control register,                               Address offset: 0x2C
RTC_TSTR      EQU  0x30   ;  RTC time stamp time register,                             Address offset: 0x30
RTC_TSDR      EQU  0x34   ;  RTC time stamp date register,                             Address offset: 0x34
RTC_TSSSR     EQU  0x38   ;  RTC time-stamp sub second register,                       Address offset: 0x38
RTC_CALR      EQU  0x3C   ;  RTC calibration register,                                 Address offset: 0x3C
RTC_TAFCR     EQU  0x40   ;  RTC tamper and alternate function configuration register, Address offset: 0x40
RTC_ALRMASSR  EQU  0x44   ;  RTC alarm A sub second register,                          Address offset: 0x44
RTC_ALRMBSSR  EQU  0x48   ;  RTC alarm B sub second register,                          Address offset: 0x48
RTC_RESERVED7 EQU  0x4C   ;  Reserved, 0x4C
RTC_BKP0R     EQU  0x50   ;  RTC backup register 1,                                    Address offset: 0x50
RTC_BKP1R     EQU  0x54   ;  RTC backup register 1,                                    Address offset: 0x54
RTC_BKP2R     EQU  0x58   ;  RTC backup register 2,                                    Address offset: 0x58
RTC_BKP3R     EQU  0x5C   ;  RTC backup register 3,                                    Address offset: 0x5C
RTC_BKP4R     EQU  0x60   ;  RTC backup register 4,                                    Address offset: 0x60
RTC_BKP5R     EQU  0x64   ;  RTC backup register 5,                                    Address offset: 0x64
RTC_BKP6R     EQU  0x68   ;  RTC backup register 6,                                    Address offset: 0x68
RTC_BKP7R     EQU  0x6C   ;  RTC backup register 7,                                    Address offset: 0x6C
RTC_BKP8R     EQU  0x70   ;  RTC backup register 8,                                    Address offset: 0x70
RTC_BKP9R     EQU  0x74   ;  RTC backup register 9,                                    Address offset: 0x74
RTC_BKP10R    EQU  0x78   ;  RTC backup register 10,                                   Address offset: 0x78
RTC_BKP11R    EQU  0x7C   ;  RTC backup register 11,                                   Address offset: 0x7C
RTC_BKP12R    EQU  0x80   ;  RTC backup register 12,                                   Address offset: 0x80
RTC_BKP13R    EQU  0x84   ;  RTC backup register 13,                                   Address offset: 0x84
RTC_BKP14R    EQU  0x88   ;  RTC backup register 14,                                   Address offset: 0x88
RTC_BKP15R    EQU  0x8C   ;  RTC backup register 15,                                   Address offset: 0x8C
RTC_BKP16R    EQU  0x90   ;  RTC backup register 16,                                   Address offset: 0x90
RTC_BKP17R    EQU  0x94   ;  RTC backup register 17,                                   Address offset: 0x94
RTC_BKP18R    EQU  0x98   ;  RTC backup register 18,                                   Address offset: 0x98
RTC_BKP19R    EQU  0x9C   ;  RTC backup register 19,                                   Address offset: 0x9C



; SD host Interface

SDIO_POWER          EQU  0x00   ;  SDIO power control register,    Address offset: 0x00
SDIO_CLKCR          EQU  0x04   ;  SDI clock control register,     Address offset: 0x04
SDIO_ARG            EQU  0x08   ;  SDIO argument register,         Address offset: 0x08
SDIO_CMD            EQU  0x0C   ;  SDIO command register,          Address offset: 0x0C
SDIO_RESPCMD        EQU  0x10   ;  SDIO command response register, Address offset: 0x10
SDIO_RESP1          EQU  0x14   ;  SDIO response 1 register,       Address offset: 0x14
SDIO_RESP2          EQU  0x18   ;  SDIO response 2 register,       Address offset: 0x18
SDIO_RESP3          EQU  0x1C   ;  SDIO response 3 register,       Address offset: 0x1C
SDIO_RESP4          EQU  0x20   ;  SDIO response 4 register,       Address offset: 0x20
SDIO_DTIMER         EQU  0x24   ;  SDIO data timer register,       Address offset: 0x24
SDIO_DLEN           EQU  0x28   ;  SDIO data length register,      Address offset: 0x28
SDIO_DCTRL          EQU  0x2C   ;  SDIO data control register,     Address offset: 0x2C
SDIO_DCOUNT         EQU  0x30   ;  SDIO data counter register,     Address offset: 0x30
SDIO_STA            EQU  0x34   ;  SDIO status register,           Address offset: 0x34
SDIO_ICR            EQU  0x38   ;  SDIO interrupt clear register,  Address offset: 0x38
SDIO_MASK           EQU  0x3C   ;  SDIO mask register,             Address offset: 0x3C
SDIO_RESERVED0_0    EQU  0x40   ;  Reserved, 0x40-0x44
SDIO_RESERVED0_1    EQU  0x44   ;  Reserved, 0x40-0x44
SDIO_FIFOCNT        EQU  0x48   ;  SDIO FIFO counter register,     Address offset: 0x48
SDIO_RESERVED1_13   EQU  0x4C   ;  Reserved, 0x4C-0x7C
SDIO_FIFO           EQU  0x80   ;  SDIO data FIFO register,        Address offset: 0x80


; Serial Peripheral Interface

SPI_TCR1        EQU  0x00   ;  SPI control register 1 (not used in I2S mode),      Address offset: 0x00
SPI_TCR2        EQU  0x04   ;  SPI control register 2,                             Address offset: 0x04
SPI_TSR         EQU  0x08   ;  SPI status register,                                Address offset: 0x08
SPI_TDR         EQU  0x0C   ;  SPI data register,                                  Address offset: 0x0C
SPI_TCRCPR      EQU  0x10   ;  SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10
SPI_TRXCRCR     EQU  0x14   ;  SPI RX CRC register (not used in I2S mode),         Address offset: 0x14
SPI_TTXCRCR     EQU  0x18   ;  SPI TX CRC register (not used in I2S mode),         Address offset: 0x18
SPI_TI2SCFGR    EQU  0x1C   ;  SPI_I2S configuration register,                     Address offset: 0x1C
SPI_TI2SPR      EQU  0x20   ;  SPI_I2S prescaler register,                         Address offset: 0x20


; TIM

TIM_CR1         EQU  0x00   ;  TIM control register 1,              Address offset: 0x00
TIM_CR2         EQU  0x04   ;  TIM control register 2,              Address offset: 0x04
TIM_SMCR        EQU  0x08   ;  TIM slave mode control register,     Address offset: 0x08
TIM_DIER        EQU  0x0C   ;  TIM DMA/interrupt enable register,   Address offset: 0x0C
TIM_SR          EQU  0x10   ;  TIM status register,                 Address offset: 0x10
TIM_EGR         EQU  0x14   ;  TIM event generation register,       Address offset: 0x14
TIM_CCMR1       EQU  0x18   ;  TIM capture/compare mode register 1, Address offset: 0x18
TIM_CCMR2       EQU  0x1C   ;  TIM capture/compare mode register 2, Address offset: 0x1C
TIM_CCER        EQU  0x20   ;  TIM capture/compare enable register, Address offset: 0x20
TIM_CNT         EQU  0x24   ;  TIM counter register,                Address offset: 0x24
TIM_PSC         EQU  0x28   ;  TIM prescaler,                       Address offset: 0x28
TIM_ARR         EQU  0x2C   ;  TIM auto-reload register,            Address offset: 0x2C
TIM_RCR         EQU  0x30   ;  TIM repetition counter register,     Address offset: 0x30
TIM_CCR1        EQU  0x34   ;  TIM capture/compare register 1,      Address offset: 0x34
TIM_CCR2        EQU  0x38   ;  TIM capture/compare register 2,      Address offset: 0x38
TIM_CCR3        EQU  0x3C   ;  TIM capture/compare register 3,      Address offset: 0x3C
TIM_CCR4        EQU  0x40   ;  TIM capture/compare register 4,      Address offset: 0x40
TIM_BDTR        EQU  0x44   ;  TIM break and dead-time register,    Address offset: 0x44
TIM_DCR         EQU  0x48   ;  TIM DMA control register,            Address offset: 0x48
TIM_DMAR        EQU  0x4C   ;  TIM DMA address for full transfer,   Address offset: 0x4C
TIM_OR          EQU  0x50   ;  TIM option register,                 Address offset: 0x50


; Universal Synchronous Asynchronous Receiver Transmitter

USART_SR        EQU  0x00   ;  USART Status register,                   Address offset: 0x00
USART_DR        EQU  0x04   ;  USART Data register,                     Address offset: 0x04
USART_BRR       EQU  0x08   ;  USART Baud rate register,                Address offset: 0x08
USART_CR1       EQU  0x0C   ;  USART Control register 1,                Address offset: 0x0C
USART_CR2       EQU  0x10   ;  USART Control register 2,                Address offset: 0x10
USART_CR3       EQU  0x14   ;  USART Control register 3,                Address offset: 0x14
USART_GTPR      EQU  0x18   ;  USART Guard time and prescaler register, Address offset: 0x18


; Window WATCHDOG

WWDG_CR    EQU  0x00   ;  WWDG Control register,       Address offset: 0x00
WWDG_CFR   EQU  0x04   ;  WWDG Configuration register, Address offset: 0x04
WWDG_SR    EQU  0x08   ;  WWDG Status register,        Address offset: 0x08


; __USB_OTG_Core_register

USB_OTG_Global_GOTGCTL              EQU  0x00   ;   USB_OTG Control and Status Register    Address offset : 0x00
USB_OTG_Global_GOTGINT              EQU  0x04   ;   USB_OTG Interrupt Register             Address offset : 0x04
USB_OTG_Global_GAHBCFG              EQU  0x08   ;   Core AHB Configuration Register        Address offset : 0x08
USB_OTG_Global_GUSBCFG              EQU  0x0C   ;   Core USB Configuration Register        Address offset : 0x0C
USB_OTG_Global_GRSTCTL              EQU  0x10   ;   Core Reset Register                    Address offset : 0x10
USB_OTG_Global_GINTSTS              EQU  0x14   ;   Core Interrupt Register                Address offset : 0x14
USB_OTG_Global_GINTMSK              EQU  0x18   ;   Core Interrupt Mask Register           Address offset : 0x18
USB_OTG_Global_GRXSTSR              EQU  0x1C   ;   Receive Sts Q Read Register            Address offset : 0x1C
USB_OTG_Global_GRXSTSP              EQU  0x20   ;   Receive Sts Q Read & POP Register      Address offset : 0x20
USB_OTG_Global_GRXFSIZ              EQU  0x24   ;   Receive FIFO Size Register             Address offset : 0x24
USB_OTG_Global_DIEPTXF0_HNPTXFSIZ   EQU  0x28   ;   EP0 / Non Periodic Tx FIFO Size Register Address offset : 0x28
USB_OTG_Global_HNPTXSTS             EQU  0x2C   ;   Non Periodic Tx FIFO/Queue Sts reg     Address offset : 0x2C
USB_OTG_Global_Reserved30_0         EQU  0x30   ;   Reserved                               Address offset : 0x30
USB_OTG_Global_Reserved30_1         EQU  0x34   ;   Reserved                               Address offset : 0x34
USB_OTG_Global_GCCFG                EQU  0x38   ;   General Purpose IO Register            Address offset : 0x38
USB_OTG_Global_CID                  EQU  0x3C   ;   User ID Register                       Address offset : 0x3C
USB_OTG_Global_Reserved40_48        EQU  0x40   ;   Reserved                               Address offset : 0x40-0xFF
USB_OTG_Global_HPTXFSIZ             EQU  0x100  ;   Host Periodic Tx FIFO Size Reg         Address offset : 0x100
USB_OTG_Global_DIEPTXF0             EQU  0x104  ;   dev Periodic Transmit FIFO


; __device_Registers

USB_OTG_Device_DCFG         EQU  0x800   ;  dev Configuration Register   Address offset : 0x800
USB_OTG_Device_DCTL         EQU  0x804   ;  dev Control Register         Address offset : 0x804
USB_OTG_Device_DSTS         EQU  0x808   ;  dev Status Register (RO)     Address offset : 0x808
USB_OTG_Device_Reserved0C   EQU  0x80C   ;  Reserved                     Address offset : 0x80C
USB_OTG_Device_DIEPMSK      EQU  0x810   ;  !< dev IN Endpoint Mask        Address offset : 0x810
USB_OTG_Device_DOEPMSK      EQU  0x814   ;  dev OUT Endpoint Mask        Address offset : 0x814
USB_OTG_Device_DAINT        EQU  0x818   ;  dev All Endpoints Itr Reg    Address offset : 0x818
USB_OTG_Device_DAINTMSK     EQU  0x81C   ;  dev All Endpoints Itr Mask   Address offset : 0x81C
USB_OTG_Device_Reserved20   EQU  0x820   ;  Reserved                     Address offset : 0x820
USB_OTG_Device_Reserved9    EQU  0x824   ;  Reserved                     Address offset : 0x824
USB_OTG_Device_DVBUSDIS     EQU  0x828   ;  dev VBUS discharge Register  Address offset : 0x828
USB_OTG_Device_DVBUSPULSE   EQU  0x82C   ;  dev VBUS Pulse Register      Address offset : 0x82C
USB_OTG_Device_DTHRCTL      EQU  0x830   ;  dev thr                      Address offset : 0x830
USB_OTG_Device_DIEPEMPMSK   EQU  0x834   ;  dev empty msk                Address offset : 0x834
USB_OTG_Device_DEACHINT     EQU  0x838   ;  dedicated EP interrupt       Address offset : 0x838
USB_OTG_Device_DEACHMSK     EQU  0x83C   ;  dedicated EP msk             Address offset : 0x83C
USB_OTG_Device_Reserved40   EQU  0x840   ;  dedicated EP mask            Address offset : 0x840
USB_OTG_Device_DINEP1MSK    EQU  0x844   ;  dedicated EP mask            Address offset : 0x844
USB_OTG_Device_Reserved44_0 EQU  0x848   ;  Reserved                     Address offset : 0x844-0x87C
USB_OTG_Device_DOUTEP1MSK   EQU  0x884   ;  dedicated EP msk             Address offset : 0x884




; __IN_Endpoint-Specific_Register

USB_OTG_INEndpoint_DIEPCTL        EQU  0x900   ;  dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h
USB_OTG_INEndpoint_Reserved04     EQU  0x904   ;  Reserved                       900h + (ep_num * 20h) + 04h
USB_OTG_INEndpoint_DIEPINT        EQU  0x908   ;  dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h
USB_OTG_INEndpoint_Reserved0C     EQU  0x90C   ;  Reserved                       900h + (ep_num * 20h) + 0Ch
USB_OTG_INEndpoint_DIEPTSIZ       EQU  0x910   ;  IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h
USB_OTG_INEndpoint_DIEPDMA        EQU  0x914   ;  IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h
USB_OTG_INEndpoint_DTXFSTS        EQU  0x918   ;  IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h
USB_OTG_INEndpoint_Reserved18     EQU  0x91C   ;  Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch


; __OUT_Endpoint-Specific_Registers

USB_OTG_OUTEndpoint_DOEPCTL       EQU  0xB00   ;  dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h
USB_OTG_OUTEndpoint_Reserved04    EQU  0xB04   ;  Reserved                      B00h + (ep_num * 20h) + 04h
USB_OTG_OUTEndpoint_DOEPINT       EQU  0xB08   ;  dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h
USB_OTG_OUTEndpoint_Reserved0C    EQU  0xB0C   ;  Reserved                      B00h + (ep_num * 20h) + 0Ch
USB_OTG_OUTEndpoint_DOEPTSIZ      EQU  0xB10   ;  dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h
USB_OTG_OUTEndpoint_DOEPDMA       EQU  0xB14   ;  dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h
USB_OTG_OUTEndpoint_Reserved18    EQU  0xB18   ;  Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch


; __Host_Mode_Register_Structures

USB_OTG_Host_HCFG;             EQU  0x400   ;  Host Configuration Register    400h
USB_OTG_Host_HFIR;             EQU  0x404   ;  Host Frame Interval Register   404h
USB_OTG_Host_HFNUM;            EQU  0x408   ;  Host Frame Nbr/Frame Remaining 408h
USB_OTG_Host_Reserved40C;      EQU  0x40C   ;  Reserved                       40Ch
USB_OTG_Host_HPTXSTS;          EQU  0x410   ;  Host Periodic Tx FIFO/ Queue Status 410h
USB_OTG_Host_HAINT;            EQU  0x414   ;  Host All Channels Interrupt Register 414h
USB_OTG_Host_HAINTMSK;         EQU  0x418   ;  Host All Channels Interrupt Mask 418h



; __Host_Channel_Specific_Registers

USB_OTG_HostChannel_HCCHAR      EQU  0x00
USB_OTG_HostChannel_HCSPLT      EQU  0x00
USB_OTG_HostChannel_HCINT       EQU  0x00
USB_OTG_HostChannel_HCINTMSK    EQU  0x00
USB_OTG_HostChannel_HCTSIZ      EQU  0x00
USB_OTG_HostChannel_HCDMA       EQU  0x00
USB_OTG_HostChannel_Reserved_0  EQU  0x00
USB_OTG_HostChannel_Reserved_1  EQU  0x00





; Peripheral_memory_map

FLASH_BASE            EQU    (0x08000000) ;  FLASH(up to 1 MB) base address in the alias region
CCMDATARAM_BASE       EQU    (0x10000000) ;  CCM(core coupled memory) data RAM(64 KB) base address in the alias region
SRAM1_BASE            EQU    (0x20000000) ;  SRAM1(112 KB) base address in the alias region
SRAM2_BASE            EQU    (0x2001C000) ;  SRAM2(16 KB) base address in the alias region
SRAM3_BASE            EQU    (0x20020000) ;  SRAM3(64 KB) base address in the alias region
PERIPH_BASE           EQU    (0x40000000) ;  Peripheral base address in the alias region
BKPSRAM_BASE          EQU    (0x40024000) ;  Backup SRAM(4 KB) base address in the alias region
CCMDATARAM_BB_BASE    EQU    (0x12000000) ;  CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region
SRAM1_BB_BASE         EQU    (0x22000000) ;  SRAM1(112 KB) base address in the bit-band region
SRAM2_BB_BASE         EQU    (0x2201C000) ;  SRAM2(16 KB) base address in the bit-band region
SRAM3_BB_BASE         EQU    (0x22020000) ;  SRAM3(64 KB) base address in the bit-band region
PERIPH_BB_BASE        EQU    (0x42000000) ;  Peripheral base address in the bit-band region
BKPSRAM_BB_BASE       EQU    (0x42024000) ;  Backup SRAM(4 KB) base address in the bit-band region
FLASH_END             EQU    (0x0803FFFF) ;  FLASH end address

;  Legacy defines
SRAM_BASE             EQU    (SRAM1_BASE)
SRAM_BB_BASE          EQU    (SRAM1_BB_BASE)


;  Peripheral memory map
APB1PERIPH_BASE       EQU    PERIPH_BASE
APB2PERIPH_BASE       EQU    (PERIPH_BASE + 0x00010000)
AHB1PERIPH_BASE       EQU    (PERIPH_BASE + 0x00020000)
AHB2PERIPH_BASE       EQU    (PERIPH_BASE + 0x10000000)

;  APB1 peripherals
TIM2_BASE             EQU    (APB1PERIPH_BASE + 0x0000)
TIM3_BASE             EQU    (APB1PERIPH_BASE + 0x0400)
TIM4_BASE             EQU    (APB1PERIPH_BASE + 0x0800)
TIM5_BASE             EQU    (APB1PERIPH_BASE + 0x0C00)
RTC_BASE              EQU    (APB1PERIPH_BASE + 0x2800)
WWDG_BASE             EQU    (APB1PERIPH_BASE + 0x2C00)
IWDG_BASE             EQU    (APB1PERIPH_BASE + 0x3000)
I2S2ext_BASE          EQU    (APB1PERIPH_BASE + 0x3400)
SPI2_BASE             EQU    (APB1PERIPH_BASE + 0x3800)
SPI3_BASE             EQU    (APB1PERIPH_BASE + 0x3C00)
I2S3ext_BASE          EQU    (APB1PERIPH_BASE + 0x4000)
USART2_BASE           EQU    (APB1PERIPH_BASE + 0x4400)
I2C1_BASE             EQU    (APB1PERIPH_BASE + 0x5400)
I2C2_BASE             EQU    (APB1PERIPH_BASE + 0x5800)
I2C3_BASE             EQU    (APB1PERIPH_BASE + 0x5C00)
PWR_BASE              EQU    (APB1PERIPH_BASE + 0x7000)

;  APB2 peripherals
TIM1_BASE             EQU    (APB2PERIPH_BASE + 0x0000)
USART1_BASE           EQU    (APB2PERIPH_BASE + 0x1000)
USART6_BASE           EQU    (APB2PERIPH_BASE + 0x1400)
ADC1_BASE             EQU    (APB2PERIPH_BASE + 0x2000)
ADC_BASE              EQU    (APB2PERIPH_BASE + 0x2300)
SDIO_BASE             EQU    (APB2PERIPH_BASE + 0x2C00)
SPI1_BASE             EQU    (APB2PERIPH_BASE + 0x3000)
SPI4_BASE             EQU    (APB2PERIPH_BASE + 0x3400)
SYSCFG_BASE           EQU    (APB2PERIPH_BASE + 0x3800)
EXTI_BASE             EQU    (APB2PERIPH_BASE + 0x3C00)
TIM9_BASE             EQU    (APB2PERIPH_BASE + 0x4000)
TIM10_BASE            EQU    (APB2PERIPH_BASE + 0x4400)
TIM11_BASE            EQU    (APB2PERIPH_BASE + 0x4800)

;  AHB1 peripherals
GPIOA_BASE            EQU    (AHB1PERIPH_BASE + 0x0000)
GPIOB_BASE            EQU    (AHB1PERIPH_BASE + 0x0400)
GPIOC_BASE            EQU    (AHB1PERIPH_BASE + 0x0800)
GPIOD_BASE            EQU    (AHB1PERIPH_BASE + 0x0C00)
GPIOE_BASE            EQU    (AHB1PERIPH_BASE + 0x1000)
GPIOH_BASE            EQU    (AHB1PERIPH_BASE + 0x1C00)
CRC_BASE              EQU    (AHB1PERIPH_BASE + 0x3000)
RCC_BASE              EQU    (AHB1PERIPH_BASE + 0x3800)
FLASH_R_BASE          EQU    (AHB1PERIPH_BASE + 0x3C00)
DMA1_BASE             EQU    (AHB1PERIPH_BASE + 0x6000)
DMA1_Stream0_BASE     EQU    (DMA1_BASE + 0x010)
DMA1_Stream1_BASE     EQU    (DMA1_BASE + 0x028)
DMA1_Stream2_BASE     EQU    (DMA1_BASE + 0x040)
DMA1_Stream3_BASE     EQU    (DMA1_BASE + 0x058)
DMA1_Stream4_BASE     EQU    (DMA1_BASE + 0x070)
DMA1_Stream5_BASE     EQU    (DMA1_BASE + 0x088)
DMA1_Stream6_BASE     EQU    (DMA1_BASE + 0x0A0)
DMA1_Stream7_BASE     EQU    (DMA1_BASE + 0x0B8)
DMA2_BASE             EQU    (AHB1PERIPH_BASE + 0x6400)
DMA2_Stream0_BASE     EQU    (DMA2_BASE + 0x010)
DMA2_Stream1_BASE     EQU    (DMA2_BASE + 0x028)
DMA2_Stream2_BASE     EQU    (DMA2_BASE + 0x040)
DMA2_Stream3_BASE     EQU    (DMA2_BASE + 0x058)
DMA2_Stream4_BASE     EQU    (DMA2_BASE + 0x070)
DMA2_Stream5_BASE     EQU    (DMA2_BASE + 0x088)
DMA2_Stream6_BASE     EQU    (DMA2_BASE + 0x0A0)
DMA2_Stream7_BASE     EQU    (DMA2_BASE + 0x0B8)

;  Debug MCU registers base address
DBGMCU_BASE           EQU    (0xE0042000)

;  USB registers base address
USB_OTG_FS_PERIPH_BASE               EQU    (0x50000000)

USB_OTG_GLOBAL_BASE                  EQU    (0x000)
USB_OTG_DEVICE_BASE                  EQU    (0x800)
USB_OTG_IN_ENDPOINT_BASE             EQU    (0x900)
USB_OTG_OUT_ENDPOINT_BASE            EQU    (0xB00)
USB_OTG_EP_REG_SIZE                  EQU    (0x20)
USB_OTG_HOST_BASE                    EQU    (0x400)
USB_OTG_HOST_PORT_BASE               EQU    (0x440)
USB_OTG_HOST_CHANNEL_BASE            EQU    (0x500)
USB_OTG_HOST_CHANNEL_SIZE            EQU    (0x20)
USB_OTG_PCGCCTL_BASE                 EQU    (0xE00)
USB_OTG_FIFO_BASE                    EQU    (0x1000)
USB_OTG_FIFO_SIZE                    EQU    (0x1000)





; Exported_constants

; Peripheral_Registers_Bits_Definition



;******************************************************************************
;*                         Peripheral Registers_Bits_Definition
;******************************************************************************

;******************************************************************************
;*
;*                        Analog to Digital Converter
;*
;******************************************************************************
;********************  Bit definition for ADC_SR register  ********************
ADC_SR_AWD                          EQU    (0x00000001)       ; Analog watchdog flag
ADC_SR_EOC                          EQU    (0x00000002)       ; End of conversion
ADC_SR_JEOC                         EQU    (0x00000004)       ; Injected channel end of conversion
ADC_SR_JSTRT                        EQU    (0x00000008)       ; Injected channel Start flag
ADC_SR_STRT                         EQU    (0x00000010)       ; Regular channel Start flag
ADC_SR_OVR                          EQU    (0x00000020)       ; Overrun flag

;*******************  Bit definition for ADC_CR1 register  ********************
ADC_CR1_AWDCH                       EQU    (0x0000001F)        ; AWDCH[4:0] bits (Analog watchdog channel select bits)
ADC_CR1_AWDCH_0                     EQU    (0x00000001)        ; Bit 0
ADC_CR1_AWDCH_1                     EQU    (0x00000002)        ; Bit 1
ADC_CR1_AWDCH_2                     EQU    (0x00000004)        ; Bit 2
ADC_CR1_AWDCH_3                     EQU    (0x00000008)        ; Bit 3
ADC_CR1_AWDCH_4                     EQU    (0x00000010)        ; Bit 4
ADC_CR1_EOCIE                       EQU    (0x00000020)        ; Interrupt enable for EOC
ADC_CR1_AWDIE                       EQU    (0x00000040)        ; AAnalog Watchdog interrupt enable
ADC_CR1_JEOCIE                      EQU    (0x00000080)        ; Interrupt enable for injected channels
ADC_CR1_SCAN                        EQU    (0x00000100)        ; Scan mode
ADC_CR1_AWDSGL                      EQU    (0x00000200)        ; Enable the watchdog on a single channel in scan mode
ADC_CR1_JAUTO                       EQU    (0x00000400)        ; Automatic injected group conversion
ADC_CR1_DISCEN                      EQU    (0x00000800)        ; Discontinuous mode on regular channels
ADC_CR1_JDISCEN                     EQU    (0x00001000)        ; Discontinuous mode on injected channels
ADC_CR1_DISCNUM                     EQU    (0x0000E000)        ; DISCNUM[2:0] bits (Discontinuous mode channel count)
ADC_CR1_DISCNUM_0                   EQU    (0x00002000)        ; Bit 0
ADC_CR1_DISCNUM_1                   EQU    (0x00004000)        ; Bit 1
ADC_CR1_DISCNUM_2                   EQU    (0x00008000)        ; Bit 2
ADC_CR1_JAWDEN                      EQU    (0x00400000)        ; Analog watchdog enable on injected channels
ADC_CR1_AWDEN                       EQU    (0x00800000)        ; Analog watchdog enable on regular channels
ADC_CR1_RES                         EQU    (0x03000000)        ; RES[2:0] bits (Resolution)
ADC_CR1_RES_0                       EQU    (0x01000000)        ; Bit 0
ADC_CR1_RES_1                       EQU    (0x02000000)        ; Bit 1
ADC_CR1_OVRIE                       EQU    (0x04000000)         ; overrun interrupt enable

;*******************  Bit definition for ADC_CR2 register  ********************
ADC_CR2_ADON                        EQU    (0x00000001)        ; A/D Converter ON / OFF
ADC_CR2_CONT                        EQU    (0x00000002)        ; Continuous Conversion
ADC_CR2_DMA                         EQU    (0x00000100)        ; Direct Memory access mode
ADC_CR2_DDS                         EQU    (0x00000200)        ; DMA disable selection (Single ADC)
ADC_CR2_EOCS                        EQU    (0x00000400)        ; End of conversion selection
ADC_CR2_ALIGN                       EQU    (0x00000800)        ; Data Alignment
ADC_CR2_JEXTSEL                     EQU    (0x000F0000)        ; JEXTSEL[3:0] bits (External event select for injected group)
ADC_CR2_JEXTSEL_0                   EQU    (0x00010000)        ; Bit 0
ADC_CR2_JEXTSEL_1                   EQU    (0x00020000)        ; Bit 1
ADC_CR2_JEXTSEL_2                   EQU    (0x00040000)        ; Bit 2
ADC_CR2_JEXTSEL_3                   EQU    (0x00080000)        ; Bit 3
ADC_CR2_JEXTEN                      EQU    (0x00300000)        ; JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp)
ADC_CR2_JEXTEN_0                    EQU    (0x00100000)        ; Bit 0
ADC_CR2_JEXTEN_1                    EQU    (0x00200000)        ; Bit 1
ADC_CR2_JSWSTART                    EQU    (0x00400000)        ; Start Conversion of injected channels
ADC_CR2_EXTSEL                      EQU    (0x0F000000)        ; EXTSEL[3:0] bits (External Event Select for regular group)
ADC_CR2_EXTSEL_0                    EQU    (0x01000000)        ; Bit 0
ADC_CR2_EXTSEL_1                    EQU    (0x02000000)        ; Bit 1
ADC_CR2_EXTSEL_2                    EQU    (0x04000000)        ; Bit 2
ADC_CR2_EXTSEL_3                    EQU    (0x08000000)        ; Bit 3
ADC_CR2_EXTEN                       EQU    (0x30000000)        ; EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp)
ADC_CR2_EXTEN_0                     EQU    (0x10000000)        ; Bit 0
ADC_CR2_EXTEN_1                     EQU    (0x20000000)        ; Bit 1
ADC_CR2_SWSTART                     EQU    (0x40000000)        ; Start Conversion of regular channels

;******************  Bit definition for ADC_SMPR1 register  *******************
ADC_SMPR1_SMP10                     EQU    (0x00000007)        ; SMP10[2:0] bits (Channel 10 Sample time selection)
ADC_SMPR1_SMP10_0                   EQU    (0x00000001)        ; Bit 0
ADC_SMPR1_SMP10_1                   EQU    (0x00000002)        ; Bit 1
ADC_SMPR1_SMP10_2                   EQU    (0x00000004)        ; Bit 2
ADC_SMPR1_SMP11                     EQU    (0x00000038)        ; SMP11[2:0] bits (Channel 11 Sample time selection)
ADC_SMPR1_SMP11_0                   EQU    (0x00000008)        ; Bit 0
ADC_SMPR1_SMP11_1                   EQU    (0x00000010)        ; Bit 1
ADC_SMPR1_SMP11_2                   EQU    (0x00000020)        ; Bit 2
ADC_SMPR1_SMP12                     EQU    (0x000001C0)        ; SMP12[2:0] bits (Channel 12 Sample time selection)
ADC_SMPR1_SMP12_0                   EQU    (0x00000040)        ; Bit 0
ADC_SMPR1_SMP12_1                   EQU    (0x00000080)        ; Bit 1
ADC_SMPR1_SMP12_2                   EQU    (0x00000100)        ; Bit 2
ADC_SMPR1_SMP13                     EQU    (0x00000E00)        ; SMP13[2:0] bits (Channel 13 Sample time selection)
ADC_SMPR1_SMP13_0                   EQU    (0x00000200)        ; Bit 0
ADC_SMPR1_SMP13_1                   EQU    (0x00000400)        ; Bit 1
ADC_SMPR1_SMP13_2                   EQU    (0x00000800)        ; Bit 2
ADC_SMPR1_SMP14                     EQU    (0x00007000)        ; SMP14[2:0] bits (Channel 14 Sample time selection)
ADC_SMPR1_SMP14_0                   EQU    (0x00001000)        ; Bit 0
ADC_SMPR1_SMP14_1                   EQU    (0x00002000)        ; Bit 1
ADC_SMPR1_SMP14_2                   EQU    (0x00004000)        ; Bit 2
ADC_SMPR1_SMP15                     EQU    (0x00038000)        ; SMP15[2:0] bits (Channel 15 Sample time selection)
ADC_SMPR1_SMP15_0                   EQU    (0x00008000)        ; Bit 0
ADC_SMPR1_SMP15_1                   EQU    (0x00010000)        ; Bit 1
ADC_SMPR1_SMP15_2                   EQU    (0x00020000)        ; Bit 2
ADC_SMPR1_SMP16                     EQU    (0x001C0000)        ; SMP16[2:0] bits (Channel 16 Sample time selection)
ADC_SMPR1_SMP16_0                   EQU    (0x00040000)        ; Bit 0
ADC_SMPR1_SMP16_1                   EQU    (0x00080000)        ; Bit 1
ADC_SMPR1_SMP16_2                   EQU    (0x00100000)        ; Bit 2
ADC_SMPR1_SMP17                     EQU    (0x00E00000)        ; SMP17[2:0] bits (Channel 17 Sample time selection)
ADC_SMPR1_SMP17_0                   EQU    (0x00200000)        ; Bit 0
ADC_SMPR1_SMP17_1                   EQU    (0x00400000)        ; Bit 1
ADC_SMPR1_SMP17_2                   EQU    (0x00800000)        ; Bit 2
ADC_SMPR1_SMP18                     EQU    (0x07000000)        ; SMP18[2:0] bits (Channel 18 Sample time selection)
ADC_SMPR1_SMP18_0                   EQU    (0x01000000)        ; Bit 0
ADC_SMPR1_SMP18_1                   EQU    (0x02000000)        ; Bit 1
ADC_SMPR1_SMP18_2                   EQU    (0x04000000)        ; Bit 2

;******************  Bit definition for ADC_SMPR2 register  *******************
ADC_SMPR2_SMP0                      EQU    (0x00000007)        ; SMP0[2:0] bits (Channel 0 Sample time selection)
ADC_SMPR2_SMP0_0                    EQU    (0x00000001)        ; Bit 0
ADC_SMPR2_SMP0_1                    EQU    (0x00000002)        ; Bit 1
ADC_SMPR2_SMP0_2                    EQU    (0x00000004)        ; Bit 2
ADC_SMPR2_SMP1                      EQU    (0x00000038)        ; SMP1[2:0] bits (Channel 1 Sample time selection)
ADC_SMPR2_SMP1_0                    EQU    (0x00000008)        ; Bit 0
ADC_SMPR2_SMP1_1                    EQU    (0x00000010)        ; Bit 1
ADC_SMPR2_SMP1_2                    EQU    (0x00000020)        ; Bit 2
ADC_SMPR2_SMP2                      EQU    (0x000001C0)        ; SMP2[2:0] bits (Channel 2 Sample time selection)
ADC_SMPR2_SMP2_0                    EQU    (0x00000040)        ; Bit 0
ADC_SMPR2_SMP2_1                    EQU    (0x00000080)        ; Bit 1
ADC_SMPR2_SMP2_2                    EQU    (0x00000100)        ; Bit 2
ADC_SMPR2_SMP3                      EQU    (0x00000E00)        ; SMP3[2:0] bits (Channel 3 Sample time selection)
ADC_SMPR2_SMP3_0                    EQU    (0x00000200)        ; Bit 0
ADC_SMPR2_SMP3_1                    EQU    (0x00000400)        ; Bit 1
ADC_SMPR2_SMP3_2                    EQU    (0x00000800)        ; Bit 2
ADC_SMPR2_SMP4                      EQU    (0x00007000)        ; SMP4[2:0] bits (Channel 4 Sample time selection)
ADC_SMPR2_SMP4_0                    EQU    (0x00001000)        ; Bit 0
ADC_SMPR2_SMP4_1                    EQU    (0x00002000)        ; Bit 1
ADC_SMPR2_SMP4_2                    EQU    (0x00004000)        ; Bit 2
ADC_SMPR2_SMP5                      EQU    (0x00038000)        ; SMP5[2:0] bits (Channel 5 Sample time selection)
ADC_SMPR2_SMP5_0                    EQU    (0x00008000)        ; Bit 0
ADC_SMPR2_SMP5_1                    EQU    (0x00010000)        ; Bit 1
ADC_SMPR2_SMP5_2                    EQU    (0x00020000)        ; Bit 2
ADC_SMPR2_SMP6                      EQU    (0x001C0000)        ; SMP6[2:0] bits (Channel 6 Sample time selection)
ADC_SMPR2_SMP6_0                    EQU    (0x00040000)        ; Bit 0
ADC_SMPR2_SMP6_1                    EQU    (0x00080000)        ; Bit 1
ADC_SMPR2_SMP6_2                    EQU    (0x00100000)        ; Bit 2
ADC_SMPR2_SMP7                      EQU    (0x00E00000)        ; SMP7[2:0] bits (Channel 7 Sample time selection)
ADC_SMPR2_SMP7_0                    EQU    (0x00200000)        ; Bit 0
ADC_SMPR2_SMP7_1                    EQU    (0x00400000)        ; Bit 1
ADC_SMPR2_SMP7_2                    EQU    (0x00800000)        ; Bit 2
ADC_SMPR2_SMP8                      EQU    (0x07000000)        ; SMP8[2:0] bits (Channel 8 Sample time selection)
ADC_SMPR2_SMP8_0                    EQU    (0x01000000)        ; Bit 0
ADC_SMPR2_SMP8_1                    EQU    (0x02000000)        ; Bit 1
ADC_SMPR2_SMP8_2                    EQU    (0x04000000)        ; Bit 2
ADC_SMPR2_SMP9                      EQU    (0x38000000)        ; SMP9[2:0] bits (Channel 9 Sample time selection)
ADC_SMPR2_SMP9_0                    EQU    (0x08000000)        ; Bit 0
ADC_SMPR2_SMP9_1                    EQU    (0x10000000)        ; Bit 1
ADC_SMPR2_SMP9_2                    EQU    (0x20000000)        ; Bit 2

;******************  Bit definition for ADC_JOFR1 register  *******************
ADC_JOFR1_JOFFSET1                  EQU    (0x0FFF)            ; Data offset for injected channel 1

;******************  Bit definition for ADC_JOFR2 register  *******************
ADC_JOFR2_JOFFSET2                  EQU    (0x0FFF)            ; Data offset for injected channel 2

;******************  Bit definition for ADC_JOFR3 register  *******************
ADC_JOFR3_JOFFSET3                  EQU    (0x0FFF)            ; Data offset for injected channel 3

;******************  Bit definition for ADC_JOFR4 register  *******************
ADC_JOFR4_JOFFSET4                  EQU    (0x0FFF)            ; Data offset for injected channel 4

;*******************  Bit definition for ADC_HTR register  ********************
ADC_HTR_HT                          EQU    (0x0FFF)            ; Analog watchdog high threshold

;*******************  Bit definition for ADC_LTR register  ********************
ADC_LTR_LT                          EQU    (0x0FFF)            ; Analog watchdog low threshold

;*******************  Bit definition for ADC_SQR1 register  *******************
ADC_SQR1_SQ13                       EQU    (0x0000001F)        ; SQ13[4:0] bits (13th conversion in regular sequence)
ADC_SQR1_SQ13_0                     EQU    (0x00000001)        ; Bit 0
ADC_SQR1_SQ13_1                     EQU    (0x00000002)        ; Bit 1
ADC_SQR1_SQ13_2                     EQU    (0x00000004)        ; Bit 2
ADC_SQR1_SQ13_3                     EQU    (0x00000008)        ; Bit 3
ADC_SQR1_SQ13_4                     EQU    (0x00000010)        ; Bit 4
ADC_SQR1_SQ14                       EQU    (0x000003E0)        ; SQ14[4:0] bits (14th conversion in regular sequence)
ADC_SQR1_SQ14_0                     EQU    (0x00000020)        ; Bit 0
ADC_SQR1_SQ14_1                     EQU    (0x00000040)        ; Bit 1
ADC_SQR1_SQ14_2                     EQU    (0x00000080)        ; Bit 2
ADC_SQR1_SQ14_3                     EQU    (0x00000100)        ; Bit 3
ADC_SQR1_SQ14_4                     EQU    (0x00000200)        ; Bit 4
ADC_SQR1_SQ15                       EQU    (0x00007C00)        ; SQ15[4:0] bits (15th conversion in regular sequence)
ADC_SQR1_SQ15_0                     EQU    (0x00000400)        ; Bit 0
ADC_SQR1_SQ15_1                     EQU    (0x00000800)        ; Bit 1
ADC_SQR1_SQ15_2                     EQU    (0x00001000)        ; Bit 2
ADC_SQR1_SQ15_3                     EQU    (0x00002000)        ; Bit 3
ADC_SQR1_SQ15_4                     EQU    (0x00004000)        ; Bit 4
ADC_SQR1_SQ16                       EQU    (0x000F8000)        ; SQ16[4:0] bits (16th conversion in regular sequence)
ADC_SQR1_SQ16_0                     EQU    (0x00008000)        ; Bit 0
ADC_SQR1_SQ16_1                     EQU    (0x00010000)        ; Bit 1
ADC_SQR1_SQ16_2                     EQU    (0x00020000)        ; Bit 2
ADC_SQR1_SQ16_3                     EQU    (0x00040000)        ; Bit 3
ADC_SQR1_SQ16_4                     EQU    (0x00080000)        ; Bit 4
ADC_SQR1_L                          EQU    (0x00F00000)        ; L[3:0] bits (Regular channel sequence length)
ADC_SQR1_L_0                        EQU    (0x00100000)        ; Bit 0
ADC_SQR1_L_1                        EQU    (0x00200000)        ; Bit 1
ADC_SQR1_L_2                        EQU    (0x00400000)        ; Bit 2
ADC_SQR1_L_3                        EQU    (0x00800000)        ; Bit 3

;*******************  Bit definition for ADC_SQR2 register  *******************
ADC_SQR2_SQ7                        EQU    (0x0000001F)        ; SQ7[4:0] bits (7th conversion in regular sequence)
ADC_SQR2_SQ7_0                      EQU    (0x00000001)        ; Bit 0
ADC_SQR2_SQ7_1                      EQU    (0x00000002)        ; Bit 1
ADC_SQR2_SQ7_2                      EQU    (0x00000004)        ; Bit 2
ADC_SQR2_SQ7_3                      EQU    (0x00000008)        ; Bit 3
ADC_SQR2_SQ7_4                      EQU    (0x00000010)        ; Bit 4
ADC_SQR2_SQ8                        EQU    (0x000003E0)        ; SQ8[4:0] bits (8th conversion in regular sequence)
ADC_SQR2_SQ8_0                      EQU    (0x00000020)        ; Bit 0
ADC_SQR2_SQ8_1                      EQU    (0x00000040)        ; Bit 1
ADC_SQR2_SQ8_2                      EQU    (0x00000080)        ; Bit 2
ADC_SQR2_SQ8_3                      EQU    (0x00000100)        ; Bit 3
ADC_SQR2_SQ8_4                      EQU    (0x00000200)        ; Bit 4
ADC_SQR2_SQ9                        EQU    (0x00007C00)        ; SQ9[4:0] bits (9th conversion in regular sequence)
ADC_SQR2_SQ9_0                      EQU    (0x00000400)        ; Bit 0
ADC_SQR2_SQ9_1                      EQU    (0x00000800)        ; Bit 1
ADC_SQR2_SQ9_2                      EQU    (0x00001000)        ; Bit 2
ADC_SQR2_SQ9_3                      EQU    (0x00002000)        ; Bit 3
ADC_SQR2_SQ9_4                      EQU    (0x00004000)        ; Bit 4
ADC_SQR2_SQ10                       EQU    (0x000F8000)        ; SQ10[4:0] bits (10th conversion in regular sequence)
ADC_SQR2_SQ10_0                     EQU    (0x00008000)        ; Bit 0
ADC_SQR2_SQ10_1                     EQU    (0x00010000)        ; Bit 1
ADC_SQR2_SQ10_2                     EQU    (0x00020000)        ; Bit 2
ADC_SQR2_SQ10_3                     EQU    (0x00040000)        ; Bit 3
ADC_SQR2_SQ10_4                     EQU    (0x00080000)        ; Bit 4
ADC_SQR2_SQ11                       EQU    (0x01F00000)        ; SQ11[4:0] bits (11th conversion in regular sequence)
ADC_SQR2_SQ11_0                     EQU    (0x00100000)        ; Bit 0
ADC_SQR2_SQ11_1                     EQU    (0x00200000)        ; Bit 1
ADC_SQR2_SQ11_2                     EQU    (0x00400000)        ; Bit 2
ADC_SQR2_SQ11_3                     EQU    (0x00800000)        ; Bit 3
ADC_SQR2_SQ11_4                     EQU    (0x01000000)        ; Bit 4
ADC_SQR2_SQ12                       EQU    (0x3E000000)        ; SQ12[4:0] bits (12th conversion in regular sequence)
ADC_SQR2_SQ12_0                     EQU    (0x02000000)        ; Bit 0
ADC_SQR2_SQ12_1                     EQU    (0x04000000)        ; Bit 1
ADC_SQR2_SQ12_2                     EQU    (0x08000000)        ; Bit 2
ADC_SQR2_SQ12_3                     EQU    (0x10000000)        ; Bit 3
ADC_SQR2_SQ12_4                     EQU    (0x20000000)        ; Bit 4

;*******************  Bit definition for ADC_SQR3 register  *******************
ADC_SQR3_SQ1                        EQU    (0x0000001F)        ; SQ1[4:0] bits (1st conversion in regular sequence)
ADC_SQR3_SQ1_0                      EQU    (0x00000001)        ; Bit 0
ADC_SQR3_SQ1_1                      EQU    (0x00000002)        ; Bit 1
ADC_SQR3_SQ1_2                      EQU    (0x00000004)        ; Bit 2
ADC_SQR3_SQ1_3                      EQU    (0x00000008)        ; Bit 3
ADC_SQR3_SQ1_4                      EQU    (0x00000010)        ; Bit 4
ADC_SQR3_SQ2                        EQU    (0x000003E0)        ; SQ2[4:0] bits (2nd conversion in regular sequence)
ADC_SQR3_SQ2_0                      EQU    (0x00000020)        ; Bit 0
ADC_SQR3_SQ2_1                      EQU    (0x00000040)        ; Bit 1
ADC_SQR3_SQ2_2                      EQU    (0x00000080)        ; Bit 2
ADC_SQR3_SQ2_3                      EQU    (0x00000100)        ; Bit 3
ADC_SQR3_SQ2_4                      EQU    (0x00000200)        ; Bit 4
ADC_SQR3_SQ3                        EQU    (0x00007C00)        ; SQ3[4:0] bits (3rd conversion in regular sequence)
ADC_SQR3_SQ3_0                      EQU    (0x00000400)        ; Bit 0
ADC_SQR3_SQ3_1                      EQU    (0x00000800)        ; Bit 1
ADC_SQR3_SQ3_2                      EQU    (0x00001000)        ; Bit 2
ADC_SQR3_SQ3_3                      EQU    (0x00002000)        ; Bit 3
ADC_SQR3_SQ3_4                      EQU    (0x00004000)        ; Bit 4
ADC_SQR3_SQ4                        EQU    (0x000F8000)        ; SQ4[4:0] bits (4th conversion in regular sequence)
ADC_SQR3_SQ4_0                      EQU    (0x00008000)        ; Bit 0
ADC_SQR3_SQ4_1                      EQU    (0x00010000)        ; Bit 1
ADC_SQR3_SQ4_2                      EQU    (0x00020000)        ; Bit 2
ADC_SQR3_SQ4_3                      EQU    (0x00040000)        ; Bit 3
ADC_SQR3_SQ4_4                      EQU    (0x00080000)        ; Bit 4
ADC_SQR3_SQ5                        EQU    (0x01F00000)        ; SQ5[4:0] bits (5th conversion in regular sequence)
ADC_SQR3_SQ5_0                      EQU    (0x00100000)        ; Bit 0
ADC_SQR3_SQ5_1                      EQU    (0x00200000)        ; Bit 1
ADC_SQR3_SQ5_2                      EQU    (0x00400000)        ; Bit 2
ADC_SQR3_SQ5_3                      EQU    (0x00800000)        ; Bit 3
ADC_SQR3_SQ5_4                      EQU    (0x01000000)        ; Bit 4
ADC_SQR3_SQ6                        EQU    (0x3E000000)        ; SQ6[4:0] bits (6th conversion in regular sequence)
ADC_SQR3_SQ6_0                      EQU    (0x02000000)        ; Bit 0
ADC_SQR3_SQ6_1                      EQU    (0x04000000)        ; Bit 1
ADC_SQR3_SQ6_2                      EQU    (0x08000000)        ; Bit 2
ADC_SQR3_SQ6_3                      EQU    (0x10000000)        ; Bit 3
ADC_SQR3_SQ6_4                      EQU    (0x20000000)        ; Bit 4

;*******************  Bit definition for ADC_JSQR register  *******************
ADC_JSQR_JSQ1                       EQU    (0x0000001F)        ; JSQ1[4:0] bits (1st conversion in injected sequence)
ADC_JSQR_JSQ1_0                     EQU    (0x00000001)        ; Bit 0
ADC_JSQR_JSQ1_1                     EQU    (0x00000002)        ; Bit 1
ADC_JSQR_JSQ1_2                     EQU    (0x00000004)        ; Bit 2
ADC_JSQR_JSQ1_3                     EQU    (0x00000008)        ; Bit 3
ADC_JSQR_JSQ1_4                     EQU    (0x00000010)        ; Bit 4
ADC_JSQR_JSQ2                       EQU    (0x000003E0)        ; JSQ2[4:0] bits (2nd conversion in injected sequence)
ADC_JSQR_JSQ2_0                     EQU    (0x00000020)        ; Bit 0
ADC_JSQR_JSQ2_1                     EQU    (0x00000040)        ; Bit 1
ADC_JSQR_JSQ2_2                     EQU    (0x00000080)        ; Bit 2
ADC_JSQR_JSQ2_3                     EQU    (0x00000100)        ; Bit 3
ADC_JSQR_JSQ2_4                     EQU    (0x00000200)        ; Bit 4
ADC_JSQR_JSQ3                       EQU    (0x00007C00)        ; JSQ3[4:0] bits (3rd conversion in injected sequence)
ADC_JSQR_JSQ3_0                     EQU    (0x00000400)        ; Bit 0
ADC_JSQR_JSQ3_1                     EQU    (0x00000800)        ; Bit 1
ADC_JSQR_JSQ3_2                     EQU    (0x00001000)        ; Bit 2
ADC_JSQR_JSQ3_3                     EQU    (0x00002000)        ; Bit 3
ADC_JSQR_JSQ3_4                     EQU    (0x00004000)        ; Bit 4
ADC_JSQR_JSQ4                       EQU    (0x000F8000)        ; JSQ4[4:0] bits (4th conversion in injected sequence)
ADC_JSQR_JSQ4_0                     EQU    (0x00008000)        ; Bit 0
ADC_JSQR_JSQ4_1                     EQU    (0x00010000)        ; Bit 1
ADC_JSQR_JSQ4_2                     EQU    (0x00020000)        ; Bit 2
ADC_JSQR_JSQ4_3                     EQU    (0x00040000)        ; Bit 3
ADC_JSQR_JSQ4_4                     EQU    (0x00080000)        ; Bit 4
ADC_JSQR_JL                         EQU    (0x00300000)        ; JL[1:0] bits (Injected Sequence length)
ADC_JSQR_JL_0                       EQU    (0x00100000)        ; Bit 0
ADC_JSQR_JL_1                       EQU    (0x00200000)        ; Bit 1

;*******************  Bit definition for ADC_JDR1 register  *******************
ADC_JDR1_JDATA                      EQU    (0xFFFF)            ; Injected data

;*******************  Bit definition for ADC_JDR2 register  *******************
ADC_JDR2_JDATA                      EQU    (0xFFFF)            ; Injected data

;*******************  Bit definition for ADC_JDR3 register  *******************
ADC_JDR3_JDATA                      EQU    (0xFFFF)            ; Injected data

;*******************  Bit definition for ADC_JDR4 register  *******************
ADC_JDR4_JDATA                      EQU    (0xFFFF)            ; Injected data

;********************  Bit definition for ADC_DR register  ********************
ADC_DR_DATA                         EQU    (0x0000FFFF)        ; Regular data
ADC_DR_ADC2DATA                     EQU    (0xFFFF0000)        ; ADC2 data

;*******************  Bit definition for ADC_CSR register  ********************
ADC_CSR_AWD1                        EQU    (0x00000001)        ; ADC1 Analog watchdog flag
ADC_CSR_EOC1                        EQU    (0x00000002)        ; ADC1 End of conversion
ADC_CSR_JEOC1                       EQU    (0x00000004)        ; ADC1 Injected channel end of conversion
ADC_CSR_JSTRT1                      EQU    (0x00000008)        ; ADC1 Injected channel Start flag
ADC_CSR_STRT1                       EQU    (0x00000010)        ; ADC1 Regular channel Start flag
ADC_CSR_DOVR1                       EQU    (0x00000020)        ; ADC1 DMA overrun  flag
ADC_CSR_AWD2                        EQU    (0x00000100)        ; ADC2 Analog watchdog flag
ADC_CSR_EOC2                        EQU    (0x00000200)        ; ADC2 End of conversion
ADC_CSR_JEOC2                       EQU    (0x00000400)        ; ADC2 Injected channel end of conversion
ADC_CSR_JSTRT2                      EQU    (0x00000800)        ; ADC2 Injected channel Start flag
ADC_CSR_STRT2                       EQU    (0x00001000)        ; ADC2 Regular channel Start flag
ADC_CSR_DOVR2                       EQU    (0x00002000)        ; ADC2 DMA overrun  flag
ADC_CSR_AWD3                        EQU    (0x00010000)        ; ADC3 Analog watchdog flag
ADC_CSR_EOC3                        EQU    (0x00020000)        ; ADC3 End of conversion
ADC_CSR_JEOC3                       EQU    (0x00040000)        ; ADC3 Injected channel end of conversion
ADC_CSR_JSTRT3                      EQU    (0x00080000)        ; ADC3 Injected channel Start flag
ADC_CSR_STRT3                       EQU    (0x00100000)        ; ADC3 Regular channel Start flag
ADC_CSR_DOVR3                       EQU    (0x00200000)        ; ADC3 DMA overrun  flag

;*******************  Bit definition for ADC_CCR register  ********************
ADC_CCR_MULTI                       EQU    (0x0000001F)        ; MULTI[4:0] bits (Multi-ADC mode selection)
ADC_CCR_MULTI_0                     EQU    (0x00000001)        ; Bit 0
ADC_CCR_MULTI_1                     EQU    (0x00000002)        ; Bit 1
ADC_CCR_MULTI_2                     EQU    (0x00000004)        ; Bit 2
ADC_CCR_MULTI_3                     EQU    (0x00000008)        ; Bit 3
ADC_CCR_MULTI_4                     EQU    (0x00000010)        ; Bit 4
ADC_CCR_DELAY                       EQU    (0x00000F00)        ; DELAY[3:0] bits (Delay between 2 sampling phases)
ADC_CCR_DELAY_0                     EQU    (0x00000100)        ; Bit 0
ADC_CCR_DELAY_1                     EQU    (0x00000200)        ; Bit 1
ADC_CCR_DELAY_2                     EQU    (0x00000400)        ; Bit 2
ADC_CCR_DELAY_3                     EQU    (0x00000800)        ; Bit 3
ADC_CCR_DDS                         EQU    (0x00002000)        ; DMA disable selection (Multi-ADC mode)
ADC_CCR_DMA                         EQU    (0x0000C000)        ; DMA[1:0] bits (Direct Memory Access mode for multimode)
ADC_CCR_DMA_0                       EQU    (0x00004000)        ; Bit 0
ADC_CCR_DMA_1                       EQU    (0x00008000)        ; Bit 1
ADC_CCR_ADCPRE                      EQU    (0x00030000)        ; ADCPRE[1:0] bits (ADC prescaler)
ADC_CCR_ADCPRE_0                    EQU    (0x00010000)        ; Bit 0
ADC_CCR_ADCPRE_1                    EQU    (0x00020000)        ; Bit 1
ADC_CCR_VBATE                       EQU    (0x00400000)        ; VBAT Enable
ADC_CCR_TSVREFE                     EQU    (0x00800000)        ; Temperature Sensor and VREFINT Enable

;*******************  Bit definition for ADC_CDR register  ********************
ADC_CDR_DATA1                      EQU    (0x0000FFFF)         ; 1st data of a pair of regular conversions
ADC_CDR_DATA2                      EQU    (0xFFFF0000)         ; 2nd data of a pair of regular conversions

;******************************************************************************
;*
;*                          CRC calculation unit
;*
;******************************************************************************
;*******************  Bit definition for CRC_DR register  *********************
CRC_DR_DR                           EQU    (0xFFFFFFFF) ;  Data register bits


;*******************  Bit definition for CRC_IDR register  ********************
CRC_IDR_IDR                         EQU    (0xFF)        ;  General-purpose 8-bit data register bits


;********************  Bit definition for CRC_CR register  ********************
CRC_CR_RESET                        EQU    (0x01)        ;  RESET bit

;******************************************************************************
;*
;*                                 Debug MCU
;*
;******************************************************************************

;******************************************************************************
;*
;*                             DMA Controller
;*
;******************************************************************************
;********************  Bits definition for DMA_SxCR register  *****************
DMA_SxCR_CHSEL                       EQU    (0x0E000000)
DMA_SxCR_CHSEL_0                     EQU    (0x02000000)
DMA_SxCR_CHSEL_1                     EQU    (0x04000000)
DMA_SxCR_CHSEL_2                     EQU    (0x08000000)
DMA_SxCR_MBURST                      EQU    (0x01800000)
DMA_SxCR_MBURST_0                    EQU    (0x00800000)
DMA_SxCR_MBURST_1                    EQU    (0x01000000)
DMA_SxCR_PBURST                      EQU    (0x00600000)
DMA_SxCR_PBURST_0                    EQU    (0x00200000)
DMA_SxCR_PBURST_1                    EQU    (0x00400000)
DMA_SxCR_ACK                         EQU    (0x00100000)
DMA_SxCR_CT                          EQU    (0x00080000)
DMA_SxCR_DBM                         EQU    (0x00040000)
DMA_SxCR_PL                          EQU    (0x00030000)
DMA_SxCR_PL_0                        EQU    (0x00010000)
DMA_SxCR_PL_1                        EQU    (0x00020000)
DMA_SxCR_PINCOS                      EQU    (0x00008000)
DMA_SxCR_MSIZE                       EQU    (0x00006000)
DMA_SxCR_MSIZE_0                     EQU    (0x00002000)
DMA_SxCR_MSIZE_1                     EQU    (0x00004000)
DMA_SxCR_PSIZE                       EQU    (0x00001800)
DMA_SxCR_PSIZE_0                     EQU    (0x00000800)
DMA_SxCR_PSIZE_1                     EQU    (0x00001000)
DMA_SxCR_MINC                        EQU    (0x00000400)
DMA_SxCR_PINC                        EQU    (0x00000200)
DMA_SxCR_CIRC                        EQU    (0x00000100)
DMA_SxCR_DIR                         EQU    (0x000000C0)
DMA_SxCR_DIR_0                       EQU    (0x00000040)
DMA_SxCR_DIR_1                       EQU    (0x00000080)
DMA_SxCR_PFCTRL                      EQU    (0x00000020)
DMA_SxCR_TCIE                        EQU    (0x00000010)
DMA_SxCR_HTIE                        EQU    (0x00000008)
DMA_SxCR_TEIE                        EQU    (0x00000004)
DMA_SxCR_DMEIE                       EQU    (0x00000002)
DMA_SxCR_EN                          EQU    (0x00000001)

;********************  Bits definition for DMA_SxCNDTR register  **************
DMA_SxNDT                            EQU    (0x0000FFFF)
DMA_SxNDT_0                          EQU    (0x00000001)
DMA_SxNDT_1                          EQU    (0x00000002)
DMA_SxNDT_2                          EQU    (0x00000004)
DMA_SxNDT_3                          EQU    (0x00000008)
DMA_SxNDT_4                          EQU    (0x00000010)
DMA_SxNDT_5                          EQU    (0x00000020)
DMA_SxNDT_6                          EQU    (0x00000040)
DMA_SxNDT_7                          EQU    (0x00000080)
DMA_SxNDT_8                          EQU    (0x00000100)
DMA_SxNDT_9                          EQU    (0x00000200)
DMA_SxNDT_10                         EQU    (0x00000400)
DMA_SxNDT_11                         EQU    (0x00000800)
DMA_SxNDT_12                         EQU    (0x00001000)
DMA_SxNDT_13                         EQU    (0x00002000)
DMA_SxNDT_14                         EQU    (0x00004000)
DMA_SxNDT_15                         EQU    (0x00008000)

;********************  Bits definition for DMA_SxFCR register  ****************
DMA_SxFCR_FEIE                       EQU    (0x00000080)
DMA_SxFCR_FS                         EQU    (0x00000038)
DMA_SxFCR_FS_0                       EQU    (0x00000008)
DMA_SxFCR_FS_1                       EQU    (0x00000010)
DMA_SxFCR_FS_2                       EQU    (0x00000020)
DMA_SxFCR_DMDIS                      EQU    (0x00000004)
DMA_SxFCR_FTH                        EQU    (0x00000003)
DMA_SxFCR_FTH_0                      EQU    (0x00000001)
DMA_SxFCR_FTH_1                      EQU    (0x00000002)

;********************  Bits definition for DMA_LISR register  *****************
DMA_LISR_TCIF3                       EQU    (0x08000000)
DMA_LISR_HTIF3                       EQU    (0x04000000)
DMA_LISR_TEIF3                       EQU    (0x02000000)
DMA_LISR_DMEIF3                      EQU    (0x01000000)
DMA_LISR_FEIF3                       EQU    (0x00400000)
DMA_LISR_TCIF2                       EQU    (0x00200000)
DMA_LISR_HTIF2                       EQU    (0x00100000)
DMA_LISR_TEIF2                       EQU    (0x00080000)
DMA_LISR_DMEIF2                      EQU    (0x00040000)
DMA_LISR_FEIF2                       EQU    (0x00010000)
DMA_LISR_TCIF1                       EQU    (0x00000800)
DMA_LISR_HTIF1                       EQU    (0x00000400)
DMA_LISR_TEIF1                       EQU    (0x00000200)
DMA_LISR_DMEIF1                      EQU    (0x00000100)
DMA_LISR_FEIF1                       EQU    (0x00000040)
DMA_LISR_TCIF0                       EQU    (0x00000020)
DMA_LISR_HTIF0                       EQU    (0x00000010)
DMA_LISR_TEIF0                       EQU    (0x00000008)
DMA_LISR_DMEIF0                      EQU    (0x00000004)
DMA_LISR_FEIF0                       EQU    (0x00000001)

;********************  Bits definition for DMA_HISR register  *****************
DMA_HISR_TCIF7                       EQU    (0x08000000)
DMA_HISR_HTIF7                       EQU    (0x04000000)
DMA_HISR_TEIF7                       EQU    (0x02000000)
DMA_HISR_DMEIF7                      EQU    (0x01000000)
DMA_HISR_FEIF7                       EQU    (0x00400000)
DMA_HISR_TCIF6                       EQU    (0x00200000)
DMA_HISR_HTIF6                       EQU    (0x00100000)
DMA_HISR_TEIF6                       EQU    (0x00080000)
DMA_HISR_DMEIF6                      EQU    (0x00040000)
DMA_HISR_FEIF6                       EQU    (0x00010000)
DMA_HISR_TCIF5                       EQU    (0x00000800)
DMA_HISR_HTIF5                       EQU    (0x00000400)
DMA_HISR_TEIF5                       EQU    (0x00000200)
DMA_HISR_DMEIF5                      EQU    (0x00000100)
DMA_HISR_FEIF5                       EQU    (0x00000040)
DMA_HISR_TCIF4                       EQU    (0x00000020)
DMA_HISR_HTIF4                       EQU    (0x00000010)
DMA_HISR_TEIF4                       EQU    (0x00000008)
DMA_HISR_DMEIF4                      EQU    (0x00000004)
DMA_HISR_FEIF4                       EQU    (0x00000001)

;********************  Bits definition for DMA_LIFCR register  ****************
DMA_LIFCR_CTCIF3                     EQU    (0x08000000)
DMA_LIFCR_CHTIF3                     EQU    (0x04000000)
DMA_LIFCR_CTEIF3                     EQU    (0x02000000)
DMA_LIFCR_CDMEIF3                    EQU    (0x01000000)
DMA_LIFCR_CFEIF3                     EQU    (0x00400000)
DMA_LIFCR_CTCIF2                     EQU    (0x00200000)
DMA_LIFCR_CHTIF2                     EQU    (0x00100000)
DMA_LIFCR_CTEIF2                     EQU    (0x00080000)
DMA_LIFCR_CDMEIF2                    EQU    (0x00040000)
DMA_LIFCR_CFEIF2                     EQU    (0x00010000)
DMA_LIFCR_CTCIF1                     EQU    (0x00000800)
DMA_LIFCR_CHTIF1                     EQU    (0x00000400)
DMA_LIFCR_CTEIF1                     EQU    (0x00000200)
DMA_LIFCR_CDMEIF1                    EQU    (0x00000100)
DMA_LIFCR_CFEIF1                     EQU    (0x00000040)
DMA_LIFCR_CTCIF0                     EQU    (0x00000020)
DMA_LIFCR_CHTIF0                     EQU    (0x00000010)
DMA_LIFCR_CTEIF0                     EQU    (0x00000008)
DMA_LIFCR_CDMEIF0                    EQU    (0x00000004)
DMA_LIFCR_CFEIF0                     EQU    (0x00000001)

;********************  Bits definition for DMA_HIFCR  register  ****************
DMA_HIFCR_CTCIF7                     EQU    (0x08000000)
DMA_HIFCR_CHTIF7                     EQU    (0x04000000)
DMA_HIFCR_CTEIF7                     EQU    (0x02000000)
DMA_HIFCR_CDMEIF7                    EQU    (0x01000000)
DMA_HIFCR_CFEIF7                     EQU    (0x00400000)
DMA_HIFCR_CTCIF6                     EQU    (0x00200000)
DMA_HIFCR_CHTIF6                     EQU    (0x00100000)
DMA_HIFCR_CTEIF6                     EQU    (0x00080000)
DMA_HIFCR_CDMEIF6                    EQU    (0x00040000)
DMA_HIFCR_CFEIF6                     EQU    (0x00010000)
DMA_HIFCR_CTCIF5                     EQU    (0x00000800)
DMA_HIFCR_CHTIF5                     EQU    (0x00000400)
DMA_HIFCR_CTEIF5                     EQU    (0x00000200)
DMA_HIFCR_CDMEIF5                    EQU    (0x00000100)
DMA_HIFCR_CFEIF5                     EQU    (0x00000040)
DMA_HIFCR_CTCIF4                     EQU    (0x00000020)
DMA_HIFCR_CHTIF4                     EQU    (0x00000010)
DMA_HIFCR_CTEIF4                     EQU    (0x00000008)
DMA_HIFCR_CDMEIF4                    EQU    (0x00000004)
DMA_HIFCR_CFEIF4                     EQU    (0x00000001)


;******************************************************************************
;*
;*                    External Interrupt/Event Controller
;*
;******************************************************************************
;*******************  Bit definition for EXTI_IMR register  *******************
EXTI_IMR_MR0                        EQU    (0x00000001)        ;  Interrupt Mask on line 0
EXTI_IMR_MR1                        EQU    (0x00000002)        ;  Interrupt Mask on line 1
EXTI_IMR_MR2                        EQU    (0x00000004)        ;  Interrupt Mask on line 2
EXTI_IMR_MR3                        EQU    (0x00000008)        ;  Interrupt Mask on line 3
EXTI_IMR_MR4                        EQU    (0x00000010)        ;  Interrupt Mask on line 4
EXTI_IMR_MR5                        EQU    (0x00000020)        ;  Interrupt Mask on line 5
EXTI_IMR_MR6                        EQU    (0x00000040)        ;  Interrupt Mask on line 6
EXTI_IMR_MR7                        EQU    (0x00000080)        ;  Interrupt Mask on line 7
EXTI_IMR_MR8                        EQU    (0x00000100)        ;  Interrupt Mask on line 8
EXTI_IMR_MR9                        EQU    (0x00000200)        ;  Interrupt Mask on line 9
EXTI_IMR_MR10                       EQU    (0x00000400)        ;  Interrupt Mask on line 10
EXTI_IMR_MR11                       EQU    (0x00000800)        ;  Interrupt Mask on line 11
EXTI_IMR_MR12                       EQU    (0x00001000)        ;  Interrupt Mask on line 12
EXTI_IMR_MR13                       EQU    (0x00002000)        ;  Interrupt Mask on line 13
EXTI_IMR_MR14                       EQU    (0x00004000)        ;  Interrupt Mask on line 14
EXTI_IMR_MR15                       EQU    (0x00008000)        ;  Interrupt Mask on line 15
EXTI_IMR_MR16                       EQU    (0x00010000)        ;  Interrupt Mask on line 16
EXTI_IMR_MR17                       EQU    (0x00020000)        ;  Interrupt Mask on line 17
EXTI_IMR_MR18                       EQU    (0x00040000)        ;  Interrupt Mask on line 18
EXTI_IMR_MR19                       EQU    (0x00080000)        ;  Interrupt Mask on line 19

;*******************  Bit definition for EXTI_EMR register  *******************
EXTI_EMR_MR0                        EQU    (0x00000001)        ;  Event Mask on line 0
EXTI_EMR_MR1                        EQU    (0x00000002)        ;  Event Mask on line 1
EXTI_EMR_MR2                        EQU    (0x00000004)        ;  Event Mask on line 2
EXTI_EMR_MR3                        EQU    (0x00000008)        ;  Event Mask on line 3
EXTI_EMR_MR4                        EQU    (0x00000010)        ;  Event Mask on line 4
EXTI_EMR_MR5                        EQU    (0x00000020)        ;  Event Mask on line 5
EXTI_EMR_MR6                        EQU    (0x00000040)        ;  Event Mask on line 6
EXTI_EMR_MR7                        EQU    (0x00000080)        ;  Event Mask on line 7
EXTI_EMR_MR8                        EQU    (0x00000100)        ;  Event Mask on line 8
EXTI_EMR_MR9                        EQU    (0x00000200)        ;  Event Mask on line 9
EXTI_EMR_MR10                       EQU    (0x00000400)        ;  Event Mask on line 10
EXTI_EMR_MR11                       EQU    (0x00000800)        ;  Event Mask on line 11
EXTI_EMR_MR12                       EQU    (0x00001000)        ;  Event Mask on line 12
EXTI_EMR_MR13                       EQU    (0x00002000)        ;  Event Mask on line 13
EXTI_EMR_MR14                       EQU    (0x00004000)        ;  Event Mask on line 14
EXTI_EMR_MR15                       EQU    (0x00008000)        ;  Event Mask on line 15
EXTI_EMR_MR16                       EQU    (0x00010000)        ;  Event Mask on line 16
EXTI_EMR_MR17                       EQU    (0x00020000)        ;  Event Mask on line 17
EXTI_EMR_MR18                       EQU    (0x00040000)        ;  Event Mask on line 18
EXTI_EMR_MR19                       EQU    (0x00080000)        ;  Event Mask on line 19

;******************  Bit definition for EXTI_RTSR register  *******************
EXTI_RTSR_TR0                       EQU    (0x00000001)        ;  Rising trigger event configuration bit of line 0
EXTI_RTSR_TR1                       EQU    (0x00000002)        ;  Rising trigger event configuration bit of line 1
EXTI_RTSR_TR2                       EQU    (0x00000004)        ;  Rising trigger event configuration bit of line 2
EXTI_RTSR_TR3                       EQU    (0x00000008)        ;  Rising trigger event configuration bit of line 3
EXTI_RTSR_TR4                       EQU    (0x00000010)        ;  Rising trigger event configuration bit of line 4
EXTI_RTSR_TR5                       EQU    (0x00000020)        ;  Rising trigger event configuration bit of line 5
EXTI_RTSR_TR6                       EQU    (0x00000040)        ;  Rising trigger event configuration bit of line 6
EXTI_RTSR_TR7                       EQU    (0x00000080)        ;  Rising trigger event configuration bit of line 7
EXTI_RTSR_TR8                       EQU    (0x00000100)        ;  Rising trigger event configuration bit of line 8
EXTI_RTSR_TR9                       EQU    (0x00000200)        ;  Rising trigger event configuration bit of line 9
EXTI_RTSR_TR10                      EQU    (0x00000400)        ;  Rising trigger event configuration bit of line 10
EXTI_RTSR_TR11                      EQU    (0x00000800)        ;  Rising trigger event configuration bit of line 11
EXTI_RTSR_TR12                      EQU    (0x00001000)        ;  Rising trigger event configuration bit of line 12
EXTI_RTSR_TR13                      EQU    (0x00002000)        ;  Rising trigger event configuration bit of line 13
EXTI_RTSR_TR14                      EQU    (0x00004000)        ;  Rising trigger event configuration bit of line 14
EXTI_RTSR_TR15                      EQU    (0x00008000)        ;  Rising trigger event configuration bit of line 15
EXTI_RTSR_TR16                      EQU    (0x00010000)        ;  Rising trigger event configuration bit of line 16
EXTI_RTSR_TR17                      EQU    (0x00020000)        ;  Rising trigger event configuration bit of line 17
EXTI_RTSR_TR18                      EQU    (0x00040000)        ;  Rising trigger event configuration bit of line 18
EXTI_RTSR_TR19                      EQU    (0x00080000)        ;  Rising trigger event configuration bit of line 19

;******************  Bit definition for EXTI_FTSR register  *******************
EXTI_FTSR_TR0                       EQU    (0x00000001)        ;  Falling trigger event configuration bit of line 0
EXTI_FTSR_TR1                       EQU    (0x00000002)        ;  Falling trigger event configuration bit of line 1
EXTI_FTSR_TR2                       EQU    (0x00000004)        ;  Falling trigger event configuration bit of line 2
EXTI_FTSR_TR3                       EQU    (0x00000008)        ;  Falling trigger event configuration bit of line 3
EXTI_FTSR_TR4                       EQU    (0x00000010)        ;  Falling trigger event configuration bit of line 4
EXTI_FTSR_TR5                       EQU    (0x00000020)        ;  Falling trigger event configuration bit of line 5
EXTI_FTSR_TR6                       EQU    (0x00000040)        ;  Falling trigger event configuration bit of line 6
EXTI_FTSR_TR7                       EQU    (0x00000080)        ;  Falling trigger event configuration bit of line 7
EXTI_FTSR_TR8                       EQU    (0x00000100)        ;  Falling trigger event configuration bit of line 8
EXTI_FTSR_TR9                       EQU    (0x00000200)        ;  Falling trigger event configuration bit of line 9
EXTI_FTSR_TR10                      EQU    (0x00000400)        ;  Falling trigger event configuration bit of line 10
EXTI_FTSR_TR11                      EQU    (0x00000800)        ;  Falling trigger event configuration bit of line 11
EXTI_FTSR_TR12                      EQU    (0x00001000)        ;  Falling trigger event configuration bit of line 12
EXTI_FTSR_TR13                      EQU    (0x00002000)        ;  Falling trigger event configuration bit of line 13
EXTI_FTSR_TR14                      EQU    (0x00004000)        ;  Falling trigger event configuration bit of line 14
EXTI_FTSR_TR15                      EQU    (0x00008000)        ;  Falling trigger event configuration bit of line 15
EXTI_FTSR_TR16                      EQU    (0x00010000)        ;  Falling trigger event configuration bit of line 16
EXTI_FTSR_TR17                      EQU    (0x00020000)        ;  Falling trigger event configuration bit of line 17
EXTI_FTSR_TR18                      EQU    (0x00040000)        ;  Falling trigger event configuration bit of line 18
EXTI_FTSR_TR19                      EQU    (0x00080000)        ;  Falling trigger event configuration bit of line 19

;******************  Bit definition for EXTI_SWIER register  ******************
EXTI_SWIER_SWIER0                   EQU    (0x00000001)        ;  Software Interrupt on line 0
EXTI_SWIER_SWIER1                   EQU    (0x00000002)        ;  Software Interrupt on line 1
EXTI_SWIER_SWIER2                   EQU    (0x00000004)        ;  Software Interrupt on line 2
EXTI_SWIER_SWIER3                   EQU    (0x00000008)        ;  Software Interrupt on line 3
EXTI_SWIER_SWIER4                   EQU    (0x00000010)        ;  Software Interrupt on line 4
EXTI_SWIER_SWIER5                   EQU    (0x00000020)        ;  Software Interrupt on line 5
EXTI_SWIER_SWIER6                   EQU    (0x00000040)        ;  Software Interrupt on line 6
EXTI_SWIER_SWIER7                   EQU    (0x00000080)        ;  Software Interrupt on line 7
EXTI_SWIER_SWIER8                   EQU    (0x00000100)        ;  Software Interrupt on line 8
EXTI_SWIER_SWIER9                   EQU    (0x00000200)        ;  Software Interrupt on line 9
EXTI_SWIER_SWIER10                  EQU    (0x00000400)        ;  Software Interrupt on line 10
EXTI_SWIER_SWIER11                  EQU    (0x00000800)        ;  Software Interrupt on line 11
EXTI_SWIER_SWIER12                  EQU    (0x00001000)        ;  Software Interrupt on line 12
EXTI_SWIER_SWIER13                  EQU    (0x00002000)        ;  Software Interrupt on line 13
EXTI_SWIER_SWIER14                  EQU    (0x00004000)        ;  Software Interrupt on line 14
EXTI_SWIER_SWIER15                  EQU    (0x00008000)        ;  Software Interrupt on line 15
EXTI_SWIER_SWIER16                  EQU    (0x00010000)        ;  Software Interrupt on line 16
EXTI_SWIER_SWIER17                  EQU    (0x00020000)        ;  Software Interrupt on line 17
EXTI_SWIER_SWIER18                  EQU    (0x00040000)        ;  Software Interrupt on line 18
EXTI_SWIER_SWIER19                  EQU    (0x00080000)        ;  Software Interrupt on line 19

;*******************  Bit definition for EXTI_PR register  ********************
EXTI_PR_PR0                         EQU    (0x00000001)        ;  Pending bit for line 0
EXTI_PR_PR1                         EQU    (0x00000002)        ;  Pending bit for line 1
EXTI_PR_PR2                         EQU    (0x00000004)        ;  Pending bit for line 2
EXTI_PR_PR3                         EQU    (0x00000008)        ;  Pending bit for line 3
EXTI_PR_PR4                         EQU    (0x00000010)        ;  Pending bit for line 4
EXTI_PR_PR5                         EQU    (0x00000020)        ;  Pending bit for line 5
EXTI_PR_PR6                         EQU    (0x00000040)        ;  Pending bit for line 6
EXTI_PR_PR7                         EQU    (0x00000080)        ;  Pending bit for line 7
EXTI_PR_PR8                         EQU    (0x00000100)        ;  Pending bit for line 8
EXTI_PR_PR9                         EQU    (0x00000200)        ;  Pending bit for line 9
EXTI_PR_PR10                        EQU    (0x00000400)        ;  Pending bit for line 10
EXTI_PR_PR11                        EQU    (0x00000800)        ;  Pending bit for line 11
EXTI_PR_PR12                        EQU    (0x00001000)        ;  Pending bit for line 12
EXTI_PR_PR13                        EQU    (0x00002000)        ;  Pending bit for line 13
EXTI_PR_PR14                        EQU    (0x00004000)        ;  Pending bit for line 14
EXTI_PR_PR15                        EQU    (0x00008000)        ;  Pending bit for line 15
EXTI_PR_PR16                        EQU    (0x00010000)        ;  Pending bit for line 16
EXTI_PR_PR17                        EQU    (0x00020000)        ;  Pending bit for line 17
EXTI_PR_PR18                        EQU    (0x00040000)        ;  Pending bit for line 18
EXTI_PR_PR19                        EQU    (0x00080000)        ;  Pending bit for line 19

;******************************************************************************
;*
;*                                    FLASH
;*
;******************************************************************************
;*******************  Bits definition for FLASH_ACR register  *****************
FLASH_ACR_LATENCY                    EQU    (0x0000000F)
FLASH_ACR_LATENCY_0WS                EQU    (0x00000000)
FLASH_ACR_LATENCY_1WS                EQU    (0x00000001)
FLASH_ACR_LATENCY_2WS                EQU    (0x00000002)
FLASH_ACR_LATENCY_3WS                EQU    (0x00000003)
FLASH_ACR_LATENCY_4WS                EQU    (0x00000004)
FLASH_ACR_LATENCY_5WS                EQU    (0x00000005)
FLASH_ACR_LATENCY_6WS                EQU    (0x00000006)
FLASH_ACR_LATENCY_7WS                EQU    (0x00000007)

FLASH_ACR_PRFTEN                     EQU    (0x00000100)
FLASH_ACR_ICEN                       EQU    (0x00000200)
FLASH_ACR_DCEN                       EQU    (0x00000400)
FLASH_ACR_ICRST                      EQU    (0x00000800)
FLASH_ACR_DCRST                      EQU    (0x00001000)
FLASH_ACR_BYTE0_ADDRESS              EQU    (0x40023C00)
FLASH_ACR_BYTE2_ADDRESS              EQU    (0x40023C03)

;*******************  Bits definition for FLASH_SR register  ******************
FLASH_SR_EOP                         EQU    (0x00000001)
FLASH_SR_SOP                         EQU    (0x00000002)
FLASH_SR_WRPERR                      EQU    (0x00000010)
FLASH_SR_PGAERR                      EQU    (0x00000020)
FLASH_SR_PGPERR                      EQU    (0x00000040)
FLASH_SR_PGSERR                      EQU    (0x00000080)
FLASH_SR_BSY                         EQU    (0x00010000)

;*******************  Bits definition for FLASH_CR register  ******************
FLASH_CR_PG                          EQU    (0x00000001)
FLASH_CR_SER                         EQU    (0x00000002)
FLASH_CR_MER                         EQU    (0x00000004)
FLASH_CR_SNB                         EQU    (0x000000F8)
FLASH_CR_SNB_0                       EQU    (0x00000008)
FLASH_CR_SNB_1                       EQU    (0x00000010)
FLASH_CR_SNB_2                       EQU    (0x00000020)
FLASH_CR_SNB_3                       EQU    (0x00000040)
FLASH_CR_SNB_4                       EQU    (0x00000080)
FLASH_CR_PSIZE                       EQU    (0x00000300)
FLASH_CR_PSIZE_0                     EQU    (0x00000100)
FLASH_CR_PSIZE_1                     EQU    (0x00000200)
FLASH_CR_STRT                        EQU    (0x00010000)
FLASH_CR_EOPIE                       EQU    (0x01000000)
FLASH_CR_LOCK                        EQU    (0x80000000)

;*******************  Bits definition for FLASH_OPTCR register  ***************
FLASH_OPTCR_OPTLOCK                 EQU    (0x00000001)
FLASH_OPTCR_OPTSTRT                 EQU    (0x00000002)
FLASH_OPTCR_BOR_LEV_0               EQU    (0x00000004)
FLASH_OPTCR_BOR_LEV_1               EQU    (0x00000008)
FLASH_OPTCR_BOR_LEV                 EQU    (0x0000000C)

FLASH_OPTCR_WDG_SW                  EQU    (0x00000020)
FLASH_OPTCR_nRST_STOP               EQU    (0x00000040)
FLASH_OPTCR_nRST_STDBY              EQU    (0x00000080)
FLASH_OPTCR_RDP                     EQU    (0x0000FF00)
FLASH_OPTCR_RDP_0                   EQU    (0x00000100)
FLASH_OPTCR_RDP_1                   EQU    (0x00000200)
FLASH_OPTCR_RDP_2                   EQU    (0x00000400)
FLASH_OPTCR_RDP_3                   EQU    (0x00000800)
FLASH_OPTCR_RDP_4                   EQU    (0x00001000)
FLASH_OPTCR_RDP_5                   EQU    (0x00002000)
FLASH_OPTCR_RDP_6                   EQU    (0x00004000)
FLASH_OPTCR_RDP_7                   EQU    (0x00008000)
FLASH_OPTCR_nWRP                    EQU    (0x0FFF0000)
FLASH_OPTCR_nWRP_0                  EQU    (0x00010000)
FLASH_OPTCR_nWRP_1                  EQU    (0x00020000)
FLASH_OPTCR_nWRP_2                  EQU    (0x00040000)
FLASH_OPTCR_nWRP_3                  EQU    (0x00080000)
FLASH_OPTCR_nWRP_4                  EQU    (0x00100000)
FLASH_OPTCR_nWRP_5                  EQU    (0x00200000)
FLASH_OPTCR_nWRP_6                  EQU    (0x00400000)
FLASH_OPTCR_nWRP_7                  EQU    (0x00800000)
FLASH_OPTCR_nWRP_8                  EQU    (0x01000000)
FLASH_OPTCR_nWRP_9                  EQU    (0x02000000)
FLASH_OPTCR_nWRP_10                 EQU    (0x04000000)
FLASH_OPTCR_nWRP_11                 EQU    (0x08000000)

;******************  Bits definition for FLASH_OPTCR1 register  ***************
FLASH_OPTCR1_nWRP                    EQU    (0x0FFF0000)
FLASH_OPTCR1_nWRP_0                  EQU    (0x00010000)
FLASH_OPTCR1_nWRP_1                  EQU    (0x00020000)
FLASH_OPTCR1_nWRP_2                  EQU    (0x00040000)
FLASH_OPTCR1_nWRP_3                  EQU    (0x00080000)
FLASH_OPTCR1_nWRP_4                  EQU    (0x00100000)
FLASH_OPTCR1_nWRP_5                  EQU    (0x00200000)
FLASH_OPTCR1_nWRP_6                  EQU    (0x00400000)
FLASH_OPTCR1_nWRP_7                  EQU    (0x00800000)
FLASH_OPTCR1_nWRP_8                  EQU    (0x01000000)
FLASH_OPTCR1_nWRP_9                  EQU    (0x02000000)
FLASH_OPTCR1_nWRP_10                 EQU    (0x04000000)
FLASH_OPTCR1_nWRP_11                 EQU    (0x08000000)

;******************************************************************************
;*
;*                            General Purpose I/O
;*
;******************************************************************************
;******************  Bits definition for GPIO_MODER register  *****************
GPIO_MODER_MODER0                    EQU    (0x00000003)
GPIO_MODER_MODER0_0                  EQU    (0x00000001)
GPIO_MODER_MODER0_1                  EQU    (0x00000002)

GPIO_MODER_MODER1                    EQU    (0x0000000C)
GPIO_MODER_MODER1_0                  EQU    (0x00000004)
GPIO_MODER_MODER1_1                  EQU    (0x00000008)

GPIO_MODER_MODER2                    EQU    (0x00000030)
GPIO_MODER_MODER2_0                  EQU    (0x00000010)
GPIO_MODER_MODER2_1                  EQU    (0x00000020)

GPIO_MODER_MODER3                    EQU    (0x000000C0)
GPIO_MODER_MODER3_0                  EQU    (0x00000040)
GPIO_MODER_MODER3_1                  EQU    (0x00000080)

GPIO_MODER_MODER4                    EQU    (0x00000300)
GPIO_MODER_MODER4_0                  EQU    (0x00000100)
GPIO_MODER_MODER4_1                  EQU    (0x00000200)

GPIO_MODER_MODER5                    EQU    (0x00000C00)
GPIO_MODER_MODER5_0                  EQU    (0x00000400)
GPIO_MODER_MODER5_1                  EQU    (0x00000800)

GPIO_MODER_MODER6                    EQU    (0x00003000)
GPIO_MODER_MODER6_0                  EQU    (0x00001000)
GPIO_MODER_MODER6_1                  EQU    (0x00002000)

GPIO_MODER_MODER7                    EQU    (0x0000C000)
GPIO_MODER_MODER7_0                  EQU    (0x00004000)
GPIO_MODER_MODER7_1                  EQU    (0x00008000)

GPIO_MODER_MODER8                    EQU    (0x00030000)
GPIO_MODER_MODER8_0                  EQU    (0x00010000)
GPIO_MODER_MODER8_1                  EQU    (0x00020000)

GPIO_MODER_MODER9                    EQU    (0x000C0000)
GPIO_MODER_MODER9_0                  EQU    (0x00040000)
GPIO_MODER_MODER9_1                  EQU    (0x00080000)

GPIO_MODER_MODER10                   EQU    (0x00300000)
GPIO_MODER_MODER10_0                 EQU    (0x00100000)
GPIO_MODER_MODER10_1                 EQU    (0x00200000)

GPIO_MODER_MODER11                   EQU    (0x00C00000)
GPIO_MODER_MODER11_0                 EQU    (0x00400000)
GPIO_MODER_MODER11_1                 EQU    (0x00800000)

GPIO_MODER_MODER12                   EQU    (0x03000000)
GPIO_MODER_MODER12_0                 EQU    (0x01000000)
GPIO_MODER_MODER12_1                 EQU    (0x02000000)

GPIO_MODER_MODER13                   EQU    (0x0C000000)
GPIO_MODER_MODER13_0                 EQU    (0x04000000)
GPIO_MODER_MODER13_1                 EQU    (0x08000000)

GPIO_MODER_MODER14                   EQU    (0x30000000)
GPIO_MODER_MODER14_0                 EQU    (0x10000000)
GPIO_MODER_MODER14_1                 EQU    (0x20000000)

GPIO_MODER_MODER15                   EQU    (0xC0000000)
GPIO_MODER_MODER15_0                 EQU    (0x40000000)
GPIO_MODER_MODER15_1                 EQU    (0x80000000)

;******************  Bits definition for GPIO_OTYPER register  ****************
GPIO_OTYPER_OT_0                     EQU    (0x00000001)
GPIO_OTYPER_OT_1                     EQU    (0x00000002)
GPIO_OTYPER_OT_2                     EQU    (0x00000004)
GPIO_OTYPER_OT_3                     EQU    (0x00000008)
GPIO_OTYPER_OT_4                     EQU    (0x00000010)
GPIO_OTYPER_OT_5                     EQU    (0x00000020)
GPIO_OTYPER_OT_6                     EQU    (0x00000040)
GPIO_OTYPER_OT_7                     EQU    (0x00000080)
GPIO_OTYPER_OT_8                     EQU    (0x00000100)
GPIO_OTYPER_OT_9                     EQU    (0x00000200)
GPIO_OTYPER_OT_10                    EQU    (0x00000400)
GPIO_OTYPER_OT_11                    EQU    (0x00000800)
GPIO_OTYPER_OT_12                    EQU    (0x00001000)
GPIO_OTYPER_OT_13                    EQU    (0x00002000)
GPIO_OTYPER_OT_14                    EQU    (0x00004000)
GPIO_OTYPER_OT_15                    EQU    (0x00008000)

;******************  Bits definition for GPIO_OSPEEDR register  ***************
GPIO_OSPEEDER_OSPEEDR0               EQU    (0x00000003)
GPIO_OSPEEDER_OSPEEDR0_0             EQU    (0x00000001)
GPIO_OSPEEDER_OSPEEDR0_1             EQU    (0x00000002)

GPIO_OSPEEDER_OSPEEDR1               EQU    (0x0000000C)
GPIO_OSPEEDER_OSPEEDR1_0             EQU    (0x00000004)
GPIO_OSPEEDER_OSPEEDR1_1             EQU    (0x00000008)

GPIO_OSPEEDER_OSPEEDR2               EQU    (0x00000030)
GPIO_OSPEEDER_OSPEEDR2_0             EQU    (0x00000010)
GPIO_OSPEEDER_OSPEEDR2_1             EQU    (0x00000020)

GPIO_OSPEEDER_OSPEEDR3               EQU    (0x000000C0)
GPIO_OSPEEDER_OSPEEDR3_0             EQU    (0x00000040)
GPIO_OSPEEDER_OSPEEDR3_1             EQU    (0x00000080)

GPIO_OSPEEDER_OSPEEDR4               EQU    (0x00000300)
GPIO_OSPEEDER_OSPEEDR4_0             EQU    (0x00000100)
GPIO_OSPEEDER_OSPEEDR4_1             EQU    (0x00000200)

GPIO_OSPEEDER_OSPEEDR5               EQU    (0x00000C00)
GPIO_OSPEEDER_OSPEEDR5_0             EQU    (0x00000400)
GPIO_OSPEEDER_OSPEEDR5_1             EQU    (0x00000800)

GPIO_OSPEEDER_OSPEEDR6               EQU    (0x00003000)
GPIO_OSPEEDER_OSPEEDR6_0             EQU    (0x00001000)
GPIO_OSPEEDER_OSPEEDR6_1             EQU    (0x00002000)

GPIO_OSPEEDER_OSPEEDR7               EQU    (0x0000C000)
GPIO_OSPEEDER_OSPEEDR7_0             EQU    (0x00004000)
GPIO_OSPEEDER_OSPEEDR7_1             EQU    (0x00008000)

GPIO_OSPEEDER_OSPEEDR8               EQU    (0x00030000)
GPIO_OSPEEDER_OSPEEDR8_0             EQU    (0x00010000)
GPIO_OSPEEDER_OSPEEDR8_1             EQU    (0x00020000)

GPIO_OSPEEDER_OSPEEDR9               EQU    (0x000C0000)
GPIO_OSPEEDER_OSPEEDR9_0             EQU    (0x00040000)
GPIO_OSPEEDER_OSPEEDR9_1             EQU    (0x00080000)

GPIO_OSPEEDER_OSPEEDR10              EQU    (0x00300000)
GPIO_OSPEEDER_OSPEEDR10_0            EQU    (0x00100000)
GPIO_OSPEEDER_OSPEEDR10_1            EQU    (0x00200000)

GPIO_OSPEEDER_OSPEEDR11              EQU    (0x00C00000)
GPIO_OSPEEDER_OSPEEDR11_0            EQU    (0x00400000)
GPIO_OSPEEDER_OSPEEDR11_1            EQU    (0x00800000)

GPIO_OSPEEDER_OSPEEDR12              EQU    (0x03000000)
GPIO_OSPEEDER_OSPEEDR12_0            EQU    (0x01000000)
GPIO_OSPEEDER_OSPEEDR12_1            EQU    (0x02000000)

GPIO_OSPEEDER_OSPEEDR13              EQU    (0x0C000000)
GPIO_OSPEEDER_OSPEEDR13_0            EQU    (0x04000000)
GPIO_OSPEEDER_OSPEEDR13_1            EQU    (0x08000000)

GPIO_OSPEEDER_OSPEEDR14              EQU    (0x30000000)
GPIO_OSPEEDER_OSPEEDR14_0            EQU    (0x10000000)
GPIO_OSPEEDER_OSPEEDR14_1            EQU    (0x20000000)

GPIO_OSPEEDER_OSPEEDR15              EQU    (0xC0000000)
GPIO_OSPEEDER_OSPEEDR15_0            EQU    (0x40000000)
GPIO_OSPEEDER_OSPEEDR15_1            EQU    (0x80000000)

;******************  Bits definition for GPIO_PUPDR register  *****************
GPIO_PUPDR_PUPDR0                    EQU    (0x00000003)
GPIO_PUPDR_PUPDR0_0                  EQU    (0x00000001)
GPIO_PUPDR_PUPDR0_1                  EQU    (0x00000002)

GPIO_PUPDR_PUPDR1                    EQU    (0x0000000C)
GPIO_PUPDR_PUPDR1_0                  EQU    (0x00000004)
GPIO_PUPDR_PUPDR1_1                  EQU    (0x00000008)

GPIO_PUPDR_PUPDR2                    EQU    (0x00000030)
GPIO_PUPDR_PUPDR2_0                  EQU    (0x00000010)
GPIO_PUPDR_PUPDR2_1                  EQU    (0x00000020)

GPIO_PUPDR_PUPDR3                    EQU    (0x000000C0)
GPIO_PUPDR_PUPDR3_0                  EQU    (0x00000040)
GPIO_PUPDR_PUPDR3_1                  EQU    (0x00000080)

GPIO_PUPDR_PUPDR4                    EQU    (0x00000300)
GPIO_PUPDR_PUPDR4_0                  EQU    (0x00000100)
GPIO_PUPDR_PUPDR4_1                  EQU    (0x00000200)

GPIO_PUPDR_PUPDR5                    EQU    (0x00000C00)
GPIO_PUPDR_PUPDR5_0                  EQU    (0x00000400)
GPIO_PUPDR_PUPDR5_1                  EQU    (0x00000800)

GPIO_PUPDR_PUPDR6                    EQU    (0x00003000)
GPIO_PUPDR_PUPDR6_0                  EQU    (0x00001000)
GPIO_PUPDR_PUPDR6_1                  EQU    (0x00002000)

GPIO_PUPDR_PUPDR7                    EQU    (0x0000C000)
GPIO_PUPDR_PUPDR7_0                  EQU    (0x00004000)
GPIO_PUPDR_PUPDR7_1                  EQU    (0x00008000)

GPIO_PUPDR_PUPDR8                    EQU    (0x00030000)
GPIO_PUPDR_PUPDR8_0                  EQU    (0x00010000)
GPIO_PUPDR_PUPDR8_1                  EQU    (0x00020000)

GPIO_PUPDR_PUPDR9                    EQU    (0x000C0000)
GPIO_PUPDR_PUPDR9_0                  EQU    (0x00040000)
GPIO_PUPDR_PUPDR9_1                  EQU    (0x00080000)

GPIO_PUPDR_PUPDR10                   EQU    (0x00300000)
GPIO_PUPDR_PUPDR10_0                 EQU    (0x00100000)
GPIO_PUPDR_PUPDR10_1                 EQU    (0x00200000)

GPIO_PUPDR_PUPDR11                   EQU    (0x00C00000)
GPIO_PUPDR_PUPDR11_0                 EQU    (0x00400000)
GPIO_PUPDR_PUPDR11_1                 EQU    (0x00800000)

GPIO_PUPDR_PUPDR12                   EQU    (0x03000000)
GPIO_PUPDR_PUPDR12_0                 EQU    (0x01000000)
GPIO_PUPDR_PUPDR12_1                 EQU    (0x02000000)

GPIO_PUPDR_PUPDR13                   EQU    (0x0C000000)
GPIO_PUPDR_PUPDR13_0                 EQU    (0x04000000)
GPIO_PUPDR_PUPDR13_1                 EQU    (0x08000000)

GPIO_PUPDR_PUPDR14                   EQU    (0x30000000)
GPIO_PUPDR_PUPDR14_0                 EQU    (0x10000000)
GPIO_PUPDR_PUPDR14_1                 EQU    (0x20000000)

GPIO_PUPDR_PUPDR15                   EQU    (0xC0000000)
GPIO_PUPDR_PUPDR15_0                 EQU    (0x40000000)
GPIO_PUPDR_PUPDR15_1                 EQU    (0x80000000)

;******************  Bits definition for GPIO_IDR register  *******************
GPIO_IDR_IDR_0                       EQU    (0x00000001)
GPIO_IDR_IDR_1                       EQU    (0x00000002)
GPIO_IDR_IDR_2                       EQU    (0x00000004)
GPIO_IDR_IDR_3                       EQU    (0x00000008)
GPIO_IDR_IDR_4                       EQU    (0x00000010)
GPIO_IDR_IDR_5                       EQU    (0x00000020)
GPIO_IDR_IDR_6                       EQU    (0x00000040)
GPIO_IDR_IDR_7                       EQU    (0x00000080)
GPIO_IDR_IDR_8                       EQU    (0x00000100)
GPIO_IDR_IDR_9                       EQU    (0x00000200)
GPIO_IDR_IDR_10                      EQU    (0x00000400)
GPIO_IDR_IDR_11                      EQU    (0x00000800)
GPIO_IDR_IDR_12                      EQU    (0x00001000)
GPIO_IDR_IDR_13                      EQU    (0x00002000)
GPIO_IDR_IDR_14                      EQU    (0x00004000)
GPIO_IDR_IDR_15                      EQU    (0x00008000)
;  Old GPIO_IDR register bits definition, maintained for legacy purpose
GPIO_OTYPER_IDR_0                    EQU GPIO_IDR_IDR_0
GPIO_OTYPER_IDR_1                    EQU GPIO_IDR_IDR_1
GPIO_OTYPER_IDR_2                    EQU GPIO_IDR_IDR_2
GPIO_OTYPER_IDR_3                    EQU GPIO_IDR_IDR_3
GPIO_OTYPER_IDR_4                    EQU GPIO_IDR_IDR_4
GPIO_OTYPER_IDR_5                    EQU GPIO_IDR_IDR_5
GPIO_OTYPER_IDR_6                    EQU GPIO_IDR_IDR_6
GPIO_OTYPER_IDR_7                    EQU GPIO_IDR_IDR_7
GPIO_OTYPER_IDR_8                    EQU GPIO_IDR_IDR_8
GPIO_OTYPER_IDR_9                    EQU GPIO_IDR_IDR_9
GPIO_OTYPER_IDR_10                   EQU GPIO_IDR_IDR_10
GPIO_OTYPER_IDR_11                   EQU GPIO_IDR_IDR_11
GPIO_OTYPER_IDR_12                   EQU GPIO_IDR_IDR_12
GPIO_OTYPER_IDR_13                   EQU GPIO_IDR_IDR_13
GPIO_OTYPER_IDR_14                   EQU GPIO_IDR_IDR_14
GPIO_OTYPER_IDR_15                   EQU GPIO_IDR_IDR_15

;******************  Bits definition for GPIO_ODR register  *******************
GPIO_ODR_ODR_0                       EQU    (0x00000001)
GPIO_ODR_ODR_1                       EQU    (0x00000002)
GPIO_ODR_ODR_2                       EQU    (0x00000004)
GPIO_ODR_ODR_3                       EQU    (0x00000008)
GPIO_ODR_ODR_4                       EQU    (0x00000010)
GPIO_ODR_ODR_5                       EQU    (0x00000020)
GPIO_ODR_ODR_6                       EQU    (0x00000040)
GPIO_ODR_ODR_7                       EQU    (0x00000080)
GPIO_ODR_ODR_8                       EQU    (0x00000100)
GPIO_ODR_ODR_9                       EQU    (0x00000200)
GPIO_ODR_ODR_10                      EQU    (0x00000400)
GPIO_ODR_ODR_11                      EQU    (0x00000800)
GPIO_ODR_ODR_12                      EQU    (0x00001000)
GPIO_ODR_ODR_13                      EQU    (0x00002000)
GPIO_ODR_ODR_14                      EQU    (0x00004000)
GPIO_ODR_ODR_15                      EQU    (0x00008000)
;  Old GPIO_ODR register bits definition, maintained for legacy purpose
GPIO_OTYPER_ODR_0                    EQU GPIO_ODR_ODR_0
GPIO_OTYPER_ODR_1                    EQU GPIO_ODR_ODR_1
GPIO_OTYPER_ODR_2                    EQU GPIO_ODR_ODR_2
GPIO_OTYPER_ODR_3                    EQU GPIO_ODR_ODR_3
GPIO_OTYPER_ODR_4                    EQU GPIO_ODR_ODR_4
GPIO_OTYPER_ODR_5                    EQU GPIO_ODR_ODR_5
GPIO_OTYPER_ODR_6                    EQU GPIO_ODR_ODR_6
GPIO_OTYPER_ODR_7                    EQU GPIO_ODR_ODR_7
GPIO_OTYPER_ODR_8                    EQU GPIO_ODR_ODR_8
GPIO_OTYPER_ODR_9                    EQU GPIO_ODR_ODR_9
GPIO_OTYPER_ODR_10                   EQU GPIO_ODR_ODR_10
GPIO_OTYPER_ODR_11                   EQU GPIO_ODR_ODR_11
GPIO_OTYPER_ODR_12                   EQU GPIO_ODR_ODR_12
GPIO_OTYPER_ODR_13                   EQU GPIO_ODR_ODR_13
GPIO_OTYPER_ODR_14                   EQU GPIO_ODR_ODR_14
GPIO_OTYPER_ODR_15                   EQU GPIO_ODR_ODR_15

;******************  Bits definition for GPIO_BSRR register  ******************
GPIO_BSRR_BS_0                       EQU    (0x00000001)
GPIO_BSRR_BS_1                       EQU    (0x00000002)
GPIO_BSRR_BS_2                       EQU    (0x00000004)
GPIO_BSRR_BS_3                       EQU    (0x00000008)
GPIO_BSRR_BS_4                       EQU    (0x00000010)
GPIO_BSRR_BS_5                       EQU    (0x00000020)
GPIO_BSRR_BS_6                       EQU    (0x00000040)
GPIO_BSRR_BS_7                       EQU    (0x00000080)
GPIO_BSRR_BS_8                       EQU    (0x00000100)
GPIO_BSRR_BS_9                       EQU    (0x00000200)
GPIO_BSRR_BS_10                      EQU    (0x00000400)
GPIO_BSRR_BS_11                      EQU    (0x00000800)
GPIO_BSRR_BS_12                      EQU    (0x00001000)
GPIO_BSRR_BS_13                      EQU    (0x00002000)
GPIO_BSRR_BS_14                      EQU    (0x00004000)
GPIO_BSRR_BS_15                      EQU    (0x00008000)
GPIO_BSRR_BR_0                       EQU    (0x00010000)
GPIO_BSRR_BR_1                       EQU    (0x00020000)
GPIO_BSRR_BR_2                       EQU    (0x00040000)
GPIO_BSRR_BR_3                       EQU    (0x00080000)
GPIO_BSRR_BR_4                       EQU    (0x00100000)
GPIO_BSRR_BR_5                       EQU    (0x00200000)
GPIO_BSRR_BR_6                       EQU    (0x00400000)
GPIO_BSRR_BR_7                       EQU    (0x00800000)
GPIO_BSRR_BR_8                       EQU    (0x01000000)
GPIO_BSRR_BR_9                       EQU    (0x02000000)
GPIO_BSRR_BR_10                      EQU    (0x04000000)
GPIO_BSRR_BR_11                      EQU    (0x08000000)
GPIO_BSRR_BR_12                      EQU    (0x10000000)
GPIO_BSRR_BR_13                      EQU    (0x20000000)
GPIO_BSRR_BR_14                      EQU    (0x40000000)
GPIO_BSRR_BR_15                      EQU    (0x80000000)

;****************** Bit definition for GPIO_LCKR register *********************
GPIO_LCKR_LCK0                       EQU    (0x00000001)
GPIO_LCKR_LCK1                       EQU    (0x00000002)
GPIO_LCKR_LCK2                       EQU    (0x00000004)
GPIO_LCKR_LCK3                       EQU    (0x00000008)
GPIO_LCKR_LCK4                       EQU    (0x00000010)
GPIO_LCKR_LCK5                       EQU    (0x00000020)
GPIO_LCKR_LCK6                       EQU    (0x00000040)
GPIO_LCKR_LCK7                       EQU    (0x00000080)
GPIO_LCKR_LCK8                       EQU    (0x00000100)
GPIO_LCKR_LCK9                       EQU    (0x00000200)
GPIO_LCKR_LCK10                      EQU    (0x00000400)
GPIO_LCKR_LCK11                      EQU    (0x00000800)
GPIO_LCKR_LCK12                      EQU    (0x00001000)
GPIO_LCKR_LCK13                      EQU    (0x00002000)
GPIO_LCKR_LCK14                      EQU    (0x00004000)
GPIO_LCKR_LCK15                      EQU    (0x00008000)
GPIO_LCKR_LCKK                       EQU    (0x00010000)

;******************************************************************************
;*
;*                      Inter-integrated Circuit Interface
;*
;******************************************************************************
;*******************  Bit definition for I2C_CR1 register  ********************
I2C_CR1_PE                          EQU    (0x00000001)     ; Peripheral Enable
I2C_CR1_SMBUS                       EQU    (0x00000002)     ; SMBus Mode
I2C_CR1_SMBTYPE                     EQU    (0x00000008)     ; SMBus Type
I2C_CR1_ENARP                       EQU    (0x00000010)     ; ARP Enable
I2C_CR1_ENPEC                       EQU    (0x00000020)     ; PEC Enable
I2C_CR1_ENGC                        EQU    (0x00000040)     ; General Call Enable
I2C_CR1_NOSTRETCH                   EQU    (0x00000080)     ; Clock Stretching Disable (Slave mode)
I2C_CR1_START                       EQU    (0x00000100)     ; Start Generation
I2C_CR1_STOP                        EQU    (0x00000200)     ; Stop Generation
I2C_CR1_ACK                         EQU    (0x00000400)     ; Acknowledge Enable
I2C_CR1_POS                         EQU    (0x00000800)     ; Acknowledge/PEC Position (for data reception)
I2C_CR1_PEC                         EQU    (0x00001000)     ; Packet Error Checking
I2C_CR1_ALERT                       EQU    (0x00002000)     ; SMBus Alert
I2C_CR1_SWRST                       EQU    (0x00008000)     ; Software Reset

;*******************  Bit definition for I2C_CR2 register  ********************
I2C_CR2_FREQ                        EQU    (0x0000003F)     ; FREQ[5:0] bits (Peripheral Clock Frequency)
I2C_CR2_FREQ_0                      EQU    (0x00000001)     ; Bit 0
I2C_CR2_FREQ_1                      EQU    (0x00000002)     ; Bit 1
I2C_CR2_FREQ_2                      EQU    (0x00000004)     ; Bit 2
I2C_CR2_FREQ_3                      EQU    (0x00000008)     ; Bit 3
I2C_CR2_FREQ_4                      EQU    (0x00000010)     ; Bit 4
I2C_CR2_FREQ_5                      EQU    (0x00000020)     ; Bit 5

I2C_CR2_ITERREN                     EQU    (0x00000100)     ; Error Interrupt Enable
I2C_CR2_ITEVTEN                     EQU    (0x00000200)     ; Event Interrupt Enable
I2C_CR2_ITBUFEN                     EQU    (0x00000400)     ; Buffer Interrupt Enable
I2C_CR2_DMAEN                       EQU    (0x00000800)     ; DMA Requests Enable
I2C_CR2_LAST                        EQU    (0x00001000)     ; DMA Last Transfer

;*******************  Bit definition for I2C_OAR1 register  *******************
I2C_OAR1_ADD1_7                     EQU    (0x000000FE)     ; Interface Address
I2C_OAR1_ADD8_9                     EQU    (0x00000300)     ; Interface Address

I2C_OAR1_ADD0                       EQU    (0x00000001)     ; Bit 0
I2C_OAR1_ADD1                       EQU    (0x00000002)     ; Bit 1
I2C_OAR1_ADD2                       EQU    (0x00000004)     ; Bit 2
I2C_OAR1_ADD3                       EQU    (0x00000008)     ; Bit 3
I2C_OAR1_ADD4                       EQU    (0x00000010)     ; Bit 4
I2C_OAR1_ADD5                       EQU    (0x00000020)     ; Bit 5
I2C_OAR1_ADD6                       EQU    (0x00000040)     ; Bit 6
I2C_OAR1_ADD7                       EQU    (0x00000080)     ; Bit 7
I2C_OAR1_ADD8                       EQU    (0x00000100)     ; Bit 8
I2C_OAR1_ADD9                       EQU    (0x00000200)     ; Bit 9

I2C_OAR1_ADDMODE                    EQU    (0x00008000)     ; Addressing Mode (Slave mode)

;*******************  Bit definition for I2C_OAR2 register  *******************
I2C_OAR2_ENDUAL                     EQU    (0x00000001)        ; Dual addressing mode enable
I2C_OAR2_ADD2                       EQU    (0x000000FE)        ; Interface address

;********************  Bit definition for I2C_DR register  ********************
I2C_DR_DR                           EQU    (0x000000FF)        ; 8-bit Data Register

;*******************  Bit definition for I2C_SR1 register  ********************
I2C_SR1_SB                          EQU    (0x00000001)     ; Start Bit (Master mode)
I2C_SR1_ADDR                        EQU    (0x00000002)     ; Address sent (master mode)/matched (slave mode)
I2C_SR1_BTF                         EQU    (0x00000004)     ; Byte Transfer Finished
I2C_SR1_ADD10                       EQU    (0x00000008)     ; 10-bit header sent (Master mode)
I2C_SR1_STOPF                       EQU    (0x00000010)     ; Stop detection (Slave mode)
I2C_SR1_RXNE                        EQU    (0x00000040)     ; Data Register not Empty (receivers)
I2C_SR1_TXE                         EQU    (0x00000080)     ; Data Register Empty (transmitters)
I2C_SR1_BERR                        EQU    (0x00000100)     ; Bus Error
I2C_SR1_ARLO                        EQU    (0x00000200)     ; Arbitration Lost (master mode)
I2C_SR1_AF                          EQU    (0x00000400)     ; Acknowledge Failure
I2C_SR1_OVR                         EQU    (0x00000800)     ; Overrun/Underrun
I2C_SR1_PECERR                      EQU    (0x00001000)     ; PEC Error in reception
I2C_SR1_TIMEOUT                     EQU    (0x00004000)     ; Timeout or Tlow Error
I2C_SR1_SMBALERT                    EQU    (0x00008000)     ; SMBus Alert

;*******************  Bit definition for I2C_SR2 register  ********************
I2C_SR2_MSL                         EQU    (0x00000001)     ; Master/Slave
I2C_SR2_BUSY                        EQU    (0x00000002)     ; Bus Busy
I2C_SR2_TRA                         EQU    (0x00000004)     ; Transmitter/Receiver
I2C_SR2_GENCALL                     EQU    (0x00000010)     ; General Call Address (Slave mode)
I2C_SR2_SMBDEFAULT                  EQU    (0x00000020)     ; SMBus Device Default Address (Slave mode)
I2C_SR2_SMBHOST                     EQU    (0x00000040)     ; SMBus Host Header (Slave mode)
I2C_SR2_DUALF                       EQU    (0x00000080)     ; Dual Flag (Slave mode)
I2C_SR2_PEC                         EQU    (0x0000FF00)     ; Packet Error Checking Register

;*******************  Bit definition for I2C_CCR register  ********************
I2C_CCR_CCR                         EQU    (0x00000FFF)     ; Clock Control Register in Fast/Standard mode (Master mode)
I2C_CCR_DUTY                        EQU    (0x00004000)     ; Fast Mode Duty Cycle
I2C_CCR_FS                          EQU    (0x00008000)     ; I2C Master Mode Selection

;******************  Bit definition for I2C_TRISE register  *******************
I2C_TRISE_TRISE                     EQU    (0x0000003F)     ; Maximum Rise Time in Fast/Standard mode (Master mode)

;******************  Bit definition for I2C_FLTR register  *******************
I2C_FLTR_DNF                        EQU    (0x0000000F)     ; Digital Noise Filter
I2C_FLTR_ANOFF                      EQU    (0x00000010)     ; Analog Noise Filter OFF

;******************************************************************************
;*
;*                           Independent WATCHDOG
;*
;******************************************************************************
;*******************  Bit definition for IWDG_KR register  ********************
IWDG_KR_KEY                         EQU    (0xFFFF)            ; Key value (write only, read 0000h)

;*******************  Bit definition for IWDG_PR register  ********************
IWDG_PR_PR                          EQU    (0x07)               ; PR[2:0] (Prescaler divider)
IWDG_PR_PR_0                        EQU    (0x01)               ; Bit 0
IWDG_PR_PR_1                        EQU    (0x02)               ; Bit 1
IWDG_PR_PR_2                        EQU    (0x04)               ; Bit 2

;*******************  Bit definition for IWDG_RLR register  *******************
IWDG_RLR_RL                         EQU    (0x0FFF)            ; Watchdog counter reload value

;*******************  Bit definition for IWDG_SR register  ********************
IWDG_SR_PVU                         EQU    (0x01)               ; Watchdog prescaler value update
IWDG_SR_RVU                         EQU    (0x02)               ; Watchdog counter reload value update


;******************************************************************************
;*
;*                             Power Control
;*
;******************************************************************************
;********************  Bit definition for PWR_CR register  ********************
PWR_CR_LPDS                         EQU    (0x00000001)     ;  Low-Power Deepsleep
PWR_CR_PDDS                         EQU    (0x00000002)     ;  Power Down Deepsleep
PWR_CR_CWUF                         EQU    (0x00000004)     ;  Clear Wakeup Flag
PWR_CR_CSBF                         EQU    (0x00000008)     ;  Clear Standby Flag
PWR_CR_PVDE                         EQU    (0x00000010)     ;  Power Voltage Detector Enable

PWR_CR_PLS                          EQU    (0x000000E0)     ;  PLS[2:0] bits (PVD Level Selection)
PWR_CR_PLS_0                        EQU    (0x00000020)     ;  Bit 0
PWR_CR_PLS_1                        EQU    (0x00000040)     ;  Bit 1
PWR_CR_PLS_2                        EQU    (0x00000080)     ;  Bit 2

;  PVD level configuration
PWR_CR_PLS_LEV0                     EQU    (0x00000000)     ;  PVD level 0
PWR_CR_PLS_LEV1                     EQU    (0x00000020)     ;  PVD level 1
PWR_CR_PLS_LEV2                     EQU    (0x00000040)     ;  PVD level 2
PWR_CR_PLS_LEV3                     EQU    (0x00000060)     ;  PVD level 3
PWR_CR_PLS_LEV4                     EQU    (0x00000080)     ;  PVD level 4
PWR_CR_PLS_LEV5                     EQU    (0x000000A0)     ;  PVD level 5
PWR_CR_PLS_LEV6                     EQU    (0x000000C0)     ;  PVD level 6
PWR_CR_PLS_LEV7                     EQU    (0x000000E0)     ;  PVD level 7

PWR_CR_DBP                          EQU    (0x00000100)     ;  Disable Backup Domain write protection
PWR_CR_FPDS                         EQU    (0x00000200)     ;  Flash power down in Stop mode
PWR_CR_LPLVDS                       EQU    (0x00000400)     ;  Low Power Regulator Low Voltage in Deep Sleep mode
PWR_CR_MRLVDS                       EQU    (0x00000800)     ;  Main Regulator Low Voltage in Deep Sleep mode
PWR_CR_ADCDC1                       EQU    (0x00002000)     ;  Refer to AN4073 on how to use this bit
PWR_CR_VOS                          EQU    (0x0000C000)     ;  VOS[1:0] bits (Regulator voltage scaling output selection)
PWR_CR_VOS_0                        EQU    (0x00004000)     ;  Bit 0
PWR_CR_VOS_1                        EQU    (0x00008000)     ;  Bit 1

;  Legacy define
PWR_CR_PMODE                        EQU    PWR_CR_VOS

;*******************  Bit definition for PWR_CSR register  ********************
PWR_CSR_WUF                         EQU    (0x00000001)     ;  Wakeup Flag
PWR_CSR_SBF                         EQU    (0x00000002)     ;  Standby Flag
PWR_CSR_PVDO                        EQU    (0x00000004)     ;  PVD Output
PWR_CSR_BRR                         EQU    (0x00000008)     ;  Backup regulator ready
PWR_CSR_EWUP                        EQU    (0x00000100)     ;  Enable WKUP pin
PWR_CSR_BRE                         EQU    (0x00000200)     ;  Backup regulator enable
PWR_CSR_VOSRDY                      EQU    (0x00004000)     ;  Regulator voltage scaling output selection ready

;  Legacy define
PWR_CSR_REGRDY                      EQU    PWR_CSR_VOSRDY

;******************************************************************************
;*
;*                         Reset and Clock Control
;*
;******************************************************************************
;********************  Bit definition for RCC_CR register  ********************
RCC_CR_HSION                        EQU    (0x00000001)
RCC_CR_HSIRDY                       EQU    (0x00000002)

RCC_CR_HSITRIM                      EQU    (0x000000F8)
RCC_CR_HSITRIM_0                    EQU    (0x00000008); Bit 0
RCC_CR_HSITRIM_1                    EQU    (0x00000010); Bit 1
RCC_CR_HSITRIM_2                    EQU    (0x00000020); Bit 2
RCC_CR_HSITRIM_3                    EQU    (0x00000040); Bit 3
RCC_CR_HSITRIM_4                    EQU    (0x00000080); Bit 4

RCC_CR_HSICAL                       EQU    (0x0000FF00)
RCC_CR_HSICAL_0                     EQU    (0x00000100); Bit 0
RCC_CR_HSICAL_1                     EQU    (0x00000200); Bit 1
RCC_CR_HSICAL_2                     EQU    (0x00000400); Bit 2
RCC_CR_HSICAL_3                     EQU    (0x00000800); Bit 3
RCC_CR_HSICAL_4                     EQU    (0x00001000); Bit 4
RCC_CR_HSICAL_5                     EQU    (0x00002000); Bit 5
RCC_CR_HSICAL_6                     EQU    (0x00004000); Bit 6
RCC_CR_HSICAL_7                     EQU    (0x00008000); Bit 7

RCC_CR_HSEON                        EQU    (0x00010000)
RCC_CR_HSERDY                       EQU    (0x00020000)
RCC_CR_HSEBYP                       EQU    (0x00040000)
RCC_CR_CSSON                        EQU    (0x00080000)
RCC_CR_PLLON                        EQU    (0x01000000)
RCC_CR_PLLRDY                       EQU    (0x02000000)
RCC_CR_PLLI2SON                     EQU    (0x04000000)
RCC_CR_PLLI2SRDY                    EQU    (0x08000000)

;********************  Bit definition for RCC_PLLCFGR register  ***************
RCC_PLLCFGR_PLLM                    EQU    (0x0000003F)
RCC_PLLCFGR_PLLM_0                  EQU    (0x00000001)
RCC_PLLCFGR_PLLM_1                  EQU    (0x00000002)
RCC_PLLCFGR_PLLM_2                  EQU    (0x00000004)
RCC_PLLCFGR_PLLM_3                  EQU    (0x00000008)
RCC_PLLCFGR_PLLM_4                  EQU    (0x00000010)
RCC_PLLCFGR_PLLM_5                  EQU    (0x00000020)

RCC_PLLCFGR_PLLN                     EQU    (0x00007FC0)
RCC_PLLCFGR_PLLN_0                   EQU    (0x00000040)
RCC_PLLCFGR_PLLN_1                   EQU    (0x00000080)
RCC_PLLCFGR_PLLN_2                   EQU    (0x00000100)
RCC_PLLCFGR_PLLN_3                   EQU    (0x00000200)
RCC_PLLCFGR_PLLN_4                   EQU    (0x00000400)
RCC_PLLCFGR_PLLN_5                   EQU    (0x00000800)
RCC_PLLCFGR_PLLN_6                   EQU    (0x00001000)
RCC_PLLCFGR_PLLN_7                   EQU    (0x00002000)
RCC_PLLCFGR_PLLN_8                   EQU    (0x00004000)

RCC_PLLCFGR_PLLP                    EQU    (0x00030000)
RCC_PLLCFGR_PLLP_0                  EQU    (0x00010000)
RCC_PLLCFGR_PLLP_1                  EQU    (0x00020000)

RCC_PLLCFGR_PLLSRC                  EQU    (0x00400000)
RCC_PLLCFGR_PLLSRC_HSE              EQU    (0x00400000)
RCC_PLLCFGR_PLLSRC_HSI              EQU    (0x00000000)

RCC_PLLCFGR_PLLQ                    EQU    (0x0F000000)
RCC_PLLCFGR_PLLQ_0                  EQU    (0x01000000)
RCC_PLLCFGR_PLLQ_1                  EQU    (0x02000000)
RCC_PLLCFGR_PLLQ_2                  EQU    (0x04000000)
RCC_PLLCFGR_PLLQ_3                  EQU    (0x08000000)

;********************  Bit definition for RCC_CFGR register  ******************
;  SW configuration
RCC_CFGR_SW                         EQU    (0x00000003)        ;  SW[1:0] bits (System clock Switch)
RCC_CFGR_SW_0                       EQU    (0x00000001)        ;  Bit 0
RCC_CFGR_SW_1                       EQU    (0x00000002)        ;  Bit 1

RCC_CFGR_SW_HSI                     EQU    (0x00000000)        ;  HSI selected as system clock
RCC_CFGR_SW_HSE                     EQU    (0x00000001)        ;  HSE selected as system clock
RCC_CFGR_SW_PLL                     EQU    (0x00000002)        ;  PLL selected as system clock

;  SWS configuration
RCC_CFGR_SWS                        EQU    (0x0000000C)        ;  SWS[1:0] bits (System Clock Switch Status)
RCC_CFGR_SWS_0                      EQU    (0x00000004)        ;  Bit 0
RCC_CFGR_SWS_1                      EQU    (0x00000008)        ;  Bit 1

RCC_CFGR_SWS_HSI                    EQU    (0x00000000)        ;  HSI oscillator used as system clock
RCC_CFGR_SWS_HSE                    EQU    (0x00000004)        ;  HSE oscillator used as system clock
RCC_CFGR_SWS_PLL                    EQU    (0x00000008)        ;  PLL used as system clock

;  HPRE configuration
RCC_CFGR_HPRE                       EQU    (0x000000F0)        ;  HPRE[3:0] bits (AHB prescaler)
RCC_CFGR_HPRE_0                     EQU    (0x00000010)        ;  Bit 0
RCC_CFGR_HPRE_1                     EQU    (0x00000020)        ;  Bit 1
RCC_CFGR_HPRE_2                     EQU    (0x00000040)        ;  Bit 2
RCC_CFGR_HPRE_3                     EQU    (0x00000080)        ;  Bit 3

RCC_CFGR_HPRE_DIV1                  EQU    (0x00000000)        ;  SYSCLK not divided
RCC_CFGR_HPRE_DIV2                  EQU    (0x00000080)        ;  SYSCLK divided by 2
RCC_CFGR_HPRE_DIV4                  EQU    (0x00000090)        ;  SYSCLK divided by 4
RCC_CFGR_HPRE_DIV8                  EQU    (0x000000A0)        ;  SYSCLK divided by 8
RCC_CFGR_HPRE_DIV16                 EQU    (0x000000B0)        ;  SYSCLK divided by 16
RCC_CFGR_HPRE_DIV64                 EQU    (0x000000C0)        ;  SYSCLK divided by 64
RCC_CFGR_HPRE_DIV128                EQU    (0x000000D0)        ;  SYSCLK divided by 128
RCC_CFGR_HPRE_DIV256                EQU    (0x000000E0)        ;  SYSCLK divided by 256
RCC_CFGR_HPRE_DIV512                EQU    (0x000000F0)        ;  SYSCLK divided by 512

;  PPRE1 configuration
RCC_CFGR_PPRE1                      EQU    (0x00001C00)        ;  PRE1[2:0] bits (APB1 prescaler)
RCC_CFGR_PPRE1_0                    EQU    (0x00000400)        ;  Bit 0
RCC_CFGR_PPRE1_1                    EQU    (0x00000800)        ;  Bit 1
RCC_CFGR_PPRE1_2                    EQU    (0x00001000)        ;  Bit 2

RCC_CFGR_PPRE1_DIV1                 EQU    (0x00000000)        ;  HCLK not divided
RCC_CFGR_PPRE1_DIV2                 EQU    (0x00001000)        ;  HCLK divided by 2
RCC_CFGR_PPRE1_DIV4                 EQU    (0x00001400)        ;  HCLK divided by 4
RCC_CFGR_PPRE1_DIV8                 EQU    (0x00001800)        ;  HCLK divided by 8
RCC_CFGR_PPRE1_DIV16                EQU    (0x00001C00)        ;  HCLK divided by 16

;  PPRE2 configuration
RCC_CFGR_PPRE2                      EQU    (0x0000E000)        ;  PRE2[2:0] bits (APB2 prescaler)
RCC_CFGR_PPRE2_0                    EQU    (0x00002000)        ;  Bit 0
RCC_CFGR_PPRE2_1                    EQU    (0x00004000)        ;  Bit 1
RCC_CFGR_PPRE2_2                    EQU    (0x00008000)        ;  Bit 2

RCC_CFGR_PPRE2_DIV1                 EQU    (0x00000000)        ;  HCLK not divided
RCC_CFGR_PPRE2_DIV2                 EQU    (0x00008000)        ;  HCLK divided by 2
RCC_CFGR_PPRE2_DIV4                 EQU    (0x0000A000)        ;  HCLK divided by 4
RCC_CFGR_PPRE2_DIV8                 EQU    (0x0000C000)        ;  HCLK divided by 8
RCC_CFGR_PPRE2_DIV16                EQU    (0x0000E000)        ;  HCLK divided by 16

;  RTCPRE configuration
RCC_CFGR_RTCPRE                     EQU    (0x001F0000)
RCC_CFGR_RTCPRE_0                   EQU    (0x00010000)
RCC_CFGR_RTCPRE_1                   EQU    (0x00020000)
RCC_CFGR_RTCPRE_2                   EQU    (0x00040000)
RCC_CFGR_RTCPRE_3                   EQU    (0x00080000)
RCC_CFGR_RTCPRE_4                   EQU    (0x00100000)

;  MCO1 configuration
RCC_CFGR_MCO1                       EQU    (0x00600000)
RCC_CFGR_MCO1_0                     EQU    (0x00200000)
RCC_CFGR_MCO1_1                     EQU    (0x00400000)

RCC_CFGR_I2SSRC                     EQU    (0x00800000)

RCC_CFGR_MCO1PRE                    EQU    (0x07000000)
RCC_CFGR_MCO1PRE_0                  EQU    (0x01000000)
RCC_CFGR_MCO1PRE_1                  EQU    (0x02000000)
RCC_CFGR_MCO1PRE_2                  EQU    (0x04000000)

RCC_CFGR_MCO2PRE                    EQU    (0x38000000)
RCC_CFGR_MCO2PRE_0                  EQU    (0x08000000)
RCC_CFGR_MCO2PRE_1                  EQU    (0x10000000)
RCC_CFGR_MCO2PRE_2                  EQU    (0x20000000)

RCC_CFGR_MCO2                       EQU    (0xC0000000)
RCC_CFGR_MCO2_0                     EQU    (0x40000000)
RCC_CFGR_MCO2_1                     EQU    (0x80000000)

;********************  Bit definition for RCC_CIR register  *******************
RCC_CIR_LSIRDYF                     EQU    (0x00000001)
RCC_CIR_LSERDYF                     EQU    (0x00000002)
RCC_CIR_HSIRDYF                     EQU    (0x00000004)
RCC_CIR_HSERDYF                     EQU    (0x00000008)
RCC_CIR_PLLRDYF                     EQU    (0x00000010)
RCC_CIR_PLLI2SRDYF                  EQU    (0x00000020)

RCC_CIR_CSSF                        EQU    (0x00000080)
RCC_CIR_LSIRDYIE                    EQU    (0x00000100)
RCC_CIR_LSERDYIE                    EQU    (0x00000200)
RCC_CIR_HSIRDYIE                    EQU    (0x00000400)
RCC_CIR_HSERDYIE                    EQU    (0x00000800)
RCC_CIR_PLLRDYIE                    EQU    (0x00001000)
RCC_CIR_PLLI2SRDYIE                 EQU    (0x00002000)

RCC_CIR_LSIRDYC                     EQU    (0x00010000)
RCC_CIR_LSERDYC                     EQU    (0x00020000)
RCC_CIR_HSIRDYC                     EQU    (0x00040000)
RCC_CIR_HSERDYC                     EQU    (0x00080000)
RCC_CIR_PLLRDYC                     EQU    (0x00100000)
RCC_CIR_PLLI2SRDYC                  EQU    (0x00200000)

RCC_CIR_CSSC                        EQU    (0x00800000)

;********************  Bit definition for RCC_AHB1RSTR register  **************
RCC_AHB1RSTR_GPIOARST               EQU    (0x00000001)
RCC_AHB1RSTR_GPIOBRST               EQU    (0x00000002)
RCC_AHB1RSTR_GPIOCRST               EQU    (0x00000004)
RCC_AHB1RSTR_GPIODRST               EQU    (0x00000008)
RCC_AHB1RSTR_GPIOERST               EQU    (0x00000010)
RCC_AHB1RSTR_GPIOHRST               EQU    (0x00000080)
RCC_AHB1RSTR_CRCRST                 EQU    (0x00001000)
RCC_AHB1RSTR_DMA1RST                EQU    (0x00200000)
RCC_AHB1RSTR_DMA2RST                EQU    (0x00400000)

;********************  Bit definition for RCC_AHB2RSTR register  **************
RCC_AHB2RSTR_OTGFSRST               EQU    (0x00000080)

;********************  Bit definition for RCC_AHB3RSTR register  **************

;********************  Bit definition for RCC_APB1RSTR register  **************
RCC_APB1RSTR_TIM2RST                EQU    (0x00000001)
RCC_APB1RSTR_TIM3RST                EQU    (0x00000002)
RCC_APB1RSTR_TIM4RST                EQU    (0x00000004)
RCC_APB1RSTR_TIM5RST                EQU    (0x00000008)
RCC_APB1RSTR_WWDGRST                EQU    (0x00000800)
RCC_APB1RSTR_SPI2RST                EQU    (0x00004000)
RCC_APB1RSTR_SPI3RST                EQU    (0x00008000)
RCC_APB1RSTR_USART2RST              EQU    (0x00020000)
RCC_APB1RSTR_I2C1RST                EQU    (0x00200000)
RCC_APB1RSTR_I2C2RST                EQU    (0x00400000)
RCC_APB1RSTR_I2C3RST                EQU    (0x00800000)
RCC_APB1RSTR_PWRRST                 EQU    (0x10000000)

;********************  Bit definition for RCC_APB2RSTR register  **************
RCC_APB2RSTR_TIM1RST                EQU    (0x00000001)
RCC_APB2RSTR_USART1RST              EQU    (0x00000010)
RCC_APB2RSTR_USART6RST              EQU    (0x00000020)
RCC_APB2RSTR_ADCRST                 EQU    (0x00000100)
RCC_APB2RSTR_SDIORST                EQU    (0x00000800)
RCC_APB2RSTR_SPI1RST                EQU    (0x00001000)
RCC_APB2RSTR_SPI4RST                EQU    (0x00002000)
RCC_APB2RSTR_SYSCFGRST              EQU    (0x00004000)
RCC_APB2RSTR_TIM9RST                EQU    (0x00010000)
RCC_APB2RSTR_TIM10RST               EQU    (0x00020000)
RCC_APB2RSTR_TIM11RST               EQU    (0x00040000)

;  Old SPI1RST bit definition, maintained for legacy purpose
RCC_APB2RSTR_SPI1                   EQU RCC_APB2RSTR_SPI1RST

;********************  Bit definition for RCC_AHB1ENR register  ***************
RCC_AHB1ENR_GPIOAEN                 EQU    (0x00000001)
RCC_AHB1ENR_GPIOBEN                 EQU    (0x00000002)
RCC_AHB1ENR_GPIOCEN                 EQU    (0x00000004)
RCC_AHB1ENR_GPIODEN                 EQU    (0x00000008)
RCC_AHB1ENR_GPIOEEN                 EQU    (0x00000010)
RCC_AHB1ENR_GPIOHEN                 EQU    (0x00000080)
RCC_AHB1ENR_CRCEN                   EQU    (0x00001000)
RCC_AHB1ENR_BKPSRAMEN               EQU    (0x00040000)
RCC_AHB1ENR_CCMDATARAMEN            EQU    (0x00100000)
RCC_AHB1ENR_DMA1EN                  EQU    (0x00200000)
RCC_AHB1ENR_DMA2EN                  EQU    (0x00400000)

;********************  Bit definition for RCC_AHB2ENR register  ***************
RCC_AHB2ENR_OTGFSEN                 EQU    (0x00000080)

;********************  Bit definition for RCC_AHB3ENR register  ***************

;********************  Bit definition for RCC_APB1ENR register  ***************
RCC_APB1ENR_TIM2EN                  EQU    (0x00000001)
RCC_APB1ENR_TIM3EN                  EQU    (0x00000002)
RCC_APB1ENR_TIM4EN                  EQU    (0x00000004)
RCC_APB1ENR_TIM5EN                  EQU    (0x00000008)
RCC_APB1ENR_WWDGEN                  EQU    (0x00000800)
RCC_APB1ENR_SPI2EN                  EQU    (0x00004000)
RCC_APB1ENR_SPI3EN                  EQU    (0x00008000)
RCC_APB1ENR_USART2EN                EQU    (0x00020000)
RCC_APB1ENR_I2C1EN                  EQU    (0x00200000)
RCC_APB1ENR_I2C2EN                  EQU    (0x00400000)
RCC_APB1ENR_I2C3EN                  EQU    (0x00800000)
RCC_APB1ENR_PWREN                   EQU    (0x10000000)

;********************  Bit definition for RCC_APB2ENR register  ***************
RCC_APB2ENR_TIM1EN                  EQU    (0x00000001)
RCC_APB2ENR_USART1EN                EQU    (0x00000010)
RCC_APB2ENR_USART6EN                EQU    (0x00000020)
RCC_APB2ENR_ADC1EN                  EQU    (0x00000100)
RCC_APB2ENR_SDIOEN                  EQU    (0x00000800)
RCC_APB2ENR_SPI1EN                  EQU    (0x00001000)
RCC_APB2ENR_SPI4EN                  EQU    (0x00002000)
RCC_APB2ENR_SYSCFGEN                EQU    (0x00004000)
RCC_APB2ENR_TIM9EN                  EQU    (0x00010000)
RCC_APB2ENR_TIM10EN                 EQU    (0x00020000)
RCC_APB2ENR_TIM11EN                 EQU    (0x00040000)

;********************  Bit definition for RCC_AHB1LPENR register  *************
RCC_AHB1LPENR_GPIOALPEN             EQU    (0x00000001)
RCC_AHB1LPENR_GPIOBLPEN             EQU    (0x00000002)
RCC_AHB1LPENR_GPIOCLPEN             EQU    (0x00000004)
RCC_AHB1LPENR_GPIODLPEN             EQU    (0x00000008)
RCC_AHB1LPENR_GPIOELPEN             EQU    (0x00000010)
RCC_AHB1LPENR_GPIOHLPEN             EQU    (0x00000080)
RCC_AHB1LPENR_CRCLPEN               EQU    (0x00001000)
RCC_AHB1LPENR_FLITFLPEN             EQU    (0x00008000)
RCC_AHB1LPENR_SRAM1LPEN             EQU    (0x00010000)
RCC_AHB1LPENR_SRAM2LPEN             EQU    (0x00020000)
RCC_AHB1LPENR_BKPSRAMLPEN           EQU    (0x00040000)
RCC_AHB1LPENR_SRAM3LPEN             EQU    (0x00080000)
RCC_AHB1LPENR_DMA1LPEN              EQU    (0x00200000)
RCC_AHB1LPENR_DMA2LPEN              EQU    (0x00400000)

;********************  Bit definition for RCC_AHB2LPENR register  *************
RCC_AHB2LPENR_OTGFSLPEN             EQU    (0x00000080)

;********************  Bit definition for RCC_AHB3LPENR register  *************

;********************  Bit definition for RCC_APB1LPENR register  *************
RCC_APB1LPENR_TIM2LPEN              EQU    (0x00000001)
RCC_APB1LPENR_TIM3LPEN              EQU    (0x00000002)
RCC_APB1LPENR_TIM4LPEN              EQU    (0x00000004)
RCC_APB1LPENR_TIM5LPEN              EQU    (0x00000008)
RCC_APB1LPENR_WWDGLPEN              EQU    (0x00000800)
RCC_APB1LPENR_SPI2LPEN              EQU    (0x00004000)
RCC_APB1LPENR_SPI3LPEN              EQU    (0x00008000)
RCC_APB1LPENR_USART2LPEN            EQU    (0x00020000)
RCC_APB1LPENR_I2C1LPEN              EQU    (0x00200000)
RCC_APB1LPENR_I2C2LPEN              EQU    (0x00400000)
RCC_APB1LPENR_I2C3LPEN              EQU    (0x00800000)
RCC_APB1LPENR_PWRLPEN               EQU    (0x10000000)
RCC_APB1LPENR_DACLPEN               EQU    (0x20000000)

;********************  Bit definition for RCC_APB2LPENR register  *************
RCC_APB2LPENR_TIM1LPEN              EQU    (0x00000001)
RCC_APB2LPENR_USART1LPEN            EQU    (0x00000010)
RCC_APB2LPENR_USART6LPEN            EQU    (0x00000020)
RCC_APB2LPENR_ADC1LPEN              EQU    (0x00000100)
RCC_APB2LPENR_SDIOLPEN              EQU    (0x00000800)
RCC_APB2LPENR_SPI1LPEN              EQU    (0x00001000)
RCC_APB2LPENR_SPI4LPEN              EQU    (0x00002000)
RCC_APB2LPENR_SYSCFGLPEN            EQU    (0x00004000)
RCC_APB2LPENR_TIM9LPEN              EQU    (0x00010000)
RCC_APB2LPENR_TIM10LPEN             EQU    (0x00020000)
RCC_APB2LPENR_TIM11LPEN             EQU    (0x00040000)

;********************  Bit definition for RCC_BDCR register  ******************
RCC_BDCR_LSEON                      EQU    (0x00000001)
RCC_BDCR_LSERDY                     EQU    (0x00000002)
RCC_BDCR_LSEBYP                     EQU    (0x00000004)

RCC_BDCR_RTCSEL                    EQU    (0x00000300)
RCC_BDCR_RTCSEL_0                  EQU    (0x00000100)
RCC_BDCR_RTCSEL_1                  EQU    (0x00000200)

RCC_BDCR_RTCEN                      EQU    (0x00008000)
RCC_BDCR_BDRST                      EQU    (0x00010000)

;********************  Bit definition for RCC_CSR register  *******************
RCC_CSR_LSION                       EQU    (0x00000001)
RCC_CSR_LSIRDY                      EQU    (0x00000002)
RCC_CSR_RMVF                        EQU    (0x01000000)
RCC_CSR_BORRSTF                     EQU    (0x02000000)
RCC_CSR_PADRSTF                     EQU    (0x04000000)
RCC_CSR_PORRSTF                     EQU    (0x08000000)
RCC_CSR_SFTRSTF                     EQU    (0x10000000)
RCC_CSR_WDGRSTF                     EQU    (0x20000000)
RCC_CSR_WWDGRSTF                    EQU    (0x40000000)
RCC_CSR_LPWRRSTF                    EQU    (0x80000000)

;********************  Bit definition for RCC_SSCGR register  *****************
RCC_SSCGR_MODPER                    EQU    (0x00001FFF)
RCC_SSCGR_INCSTEP                   EQU    (0x0FFFE000)
RCC_SSCGR_SPREADSEL                 EQU    (0x40000000)
RCC_SSCGR_SSCGEN                    EQU    (0x80000000)

;********************  Bit definition for RCC_PLLI2SCFGR register  ************
RCC_PLLI2SCFGR_PLLI2SN              EQU    (0x00007FC0)
RCC_PLLI2SCFGR_PLLI2SN_0            EQU    (0x00000040)
RCC_PLLI2SCFGR_PLLI2SN_1            EQU    (0x00000080)
RCC_PLLI2SCFGR_PLLI2SN_2            EQU    (0x00000100)
RCC_PLLI2SCFGR_PLLI2SN_3            EQU    (0x00000200)
RCC_PLLI2SCFGR_PLLI2SN_4            EQU    (0x00000400)
RCC_PLLI2SCFGR_PLLI2SN_5            EQU    (0x00000800)
RCC_PLLI2SCFGR_PLLI2SN_6            EQU    (0x00001000)
RCC_PLLI2SCFGR_PLLI2SN_7            EQU    (0x00002000)
RCC_PLLI2SCFGR_PLLI2SN_8            EQU    (0x00004000)

RCC_PLLI2SCFGR_PLLI2SR              EQU    (0x70000000)
RCC_PLLI2SCFGR_PLLI2SR_0            EQU    (0x10000000)
RCC_PLLI2SCFGR_PLLI2SR_1            EQU    (0x20000000)
RCC_PLLI2SCFGR_PLLI2SR_2            EQU    (0x40000000)

;******************************************************************************
;*
;*                           Real-Time Clock (RTC)
;*
;******************************************************************************
;********************  Bits definition for RTC_TR register  *******************
RTC_TR_PM                            EQU    (0x00400000)
RTC_TR_HT                            EQU    (0x00300000)
RTC_TR_HT_0                          EQU    (0x00100000)
RTC_TR_HT_1                          EQU    (0x00200000)
RTC_TR_HU                            EQU    (0x000F0000)
RTC_TR_HU_0                          EQU    (0x00010000)
RTC_TR_HU_1                          EQU    (0x00020000)
RTC_TR_HU_2                          EQU    (0x00040000)
RTC_TR_HU_3                          EQU    (0x00080000)
RTC_TR_MNT                           EQU    (0x00007000)
RTC_TR_MNT_0                         EQU    (0x00001000)
RTC_TR_MNT_1                         EQU    (0x00002000)
RTC_TR_MNT_2                         EQU    (0x00004000)
RTC_TR_MNU                           EQU    (0x00000F00)
RTC_TR_MNU_0                         EQU    (0x00000100)
RTC_TR_MNU_1                         EQU    (0x00000200)
RTC_TR_MNU_2                         EQU    (0x00000400)
RTC_TR_MNU_3                         EQU    (0x00000800)
RTC_TR_ST                            EQU    (0x00000070)
RTC_TR_ST_0                          EQU    (0x00000010)
RTC_TR_ST_1                          EQU    (0x00000020)
RTC_TR_ST_2                          EQU    (0x00000040)
RTC_TR_SU                            EQU    (0x0000000F)
RTC_TR_SU_0                          EQU    (0x00000001)
RTC_TR_SU_1                          EQU    (0x00000002)
RTC_TR_SU_2                          EQU    (0x00000004)
RTC_TR_SU_3                          EQU    (0x00000008)

;********************  Bits definition for RTC_DR register  *******************
RTC_DR_YT                            EQU    (0x00F00000)
RTC_DR_YT_0                          EQU    (0x00100000)
RTC_DR_YT_1                          EQU    (0x00200000)
RTC_DR_YT_2                          EQU    (0x00400000)
RTC_DR_YT_3                          EQU    (0x00800000)
RTC_DR_YU                            EQU    (0x000F0000)
RTC_DR_YU_0                          EQU    (0x00010000)
RTC_DR_YU_1                          EQU    (0x00020000)
RTC_DR_YU_2                          EQU    (0x00040000)
RTC_DR_YU_3                          EQU    (0x00080000)
RTC_DR_WDU                           EQU    (0x0000E000)
RTC_DR_WDU_0                         EQU    (0x00002000)
RTC_DR_WDU_1                         EQU    (0x00004000)
RTC_DR_WDU_2                         EQU    (0x00008000)
RTC_DR_MT                            EQU    (0x00001000)
RTC_DR_MU                            EQU    (0x00000F00)
RTC_DR_MU_0                          EQU    (0x00000100)
RTC_DR_MU_1                          EQU    (0x00000200)
RTC_DR_MU_2                          EQU    (0x00000400)
RTC_DR_MU_3                          EQU    (0x00000800)
RTC_DR_DT                            EQU    (0x00000030)
RTC_DR_DT_0                          EQU    (0x00000010)
RTC_DR_DT_1                          EQU    (0x00000020)
RTC_DR_DU                            EQU    (0x0000000F)
RTC_DR_DU_0                          EQU    (0x00000001)
RTC_DR_DU_1                          EQU    (0x00000002)
RTC_DR_DU_2                          EQU    (0x00000004)
RTC_DR_DU_3                          EQU    (0x00000008)

;********************  Bits definition for RTC_CR register  *******************
RTC_CR_COE                           EQU    (0x00800000)
RTC_CR_OSEL                          EQU    (0x00600000)
RTC_CR_OSEL_0                        EQU    (0x00200000)
RTC_CR_OSEL_1                        EQU    (0x00400000)
RTC_CR_POL                           EQU    (0x00100000)
RTC_CR_COSEL                         EQU    (0x00080000)
RTC_CR_BCK                           EQU    (0x00040000)
RTC_CR_SUB1H                         EQU    (0x00020000)
RTC_CR_ADD1H                         EQU    (0x00010000)
RTC_CR_TSIE                          EQU    (0x00008000)
RTC_CR_WUTIE                         EQU    (0x00004000)
RTC_CR_ALRBIE                        EQU    (0x00002000)
RTC_CR_ALRAIE                        EQU    (0x00001000)
RTC_CR_TSE                           EQU    (0x00000800)
RTC_CR_WUTE                          EQU    (0x00000400)
RTC_CR_ALRBE                         EQU    (0x00000200)
RTC_CR_ALRAE                         EQU    (0x00000100)
RTC_CR_DCE                           EQU    (0x00000080)
RTC_CR_FMT                           EQU    (0x00000040)
RTC_CR_BYPSHAD                       EQU    (0x00000020)
RTC_CR_REFCKON                       EQU    (0x00000010)
RTC_CR_TSEDGE                        EQU    (0x00000008)
RTC_CR_WUCKSEL                       EQU    (0x00000007)
RTC_CR_WUCKSEL_0                     EQU    (0x00000001)
RTC_CR_WUCKSEL_1                     EQU    (0x00000002)
RTC_CR_WUCKSEL_2                     EQU    (0x00000004)

;********************  Bits definition for RTC_ISR register  ******************
RTC_ISR_RECALPF                      EQU    (0x00010000)
RTC_ISR_TAMP1F                       EQU    (0x00002000)
RTC_ISR_TAMP2F                       EQU    (0x00004000)
RTC_ISR_TSOVF                        EQU    (0x00001000)
RTC_ISR_TSF                          EQU    (0x00000800)
RTC_ISR_WUTF                         EQU    (0x00000400)
RTC_ISR_ALRBF                        EQU    (0x00000200)
RTC_ISR_ALRAF                        EQU    (0x00000100)
RTC_ISR_INIT                         EQU    (0x00000080)
RTC_ISR_INITF                        EQU    (0x00000040)
RTC_ISR_RSF                          EQU    (0x00000020)
RTC_ISR_INITS                        EQU    (0x00000010)
RTC_ISR_SHPF                         EQU    (0x00000008)
RTC_ISR_WUTWF                        EQU    (0x00000004)
RTC_ISR_ALRBWF                       EQU    (0x00000002)
RTC_ISR_ALRAWF                       EQU    (0x00000001)

;********************  Bits definition for RTC_PRER register  *****************
RTC_PRER_PREDIV_A                    EQU    (0x007F0000)
RTC_PRER_PREDIV_S                    EQU    (0x00001FFF)

;********************  Bits definition for RTC_WUTR register  *****************
RTC_WUTR_WUT                         EQU    (0x0000FFFF)

;********************  Bits definition for RTC_CALIBR register  ***************
RTC_CALIBR_DCS                       EQU    (0x00000080)
RTC_CALIBR_DC                        EQU    (0x0000001F)

;********************  Bits definition for RTC_ALRMAR register  ***************
RTC_ALRMAR_MSK4                      EQU    (0x80000000)
RTC_ALRMAR_WDSEL                     EQU    (0x40000000)
RTC_ALRMAR_DT                        EQU    (0x30000000)
RTC_ALRMAR_DT_0                      EQU    (0x10000000)
RTC_ALRMAR_DT_1                      EQU    (0x20000000)
RTC_ALRMAR_DU                        EQU    (0x0F000000)
RTC_ALRMAR_DU_0                      EQU    (0x01000000)
RTC_ALRMAR_DU_1                      EQU    (0x02000000)
RTC_ALRMAR_DU_2                      EQU    (0x04000000)
RTC_ALRMAR_DU_3                      EQU    (0x08000000)
RTC_ALRMAR_MSK3                      EQU    (0x00800000)
RTC_ALRMAR_PM                        EQU    (0x00400000)
RTC_ALRMAR_HT                        EQU    (0x00300000)
RTC_ALRMAR_HT_0                      EQU    (0x00100000)
RTC_ALRMAR_HT_1                      EQU    (0x00200000)
RTC_ALRMAR_HU                        EQU    (0x000F0000)
RTC_ALRMAR_HU_0                      EQU    (0x00010000)
RTC_ALRMAR_HU_1                      EQU    (0x00020000)
RTC_ALRMAR_HU_2                      EQU    (0x00040000)
RTC_ALRMAR_HU_3                      EQU    (0x00080000)
RTC_ALRMAR_MSK2                      EQU    (0x00008000)
RTC_ALRMAR_MNT                       EQU    (0x00007000)
RTC_ALRMAR_MNT_0                     EQU    (0x00001000)
RTC_ALRMAR_MNT_1                     EQU    (0x00002000)
RTC_ALRMAR_MNT_2                     EQU    (0x00004000)
RTC_ALRMAR_MNU                       EQU    (0x00000F00)
RTC_ALRMAR_MNU_0                     EQU    (0x00000100)
RTC_ALRMAR_MNU_1                     EQU    (0x00000200)
RTC_ALRMAR_MNU_2                     EQU    (0x00000400)
RTC_ALRMAR_MNU_3                     EQU    (0x00000800)
RTC_ALRMAR_MSK1                      EQU    (0x00000080)
RTC_ALRMAR_ST                        EQU    (0x00000070)
RTC_ALRMAR_ST_0                      EQU    (0x00000010)
RTC_ALRMAR_ST_1                      EQU    (0x00000020)
RTC_ALRMAR_ST_2                      EQU    (0x00000040)
RTC_ALRMAR_SU                        EQU    (0x0000000F)
RTC_ALRMAR_SU_0                      EQU    (0x00000001)
RTC_ALRMAR_SU_1                      EQU    (0x00000002)
RTC_ALRMAR_SU_2                      EQU    (0x00000004)
RTC_ALRMAR_SU_3                      EQU    (0x00000008)

;********************  Bits definition for RTC_ALRMBR register  ***************
RTC_ALRMBR_MSK4                      EQU    (0x80000000)
RTC_ALRMBR_WDSEL                     EQU    (0x40000000)
RTC_ALRMBR_DT                        EQU    (0x30000000)
RTC_ALRMBR_DT_0                      EQU    (0x10000000)
RTC_ALRMBR_DT_1                      EQU    (0x20000000)
RTC_ALRMBR_DU                        EQU    (0x0F000000)
RTC_ALRMBR_DU_0                      EQU    (0x01000000)
RTC_ALRMBR_DU_1                      EQU    (0x02000000)
RTC_ALRMBR_DU_2                      EQU    (0x04000000)
RTC_ALRMBR_DU_3                      EQU    (0x08000000)
RTC_ALRMBR_MSK3                      EQU    (0x00800000)
RTC_ALRMBR_PM                        EQU    (0x00400000)
RTC_ALRMBR_HT                        EQU    (0x00300000)
RTC_ALRMBR_HT_0                      EQU    (0x00100000)
RTC_ALRMBR_HT_1                      EQU    (0x00200000)
RTC_ALRMBR_HU                        EQU    (0x000F0000)
RTC_ALRMBR_HU_0                      EQU    (0x00010000)
RTC_ALRMBR_HU_1                      EQU    (0x00020000)
RTC_ALRMBR_HU_2                      EQU    (0x00040000)
RTC_ALRMBR_HU_3                      EQU    (0x00080000)
RTC_ALRMBR_MSK2                      EQU    (0x00008000)
RTC_ALRMBR_MNT                       EQU    (0x00007000)
RTC_ALRMBR_MNT_0                     EQU    (0x00001000)
RTC_ALRMBR_MNT_1                     EQU    (0x00002000)
RTC_ALRMBR_MNT_2                     EQU    (0x00004000)
RTC_ALRMBR_MNU                       EQU    (0x00000F00)
RTC_ALRMBR_MNU_0                     EQU    (0x00000100)
RTC_ALRMBR_MNU_1                     EQU    (0x00000200)
RTC_ALRMBR_MNU_2                     EQU    (0x00000400)
RTC_ALRMBR_MNU_3                     EQU    (0x00000800)
RTC_ALRMBR_MSK1                      EQU    (0x00000080)
RTC_ALRMBR_ST                        EQU    (0x00000070)
RTC_ALRMBR_ST_0                      EQU    (0x00000010)
RTC_ALRMBR_ST_1                      EQU    (0x00000020)
RTC_ALRMBR_ST_2                      EQU    (0x00000040)
RTC_ALRMBR_SU                        EQU    (0x0000000F)
RTC_ALRMBR_SU_0                      EQU    (0x00000001)
RTC_ALRMBR_SU_1                      EQU    (0x00000002)
RTC_ALRMBR_SU_2                      EQU    (0x00000004)
RTC_ALRMBR_SU_3                      EQU    (0x00000008)

;********************  Bits definition for RTC_WPR register  ******************
RTC_WPR_KEY                          EQU    (0x000000FF)

;********************  Bits definition for RTC_SSR register  ******************
RTC_SSR_SS                           EQU    (0x0000FFFF)

;********************  Bits definition for RTC_SHIFTR register  ***************
RTC_SHIFTR_SUBFS                     EQU    (0x00007FFF)
RTC_SHIFTR_ADD1S                     EQU    (0x80000000)

;********************  Bits definition for RTC_TSTR register  *****************
RTC_TSTR_PM                          EQU    (0x00400000)
RTC_TSTR_HT                          EQU    (0x00300000)
RTC_TSTR_HT_0                        EQU    (0x00100000)
RTC_TSTR_HT_1                        EQU    (0x00200000)
RTC_TSTR_HU                          EQU    (0x000F0000)
RTC_TSTR_HU_0                        EQU    (0x00010000)
RTC_TSTR_HU_1                        EQU    (0x00020000)
RTC_TSTR_HU_2                        EQU    (0x00040000)
RTC_TSTR_HU_3                        EQU    (0x00080000)
RTC_TSTR_MNT                         EQU    (0x00007000)
RTC_TSTR_MNT_0                       EQU    (0x00001000)
RTC_TSTR_MNT_1                       EQU    (0x00002000)
RTC_TSTR_MNT_2                       EQU    (0x00004000)
RTC_TSTR_MNU                         EQU    (0x00000F00)
RTC_TSTR_MNU_0                       EQU    (0x00000100)
RTC_TSTR_MNU_1                       EQU    (0x00000200)
RTC_TSTR_MNU_2                       EQU    (0x00000400)
RTC_TSTR_MNU_3                       EQU    (0x00000800)
RTC_TSTR_ST                          EQU    (0x00000070)
RTC_TSTR_ST_0                        EQU    (0x00000010)
RTC_TSTR_ST_1                        EQU    (0x00000020)
RTC_TSTR_ST_2                        EQU    (0x00000040)
RTC_TSTR_SU                          EQU    (0x0000000F)
RTC_TSTR_SU_0                        EQU    (0x00000001)
RTC_TSTR_SU_1                        EQU    (0x00000002)
RTC_TSTR_SU_2                        EQU    (0x00000004)
RTC_TSTR_SU_3                        EQU    (0x00000008)

;********************  Bits definition for RTC_TSDR register  *****************
RTC_TSDR_WDU                         EQU    (0x0000E000)
RTC_TSDR_WDU_0                       EQU    (0x00002000)
RTC_TSDR_WDU_1                       EQU    (0x00004000)
RTC_TSDR_WDU_2                       EQU    (0x00008000)
RTC_TSDR_MT                          EQU    (0x00001000)
RTC_TSDR_MU                          EQU    (0x00000F00)
RTC_TSDR_MU_0                        EQU    (0x00000100)
RTC_TSDR_MU_1                        EQU    (0x00000200)
RTC_TSDR_MU_2                        EQU    (0x00000400)
RTC_TSDR_MU_3                        EQU    (0x00000800)
RTC_TSDR_DT                          EQU    (0x00000030)
RTC_TSDR_DT_0                        EQU    (0x00000010)
RTC_TSDR_DT_1                        EQU    (0x00000020)
RTC_TSDR_DU                          EQU    (0x0000000F)
RTC_TSDR_DU_0                        EQU    (0x00000001)
RTC_TSDR_DU_1                        EQU    (0x00000002)
RTC_TSDR_DU_2                        EQU    (0x00000004)
RTC_TSDR_DU_3                        EQU    (0x00000008)

;********************  Bits definition for RTC_TSSSR register  ****************
RTC_TSSSR_SS                         EQU    (0x0000FFFF)

;********************  Bits definition for RTC_CAL register  *****************
RTC_CALR_CALP                        EQU    (0x00008000)
RTC_CALR_CALW8                       EQU    (0x00004000)
RTC_CALR_CALW16                      EQU    (0x00002000)
RTC_CALR_CALM                        EQU    (0x000001FF)
RTC_CALR_CALM_0                      EQU    (0x00000001)
RTC_CALR_CALM_1                      EQU    (0x00000002)
RTC_CALR_CALM_2                      EQU    (0x00000004)
RTC_CALR_CALM_3                      EQU    (0x00000008)
RTC_CALR_CALM_4                      EQU    (0x00000010)
RTC_CALR_CALM_5                      EQU    (0x00000020)
RTC_CALR_CALM_6                      EQU    (0x00000040)
RTC_CALR_CALM_7                      EQU    (0x00000080)
RTC_CALR_CALM_8                      EQU    (0x00000100)

;********************  Bits definition for RTC_TAFCR register  ****************
RTC_TAFCR_ALARMOUTTYPE               EQU    (0x00040000)
RTC_TAFCR_TSINSEL                    EQU    (0x00020000)
RTC_TAFCR_TAMPINSEL                  EQU    (0x00010000)
RTC_TAFCR_TAMPPUDIS                  EQU    (0x00008000)
RTC_TAFCR_TAMPPRCH                   EQU    (0x00006000)
RTC_TAFCR_TAMPPRCH_0                 EQU    (0x00002000)
RTC_TAFCR_TAMPPRCH_1                 EQU    (0x00004000)
RTC_TAFCR_TAMPFLT                    EQU    (0x00001800)
RTC_TAFCR_TAMPFLT_0                  EQU    (0x00000800)
RTC_TAFCR_TAMPFLT_1                  EQU    (0x00001000)
RTC_TAFCR_TAMPFREQ                   EQU    (0x00000700)
RTC_TAFCR_TAMPFREQ_0                 EQU    (0x00000100)
RTC_TAFCR_TAMPFREQ_1                 EQU    (0x00000200)
RTC_TAFCR_TAMPFREQ_2                 EQU    (0x00000400)
RTC_TAFCR_TAMPTS                     EQU    (0x00000080)
RTC_TAFCR_TAMP2TRG                   EQU    (0x00000010)
RTC_TAFCR_TAMP2E                     EQU    (0x00000008)
RTC_TAFCR_TAMPIE                     EQU    (0x00000004)
RTC_TAFCR_TAMP1TRG                   EQU    (0x00000002)
RTC_TAFCR_TAMP1E                     EQU    (0x00000001)

;********************  Bits definition for RTC_ALRMASSR register  *************
RTC_ALRMASSR_MASKSS                  EQU    (0x0F000000)
RTC_ALRMASSR_MASKSS_0                EQU    (0x01000000)
RTC_ALRMASSR_MASKSS_1                EQU    (0x02000000)
RTC_ALRMASSR_MASKSS_2                EQU    (0x04000000)
RTC_ALRMASSR_MASKSS_3                EQU    (0x08000000)
RTC_ALRMASSR_SS                      EQU    (0x00007FFF)

;********************  Bits definition for RTC_ALRMBSSR register  *************
RTC_ALRMBSSR_MASKSS                  EQU    (0x0F000000)
RTC_ALRMBSSR_MASKSS_0                EQU    (0x01000000)
RTC_ALRMBSSR_MASKSS_1                EQU    (0x02000000)
RTC_ALRMBSSR_MASKSS_2                EQU    (0x04000000)
RTC_ALRMBSSR_MASKSS_3                EQU    (0x08000000)
RTC_ALRMBSSR_SS                      EQU    (0x00007FFF)

;********************  Bits definition for RTC_BKP0R register  ****************
; RTC_BKP0R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP1R register  ****************
; RTC_BKP1R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP2R register  ****************
; RTC_BKP2R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP3R register  ****************
; RTC_BKP3R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP4R register  ****************
; RTC_BKP4R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP5R register  ****************
; RTC_BKP5R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP6R register  ****************
; RTC_BKP6R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP7R register  ****************
; RTC_BKP7R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP8R register  ****************
; RTC_BKP8R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP9R register  ****************
; RTC_BKP9R                            EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP10R register  ***************
; RTC_BKP10R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP11R register  ***************
; RTC_BKP11R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP12R register  ***************
; RTC_BKP12R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP13R register  ***************
; RTC_BKP13R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP14R register  ***************
; RTC_BKP14R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP15R register  ***************
; RTC_BKP15R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP16R register  ***************
; RTC_BKP16R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP17R register  ***************
; RTC_BKP17R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP18R register  ***************
; RTC_BKP18R                           EQU    (0xFFFFFFFF)

;********************  Bits definition for RTC_BKP19R register  ***************
; RTC_BKP19R                           EQU    (0xFFFFFFFF)



;******************************************************************************
;*
;*                          SD host Interface
;*
;******************************************************************************
;******************  Bit definition for SDIO_POWER register  ******************
SDIO_POWER_PWRCTRL                  EQU    (0x03)               ; PWRCTRL[1:0] bits (Power supply control bits)
SDIO_POWER_PWRCTRL_0                EQU    (0x01)               ; Bit 0
SDIO_POWER_PWRCTRL_1                EQU    (0x02)               ; Bit 1

;******************  Bit definition for SDIO_CLKCR register  ******************
SDIO_CLKCR_CLKDIV                   EQU    (0x00FF)            ; Clock divide factor
SDIO_CLKCR_CLKEN                    EQU    (0x0100)            ; Clock enable bit
SDIO_CLKCR_PWRSAV                   EQU    (0x0200)            ; Power saving configuration bit
SDIO_CLKCR_BYPASS                   EQU    (0x0400)            ; Clock divider bypass enable bit

SDIO_CLKCR_WIDBUS                   EQU    (0x1800)            ; WIDBUS[1:0] bits (Wide bus mode enable bit)
SDIO_CLKCR_WIDBUS_0                 EQU    (0x0800)            ; Bit 0
SDIO_CLKCR_WIDBUS_1                 EQU    (0x1000)            ; Bit 1

SDIO_CLKCR_NEGEDGE                  EQU    (0x2000)            ; SDIO_CK dephasing selection bit
SDIO_CLKCR_HWFC_EN                  EQU    (0x4000)            ; HW Flow Control enable

;*******************  Bit definition for SDIO_ARG register  *******************
SDIO_ARG_CMDARG                     EQU    (0xFFFFFFFF)            ; Command argument

;*******************  Bit definition for SDIO_CMD register  *******************
SDIO_CMD_CMDINDEX                   EQU    (0x003F)            ; Command Index

SDIO_CMD_WAITRESP                   EQU    (0x00C0)            ; WAITRESP[1:0] bits (Wait for response bits)
SDIO_CMD_WAITRESP_0                 EQU    (0x0040)            ;  Bit 0
SDIO_CMD_WAITRESP_1                 EQU    (0x0080)            ;  Bit 1

SDIO_CMD_WAITINT                    EQU    (0x0100)            ; CPSM Waits for Interrupt Request
SDIO_CMD_WAITPEND                   EQU    (0x0200)            ; CPSM Waits for ends of data transfer (CmdPend internal signal)
SDIO_CMD_CPSMEN                     EQU    (0x0400)            ; Command path state machine (CPSM) Enable bit
SDIO_CMD_SDIOSUSPEND                EQU    (0x0800)            ; SD I/O suspend command
SDIO_CMD_ENCMDCOMPL                 EQU    (0x1000)            ; Enable CMD completion
SDIO_CMD_NIEN                       EQU    (0x2000)            ; Not Interrupt Enable
SDIO_CMD_CEATACMD                   EQU    (0x4000)            ; CE-ATA command

;****************  Bit definition for SDIO_RESPCMD register  *****************
SDIO_RESPCMD_RESPCMD                EQU    (0x3F)               ; Response command index

;******************  Bit definition for SDIO_RESP0 register  ******************
SDIO_RESP0_CARDSTATUS0              EQU    (0xFFFFFFFF)        ; Card Status

;******************  Bit definition for SDIO_RESP1 register  ******************
SDIO_RESP1_CARDSTATUS1              EQU    (0xFFFFFFFF)        ; Card Status

;******************  Bit definition for SDIO_RESP2 register  ******************
SDIO_RESP2_CARDSTATUS2              EQU    (0xFFFFFFFF)        ; Card Status

;******************  Bit definition for SDIO_RESP3 register  ******************
SDIO_RESP3_CARDSTATUS3              EQU    (0xFFFFFFFF)        ; Card Status

;******************  Bit definition for SDIO_RESP4 register  ******************
SDIO_RESP4_CARDSTATUS4              EQU    (0xFFFFFFFF)        ; Card Status

;******************  Bit definition for SDIO_DTIMER register  *****************
SDIO_DTIMER_DATATIME                EQU    (0xFFFFFFFF)        ; Data timeout period.

;******************  Bit definition for SDIO_DLEN register  *******************
SDIO_DLEN_DATALENGTH                EQU    (0x01FFFFFF)        ; Data length value

;******************  Bit definition for SDIO_DCTRL register  ******************
SDIO_DCTRL_DTEN                     EQU    (0x0001)            ; Data transfer enabled bit
SDIO_DCTRL_DTDIR                    EQU    (0x0002)            ; Data transfer direction selection
SDIO_DCTRL_DTMODE                   EQU    (0x0004)            ; Data transfer mode selection
SDIO_DCTRL_DMAEN                    EQU    (0x0008)            ; DMA enabled bit

SDIO_DCTRL_DBLOCKSIZE               EQU    (0x00F0)            ; DBLOCKSIZE[3:0] bits (Data block size)
SDIO_DCTRL_DBLOCKSIZE_0             EQU    (0x0010)            ; Bit 0
SDIO_DCTRL_DBLOCKSIZE_1             EQU    (0x0020)            ; Bit 1
SDIO_DCTRL_DBLOCKSIZE_2             EQU    (0x0040)            ; Bit 2
SDIO_DCTRL_DBLOCKSIZE_3             EQU    (0x0080)            ; Bit 3

SDIO_DCTRL_RWSTART                  EQU    (0x0100)            ; Read wait start
SDIO_DCTRL_RWSTOP                   EQU    (0x0200)            ; Read wait stop
SDIO_DCTRL_RWMOD                    EQU    (0x0400)            ; Read wait mode
SDIO_DCTRL_SDIOEN                   EQU    (0x0800)            ; SD I/O enable functions

;******************  Bit definition for SDIO_DCOUNT register  *****************
SDIO_DCOUNT_DATACOUNT               EQU    (0x01FFFFFF)        ; Data count value

;******************  Bit definition for SDIO_STA register  ********************
SDIO_STA_CCRCFAIL                   EQU    (0x00000001)        ; Command response received (CRC check failed)
SDIO_STA_DCRCFAIL                   EQU    (0x00000002)        ; Data block sent/received (CRC check failed)
SDIO_STA_CTIMEOUT                   EQU    (0x00000004)        ; Command response timeout
SDIO_STA_DTIMEOUT                   EQU    (0x00000008)        ; Data timeout
SDIO_STA_TXUNDERR                   EQU    (0x00000010)        ; Transmit FIFO underrun error
SDIO_STA_RXOVERR                    EQU    (0x00000020)        ; Received FIFO overrun error
SDIO_STA_CMDREND                    EQU    (0x00000040)        ; Command response received (CRC check passed)
SDIO_STA_CMDSENT                    EQU    (0x00000080)        ; Command sent (no response required)
SDIO_STA_DATAEND                    EQU    (0x00000100)        ; Data end (data counter, SDIDCOUNT, is zero)
SDIO_STA_STBITERR                   EQU    (0x00000200)        ; Start bit not detected on all data signals in wide bus mode
SDIO_STA_DBCKEND                    EQU    (0x00000400)        ; Data block sent/received (CRC check passed)
SDIO_STA_CMDACT                     EQU    (0x00000800)        ; Command transfer in progress
SDIO_STA_TXACT                      EQU    (0x00001000)        ; Data transmit in progress
SDIO_STA_RXACT                      EQU    (0x00002000)        ; Data receive in progress
SDIO_STA_TXFIFOHE                   EQU    (0x00004000)        ; Transmit FIFO Half Empty: at least 8 words can be written into the FIFO
SDIO_STA_RXFIFOHF                   EQU    (0x00008000)        ; Receive FIFO Half Full: there are at least 8 words in the FIFO
SDIO_STA_TXFIFOF                    EQU    (0x00010000)        ; Transmit FIFO full
SDIO_STA_RXFIFOF                    EQU    (0x00020000)        ; Receive FIFO full
SDIO_STA_TXFIFOE                    EQU    (0x00040000)        ; Transmit FIFO empty
SDIO_STA_RXFIFOE                    EQU    (0x00080000)        ; Receive FIFO empty
SDIO_STA_TXDAVL                     EQU    (0x00100000)        ; Data available in transmit FIFO
SDIO_STA_RXDAVL                     EQU    (0x00200000)        ; Data available in receive FIFO
SDIO_STA_SDIOIT                     EQU    (0x00400000)        ; SDIO interrupt received
SDIO_STA_CEATAEND                   EQU    (0x00800000)        ; CE-ATA command completion signal received for CMD61

;*******************  Bit definition for SDIO_ICR register  *******************
SDIO_ICR_CCRCFAILC                  EQU    (0x00000001)        ; CCRCFAIL flag clear bit
SDIO_ICR_DCRCFAILC                  EQU    (0x00000002)        ; DCRCFAIL flag clear bit
SDIO_ICR_CTIMEOUTC                  EQU    (0x00000004)        ; CTIMEOUT flag clear bit
SDIO_ICR_DTIMEOUTC                  EQU    (0x00000008)        ; DTIMEOUT flag clear bit
SDIO_ICR_TXUNDERRC                  EQU    (0x00000010)        ; TXUNDERR flag clear bit
SDIO_ICR_RXOVERRC                   EQU    (0x00000020)        ; RXOVERR flag clear bit
SDIO_ICR_CMDRENDC                   EQU    (0x00000040)        ; CMDREND flag clear bit
SDIO_ICR_CMDSENTC                   EQU    (0x00000080)        ; CMDSENT flag clear bit
SDIO_ICR_DATAENDC                   EQU    (0x00000100)        ; DATAEND flag clear bit
SDIO_ICR_STBITERRC                  EQU    (0x00000200)        ; STBITERR flag clear bit
SDIO_ICR_DBCKENDC                   EQU    (0x00000400)        ; DBCKEND flag clear bit
SDIO_ICR_SDIOITC                    EQU    (0x00400000)        ; SDIOIT flag clear bit
SDIO_ICR_CEATAENDC                  EQU    (0x00800000)        ; CEATAEND flag clear bit

;******************  Bit definition for SDIO_MASK register  *******************
SDIO_MASK_CCRCFAILIE                EQU    (0x00000001)        ; Command CRC Fail Interrupt Enable
SDIO_MASK_DCRCFAILIE                EQU    (0x00000002)        ; Data CRC Fail Interrupt Enable
SDIO_MASK_CTIMEOUTIE                EQU    (0x00000004)        ; Command TimeOut Interrupt Enable
SDIO_MASK_DTIMEOUTIE                EQU    (0x00000008)        ; Data TimeOut Interrupt Enable
SDIO_MASK_TXUNDERRIE                EQU    (0x00000010)        ; Tx FIFO UnderRun Error Interrupt Enable
SDIO_MASK_RXOVERRIE                 EQU    (0x00000020)        ; Rx FIFO OverRun Error Interrupt Enable
SDIO_MASK_CMDRENDIE                 EQU    (0x00000040)        ; Command Response Received Interrupt Enable
SDIO_MASK_CMDSENTIE                 EQU    (0x00000080)        ; Command Sent Interrupt Enable
SDIO_MASK_DATAENDIE                 EQU    (0x00000100)        ; Data End Interrupt Enable
SDIO_MASK_STBITERRIE                EQU    (0x00000200)        ; Start Bit Error Interrupt Enable
SDIO_MASK_DBCKENDIE                 EQU    (0x00000400)        ; Data Block End Interrupt Enable
SDIO_MASK_CMDACTIE                  EQU    (0x00000800)        ; CCommand Acting Interrupt Enable
SDIO_MASK_TXACTIE                   EQU    (0x00001000)        ; Data Transmit Acting Interrupt Enable
SDIO_MASK_RXACTIE                   EQU    (0x00002000)        ; Data receive acting interrupt enabled
SDIO_MASK_TXFIFOHEIE                EQU    (0x00004000)        ; Tx FIFO Half Empty interrupt Enable
SDIO_MASK_RXFIFOHFIE                EQU    (0x00008000)        ; Rx FIFO Half Full interrupt Enable
SDIO_MASK_TXFIFOFIE                 EQU    (0x00010000)        ; Tx FIFO Full interrupt Enable
SDIO_MASK_RXFIFOFIE                 EQU    (0x00020000)        ; Rx FIFO Full interrupt Enable
SDIO_MASK_TXFIFOEIE                 EQU    (0x00040000)        ; Tx FIFO Empty interrupt Enable
SDIO_MASK_RXFIFOEIE                 EQU    (0x00080000)        ; Rx FIFO Empty interrupt Enable
SDIO_MASK_TXDAVLIE                  EQU    (0x00100000)        ; Data available in Tx FIFO interrupt Enable
SDIO_MASK_RXDAVLIE                  EQU    (0x00200000)        ; Data available in Rx FIFO interrupt Enable
SDIO_MASK_SDIOITIE                  EQU    (0x00400000)        ; SDIO Mode Interrupt Received interrupt Enable
SDIO_MASK_CEATAENDIE                EQU    (0x00800000)        ; CE-ATA command completion signal received Interrupt Enable

;****************  Bit definition for SDIO_FIFOCNT register  *****************
SDIO_FIFOCNT_FIFOCOUNT              EQU    (0x00FFFFFF)        ; Remaining number of words to be written to or read from the FIFO

;******************  Bit definition for SDIO_FIFO register  *******************
SDIO_FIFO_FIFODATA                  EQU    (0xFFFFFFFF)        ; Receive and transmit FIFO data

;******************************************************************************
;*
;*                        Serial Peripheral Interface
;*
;******************************************************************************
;*******************  Bit definition for SPI_CR1 register  ********************
SPI_CR1_CPHA                        EQU    (0x00000001)            ; Clock Phase
SPI_CR1_CPOL                        EQU    (0x00000002)            ; Clock Polarity
SPI_CR1_MSTR                        EQU    (0x00000004)            ; Master Selection

SPI_CR1_BR                          EQU    (0x00000038)            ; BR[2:0] bits (Baud Rate Control)
SPI_CR1_BR_0                        EQU    (0x00000008)            ; Bit 0
SPI_CR1_BR_1                        EQU    (0x00000010)            ; Bit 1
SPI_CR1_BR_2                        EQU    (0x00000020)            ; Bit 2

SPI_CR1_SPE                         EQU    (0x00000040)            ; SPI Enable
SPI_CR1_LSBFIRST                    EQU    (0x00000080)            ; Frame Format
SPI_CR1_SSI                         EQU    (0x00000100)            ; Internal slave select
SPI_CR1_SSM                         EQU    (0x00000200)            ; Software slave management
SPI_CR1_RXONLY                      EQU    (0x00000400)            ; Receive only
SPI_CR1_DFF                         EQU    (0x00000800)            ; Data Frame Format
SPI_CR1_CRCNEXT                     EQU    (0x00001000)            ; Transmit CRC next
SPI_CR1_CRCEN                       EQU    (0x00002000)            ; Hardware CRC calculation enable
SPI_CR1_BIDIOE                      EQU    (0x00004000)            ; Output enable in bidirectional mode
SPI_CR1_BIDIMODE                    EQU    (0x00008000)            ; Bidirectional data mode enable

;*******************  Bit definition for SPI_CR2 register  ********************
SPI_CR2_RXDMAEN                     EQU    (0x00000001)               ; Rx Buffer DMA Enable
SPI_CR2_TXDMAEN                     EQU    (0x00000002)               ; Tx Buffer DMA Enable
SPI_CR2_SSOE                        EQU    (0x00000004)               ; SS Output Enable
SPI_CR2_FRF                         EQU    (0x00000010)               ; Frame Format
SPI_CR2_ERRIE                       EQU    (0x00000020)               ; Error Interrupt Enable
SPI_CR2_RXNEIE                      EQU    (0x00000040)               ; RX buffer Not Empty Interrupt Enable
SPI_CR2_TXEIE                       EQU    (0x00000080)               ; Tx buffer Empty Interrupt Enable

;********************  Bit definition for SPI_SR register  ********************
SPI_SR_RXNE                         EQU    (0x00000001)               ; Receive buffer Not Empty
SPI_SR_TXE                          EQU    (0x00000002)               ; Transmit buffer Empty
SPI_SR_CHSIDE                       EQU    (0x00000004)               ; Channel side
SPI_SR_UDR                          EQU    (0x00000008)               ; Underrun flag
SPI_SR_CRCERR                       EQU    (0x00000010)               ; CRC Error flag
SPI_SR_MODF                         EQU    (0x00000020)               ; Mode fault
SPI_SR_OVR                          EQU    (0x00000040)               ; Overrun flag
SPI_SR_BSY                          EQU    (0x00000080)               ; Busy flag
SPI_SR_FRE                          EQU    (0x00000100)               ; Frame format error flag

;********************  Bit definition for SPI_DR register  ********************
SPI_DR_DR                           EQU    (0x0000FFFF)            ; Data Register

;*******************  Bit definition for SPI_CRCPR register  ******************
SPI_CRCPR_CRCPOLY                   EQU    (0x0000FFFF)            ; CRC polynomial register

;******************  Bit definition for SPI_RXCRCR register  ******************
SPI_RXCRCR_RXCRC                    EQU    (0x0000FFFF)            ; Rx CRC Register

;******************  Bit definition for SPI_TXCRCR register  ******************
SPI_TXCRCR_TXCRC                    EQU    (0x0000FFFF)            ; Tx CRC Register

;******************  Bit definition for SPI_I2SCFGR register  *****************
SPI_I2SCFGR_CHLEN                   EQU    (0x00000001)            ; Channel length (number of bits per audio channel)

SPI_I2SCFGR_DATLEN                  EQU    (0x00000006)            ; DATLEN[1:0] bits (Data length to be transferred)
SPI_I2SCFGR_DATLEN_0                EQU    (0x00000002)            ; Bit 0
SPI_I2SCFGR_DATLEN_1                EQU    (0x00000004)            ; Bit 1

SPI_I2SCFGR_CKPOL                   EQU    (0x00000008)            ; steady state clock polarity

SPI_I2SCFGR_I2SSTD                  EQU    (0x00000030)            ; I2SSTD[1:0] bits (I2S standard selection)
SPI_I2SCFGR_I2SSTD_0                EQU    (0x00000010)            ; Bit 0
SPI_I2SCFGR_I2SSTD_1                EQU    (0x00000020)            ; Bit 1

SPI_I2SCFGR_PCMSYNC                 EQU    (0x00000080)            ; PCM frame synchronization

SPI_I2SCFGR_I2SCFG                  EQU    (0x00000300)            ; I2SCFG[1:0] bits (I2S configuration mode)
SPI_I2SCFGR_I2SCFG_0                EQU    (0x00000100)            ; Bit 0
SPI_I2SCFGR_I2SCFG_1                EQU    (0x00000200)            ; Bit 1

SPI_I2SCFGR_I2SE                    EQU    (0x00000400)            ; I2S Enable
SPI_I2SCFGR_I2SMOD                  EQU    (0x00000800)            ; I2S mode selection

;******************  Bit definition for SPI_I2SPR register  *******************
SPI_I2SPR_I2SDIV                    EQU    (0x000000FF)            ; I2S Linear prescaler
SPI_I2SPR_ODD                       EQU    (0x00000100)            ; Odd factor for the prescaler
SPI_I2SPR_MCKOE                     EQU    (0x00000200)            ; Master Clock Output Enable

;******************************************************************************
;*
;*                                 SYSCFG
;*
;******************************************************************************
;******************  Bit definition for SYSCFG_MEMRMP register  ***************
SYSCFG_MEMRMP_MEM_MODE          EQU    (0x00000007) ;  SYSCFG_Memory Remap Config
SYSCFG_MEMRMP_MEM_MODE_0        EQU    (0x00000001)
SYSCFG_MEMRMP_MEM_MODE_1        EQU    (0x00000002)
SYSCFG_MEMRMP_MEM_MODE_2        EQU    (0x00000004)

;******************  Bit definition for SYSCFG_PMC register  ******************
SYSCFG_PMC_ADC1DC2              EQU    (0x00010000) ;  Refer to AN4073 on how to use this bit

;****************  Bit definition for SYSCFG_EXTICR1 register  ***************
SYSCFG_EXTICR1_EXTI0            EQU    (0x000F) ; EXTI 0 configuration
SYSCFG_EXTICR1_EXTI1            EQU    (0x00F0) ; EXTI 1 configuration
SYSCFG_EXTICR1_EXTI2            EQU    (0x0F00) ; EXTI 2 configuration
SYSCFG_EXTICR1_EXTI3            EQU    (0xF000) ; EXTI 3 configuration

;   EXTI0 configuration

SYSCFG_EXTICR1_EXTI0_PA         EQU    (0x0000) ; PA[0] pin
SYSCFG_EXTICR1_EXTI0_PB         EQU    (0x0001) ; PB[0] pin
SYSCFG_EXTICR1_EXTI0_PC         EQU    (0x0002) ; PC[0] pin
SYSCFG_EXTICR1_EXTI0_PD         EQU    (0x0003) ; PD[0] pin
SYSCFG_EXTICR1_EXTI0_PE         EQU    (0x0004) ; PE[0] pin
SYSCFG_EXTICR1_EXTI0_PH         EQU    (0x0007) ; PH[0] pin


;   EXTI1 configuration

SYSCFG_EXTICR1_EXTI1_PA         EQU    (0x0000) ; PA[1] pin
SYSCFG_EXTICR1_EXTI1_PB         EQU    (0x0010) ; PB[1] pin
SYSCFG_EXTICR1_EXTI1_PC         EQU    (0x0020) ; PC[1] pin
SYSCFG_EXTICR1_EXTI1_PD         EQU    (0x0030) ; PD[1] pin
SYSCFG_EXTICR1_EXTI1_PE         EQU    (0x0040) ; PE[1] pin
SYSCFG_EXTICR1_EXTI1_PH         EQU    (0x0070) ; PH[1] pin


;   EXTI2 configuration

SYSCFG_EXTICR1_EXTI2_PA         EQU    (0x0000) ; PA[2] pin
SYSCFG_EXTICR1_EXTI2_PB         EQU    (0x0100) ; PB[2] pin
SYSCFG_EXTICR1_EXTI2_PC         EQU    (0x0200) ; PC[2] pin
SYSCFG_EXTICR1_EXTI2_PD         EQU    (0x0300) ; PD[2] pin
SYSCFG_EXTICR1_EXTI2_PE         EQU    (0x0400) ; PE[2] pin
SYSCFG_EXTICR1_EXTI2_PH         EQU    (0x0700) ; PH[2] pin


;   EXTI3 configuration

SYSCFG_EXTICR1_EXTI3_PA         EQU    (0x0000) ; PA[3] pin
SYSCFG_EXTICR1_EXTI3_PB         EQU    (0x1000) ; PB[3] pin
SYSCFG_EXTICR1_EXTI3_PC         EQU    (0x2000) ; PC[3] pin
SYSCFG_EXTICR1_EXTI3_PD         EQU    (0x3000) ; PD[3] pin
SYSCFG_EXTICR1_EXTI3_PE         EQU    (0x4000) ; PE[3] pin
SYSCFG_EXTICR1_EXTI3_PH         EQU    (0x7000) ; PH[3] pin

;****************  Bit definition for SYSCFG_EXTICR2 register  ***************
SYSCFG_EXTICR2_EXTI4            EQU    (0x000F) ; EXTI 4 configuration
SYSCFG_EXTICR2_EXTI5            EQU    (0x00F0) ; EXTI 5 configuration
SYSCFG_EXTICR2_EXTI6            EQU    (0x0F00) ; EXTI 6 configuration
SYSCFG_EXTICR2_EXTI7            EQU    (0xF000) ; EXTI 7 configuration

;   EXTI4 configuration

SYSCFG_EXTICR2_EXTI4_PA         EQU    (0x0000) ; PA[4] pin
SYSCFG_EXTICR2_EXTI4_PB         EQU    (0x0001) ; PB[4] pin
SYSCFG_EXTICR2_EXTI4_PC         EQU    (0x0002) ; PC[4] pin
SYSCFG_EXTICR2_EXTI4_PD         EQU    (0x0003) ; PD[4] pin
SYSCFG_EXTICR2_EXTI4_PE         EQU    (0x0004) ; PE[4] pin
SYSCFG_EXTICR2_EXTI4_PH         EQU    (0x0007) ; PH[4] pin


;   EXTI5 configuration

SYSCFG_EXTICR2_EXTI5_PA         EQU    (0x0000) ; PA[5] pin
SYSCFG_EXTICR2_EXTI5_PB         EQU    (0x0010) ; PB[5] pin
SYSCFG_EXTICR2_EXTI5_PC         EQU    (0x0020) ; PC[5] pin
SYSCFG_EXTICR2_EXTI5_PD         EQU    (0x0030) ; PD[5] pin
SYSCFG_EXTICR2_EXTI5_PE         EQU    (0x0040) ; PE[5] pin
SYSCFG_EXTICR2_EXTI5_PH         EQU    (0x0070) ; PH[5] pin


;   EXTI6 configuration

SYSCFG_EXTICR2_EXTI6_PA         EQU    (0x0000) ; PA[6] pin
SYSCFG_EXTICR2_EXTI6_PB         EQU    (0x0100) ; PB[6] pin
SYSCFG_EXTICR2_EXTI6_PC         EQU    (0x0200) ; PC[6] pin
SYSCFG_EXTICR2_EXTI6_PD         EQU    (0x0300) ; PD[6] pin
SYSCFG_EXTICR2_EXTI6_PE         EQU    (0x0400) ; PE[6] pin
SYSCFG_EXTICR2_EXTI6_PH         EQU    (0x0700) ; PH[6] pin


;   EXTI7 configuration

SYSCFG_EXTICR2_EXTI7_PA         EQU    (0x0000) ; PA[7] pin
SYSCFG_EXTICR2_EXTI7_PB         EQU    (0x1000) ; PB[7] pin
SYSCFG_EXTICR2_EXTI7_PC         EQU    (0x2000) ; PC[7] pin
SYSCFG_EXTICR2_EXTI7_PD         EQU    (0x3000) ; PD[7] pin
SYSCFG_EXTICR2_EXTI7_PE         EQU    (0x4000) ; PE[7] pin
SYSCFG_EXTICR2_EXTI7_PH         EQU    (0x7000) ; PH[7] pin


;****************  Bit definition for SYSCFG_EXTICR3 register  ***************
SYSCFG_EXTICR3_EXTI8            EQU    (0x000F) ; EXTI 8 configuration
SYSCFG_EXTICR3_EXTI9            EQU    (0x00F0) ; EXTI 9 configuration
SYSCFG_EXTICR3_EXTI10           EQU    (0x0F00) ; EXTI 10 configuration
SYSCFG_EXTICR3_EXTI11           EQU    (0xF000) ; EXTI 11 configuration


;   EXTI8 configuration

SYSCFG_EXTICR3_EXTI8_PA         EQU    (0x0000) ; PA[8] pin
SYSCFG_EXTICR3_EXTI8_PB         EQU    (0x0001) ; PB[8] pin
SYSCFG_EXTICR3_EXTI8_PC         EQU    (0x0002) ; PC[8] pin
SYSCFG_EXTICR3_EXTI8_PD         EQU    (0x0003) ; PD[8] pin
SYSCFG_EXTICR3_EXTI8_PE         EQU    (0x0004) ; PE[8] pin
SYSCFG_EXTICR3_EXTI8_PH         EQU    (0x0007) ; PH[8] pin


;   EXTI9 configuration

SYSCFG_EXTICR3_EXTI9_PA         EQU    (0x0000) ; PA[9] pin
SYSCFG_EXTICR3_EXTI9_PB         EQU    (0x0010) ; PB[9] pin
SYSCFG_EXTICR3_EXTI9_PC         EQU    (0x0020) ; PC[9] pin
SYSCFG_EXTICR3_EXTI9_PD         EQU    (0x0030) ; PD[9] pin
SYSCFG_EXTICR3_EXTI9_PE         EQU    (0x0040) ; PE[9] pin
SYSCFG_EXTICR3_EXTI9_PH         EQU    (0x0070) ; PH[9] pin


;   EXTI10 configuration

SYSCFG_EXTICR3_EXTI10_PA        EQU    (0x0000) ; PA[10] pin
SYSCFG_EXTICR3_EXTI10_PB        EQU    (0x0100) ; PB[10] pin
SYSCFG_EXTICR3_EXTI10_PC        EQU    (0x0200) ; PC[10] pin
SYSCFG_EXTICR3_EXTI10_PD        EQU    (0x0300) ; PD[10] pin
SYSCFG_EXTICR3_EXTI10_PE        EQU    (0x0400) ; PE[10] pin
SYSCFG_EXTICR3_EXTI10_PH        EQU    (0x0700) ; PH[10] pin


;   EXTI11 configuration

SYSCFG_EXTICR3_EXTI11_PA        EQU    (0x0000) ; PA[11] pin
SYSCFG_EXTICR3_EXTI11_PB        EQU    (0x1000) ; PB[11] pin
SYSCFG_EXTICR3_EXTI11_PC        EQU    (0x2000) ; PC[11] pin
SYSCFG_EXTICR3_EXTI11_PD        EQU    (0x3000) ; PD[11] pin
SYSCFG_EXTICR3_EXTI11_PE        EQU    (0x4000) ; PE[11] pin
SYSCFG_EXTICR3_EXTI11_PH        EQU    (0x7000) ; PH[11] pin

;****************  Bit definition for SYSCFG_EXTICR4 register  ***************

SYSCFG_EXTICR4_EXTI12           EQU    (0x000F) ; EXTI 12 configuration
SYSCFG_EXTICR4_EXTI13           EQU    (0x00F0) ; EXTI 13 configuration
SYSCFG_EXTICR4_EXTI14           EQU    (0x0F00) ; EXTI 14 configuration
SYSCFG_EXTICR4_EXTI15           EQU    (0xF000) ; EXTI 15 configuration

;   EXTI12 configuration

SYSCFG_EXTICR4_EXTI12_PA        EQU    (0x0000) ; PA[12] pin
SYSCFG_EXTICR4_EXTI12_PB        EQU    (0x0001) ; PB[12] pin
SYSCFG_EXTICR4_EXTI12_PC        EQU    (0x0002) ; PC[12] pin
SYSCFG_EXTICR4_EXTI12_PD        EQU    (0x0003) ; PD[12] pin
SYSCFG_EXTICR4_EXTI12_PE        EQU    (0x0004) ; PE[12] pin
SYSCFG_EXTICR4_EXTI12_PH        EQU    (0x0007) ; PH[12] pin


;   EXTI13 configuration

SYSCFG_EXTICR4_EXTI13_PA        EQU    (0x0000) ; PA[13] pin
SYSCFG_EXTICR4_EXTI13_PB        EQU    (0x0010) ; PB[13] pin
SYSCFG_EXTICR4_EXTI13_PC        EQU    (0x0020) ; PC[13] pin
SYSCFG_EXTICR4_EXTI13_PD        EQU    (0x0030) ; PD[13] pin
SYSCFG_EXTICR4_EXTI13_PE        EQU    (0x0040) ; PE[13] pin
SYSCFG_EXTICR4_EXTI13_PH        EQU    (0x0070) ; PH[13] pin


;   EXTI14 configuration

SYSCFG_EXTICR4_EXTI14_PA        EQU    (0x0000) ; PA[14] pin
SYSCFG_EXTICR4_EXTI14_PB        EQU    (0x0100) ; PB[14] pin
SYSCFG_EXTICR4_EXTI14_PC        EQU    (0x0200) ; PC[14] pin
SYSCFG_EXTICR4_EXTI14_PD        EQU    (0x0300) ; PD[14] pin
SYSCFG_EXTICR4_EXTI14_PE        EQU    (0x0400) ; PE[14] pin
SYSCFG_EXTICR4_EXTI14_PH        EQU    (0x0700) ; PH[14] pin


;   EXTI15 configuration

SYSCFG_EXTICR4_EXTI15_PA        EQU    (0x0000) ; PA[15] pin
SYSCFG_EXTICR4_EXTI15_PB        EQU    (0x1000) ; PB[15] pin
SYSCFG_EXTICR4_EXTI15_PC        EQU    (0x2000) ; PC[15] pin
SYSCFG_EXTICR4_EXTI15_PD        EQU    (0x3000) ; PD[15] pin
SYSCFG_EXTICR4_EXTI15_PE        EQU    (0x4000) ; PE[15] pin
SYSCFG_EXTICR4_EXTI15_PH        EQU    (0x7000) ; PH[15] pin

;******************  Bit definition for SYSCFG_CMPCR register  ****************
SYSCFG_CMPCR_CMP_PD             EQU    (0x00000001) ; Compensation cell ready flag
SYSCFG_CMPCR_READY              EQU    (0x00000100) ; Compensation cell power-down

;******************************************************************************
;*
;*                                    TIM
;*
;******************************************************************************
;*******************  Bit definition for TIM_CR1 register  ********************
TIM_CR1_CEN                         EQU    (0x0001)            ; Counter enable
TIM_CR1_UDIS                        EQU    (0x0002)            ; Update disable
TIM_CR1_URS                         EQU    (0x0004)            ; Update request source
TIM_CR1_OPM                         EQU    (0x0008)            ; One pulse mode
TIM_CR1_DIR                         EQU    (0x0010)            ; Direction

TIM_CR1_CMS                         EQU    (0x0060)            ; CMS[1:0] bits (Center-aligned mode selection)
TIM_CR1_CMS_0                       EQU    (0x0020)            ; Bit 0
TIM_CR1_CMS_1                       EQU    (0x0040)            ; Bit 1

TIM_CR1_ARPE                        EQU    (0x0080)            ; Auto-reload preload enable

TIM_CR1_CKD                         EQU    (0x0300)            ; CKD[1:0] bits (clock division)
TIM_CR1_CKD_0                       EQU    (0x0100)            ; Bit 0
TIM_CR1_CKD_1                       EQU    (0x0200)            ; Bit 1

;*******************  Bit definition for TIM_CR2 register  ********************
TIM_CR2_CCPC                        EQU    (0x0001)            ; Capture/Compare Preloaded Control
TIM_CR2_CCUS                        EQU    (0x0004)            ; Capture/Compare Control Update Selection
TIM_CR2_CCDS                        EQU    (0x0008)            ; Capture/Compare DMA Selection

TIM_CR2_MMS                         EQU    (0x0070)            ; MMS[2:0] bits (Master Mode Selection)
TIM_CR2_MMS_0                       EQU    (0x0010)            ; Bit 0
TIM_CR2_MMS_1                       EQU    (0x0020)            ; Bit 1
TIM_CR2_MMS_2                       EQU    (0x0040)            ; Bit 2

TIM_CR2_TI1S                        EQU    (0x0080)            ; TI1 Selection
TIM_CR2_OIS1                        EQU    (0x0100)            ; Output Idle state 1 (OC1 output)
TIM_CR2_OIS1N                       EQU    (0x0200)            ; Output Idle state 1 (OC1N output)
TIM_CR2_OIS2                        EQU    (0x0400)            ; Output Idle state 2 (OC2 output)
TIM_CR2_OIS2N                       EQU    (0x0800)            ; Output Idle state 2 (OC2N output)
TIM_CR2_OIS3                        EQU    (0x1000)            ; Output Idle state 3 (OC3 output)
TIM_CR2_OIS3N                       EQU    (0x2000)            ; Output Idle state 3 (OC3N output)
TIM_CR2_OIS4                        EQU    (0x4000)            ; Output Idle state 4 (OC4 output)

;*******************  Bit definition for TIM_SMCR register  *******************
TIM_SMCR_SMS                        EQU    (0x0007)            ; SMS[2:0] bits (Slave mode selection)
TIM_SMCR_SMS_0                      EQU    (0x0001)            ; Bit 0
TIM_SMCR_SMS_1                      EQU    (0x0002)            ; Bit 1
TIM_SMCR_SMS_2                      EQU    (0x0004)            ; Bit 2

TIM_SMCR_TS                         EQU    (0x0070)            ; TS[2:0] bits (Trigger selection)
TIM_SMCR_TS_0                       EQU    (0x0010)            ; Bit 0
TIM_SMCR_TS_1                       EQU    (0x0020)            ; Bit 1
TIM_SMCR_TS_2                       EQU    (0x0040)            ; Bit 2

TIM_SMCR_MSM                        EQU    (0x0080)            ; Master/slave mode

TIM_SMCR_ETF                        EQU    (0x0F00)            ; ETF[3:0] bits (External trigger filter)
TIM_SMCR_ETF_0                      EQU    (0x0100)            ; Bit 0
TIM_SMCR_ETF_1                      EQU    (0x0200)            ; Bit 1
TIM_SMCR_ETF_2                      EQU    (0x0400)            ; Bit 2
TIM_SMCR_ETF_3                      EQU    (0x0800)            ; Bit 3

TIM_SMCR_ETPS                       EQU    (0x3000)            ; ETPS[1:0] bits (External trigger prescaler)
TIM_SMCR_ETPS_0                     EQU    (0x1000)            ; Bit 0
TIM_SMCR_ETPS_1                     EQU    (0x2000)            ; Bit 1

TIM_SMCR_ECE                        EQU    (0x4000)            ; External clock enable
TIM_SMCR_ETP                        EQU    (0x8000)            ; External trigger polarity

;*******************  Bit definition for TIM_DIER register  *******************
TIM_DIER_UIE                        EQU    (0x0001)            ; Update interrupt enable
TIM_DIER_CC1IE                      EQU    (0x0002)            ; Capture/Compare 1 interrupt enable
TIM_DIER_CC2IE                      EQU    (0x0004)            ; Capture/Compare 2 interrupt enable
TIM_DIER_CC3IE                      EQU    (0x0008)            ; Capture/Compare 3 interrupt enable
TIM_DIER_CC4IE                      EQU    (0x0010)            ; Capture/Compare 4 interrupt enable
TIM_DIER_COMIE                      EQU    (0x0020)            ; COM interrupt enable
TIM_DIER_TIE                        EQU    (0x0040)            ; Trigger interrupt enable
TIM_DIER_BIE                        EQU    (0x0080)            ; Break interrupt enable
TIM_DIER_UDE                        EQU    (0x0100)            ; Update DMA request enable
TIM_DIER_CC1DE                      EQU    (0x0200)            ; Capture/Compare 1 DMA request enable
TIM_DIER_CC2DE                      EQU    (0x0400)            ; Capture/Compare 2 DMA request enable
TIM_DIER_CC3DE                      EQU    (0x0800)            ; Capture/Compare 3 DMA request enable
TIM_DIER_CC4DE                      EQU    (0x1000)            ; Capture/Compare 4 DMA request enable
TIM_DIER_COMDE                      EQU    (0x2000)            ; COM DMA request enable
TIM_DIER_TDE                        EQU    (0x4000)            ; Trigger DMA request enable

;********************  Bit definition for TIM_SR register  ********************
TIM_SR_UIF                          EQU    (0x0001)            ; Update interrupt Flag
TIM_SR_CC1IF                        EQU    (0x0002)            ; Capture/Compare 1 interrupt Flag
TIM_SR_CC2IF                        EQU    (0x0004)            ; Capture/Compare 2 interrupt Flag
TIM_SR_CC3IF                        EQU    (0x0008)            ; Capture/Compare 3 interrupt Flag
TIM_SR_CC4IF                        EQU    (0x0010)            ; Capture/Compare 4 interrupt Flag
TIM_SR_COMIF                        EQU    (0x0020)            ; COM interrupt Flag
TIM_SR_TIF                          EQU    (0x0040)            ; Trigger interrupt Flag
TIM_SR_BIF                          EQU    (0x0080)            ; Break interrupt Flag
TIM_SR_CC1OF                        EQU    (0x0200)            ; Capture/Compare 1 Overcapture Flag
TIM_SR_CC2OF                        EQU    (0x0400)            ; Capture/Compare 2 Overcapture Flag
TIM_SR_CC3OF                        EQU    (0x0800)            ; Capture/Compare 3 Overcapture Flag
TIM_SR_CC4OF                        EQU    (0x1000)            ; Capture/Compare 4 Overcapture Flag

;*******************  Bit definition for TIM_EGR register  ********************
TIM_EGR_UG                          EQU    (0x01)               ; Update Generation
TIM_EGR_CC1G                        EQU    (0x02)               ; Capture/Compare 1 Generation
TIM_EGR_CC2G                        EQU    (0x04)               ; Capture/Compare 2 Generation
TIM_EGR_CC3G                        EQU    (0x08)               ; Capture/Compare 3 Generation
TIM_EGR_CC4G                        EQU    (0x10)               ; Capture/Compare 4 Generation
TIM_EGR_COMG                        EQU    (0x20)               ; Capture/Compare Control Update Generation
TIM_EGR_TG                          EQU    (0x40)               ; Trigger Generation
TIM_EGR_BG                          EQU    (0x80)               ; Break Generation

;******************  Bit definition for TIM_CCMR1 register  *******************
TIM_CCMR1_CC1S                      EQU    (0x0003)            ; CC1S[1:0] bits (Capture/Compare 1 Selection)
TIM_CCMR1_CC1S_0                    EQU    (0x0001)            ; Bit 0
TIM_CCMR1_CC1S_1                    EQU    (0x0002)            ; Bit 1

TIM_CCMR1_OC1FE                     EQU    (0x0004)            ; Output Compare 1 Fast enable
TIM_CCMR1_OC1PE                     EQU    (0x0008)            ; Output Compare 1 Preload enable

TIM_CCMR1_OC1M                      EQU    (0x0070)            ; OC1M[2:0] bits (Output Compare 1 Mode)
TIM_CCMR1_OC1M_0                    EQU    (0x0010)            ; Bit 0
TIM_CCMR1_OC1M_1                    EQU    (0x0020)            ; Bit 1
TIM_CCMR1_OC1M_2                    EQU    (0x0040)            ; Bit 2

TIM_CCMR1_OC1CE                     EQU    (0x0080)            ; Output Compare 1Clear Enable

TIM_CCMR1_CC2S                      EQU    (0x0300)            ; CC2S[1:0] bits (Capture/Compare 2 Selection)
TIM_CCMR1_CC2S_0                    EQU    (0x0100)            ; Bit 0
TIM_CCMR1_CC2S_1                    EQU    (0x0200)            ; Bit 1

TIM_CCMR1_OC2FE                     EQU    (0x0400)            ; Output Compare 2 Fast enable
TIM_CCMR1_OC2PE                     EQU    (0x0800)            ; Output Compare 2 Preload enable

TIM_CCMR1_OC2M                      EQU    (0x7000)            ; OC2M[2:0] bits (Output Compare 2 Mode)
TIM_CCMR1_OC2M_0                    EQU    (0x1000)            ; Bit 0
TIM_CCMR1_OC2M_1                    EQU    (0x2000)            ; Bit 1
TIM_CCMR1_OC2M_2                    EQU    (0x4000)            ; Bit 2

TIM_CCMR1_OC2CE                     EQU    (0x8000)            ; Output Compare 2 Clear Enable

;ÑÑ------------------------------------------------------------------------

TIM_CCMR1_IC1PSC                    EQU    (0x000C)            ; IC1PSC[1:0] bits (Input Capture 1 Prescaler)
TIM_CCMR1_IC1PSC_0                  EQU    (0x0004)            ; Bit 0
TIM_CCMR1_IC1PSC_1                  EQU    (0x0008)            ; Bit 1

TIM_CCMR1_IC1F                      EQU    (0x00F0)            ; IC1F[3:0] bits (Input Capture 1 Filter)
TIM_CCMR1_IC1F_0                    EQU    (0x0010)            ; Bit 0
TIM_CCMR1_IC1F_1                    EQU    (0x0020)            ; Bit 1
TIM_CCMR1_IC1F_2                    EQU    (0x0040)            ; Bit 2
TIM_CCMR1_IC1F_3                    EQU    (0x0080)            ; Bit 3

TIM_CCMR1_IC2PSC                    EQU    (0x0C00)            ; IC2PSC[1:0] bits (Input Capture 2 Prescaler)
TIM_CCMR1_IC2PSC_0                  EQU    (0x0400)            ; Bit 0
TIM_CCMR1_IC2PSC_1                  EQU    (0x0800)            ; Bit 1

TIM_CCMR1_IC2F                      EQU    (0xF000)            ; IC2F[3:0] bits (Input Capture 2 Filter)
TIM_CCMR1_IC2F_0                    EQU    (0x1000)            ; Bit 0
TIM_CCMR1_IC2F_1                    EQU    (0x2000)            ; Bit 1
TIM_CCMR1_IC2F_2                    EQU    (0x4000)            ; Bit 2
TIM_CCMR1_IC2F_3                    EQU    (0x8000)            ; Bit 3

;******************  Bit definition for TIM_CCMR2 register  *******************
TIM_CCMR2_CC3S                      EQU    (0x0003)            ; CC3S[1:0] bits (Capture/Compare 3 Selection)
TIM_CCMR2_CC3S_0                    EQU    (0x0001)            ; Bit 0
TIM_CCMR2_CC3S_1                    EQU    (0x0002)            ; Bit 1

TIM_CCMR2_OC3FE                     EQU    (0x0004)            ; Output Compare 3 Fast enable
TIM_CCMR2_OC3PE                     EQU    (0x0008)            ; Output Compare 3 Preload enable

TIM_CCMR2_OC3M                      EQU    (0x0070)            ; OC3M[2:0] bits (Output Compare 3 Mode)
TIM_CCMR2_OC3M_0                    EQU    (0x0010)            ; Bit 0
TIM_CCMR2_OC3M_1                    EQU    (0x0020)            ; Bit 1
TIM_CCMR2_OC3M_2                    EQU    (0x0040)            ; Bit 2

TIM_CCMR2_OC3CE                     EQU    (0x0080)            ; Output Compare 3 Clear Enable

TIM_CCMR2_CC4S                      EQU    (0x0300)            ; CC4S[1:0] bits (Capture/Compare 4 Selection)
TIM_CCMR2_CC4S_0                    EQU    (0x0100)            ; Bit 0
TIM_CCMR2_CC4S_1                    EQU    (0x0200)            ; Bit 1

TIM_CCMR2_OC4FE                     EQU    (0x0400)            ; Output Compare 4 Fast enable
TIM_CCMR2_OC4PE                     EQU    (0x0800)            ; Output Compare 4 Preload enable

TIM_CCMR2_OC4M                      EQU    (0x7000)            ; OC4M[2:0] bits (Output Compare 4 Mode)
TIM_CCMR2_OC4M_0                    EQU    (0x1000)            ; Bit 0
TIM_CCMR2_OC4M_1                    EQU    (0x2000)            ; Bit 1
TIM_CCMR2_OC4M_2                    EQU    (0x4000)            ; Bit 2

TIM_CCMR2_OC4CE                     EQU    (0x8000)            ; Output Compare 4 Clear Enable

;ÑÑ------------------------------------------------------------------------

TIM_CCMR2_IC3PSC                    EQU    (0x000C)            ; IC3PSC[1:0] bits (Input Capture 3 Prescaler)
TIM_CCMR2_IC3PSC_0                  EQU    (0x0004)            ; Bit 0
TIM_CCMR2_IC3PSC_1                  EQU    (0x0008)            ; Bit 1

TIM_CCMR2_IC3F                      EQU    (0x00F0)            ; IC3F[3:0] bits (Input Capture 3 Filter)
TIM_CCMR2_IC3F_0                    EQU    (0x0010)            ; Bit 0
TIM_CCMR2_IC3F_1                    EQU    (0x0020)            ; Bit 1
TIM_CCMR2_IC3F_2                    EQU    (0x0040)            ; Bit 2
TIM_CCMR2_IC3F_3                    EQU    (0x0080)            ; Bit 3

TIM_CCMR2_IC4PSC                    EQU    (0x0C00)            ; IC4PSC[1:0] bits (Input Capture 4 Prescaler)
TIM_CCMR2_IC4PSC_0                  EQU    (0x0400)            ; Bit 0
TIM_CCMR2_IC4PSC_1                  EQU    (0x0800)            ; Bit 1

TIM_CCMR2_IC4F                      EQU    (0xF000)            ; IC4F[3:0] bits (Input Capture 4 Filter)
TIM_CCMR2_IC4F_0                    EQU    (0x1000)            ; Bit 0
TIM_CCMR2_IC4F_1                    EQU    (0x2000)            ; Bit 1
TIM_CCMR2_IC4F_2                    EQU    (0x4000)            ; Bit 2
TIM_CCMR2_IC4F_3                    EQU    (0x8000)            ; Bit 3

;*******************  Bit definition for TIM_CCER register  *******************
TIM_CCER_CC1E                       EQU    (0x0001)            ; Capture/Compare 1 output enable
TIM_CCER_CC1P                       EQU    (0x0002)            ; Capture/Compare 1 output Polarity
TIM_CCER_CC1NE                      EQU    (0x0004)            ; Capture/Compare 1 Complementary output enable
TIM_CCER_CC1NP                      EQU    (0x0008)            ; Capture/Compare 1 Complementary output Polarity
TIM_CCER_CC2E                       EQU    (0x0010)            ; Capture/Compare 2 output enable
TIM_CCER_CC2P                       EQU    (0x0020)            ; Capture/Compare 2 output Polarity
TIM_CCER_CC2NE                      EQU    (0x0040)            ; Capture/Compare 2 Complementary output enable
TIM_CCER_CC2NP                      EQU    (0x0080)            ; Capture/Compare 2 Complementary output Polarity
TIM_CCER_CC3E                       EQU    (0x0100)            ; Capture/Compare 3 output enable
TIM_CCER_CC3P                       EQU    (0x0200)            ; Capture/Compare 3 output Polarity
TIM_CCER_CC3NE                      EQU    (0x0400)            ; Capture/Compare 3 Complementary output enable
TIM_CCER_CC3NP                      EQU    (0x0800)            ; Capture/Compare 3 Complementary output Polarity
TIM_CCER_CC4E                       EQU    (0x1000)            ; Capture/Compare 4 output enable
TIM_CCER_CC4P                       EQU    (0x2000)            ; Capture/Compare 4 output Polarity
TIM_CCER_CC4NP                      EQU    (0x8000)            ; Capture/Compare 4 Complementary output Polarity

;*******************  Bit definition for TIM_CNT register  ********************
TIM_CNT_CNT                         EQU    (0xFFFF)            ; Counter Value

;*******************  Bit definition for TIM_PSC register  ********************
TIM_PSC_PSC                         EQU    (0xFFFF)            ; Prescaler Value

;*******************  Bit definition for TIM_ARR register  ********************
TIM_ARR_ARR                         EQU    (0xFFFF)            ; actual auto-reload Value

;*******************  Bit definition for TIM_RCR register  ********************
TIM_RCR_REP                         EQU    (0xFF)               ; Repetition Counter Value

;*******************  Bit definition for TIM_CCR1 register  *******************
TIM_CCR1_CCR1                       EQU    (0xFFFF)            ; Capture/Compare 1 Value

;*******************  Bit definition for TIM_CCR2 register  *******************
TIM_CCR2_CCR2                       EQU    (0xFFFF)            ; Capture/Compare 2 Value

;*******************  Bit definition for TIM_CCR3 register  *******************
TIM_CCR3_CCR3                       EQU    (0xFFFF)            ; Capture/Compare 3 Value

;*******************  Bit definition for TIM_CCR4 register  *******************
TIM_CCR4_CCR4                       EQU    (0xFFFF)            ; Capture/Compare 4 Value

;*******************  Bit definition for TIM_BDTR register  *******************
TIM_BDTR_DTG                        EQU    (0x00FF)            ; DTG[0:7] bits (Dead-Time Generator set-up)
TIM_BDTR_DTG_0                      EQU    (0x0001)            ; Bit 0
TIM_BDTR_DTG_1                      EQU    (0x0002)            ; Bit 1
TIM_BDTR_DTG_2                      EQU    (0x0004)            ; Bit 2
TIM_BDTR_DTG_3                      EQU    (0x0008)            ; Bit 3
TIM_BDTR_DTG_4                      EQU    (0x0010)            ; Bit 4
TIM_BDTR_DTG_5                      EQU    (0x0020)            ; Bit 5
TIM_BDTR_DTG_6                      EQU    (0x0040)            ; Bit 6
TIM_BDTR_DTG_7                      EQU    (0x0080)            ; Bit 7

TIM_BDTR_LOCK                       EQU    (0x0300)            ; LOCK[1:0] bits (Lock Configuration)
TIM_BDTR_LOCK_0                     EQU    (0x0100)            ; Bit 0
TIM_BDTR_LOCK_1                     EQU    (0x0200)            ; Bit 1

TIM_BDTR_OSSI                       EQU    (0x0400)            ; Off-State Selection for Idle mode
TIM_BDTR_OSSR                       EQU    (0x0800)            ; Off-State Selection for Run mode
TIM_BDTR_BKE                        EQU    (0x1000)            ; Break enable
TIM_BDTR_BKP                        EQU    (0x2000)            ; Break Polarity
TIM_BDTR_AOE                        EQU    (0x4000)            ; Automatic Output enable
TIM_BDTR_MOE                        EQU    (0x8000)            ; Main Output enable

;*******************  Bit definition for TIM_DCR register  ********************
TIM_DCR_DBA                         EQU    (0x001F)            ; DBA[4:0] bits (DMA Base Address)
TIM_DCR_DBA_0                       EQU    (0x0001)            ; Bit 0
TIM_DCR_DBA_1                       EQU    (0x0002)            ; Bit 1
TIM_DCR_DBA_2                       EQU    (0x0004)            ; Bit 2
TIM_DCR_DBA_3                       EQU    (0x0008)            ; Bit 3
TIM_DCR_DBA_4                       EQU    (0x0010)            ; Bit 4

TIM_DCR_DBL                         EQU    (0x1F00)            ; DBL[4:0] bits (DMA Burst Length)
TIM_DCR_DBL_0                       EQU    (0x0100)            ; Bit 0
TIM_DCR_DBL_1                       EQU    (0x0200)            ; Bit 1
TIM_DCR_DBL_2                       EQU    (0x0400)            ; Bit 2
TIM_DCR_DBL_3                       EQU    (0x0800)            ; Bit 3
TIM_DCR_DBL_4                       EQU    (0x1000)            ; Bit 4

;*******************  Bit definition for TIM_DMAR register  *******************
TIM_DMAR_DMAB                       EQU    (0xFFFF)            ; DMA register for burst accesses

;*******************  Bit definition for TIM_OR register  *********************
TIM_OR_TI4_RMP                       EQU    (0x00C0)            ; TI4_RMP[1:0] bits (TIM5 Input 4 remap)
TIM_OR_TI4_RMP_0                     EQU    (0x0040)            ; Bit 0
TIM_OR_TI4_RMP_1                     EQU    (0x0080)            ; Bit 1
TIM_OR_ITR1_RMP                      EQU    (0x0C00)            ; ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap)
TIM_OR_ITR1_RMP_0                    EQU    (0x0400)            ; Bit 0
TIM_OR_ITR1_RMP_1                    EQU    (0x0800)            ; Bit 1


;******************************************************************************
;*
;*         Universal Synchronous Asynchronous Receiver Transmitter
;*
;******************************************************************************
;*******************  Bit definition for USART_SR register  *******************
USART_SR_PE                         EQU    (0x0001)            ; Parity Error
USART_SR_FE                         EQU    (0x0002)            ; Framing Error
USART_SR_NE                         EQU    (0x0004)            ; Noise Error Flag
USART_SR_ORE                        EQU    (0x0008)            ; OverRun Error
USART_SR_IDLE                       EQU    (0x0010)            ; IDLE line detected
USART_SR_RXNE                       EQU    (0x0020)            ; Read Data Register Not Empty
USART_SR_TC                         EQU    (0x0040)            ; Transmission Complete
USART_SR_TXE                        EQU    (0x0080)            ; Transmit Data Register Empty
USART_SR_LBD                        EQU    (0x0100)            ; LIN Break Detection Flag
USART_SR_CTS                        EQU    (0x0200)            ; CTS Flag

;*******************  Bit definition for USART_DR register  *******************
USART_DR_DR                         EQU    (0x01FF)            ; Data value

;******************  Bit definition for USART_BRR register  *******************
USART_BRR_DIV_Fraction              EQU    (0x000F)            ; Fraction of USARTDIV
USART_BRR_DIV_Mantissa              EQU    (0xFFF0)            ; Mantissa of USARTDIV

;******************  Bit definition for USART_CR1 register  *******************
USART_CR1_SBK                       EQU    (0x0001)            ; Send Break
USART_CR1_RWU                       EQU    (0x0002)            ; Receiver wakeup
USART_CR1_RE                        EQU    (0x0004)            ; Receiver Enable
USART_CR1_TE                        EQU    (0x0008)            ; Transmitter Enable
USART_CR1_IDLEIE                    EQU    (0x0010)            ; IDLE Interrupt Enable
USART_CR1_RXNEIE                    EQU    (0x0020)            ; RXNE Interrupt Enable
USART_CR1_TCIE                      EQU    (0x0040)            ; Transmission Complete Interrupt Enable
USART_CR1_TXEIE                     EQU    (0x0080)            ; PE Interrupt Enable
USART_CR1_PEIE                      EQU    (0x0100)            ; PE Interrupt Enable
USART_CR1_PS                        EQU    (0x0200)            ; Parity Selection
USART_CR1_PCE                       EQU    (0x0400)            ; Parity Control Enable
USART_CR1_WAKE                      EQU    (0x0800)            ; Wakeup method
USART_CR1_M                         EQU    (0x1000)            ; Word length
USART_CR1_UE                        EQU    (0x2000)            ; USART Enable
USART_CR1_OVER8                     EQU    (0x8000)            ; USART Oversampling by 8 enable

;******************  Bit definition for USART_CR2 register  *******************
USART_CR2_ADD                       EQU    (0x000F)            ; Address of the USART node
USART_CR2_LBDL                      EQU    (0x0020)            ; LIN Break Detection Length
USART_CR2_LBDIE                     EQU    (0x0040)            ; LIN Break Detection Interrupt Enable
USART_CR2_LBCL                      EQU    (0x0100)            ; Last Bit Clock pulse
USART_CR2_CPHA                      EQU    (0x0200)            ; Clock Phase
USART_CR2_CPOL                      EQU    (0x0400)            ; Clock Polarity
USART_CR2_CLKEN                     EQU    (0x0800)            ; Clock Enable

USART_CR2_STOP                      EQU    (0x3000)            ; STOP[1:0] bits (STOP bits)
USART_CR2_STOP_0                    EQU    (0x1000)            ; Bit 0
USART_CR2_STOP_1                    EQU    (0x2000)            ; Bit 1

USART_CR2_LINEN                     EQU    (0x4000)            ; LIN mode enable

;******************  Bit definition for USART_CR3 register  *******************
USART_CR3_EIE                       EQU    (0x0001)            ; Error Interrupt Enable
USART_CR3_IREN                      EQU    (0x0002)            ; IrDA mode Enable
USART_CR3_IRLP                      EQU    (0x0004)            ; IrDA Low-Power
USART_CR3_HDSEL                     EQU    (0x0008)            ; Half-Duplex Selection
USART_CR3_NACK                      EQU    (0x0010)            ; Smartcard NACK enable
USART_CR3_SCEN                      EQU    (0x0020)            ; Smartcard mode enable
USART_CR3_DMAR                      EQU    (0x0040)            ; DMA Enable Receiver
USART_CR3_DMAT                      EQU    (0x0080)            ; DMA Enable Transmitter
USART_CR3_RTSE                      EQU    (0x0100)            ; RTS Enable
USART_CR3_CTSE                      EQU    (0x0200)            ; CTS Enable
USART_CR3_CTSIE                     EQU    (0x0400)            ; CTS Interrupt Enable
USART_CR3_ONEBIT                    EQU    (0x0800)            ; USART One bit method enable

;******************  Bit definition for USART_GTPR register  ******************
USART_GTPR_PSC                      EQU    (0x00FF)            ; PSC[7:0] bits (Prescaler value)
USART_GTPR_PSC_0                    EQU    (0x0001)            ; Bit 0
USART_GTPR_PSC_1                    EQU    (0x0002)            ; Bit 1
USART_GTPR_PSC_2                    EQU    (0x0004)            ; Bit 2
USART_GTPR_PSC_3                    EQU    (0x0008)            ; Bit 3
USART_GTPR_PSC_4                    EQU    (0x0010)            ; Bit 4
USART_GTPR_PSC_5                    EQU    (0x0020)            ; Bit 5
USART_GTPR_PSC_6                    EQU    (0x0040)            ; Bit 6
USART_GTPR_PSC_7                    EQU    (0x0080)            ; Bit 7

USART_GTPR_GT                       EQU    (0xFF00)            ; Guard time value

;******************************************************************************
;*
;*                            Window WATCHDOG
;*
;******************************************************************************
;*******************  Bit definition for WWDG_CR register  ********************
WWDG_CR_T                           EQU    (0x7F)               ; T[6:0] bits (7-Bit counter (MSB to LSB))
WWDG_CR_T0                          EQU    (0x01)               ; Bit 0
WWDG_CR_T1                          EQU    (0x02)               ; Bit 1
WWDG_CR_T2                          EQU    (0x04)               ; Bit 2
WWDG_CR_T3                          EQU    (0x08)               ; Bit 3
WWDG_CR_T4                          EQU    (0x10)               ; Bit 4
WWDG_CR_T5                          EQU    (0x20)               ; Bit 5
WWDG_CR_T6                          EQU    (0x40)               ; Bit 6

WWDG_CR_WDGA                        EQU    (0x80)               ; Activation bit

;*******************  Bit definition for WWDG_CFR register  *******************
WWDG_CFR_W                          EQU    (0x007F)            ; W[6:0] bits (7-bit window value)
WWDG_CFR_W0                         EQU    (0x0001)            ; Bit 0
WWDG_CFR_W1                         EQU    (0x0002)            ; Bit 1
WWDG_CFR_W2                         EQU    (0x0004)            ; Bit 2
WWDG_CFR_W3                         EQU    (0x0008)            ; Bit 3
WWDG_CFR_W4                         EQU    (0x0010)            ; Bit 4
WWDG_CFR_W5                         EQU    (0x0020)            ; Bit 5
WWDG_CFR_W6                         EQU    (0x0040)            ; Bit 6

WWDG_CFR_WDGTB                      EQU    (0x0180)            ; WDGTB[1:0] bits (Timer Base)
WWDG_CFR_WDGTB0                     EQU    (0x0080)            ; Bit 0
WWDG_CFR_WDGTB1                     EQU    (0x0100)            ; Bit 1

WWDG_CFR_EWI                        EQU    (0x0200)            ; Early Wakeup Interrupt

;*******************  Bit definition for WWDG_SR register  ********************
WWDG_SR_EWIF                        EQU    (0x01)               ; Early Wakeup Interrupt Flag


;******************************************************************************
;*
;*                                DBG
;*
;******************************************************************************
;********************  Bit definition for DBGMCU_IDCODE register  *************
DBGMCU_IDCODE_DEV_ID                EQU    (0x00000FFF)
DBGMCU_IDCODE_REV_ID                EQU    (0xFFFF0000)

;********************  Bit definition for DBGMCU_CR register  *****************
DBGMCU_CR_DBG_SLEEP                 EQU    (0x00000001)
DBGMCU_CR_DBG_STOP                  EQU    (0x00000002)
DBGMCU_CR_DBG_STANDBY               EQU    (0x00000004)
DBGMCU_CR_TRACE_IOEN                EQU    (0x00000020)

DBGMCU_CR_TRACE_MODE                EQU    (0x000000C0)
DBGMCU_CR_TRACE_MODE_0              EQU    (0x00000040); Bit 0
DBGMCU_CR_TRACE_MODE_1              EQU    (0x00000080); Bit 1

;********************  Bit definition for DBGMCU_APB1_FZ register  ************
DBGMCU_APB1_FZ_DBG_TIM2_STOP            EQU    (0x00000001)
DBGMCU_APB1_FZ_DBG_TIM3_STOP            EQU    (0x00000002)
DBGMCU_APB1_FZ_DBG_TIM4_STOP            EQU    (0x00000004)
DBGMCU_APB1_FZ_DBG_TIM5_STOP            EQU    (0x00000008)
DBGMCU_APB1_FZ_DBG_TIM6_STOP            EQU    (0x00000010)
DBGMCU_APB1_FZ_DBG_TIM7_STOP            EQU    (0x00000020)
DBGMCU_APB1_FZ_DBG_TIM12_STOP           EQU    (0x00000040)
DBGMCU_APB1_FZ_DBG_TIM13_STOP           EQU    (0x00000080)
DBGMCU_APB1_FZ_DBG_TIM14_STOP           EQU    (0x00000100)
DBGMCU_APB1_FZ_DBG_RTC_STOP             EQU    (0x00000400)
DBGMCU_APB1_FZ_DBG_WWDG_STOP            EQU    (0x00000800)
DBGMCU_APB1_FZ_DBG_IWDG_STOP            EQU    (0x00001000)
DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   EQU    (0x00200000)
DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   EQU    (0x00400000)
DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT   EQU    (0x00800000)
DBGMCU_APB1_FZ_DBG_CAN1_STOP            EQU    (0x02000000)
DBGMCU_APB1_FZ_DBG_CAN2_STOP            EQU    (0x04000000)
;  Old IWDGSTOP bit definition, maintained for legacy purpose
DBGMCU_APB1_FZ_DBG_IWDEG_STOP           EQU DBGMCU_APB1_FZ_DBG_IWDG_STOP

;********************  Bit definition for DBGMCU_APB2_FZ register  ************
DBGMCU_APB2_FZ_DBG_TIM1_STOP        EQU    (0x00000001)
DBGMCU_APB2_FZ_DBG_TIM8_STOP        EQU    (0x00000002)
DBGMCU_APB2_FZ_DBG_TIM9_STOP        EQU    (0x00010000)
DBGMCU_APB2_FZ_DBG_TIM10_STOP       EQU    (0x00020000)
DBGMCU_APB2_FZ_DBG_TIM11_STOP       EQU    (0x00040000)

;******************************************************************************
;*
;*                                       USB_OTG
;*
;******************************************************************************
;********************  Bit definition forUSB_OTG_GOTGCTL register  ********************
USB_OTG_GOTGCTL_SRQSCS                  EQU    (0x00000001)            ;  Session request success
USB_OTG_GOTGCTL_SRQ                     EQU    (0x00000002)            ;  Session request
USB_OTG_GOTGCTL_HNGSCS                  EQU    (0x00000100)            ;  Host negotiation success
USB_OTG_GOTGCTL_HNPRQ                   EQU    (0x00000200)            ;  HNP request
USB_OTG_GOTGCTL_HSHNPEN                 EQU    (0x00000400)            ;  Host set HNP enable
USB_OTG_GOTGCTL_DHNPEN                  EQU    (0x00000800)            ;  Device HNP enabled
USB_OTG_GOTGCTL_CIDSTS                  EQU    (0x00010000)            ;  Connector ID status
USB_OTG_GOTGCTL_DBCT                    EQU    (0x00020000)            ;  Long/short debounce time
USB_OTG_GOTGCTL_ASVLD                   EQU    (0x00040000)            ;  A-session valid
USB_OTG_GOTGCTL_BSVLD                   EQU    (0x00080000)            ;  B-session valid

;********************  Bit definition forUSB_OTG_HCFG register  ********************

USB_OTG_HCFG_FSLSPCS                 EQU    (0x00000003)            ;  FS/LS PHY clock select
USB_OTG_HCFG_FSLSPCS_0               EQU    (0x00000001)            ; Bit 0
USB_OTG_HCFG_FSLSPCS_1               EQU    (0x00000002)            ; Bit 1
USB_OTG_HCFG_FSLSS                   EQU    (0x00000004)            ;  FS- and LS-only support

;********************  Bit definition forUSB_OTG_DCFG register  ********************

USB_OTG_DCFG_DSPD                    EQU    (0x00000003)            ;  Device speed
USB_OTG_DCFG_DSPD_0                  EQU    (0x00000001)            ; Bit 0
USB_OTG_DCFG_DSPD_1                  EQU    (0x00000002)            ; Bit 1
USB_OTG_DCFG_NZLSOHSK                EQU    (0x00000004)            ;  Nonzero-length status OUT handshake

USB_OTG_DCFG_DAD                     EQU    (0x000007F0)            ;  Device address
USB_OTG_DCFG_DAD_0                   EQU    (0x00000010)            ; Bit 0
USB_OTG_DCFG_DAD_1                   EQU    (0x00000020)            ; Bit 1
USB_OTG_DCFG_DAD_2                   EQU    (0x00000040)            ; Bit 2
USB_OTG_DCFG_DAD_3                   EQU    (0x00000080)            ; Bit 3
USB_OTG_DCFG_DAD_4                   EQU    (0x00000100)            ; Bit 4
USB_OTG_DCFG_DAD_5                   EQU    (0x00000200)            ; Bit 5
USB_OTG_DCFG_DAD_6                   EQU    (0x00000400)            ; Bit 6

USB_OTG_DCFG_PFIVL                   EQU    (0x00001800)            ;  Periodic (micro)frame interval
USB_OTG_DCFG_PFIVL_0                 EQU    (0x00000800)            ; Bit 0
USB_OTG_DCFG_PFIVL_1                 EQU    (0x00001000)            ; Bit 1

USB_OTG_DCFG_PERSCHIVL               EQU    (0x03000000)            ;  Periodic scheduling interval
USB_OTG_DCFG_PERSCHIVL_0             EQU    (0x01000000)            ; Bit 0
USB_OTG_DCFG_PERSCHIVL_1             EQU    (0x02000000)            ; Bit 1

;********************  Bit definition forUSB_OTG_PCGCR register  ********************
USB_OTG_PCGCR_STPPCLK                 EQU    (0x00000001)            ;  Stop PHY clock
USB_OTG_PCGCR_GATEHCLK                EQU    (0x00000002)            ;  Gate HCLK
USB_OTG_PCGCR_PHYSUSP                 EQU    (0x00000010)            ;  PHY suspended

;********************  Bit definition forUSB_OTG_GOTGINT register  ********************
USB_OTG_GOTGINT_SEDET                   EQU    (0x00000004)            ;  Session end detected
USB_OTG_GOTGINT_SRSSCHG                 EQU    (0x00000100)            ;  Session request success status change
USB_OTG_GOTGINT_HNSSCHG                 EQU    (0x00000200)            ;  Host negotiation success status change
USB_OTG_GOTGINT_HNGDET                  EQU    (0x00020000)            ;  Host negotiation detected
USB_OTG_GOTGINT_ADTOCHG                 EQU    (0x00040000)            ;  A-device timeout change
USB_OTG_GOTGINT_DBCDNE                  EQU    (0x00080000)            ;  Debounce done

;********************  Bit definition forUSB_OTG_DCTL register  ********************
USB_OTG_DCTL_RWUSIG                  EQU    (0x00000001)            ;  Remote wakeup signaling
USB_OTG_DCTL_SDIS                    EQU    (0x00000002)            ;  Soft disconnect
USB_OTG_DCTL_GINSTS                  EQU    (0x00000004)            ;  Global IN NAK status
USB_OTG_DCTL_GONSTS                  EQU    (0x00000008)            ;  Global OUT NAK status

USB_OTG_DCTL_TCTL                    EQU    (0x00000070)            ;  Test control
USB_OTG_DCTL_TCTL_0                  EQU    (0x00000010)            ; Bit 0
USB_OTG_DCTL_TCTL_1                  EQU    (0x00000020)            ; Bit 1
USB_OTG_DCTL_TCTL_2                  EQU    (0x00000040)            ; Bit 2
USB_OTG_DCTL_SGINAK                  EQU    (0x00000080)            ;  Set global IN NAK
USB_OTG_DCTL_CGINAK                  EQU    (0x00000100)            ;  Clear global IN NAK
USB_OTG_DCTL_SGONAK                  EQU    (0x00000200)            ;  Set global OUT NAK
USB_OTG_DCTL_CGONAK                  EQU    (0x00000400)            ;  Clear global OUT NAK
USB_OTG_DCTL_POPRGDNE                EQU    (0x00000800)            ;  Power-on programming done

;********************  Bit definition forUSB_OTG_HFIR register  ********************
USB_OTG_HFIR_FRIVL                   EQU    (0x0000FFFF)            ;  Frame interval

;********************  Bit definition forUSB_OTG_HFNUM register  ********************
USB_OTG_HFNUM_FRNUM                   EQU    (0x0000FFFF)            ;  Frame number
USB_OTG_HFNUM_FTREM                   EQU    (0xFFFF0000)            ;  Frame time remaining

;********************  Bit definition forUSB_OTG_DSTS register  ********************
USB_OTG_DSTS_SUSPSTS                 EQU    (0x00000001)            ;  Suspend status

USB_OTG_DSTS_ENUMSPD                 EQU    (0x00000006)            ;  Enumerated speed
USB_OTG_DSTS_ENUMSPD_0               EQU    (0x00000002)            ; Bit 0
USB_OTG_DSTS_ENUMSPD_1               EQU    (0x00000004)            ; Bit 1
USB_OTG_DSTS_EERR                    EQU    (0x00000008)            ;  Erratic error
USB_OTG_DSTS_FNSOF                   EQU    (0x003FFF00)            ;  Frame number of the received SOF

;********************  Bit definition forUSB_OTG_GAHBCFG register  ********************
USB_OTG_GAHBCFG_GINT                    EQU    (0x00000001)            ;  Global interrupt mask

USB_OTG_GAHBCFG_HBSTLEN                 EQU    (0x0000001E)            ;  Burst length/type
USB_OTG_GAHBCFG_HBSTLEN_0               EQU    (0x00000002)            ; Bit 0
USB_OTG_GAHBCFG_HBSTLEN_1               EQU    (0x00000004)            ; Bit 1
USB_OTG_GAHBCFG_HBSTLEN_2               EQU    (0x00000008)            ; Bit 2
USB_OTG_GAHBCFG_HBSTLEN_3               EQU    (0x00000010)            ; Bit 3
USB_OTG_GAHBCFG_DMAEN                   EQU    (0x00000020)            ;  DMA enable
USB_OTG_GAHBCFG_TXFELVL                 EQU    (0x00000080)            ;  TxFIFO empty level
USB_OTG_GAHBCFG_PTXFELVL                EQU    (0x00000100)            ;  Periodic TxFIFO empty level

;********************  Bit definition forUSB_OTG_GUSBCFG register  ********************

USB_OTG_GUSBCFG_TOCAL                   EQU    (0x00000007)            ;  FS timeout calibration
USB_OTG_GUSBCFG_TOCAL_0                 EQU    (0x00000001)            ; Bit 0
USB_OTG_GUSBCFG_TOCAL_1                 EQU    (0x00000002)            ; Bit 1
USB_OTG_GUSBCFG_TOCAL_2                 EQU    (0x00000004)            ; Bit 2
USB_OTG_GUSBCFG_PHYSEL                  EQU    (0x00000040)            ;  USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select
USB_OTG_GUSBCFG_SRPCAP                  EQU    (0x00000100)            ;  SRP-capable
USB_OTG_GUSBCFG_HNPCAP                  EQU    (0x00000200)            ;  HNP-capable

USB_OTG_GUSBCFG_TRDT                    EQU    (0x00003C00)            ;  USB turnaround time
USB_OTG_GUSBCFG_TRDT_0                  EQU    (0x00000400)            ; Bit 0
USB_OTG_GUSBCFG_TRDT_1                  EQU    (0x00000800)            ; Bit 1
USB_OTG_GUSBCFG_TRDT_2                  EQU    (0x00001000)            ; Bit 2
USB_OTG_GUSBCFG_TRDT_3                  EQU    (0x00002000)            ; Bit 3
USB_OTG_GUSBCFG_PHYLPCS                 EQU    (0x00008000)            ;  PHY Low-power clock select
USB_OTG_GUSBCFG_ULPIFSLS                EQU    (0x00020000)            ;  ULPI FS/LS select
USB_OTG_GUSBCFG_ULPIAR                  EQU    (0x00040000)            ;  ULPI Auto-resume
USB_OTG_GUSBCFG_ULPICSM                 EQU    (0x00080000)            ;  ULPI Clock SuspendM
USB_OTG_GUSBCFG_ULPIEVBUSD              EQU    (0x00100000)            ;  ULPI External VBUS Drive
USB_OTG_GUSBCFG_ULPIEVBUSI              EQU    (0x00200000)            ;  ULPI external VBUS indicator
USB_OTG_GUSBCFG_TSDPS                   EQU    (0x00400000)            ;  TermSel DLine pulsing selection
USB_OTG_GUSBCFG_PCCI                    EQU    (0x00800000)            ;  Indicator complement
USB_OTG_GUSBCFG_PTCI                    EQU    (0x01000000)            ;  Indicator pass through
USB_OTG_GUSBCFG_ULPIIPD                 EQU    (0x02000000)            ;  ULPI interface protect disable
USB_OTG_GUSBCFG_FHMOD                   EQU    (0x20000000)            ;  Forced host mode
USB_OTG_GUSBCFG_FDMOD                   EQU    (0x40000000)            ;  Forced peripheral mode
USB_OTG_GUSBCFG_CTXPKT                  EQU    (0x80000000)            ;  Corrupt Tx packet

;********************  Bit definition forUSB_OTG_GRSTCTL register  ********************
USB_OTG_GRSTCTL_CSRST                   EQU    (0x00000001)            ;  Core soft reset
USB_OTG_GRSTCTL_HSRST                   EQU    (0x00000002)            ;  HCLK soft reset
USB_OTG_GRSTCTL_FCRST                   EQU    (0x00000004)            ;  Host frame counter reset
USB_OTG_GRSTCTL_RXFFLSH                 EQU    (0x00000010)            ;  RxFIFO flush
USB_OTG_GRSTCTL_TXFFLSH                 EQU    (0x00000020)            ;  TxFIFO flush

USB_OTG_GRSTCTL_TXFNUM                  EQU    (0x000007C0)            ;  TxFIFO number
USB_OTG_GRSTCTL_TXFNUM_0                EQU    (0x00000040)            ; Bit 0
USB_OTG_GRSTCTL_TXFNUM_1                EQU    (0x00000080)            ; Bit 1
USB_OTG_GRSTCTL_TXFNUM_2                EQU    (0x00000100)            ; Bit 2
USB_OTG_GRSTCTL_TXFNUM_3                EQU    (0x00000200)            ; Bit 3
USB_OTG_GRSTCTL_TXFNUM_4                EQU    (0x00000400)            ; Bit 4
USB_OTG_GRSTCTL_DMAREQ                  EQU    (0x40000000)            ;  DMA request signal
USB_OTG_GRSTCTL_AHBIDL                  EQU    (0x80000000)            ;  AHB master idle

;********************  Bit definition forUSB_OTG_DIEPMSK register  ********************
USB_OTG_DIEPMSK_XFRCM                   EQU    (0x00000001)            ;  Transfer completed interrupt mask
USB_OTG_DIEPMSK_EPDM                    EQU    (0x00000002)            ;  Endpoint disabled interrupt mask
USB_OTG_DIEPMSK_TOM                     EQU    (0x00000008)            ;  Timeout condition mask (nonisochronous endpoints)
USB_OTG_DIEPMSK_ITTXFEMSK               EQU    (0x00000010)            ;  IN token received when TxFIFO empty mask
USB_OTG_DIEPMSK_INEPNMM                 EQU    (0x00000020)            ;  IN token received with EP mismatch mask
USB_OTG_DIEPMSK_INEPNEM                 EQU    (0x00000040)            ;  IN endpoint NAK effective mask
USB_OTG_DIEPMSK_TXFURM                  EQU    (0x00000100)            ;  FIFO underrun mask
USB_OTG_DIEPMSK_BIM                     EQU    (0x00000200)            ;  BNA interrupt mask

;********************  Bit definition forUSB_OTG_HPTXSTS register  ********************
USB_OTG_HPTXSTS_PTXFSAVL                EQU    (0x0000FFFF)            ;  Periodic transmit data FIFO space available

USB_OTG_HPTXSTS_PTXQSAV                 EQU    (0x00FF0000)            ;  Periodic transmit request queue space available
USB_OTG_HPTXSTS_PTXQSAV_0               EQU    (0x00010000)            ; Bit 0
USB_OTG_HPTXSTS_PTXQSAV_1               EQU    (0x00020000)            ; Bit 1
USB_OTG_HPTXSTS_PTXQSAV_2               EQU    (0x00040000)            ; Bit 2
USB_OTG_HPTXSTS_PTXQSAV_3               EQU    (0x00080000)            ; Bit 3
USB_OTG_HPTXSTS_PTXQSAV_4               EQU    (0x00100000)            ; Bit 4
USB_OTG_HPTXSTS_PTXQSAV_5               EQU    (0x00200000)            ; Bit 5
USB_OTG_HPTXSTS_PTXQSAV_6               EQU    (0x00400000)            ; Bit 6
USB_OTG_HPTXSTS_PTXQSAV_7               EQU    (0x00800000)            ; Bit 7

USB_OTG_HPTXSTS_PTXQTOP                 EQU    (0xFF000000)            ;  Top of the periodic transmit request queue
USB_OTG_HPTXSTS_PTXQTOP_0               EQU    (0x01000000)            ; Bit 0
USB_OTG_HPTXSTS_PTXQTOP_1               EQU    (0x02000000)            ; Bit 1
USB_OTG_HPTXSTS_PTXQTOP_2               EQU    (0x04000000)            ; Bit 2
USB_OTG_HPTXSTS_PTXQTOP_3               EQU    (0x08000000)            ; Bit 3
USB_OTG_HPTXSTS_PTXQTOP_4               EQU    (0x10000000)            ; Bit 4
USB_OTG_HPTXSTS_PTXQTOP_5               EQU    (0x20000000)            ; Bit 5
USB_OTG_HPTXSTS_PTXQTOP_6               EQU    (0x40000000)            ; Bit 6
USB_OTG_HPTXSTS_PTXQTOP_7               EQU    (0x80000000)            ; Bit 7

;********************  Bit definition forUSB_OTG_HAINT register  ********************
USB_OTG_HAINT_HAINT                   EQU    (0x0000FFFF)            ;  Channel interrupts

;********************  Bit definition forUSB_OTG_DOEPMSK register  ********************
USB_OTG_DOEPMSK_XFRCM                   EQU    (0x00000001)            ;  Transfer completed interrupt mask
USB_OTG_DOEPMSK_EPDM                    EQU    (0x00000002)            ;  Endpoint disabled interrupt mask
USB_OTG_DOEPMSK_STUPM                   EQU    (0x00000008)            ;  SETUP phase done mask
USB_OTG_DOEPMSK_OTEPDM                  EQU    (0x00000010)            ;  OUT token received when endpoint disabled mask
USB_OTG_DOEPMSK_B2BSTUP                 EQU    (0x00000040)            ;  Back-to-back SETUP packets received mask
USB_OTG_DOEPMSK_OPEM                    EQU    (0x00000100)            ;  OUT packet error mask
USB_OTG_DOEPMSK_BOIM                    EQU    (0x00000200)            ;  BNA interrupt mask

;********************  Bit definition forUSB_OTG_GINTSTS register  ********************
USB_OTG_GINTSTS_CMOD                    EQU    (0x00000001)            ;  Current mode of operation
USB_OTG_GINTSTS_MMIS                    EQU    (0x00000002)            ;  Mode mismatch interrupt
USB_OTG_GINTSTS_OTGINT                  EQU    (0x00000004)            ;  OTG interrupt
USB_OTG_GINTSTS_SOF                     EQU    (0x00000008)            ;  Start of frame
USB_OTG_GINTSTS_RXFLVL                  EQU    (0x00000010)            ;  RxFIFO nonempty
USB_OTG_GINTSTS_NPTXFE                  EQU    (0x00000020)            ;  Nonperiodic TxFIFO empty
USB_OTG_GINTSTS_GINAKEFF                EQU    (0x00000040)            ;  Global IN nonperiodic NAK effective
USB_OTG_GINTSTS_BOUTNAKEFF              EQU    (0x00000080)            ;  Global OUT NAK effective
USB_OTG_GINTSTS_ESUSP                   EQU    (0x00000400)            ;  Early suspend
USB_OTG_GINTSTS_USBSUSP                 EQU    (0x00000800)            ;  USB suspend
USB_OTG_GINTSTS_USBRST                  EQU    (0x00001000)            ;  USB reset
USB_OTG_GINTSTS_ENUMDNE                 EQU    (0x00002000)            ;  Enumeration done
USB_OTG_GINTSTS_ISOODRP                 EQU    (0x00004000)            ;  Isochronous OUT packet dropped interrupt
USB_OTG_GINTSTS_EOPF                    EQU    (0x00008000)            ;  End of periodic frame interrupt
USB_OTG_GINTSTS_IEPINT                  EQU    (0x00040000)            ;  IN endpoint interrupt
USB_OTG_GINTSTS_OEPINT                  EQU    (0x00080000)            ;  OUT endpoint interrupt
USB_OTG_GINTSTS_IISOIXFR                EQU    (0x00100000)            ;  Incomplete isochronous IN transfer
USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       EQU    (0x00200000)            ;  Incomplete periodic transfer
USB_OTG_GINTSTS_DATAFSUSP               EQU    (0x00400000)            ;  Data fetch suspended
USB_OTG_GINTSTS_HPRTINT                 EQU    (0x01000000)            ;  Host port interrupt
USB_OTG_GINTSTS_HCINT                   EQU    (0x02000000)            ;  Host channels interrupt
USB_OTG_GINTSTS_PTXFE                   EQU    (0x04000000)            ;  Periodic TxFIFO empty
USB_OTG_GINTSTS_CIDSCHG                 EQU    (0x10000000)            ;  Connector ID status change
USB_OTG_GINTSTS_DISCINT                 EQU    (0x20000000)            ;  Disconnect detected interrupt
USB_OTG_GINTSTS_SRQINT                  EQU    (0x40000000)            ;  Session request/new session detected interrupt
USB_OTG_GINTSTS_WKUINT                  EQU    (0x80000000)            ;  Resume/remote wakeup detected interrupt

;********************  Bit definition forUSB_OTG_GINTMSK register  ********************
USB_OTG_GINTMSK_MMISM                   EQU    (0x00000002)            ;  Mode mismatch interrupt mask
USB_OTG_GINTMSK_OTGINT                  EQU    (0x00000004)            ;  OTG interrupt mask
USB_OTG_GINTMSK_SOFM                    EQU    (0x00000008)            ;  Start of frame mask
USB_OTG_GINTMSK_RXFLVLM                 EQU    (0x00000010)            ;  Receive FIFO nonempty mask
USB_OTG_GINTMSK_NPTXFEM                 EQU    (0x00000020)            ;  Nonperiodic TxFIFO empty mask
USB_OTG_GINTMSK_GINAKEFFM               EQU    (0x00000040)            ;  Global nonperiodic IN NAK effective mask
USB_OTG_GINTMSK_GONAKEFFM               EQU    (0x00000080)            ;  Global OUT NAK effective mask
USB_OTG_GINTMSK_ESUSPM                  EQU    (0x00000400)            ;  Early suspend mask
USB_OTG_GINTMSK_USBSUSPM                EQU    (0x00000800)            ;  USB suspend mask
USB_OTG_GINTMSK_USBRST                  EQU    (0x00001000)            ;  USB reset mask
USB_OTG_GINTMSK_ENUMDNEM                EQU    (0x00002000)            ;  Enumeration done mask
USB_OTG_GINTMSK_ISOODRPM                EQU    (0x00004000)            ;  Isochronous OUT packet dropped interrupt mask
USB_OTG_GINTMSK_EOPFM                   EQU    (0x00008000)            ;  End of periodic frame interrupt mask
USB_OTG_GINTMSK_EPMISM                  EQU    (0x00020000)            ;  Endpoint mismatch interrupt mask
USB_OTG_GINTMSK_IEPINT                  EQU    (0x00040000)            ;  IN endpoints interrupt mask
USB_OTG_GINTMSK_OEPINT                  EQU    (0x00080000)            ;  OUT endpoints interrupt mask
USB_OTG_GINTMSK_IISOIXFRM               EQU    (0x00100000)            ;  Incomplete isochronous IN transfer mask
USB_OTG_GINTMSK_PXFRM_IISOOXFRM         EQU    (0x00200000)            ;  Incomplete periodic transfer mask
USB_OTG_GINTMSK_FSUSPM                  EQU    (0x00400000)            ;  Data fetch suspended mask
USB_OTG_GINTMSK_PRTIM                   EQU    (0x01000000)            ;  Host port interrupt mask
USB_OTG_GINTMSK_HCIM                    EQU    (0x02000000)            ;  Host channels interrupt mask
USB_OTG_GINTMSK_PTXFEM                  EQU    (0x04000000)            ;  Periodic TxFIFO empty mask
USB_OTG_GINTMSK_CIDSCHGM                EQU    (0x10000000)            ;  Connector ID status change mask
USB_OTG_GINTMSK_DISCINT                 EQU    (0x20000000)            ;  Disconnect detected interrupt mask
USB_OTG_GINTMSK_SRQIM                   EQU    (0x40000000)            ;  Session request/new session detected interrupt mask
USB_OTG_GINTMSK_WUIM                    EQU    (0x80000000)            ;  Resume/remote wakeup detected interrupt mask

;********************  Bit definition forUSB_OTG_DAINT register  ********************
USB_OTG_DAINT_IEPINT                  EQU    (0x0000FFFF)            ;  IN endpoint interrupt bits
USB_OTG_DAINT_OEPINT                  EQU    (0xFFFF0000)            ;  OUT endpoint interrupt bits

;********************  Bit definition forUSB_OTG_HAINTMSK register  ********************
USB_OTG_HAINTMSK_HAINTM                  EQU    (0x0000FFFF)            ;  Channel interrupt mask

;********************  Bit definition for USB_OTG_GRXSTSP register  ********************
USB_OTG_GRXSTSP_EPNUM                    EQU    (0x0000000F)            ;  IN EP interrupt mask bits
USB_OTG_GRXSTSP_BCNT                     EQU    (0x00007FF0)            ;  OUT EP interrupt mask bits
USB_OTG_GRXSTSP_DPID                     EQU    (0x00018000)            ;  OUT EP interrupt mask bits
USB_OTG_GRXSTSP_PKTSTS                   EQU    (0x001E0000)            ;  OUT EP interrupt mask bits

;********************  Bit definition forUSB_OTG_DAINTMSK register  ********************
USB_OTG_DAINTMSK_IEPM                    EQU    (0x0000FFFF)            ;  IN EP interrupt mask bits
USB_OTG_DAINTMSK_OEPM                    EQU    (0xFFFF0000)            ;  OUT EP interrupt mask bits

;********************  Bit definition for OTG register  ********************

USB_OTG_CHNUM                   EQU    (0x0000000F)            ;  Channel number
USB_OTG_CHNUM_0                 EQU    (0x00000001)            ; Bit 0
USB_OTG_CHNUM_1                 EQU    (0x00000002)            ; Bit 1
USB_OTG_CHNUM_2                 EQU    (0x00000004)            ; Bit 2
USB_OTG_CHNUM_3                 EQU    (0x00000008)            ; Bit 3
USB_OTG_BCNT                    EQU    (0x00007FF0)            ;  Byte count

USB_OTG_DPID                    EQU    (0x00018000)            ;  Data PID
USB_OTG_DPID_0                  EQU    (0x00008000)            ; Bit 0
USB_OTG_DPID_1                  EQU    (0x00010000)            ; Bit 1

USB_OTG_PKTSTS                  EQU    (0x001E0000)            ;  Packet status
USB_OTG_PKTSTS_0                EQU    (0x00020000)            ; Bit 0
USB_OTG_PKTSTS_1                EQU    (0x00040000)            ; Bit 1
USB_OTG_PKTSTS_2                EQU    (0x00080000)            ; Bit 2
USB_OTG_PKTSTS_3                EQU    (0x00100000)            ; Bit 3

USB_OTG_EPNUM                   EQU    (0x0000000F)            ;  Endpoint number
USB_OTG_EPNUM_0                 EQU    (0x00000001)            ; Bit 0
USB_OTG_EPNUM_1                 EQU    (0x00000002)            ; Bit 1
USB_OTG_EPNUM_2                 EQU    (0x00000004)            ; Bit 2
USB_OTG_EPNUM_3                 EQU    (0x00000008)            ; Bit 3

USB_OTG_FRMNUM                  EQU    (0x01E00000)            ;  Frame number
USB_OTG_FRMNUM_0                EQU    (0x00200000)            ; Bit 0
USB_OTG_FRMNUM_1                EQU    (0x00400000)            ; Bit 1
USB_OTG_FRMNUM_2                EQU    (0x00800000)            ; Bit 2
USB_OTG_FRMNUM_3                EQU    (0x01000000)            ; Bit 3

;********************  Bit definition for OTG register  ********************

;USB_OTG_CHNUM                   EQU    (0x0000000F)            ;  Channel number
;USB_OTG_CHNUM_0                 EQU    (0x00000001)            ; Bit 0
;USB_OTG_CHNUM_1                 EQU    (0x00000002)            ; Bit 1
;USB_OTG_CHNUM_2                 EQU    (0x00000004)            ; Bit 2
;USB_OTG_CHNUM_3                 EQU    (0x00000008)            ; Bit 3
;USB_OTG_BCNT                    EQU    (0x00007FF0)            ;  Byte count

;USB_OTG_DPID                    EQU    (0x00018000)            ;  Data PID
;USB_OTG_DPID_0                  EQU    (0x00008000)            ; Bit 0
;USB_OTG_DPID_1                  EQU    (0x00010000)            ; Bit 1

;USB_OTG_PKTSTS                  EQU    (0x001E0000)            ;  Packet status
;USB_OTG_PKTSTS_0                EQU    (0x00020000)            ; Bit 0
;USB_OTG_PKTSTS_1                EQU    (0x00040000)            ; Bit 1
;USB_OTG_PKTSTS_2                EQU    (0x00080000)            ; Bit 2
;USB_OTG_PKTSTS_3                EQU    (0x00100000)            ; Bit 3

;USB_OTG_EPNUM                   EQU    (0x0000000F)            ;  Endpoint number
;USB_OTG_EPNUM_0                 EQU    (0x00000001)            ; Bit 0
;USB_OTG_EPNUM_1                 EQU    (0x00000002)            ; Bit 1
;USB_OTG_EPNUM_2                 EQU    (0x00000004)            ; Bit 2
;USB_OTG_EPNUM_3                 EQU    (0x00000008)            ; Bit 3

;USB_OTG_FRMNUM                  EQU    (0x01E00000)            ;  Frame number
;USB_OTG_FRMNUM_0                EQU    (0x00200000)            ; Bit 0
;USB_OTG_FRMNUM_1                EQU    (0x00400000)            ; Bit 1
;USB_OTG_FRMNUM_2                EQU    (0x00800000)            ; Bit 2
;USB_OTG_FRMNUM_3                EQU    (0x01000000)            ; Bit 3

;********************  Bit definition forUSB_OTG_GRXFSIZ register  ********************
USB_OTG_GRXFSIZ_RXFD                    EQU    (0x0000FFFF)            ;  RxFIFO depth

;********************  Bit definition forUSB_OTG_DVBUSDIS register  ********************
USB_OTG_DVBUSDIS_VBUSDT                  EQU    (0x0000FFFF)            ;  Device VBUS discharge time

;********************  Bit definition for OTG register  ********************
USB_OTG_NPTXFSA                 EQU    (0x0000FFFF)            ;  Nonperiodic transmit RAM start address
USB_OTG_NPTXFD                  EQU    (0xFFFF0000)            ;  Nonperiodic TxFIFO depth
USB_OTG_TX0FSA                  EQU    (0x0000FFFF)            ;  Endpoint 0 transmit RAM start address
USB_OTG_TX0FD                   EQU    (0xFFFF0000)            ;  Endpoint 0 TxFIFO depth

;********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************
USB_OTG_DVBUSPULSE_DVBUSP                  EQU    (0x00000FFF)            ;  Device VBUS pulsing time

;********************  Bit definition forUSB_OTG_GNPTXSTS register  ********************
USB_OTG_GNPTXSTS_NPTXFSAV                EQU    (0x0000FFFF)            ;  Nonperiodic TxFIFO space available

USB_OTG_GNPTXSTS_NPTQXSAV                EQU    (0x00FF0000)            ;  Nonperiodic transmit request queue space available
USB_OTG_GNPTXSTS_NPTQXSAV_0              EQU    (0x00010000)            ; Bit 0
USB_OTG_GNPTXSTS_NPTQXSAV_1              EQU    (0x00020000)            ; Bit 1
USB_OTG_GNPTXSTS_NPTQXSAV_2              EQU    (0x00040000)            ; Bit 2
USB_OTG_GNPTXSTS_NPTQXSAV_3              EQU    (0x00080000)            ; Bit 3
USB_OTG_GNPTXSTS_NPTQXSAV_4              EQU    (0x00100000)            ; Bit 4
USB_OTG_GNPTXSTS_NPTQXSAV_5              EQU    (0x00200000)            ; Bit 5
USB_OTG_GNPTXSTS_NPTQXSAV_6              EQU    (0x00400000)            ; Bit 6
USB_OTG_GNPTXSTS_NPTQXSAV_7              EQU    (0x00800000)            ; Bit 7

USB_OTG_GNPTXSTS_NPTXQTOP                EQU    (0x7F000000)            ;  Top of the nonperiodic transmit request queue
USB_OTG_GNPTXSTS_NPTXQTOP_0              EQU    (0x01000000)            ; Bit 0
USB_OTG_GNPTXSTS_NPTXQTOP_1              EQU    (0x02000000)            ; Bit 1
USB_OTG_GNPTXSTS_NPTXQTOP_2              EQU    (0x04000000)            ; Bit 2
USB_OTG_GNPTXSTS_NPTXQTOP_3              EQU    (0x08000000)            ; Bit 3
USB_OTG_GNPTXSTS_NPTXQTOP_4              EQU    (0x10000000)            ; Bit 4
USB_OTG_GNPTXSTS_NPTXQTOP_5              EQU    (0x20000000)            ; Bit 5
USB_OTG_GNPTXSTS_NPTXQTOP_6              EQU    (0x40000000)            ; Bit 6

;********************  Bit definition forUSB_OTG_DTHRCTL register  ********************
USB_OTG_DTHRCTL_NONISOTHREN             EQU    (0x00000001)            ;  Nonisochronous IN endpoints threshold enable
USB_OTG_DTHRCTL_ISOTHREN                EQU    (0x00000002)            ;  ISO IN endpoint threshold enable

USB_OTG_DTHRCTL_TXTHRLEN                EQU    (0x000007FC)            ;  Transmit threshold length
USB_OTG_DTHRCTL_TXTHRLEN_0              EQU    (0x00000004)            ; Bit 0
USB_OTG_DTHRCTL_TXTHRLEN_1              EQU    (0x00000008)            ; Bit 1
USB_OTG_DTHRCTL_TXTHRLEN_2              EQU    (0x00000010)            ; Bit 2
USB_OTG_DTHRCTL_TXTHRLEN_3              EQU    (0x00000020)            ; Bit 3
USB_OTG_DTHRCTL_TXTHRLEN_4              EQU    (0x00000040)            ; Bit 4
USB_OTG_DTHRCTL_TXTHRLEN_5              EQU    (0x00000080)            ; Bit 5
USB_OTG_DTHRCTL_TXTHRLEN_6              EQU    (0x00000100)            ; Bit 6
USB_OTG_DTHRCTL_TXTHRLEN_7              EQU    (0x00000200)            ; Bit 7
USB_OTG_DTHRCTL_TXTHRLEN_8              EQU    (0x00000400)            ; Bit 8
USB_OTG_DTHRCTL_RXTHREN                 EQU    (0x00010000)            ;  Receive threshold enable

USB_OTG_DTHRCTL_RXTHRLEN                EQU    (0x03FE0000)            ;  Receive threshold length
USB_OTG_DTHRCTL_RXTHRLEN_0              EQU    (0x00020000)            ; Bit 0
USB_OTG_DTHRCTL_RXTHRLEN_1              EQU    (0x00040000)            ; Bit 1
USB_OTG_DTHRCTL_RXTHRLEN_2              EQU    (0x00080000)            ; Bit 2
USB_OTG_DTHRCTL_RXTHRLEN_3              EQU    (0x00100000)            ; Bit 3
USB_OTG_DTHRCTL_RXTHRLEN_4              EQU    (0x00200000)            ; Bit 4
USB_OTG_DTHRCTL_RXTHRLEN_5              EQU    (0x00400000)            ; Bit 5
USB_OTG_DTHRCTL_RXTHRLEN_6              EQU    (0x00800000)            ; Bit 6
USB_OTG_DTHRCTL_RXTHRLEN_7              EQU    (0x01000000)            ; Bit 7
USB_OTG_DTHRCTL_RXTHRLEN_8              EQU    (0x02000000)            ; Bit 8
USB_OTG_DTHRCTL_ARPEN                   EQU    (0x08000000)            ;  Arbiter parking enable

;********************  Bit definition forUSB_OTG_DIEPEMPMSK register  ********************
USB_OTG_DIEPEMPMSK_INEPTXFEM               EQU    (0x0000FFFF)            ;  IN EP Tx FIFO empty interrupt mask bits

;********************  Bit definition forUSB_OTG_DEACHINT register  ********************
USB_OTG_DEACHINT_IEP1INT                 EQU    (0x00000002)            ;  IN endpoint 1interrupt bit
USB_OTG_DEACHINT_OEP1INT                 EQU    (0x00020000)            ;  OUT endpoint 1 interrupt bit

;********************  Bit definition forUSB_OTG_GCCFG register  ********************
USB_OTG_GCCFG_PWRDWN                  EQU    (0x00010000)            ;  Power down
USB_OTG_GCCFG_I2CPADEN                EQU    (0x00020000)            ;  Enable I2C bus connection for the external I2C PHY interface
USB_OTG_GCCFG_VBUSASEN                EQU    (0x00040000)            ;  Enable the VBUS sensing device
USB_OTG_GCCFG_VBUSBSEN                EQU    (0x00080000)            ;  Enable the VBUS sensing device
USB_OTG_GCCFG_SOFOUTEN                EQU    (0x00100000)            ;  SOF output enable
USB_OTG_GCCFG_NOVBUSSENS              EQU    (0x00200000)            ;  VBUS sensing disable option

;********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************
USB_OTG_DEACHINTMSK_IEP1INTM                EQU    (0x00000002)            ;  IN Endpoint 1 interrupt mask bit
USB_OTG_DEACHINTMSK_OEP1INTM                EQU    (0x00020000)            ;  OUT Endpoint 1 interrupt mask bit

;********************  Bit definition forUSB_OTG_CID register  ********************
USB_OTG_CID_PRODUCT_ID              EQU    (0xFFFFFFFF)            ;  Product ID field

;********************  Bit definition forUSB_OTG_DIEPEACHMSK1 register  ********************
USB_OTG_DIEPEACHMSK1_XFRCM                   EQU    (0x00000001)            ;  Transfer completed interrupt mask
USB_OTG_DIEPEACHMSK1_EPDM                    EQU    (0x00000002)            ;  Endpoint disabled interrupt mask
USB_OTG_DIEPEACHMSK1_TOM                     EQU    (0x00000008)            ;  Timeout condition mask (nonisochronous endpoints)
USB_OTG_DIEPEACHMSK1_ITTXFEMSK               EQU    (0x00000010)            ;  IN token received when TxFIFO empty mask
USB_OTG_DIEPEACHMSK1_INEPNMM                 EQU    (0x00000020)            ;  IN token received with EP mismatch mask
USB_OTG_DIEPEACHMSK1_INEPNEM                 EQU    (0x00000040)            ;  IN endpoint NAK effective mask
USB_OTG_DIEPEACHMSK1_TXFURM                  EQU    (0x00000100)            ;  FIFO underrun mask
USB_OTG_DIEPEACHMSK1_BIM                     EQU    (0x00000200)            ;  BNA interrupt mask
USB_OTG_DIEPEACHMSK1_NAKM                    EQU    (0x00002000)            ;  NAK interrupt mask

;********************  Bit definition forUSB_OTG_HPRT register  ********************
USB_OTG_HPRT_PCSTS                   EQU    (0x00000001)            ;  Port connect status
USB_OTG_HPRT_PCDET                   EQU    (0x00000002)            ;  Port connect detected
USB_OTG_HPRT_PENA                    EQU    (0x00000004)            ;  Port enable
USB_OTG_HPRT_PENCHNG                 EQU    (0x00000008)            ;  Port enable/disable change
USB_OTG_HPRT_POCA                    EQU    (0x00000010)            ;  Port overcurrent active
USB_OTG_HPRT_POCCHNG                 EQU    (0x00000020)            ;  Port overcurrent change
USB_OTG_HPRT_PRES                    EQU    (0x00000040)            ;  Port resume
USB_OTG_HPRT_PSUSP                   EQU    (0x00000080)            ;  Port suspend
USB_OTG_HPRT_PRST                    EQU    (0x00000100)            ;  Port reset

USB_OTG_HPRT_PLSTS                   EQU    (0x00000C00)            ;  Port line status
USB_OTG_HPRT_PLSTS_0                 EQU    (0x00000400)            ; Bit 0
USB_OTG_HPRT_PLSTS_1                 EQU    (0x00000800)            ; Bit 1
USB_OTG_HPRT_PPWR                    EQU    (0x00001000)            ;  Port power

USB_OTG_HPRT_PTCTL                   EQU    (0x0001E000)            ;  Port test control
USB_OTG_HPRT_PTCTL_0                 EQU    (0x00002000)            ; Bit 0
USB_OTG_HPRT_PTCTL_1                 EQU    (0x00004000)            ; Bit 1
USB_OTG_HPRT_PTCTL_2                 EQU    (0x00008000)            ; Bit 2
USB_OTG_HPRT_PTCTL_3                 EQU    (0x00010000)            ; Bit 3

USB_OTG_HPRT_PSPD                    EQU    (0x00060000)            ;  Port speed
USB_OTG_HPRT_PSPD_0                  EQU    (0x00020000)            ; Bit 0
USB_OTG_HPRT_PSPD_1                  EQU    (0x00040000)            ; Bit 1

;********************  Bit definition forUSB_OTG_DOEPEACHMSK1 register  ********************
USB_OTG_DOEPEACHMSK1_XFRCM                   EQU    (0x00000001)            ;  Transfer completed interrupt mask
USB_OTG_DOEPEACHMSK1_EPDM                    EQU    (0x00000002)            ;  Endpoint disabled interrupt mask
USB_OTG_DOEPEACHMSK1_TOM                     EQU    (0x00000008)            ;  Timeout condition mask
USB_OTG_DOEPEACHMSK1_ITTXFEMSK               EQU    (0x00000010)            ;  IN token received when TxFIFO empty mask
USB_OTG_DOEPEACHMSK1_INEPNMM                 EQU    (0x00000020)            ;  IN token received with EP mismatch mask
USB_OTG_DOEPEACHMSK1_INEPNEM                 EQU    (0x00000040)            ;  IN endpoint NAK effective mask
USB_OTG_DOEPEACHMSK1_TXFURM                  EQU    (0x00000100)            ;  OUT packet error mask
USB_OTG_DOEPEACHMSK1_BIM                     EQU    (0x00000200)            ;  BNA interrupt mask
USB_OTG_DOEPEACHMSK1_BERRM                   EQU    (0x00001000)            ;  Bubble error interrupt mask
USB_OTG_DOEPEACHMSK1_NAKM                    EQU    (0x00002000)            ;  NAK interrupt mask
USB_OTG_DOEPEACHMSK1_NYETM                   EQU    (0x00004000)            ;  NYET interrupt mask

;********************  Bit definition forUSB_OTG_HPTXFSIZ register  ********************
USB_OTG_HPTXFSIZ_PTXSA                   EQU    (0x0000FFFF)            ;  Host periodic TxFIFO start address
USB_OTG_HPTXFSIZ_PTXFD                   EQU    (0xFFFF0000)            ;  Host periodic TxFIFO depth

;********************  Bit definition forUSB_OTG_DIEPCTL register  ********************
USB_OTG_DIEPCTL_MPSIZ                   EQU    (0x000007FF)            ;  Maximum packet size
USB_OTG_DIEPCTL_USBAEP                  EQU    (0x00008000)            ;  USB active endpoint
USB_OTG_DIEPCTL_EONUM_DPID              EQU    (0x00010000)            ;  Even/odd frame
USB_OTG_DIEPCTL_NAKSTS                  EQU    (0x00020000)            ;  NAK status

USB_OTG_DIEPCTL_EPTYP                   EQU    (0x000C0000)            ;  Endpoint type
USB_OTG_DIEPCTL_EPTYP_0                 EQU    (0x00040000)            ; Bit 0
USB_OTG_DIEPCTL_EPTYP_1                 EQU    (0x00080000)            ; Bit 1
USB_OTG_DIEPCTL_STALL                   EQU    (0x00200000)            ;  STALL handshake

USB_OTG_DIEPCTL_TXFNUM                  EQU    (0x03C00000)            ;  TxFIFO number
USB_OTG_DIEPCTL_TXFNUM_0                EQU    (0x00400000)            ; Bit 0
USB_OTG_DIEPCTL_TXFNUM_1                EQU    (0x00800000)            ; Bit 1
USB_OTG_DIEPCTL_TXFNUM_2                EQU    (0x01000000)            ; Bit 2
USB_OTG_DIEPCTL_TXFNUM_3                EQU    (0x02000000)            ; Bit 3
USB_OTG_DIEPCTL_CNAK                    EQU    (0x04000000)            ;  Clear NAK
USB_OTG_DIEPCTL_SNAK                    EQU    (0x08000000)            ;  Set NAK
USB_OTG_DIEPCTL_SD0PID_SEVNFRM          EQU    (0x10000000)            ;  Set DATA0 PID
USB_OTG_DIEPCTL_SODDFRM                 EQU    (0x20000000)            ;  Set odd frame
USB_OTG_DIEPCTL_EPDIS                   EQU    (0x40000000)            ;  Endpoint disable
USB_OTG_DIEPCTL_EPENA                   EQU    (0x80000000)            ;  Endpoint enable

;********************  Bit definition forUSB_OTG_HCCHAR register  ********************
USB_OTG_HCCHAR_MPSIZ                   EQU    (0x000007FF)            ;  Maximum packet size

USB_OTG_HCCHAR_EPNUM                   EQU    (0x00007800)            ;  Endpoint number
USB_OTG_HCCHAR_EPNUM_0                 EQU    (0x00000800)            ; Bit 0
USB_OTG_HCCHAR_EPNUM_1                 EQU    (0x00001000)            ; Bit 1
USB_OTG_HCCHAR_EPNUM_2                 EQU    (0x00002000)            ; Bit 2
USB_OTG_HCCHAR_EPNUM_3                 EQU    (0x00004000)            ; Bit 3
USB_OTG_HCCHAR_EPDIR                   EQU    (0x00008000)            ;  Endpoint direction
USB_OTG_HCCHAR_LSDEV                   EQU    (0x00020000)            ;  Low-speed device

USB_OTG_HCCHAR_EPTYP                   EQU    (0x000C0000)            ;  Endpoint type
USB_OTG_HCCHAR_EPTYP_0                 EQU    (0x00040000)            ; Bit 0
USB_OTG_HCCHAR_EPTYP_1                 EQU    (0x00080000)            ; Bit 1

USB_OTG_HCCHAR_MC                      EQU    (0x00300000)            ;  Multi Count (MC) / Error Count (EC)
USB_OTG_HCCHAR_MC_0                    EQU    (0x00100000)            ; Bit 0
USB_OTG_HCCHAR_MC_1                    EQU    (0x00200000)            ; Bit 1

USB_OTG_HCCHAR_DAD                     EQU    (0x1FC00000)            ;  Device address
USB_OTG_HCCHAR_DAD_0                   EQU    (0x00400000)            ; Bit 0
USB_OTG_HCCHAR_DAD_1                   EQU    (0x00800000)            ; Bit 1
USB_OTG_HCCHAR_DAD_2                   EQU    (0x01000000)            ; Bit 2
USB_OTG_HCCHAR_DAD_3                   EQU    (0x02000000)            ; Bit 3
USB_OTG_HCCHAR_DAD_4                   EQU    (0x04000000)            ; Bit 4
USB_OTG_HCCHAR_DAD_5                   EQU    (0x08000000)            ; Bit 5
USB_OTG_HCCHAR_DAD_6                   EQU    (0x10000000)            ; Bit 6
USB_OTG_HCCHAR_ODDFRM                  EQU    (0x20000000)            ;  Odd frame
USB_OTG_HCCHAR_CHDIS                   EQU    (0x40000000)            ;  Channel disable
USB_OTG_HCCHAR_CHENA                   EQU    (0x80000000)            ;  Channel enable

;********************  Bit definition forUSB_OTG_HCSPLT register  ********************

USB_OTG_HCSPLT_PRTADDR                 EQU    (0x0000007F)            ;  Port address
USB_OTG_HCSPLT_PRTADDR_0               EQU    (0x00000001)            ; Bit 0
USB_OTG_HCSPLT_PRTADDR_1               EQU    (0x00000002)            ; Bit 1
USB_OTG_HCSPLT_PRTADDR_2               EQU    (0x00000004)            ; Bit 2
USB_OTG_HCSPLT_PRTADDR_3               EQU    (0x00000008)            ; Bit 3
USB_OTG_HCSPLT_PRTADDR_4               EQU    (0x00000010)            ; Bit 4
USB_OTG_HCSPLT_PRTADDR_5               EQU    (0x00000020)            ; Bit 5
USB_OTG_HCSPLT_PRTADDR_6               EQU    (0x00000040)            ; Bit 6

USB_OTG_HCSPLT_HUBADDR                 EQU    (0x00003F80)            ;  Hub address
USB_OTG_HCSPLT_HUBADDR_0               EQU    (0x00000080)            ; Bit 0
USB_OTG_HCSPLT_HUBADDR_1               EQU    (0x00000100)            ; Bit 1
USB_OTG_HCSPLT_HUBADDR_2               EQU    (0x00000200)            ; Bit 2
USB_OTG_HCSPLT_HUBADDR_3               EQU    (0x00000400)            ; Bit 3
USB_OTG_HCSPLT_HUBADDR_4               EQU    (0x00000800)            ; Bit 4
USB_OTG_HCSPLT_HUBADDR_5               EQU    (0x00001000)            ; Bit 5
USB_OTG_HCSPLT_HUBADDR_6               EQU    (0x00002000)            ; Bit 6

USB_OTG_HCSPLT_XACTPOS                 EQU    (0x0000C000)            ;  XACTPOS
USB_OTG_HCSPLT_XACTPOS_0               EQU    (0x00004000)            ; Bit 0
USB_OTG_HCSPLT_XACTPOS_1               EQU    (0x00008000)            ; Bit 1
USB_OTG_HCSPLT_COMPLSPLT               EQU    (0x00010000)            ;  Do complete split
USB_OTG_HCSPLT_SPLITEN                 EQU    (0x80000000)            ;  Split enable

;********************  Bit definition forUSB_OTG_HCINT register  ********************
USB_OTG_HCINT_XFRC                    EQU    (0x00000001)            ;  Transfer completed
USB_OTG_HCINT_CHH                     EQU    (0x00000002)            ;  Channel halted
USB_OTG_HCINT_AHBERR                  EQU    (0x00000004)            ;  AHB error
USB_OTG_HCINT_STALL                   EQU    (0x00000008)            ;  STALL response received interrupt
USB_OTG_HCINT_NAK                     EQU    (0x00000010)            ;  NAK response received interrupt
USB_OTG_HCINT_ACK                     EQU    (0x00000020)            ;  ACK response received/transmitted interrupt
USB_OTG_HCINT_NYET                    EQU    (0x00000040)            ;  Response received interrupt
USB_OTG_HCINT_TXERR                   EQU    (0x00000080)            ;  Transaction error
USB_OTG_HCINT_BBERR                   EQU    (0x00000100)            ;  Babble error
USB_OTG_HCINT_FRMOR                   EQU    (0x00000200)            ;  Frame overrun
USB_OTG_HCINT_DTERR                   EQU    (0x00000400)            ;  Data toggle error

;********************  Bit definition forUSB_OTG_DIEPINT register  ********************
USB_OTG_DIEPINT_XFRC                    EQU    (0x00000001)            ;  Transfer completed interrupt
USB_OTG_DIEPINT_EPDISD                  EQU    (0x00000002)            ;  Endpoint disabled interrupt
USB_OTG_DIEPINT_TOC                     EQU    (0x00000008)            ;  Timeout condition
USB_OTG_DIEPINT_ITTXFE                  EQU    (0x00000010)            ;  IN token received when TxFIFO is empty
USB_OTG_DIEPINT_INEPNE                  EQU    (0x00000040)            ;  IN endpoint NAK effective
USB_OTG_DIEPINT_TXFE                    EQU    (0x00000080)            ;  Transmit FIFO empty
USB_OTG_DIEPINT_TXFIFOUDRN              EQU    (0x00000100)            ;  Transmit Fifo Underrun
USB_OTG_DIEPINT_BNA                     EQU    (0x00000200)            ;  Buffer not available interrupt
USB_OTG_DIEPINT_PKTDRPSTS               EQU    (0x00000800)            ;  Packet dropped status
USB_OTG_DIEPINT_BERR                    EQU    (0x00001000)            ;  Babble error interrupt
USB_OTG_DIEPINT_NAK                     EQU    (0x00002000)            ;  NAK interrupt

;********************  Bit definition forUSB_OTG_HCINTMSK register  ********************
USB_OTG_HCINTMSK_XFRCM                   EQU    (0x00000001)            ;  Transfer completed mask
USB_OTG_HCINTMSK_CHHM                    EQU    (0x00000002)            ;  Channel halted mask
USB_OTG_HCINTMSK_AHBERR                  EQU    (0x00000004)            ;  AHB error
USB_OTG_HCINTMSK_STALLM                  EQU    (0x00000008)            ;  STALL response received interrupt mask
USB_OTG_HCINTMSK_NAKM                    EQU    (0x00000010)            ;  NAK response received interrupt mask
USB_OTG_HCINTMSK_ACKM                    EQU    (0x00000020)            ;  ACK response received/transmitted interrupt mask
USB_OTG_HCINTMSK_NYET                    EQU    (0x00000040)            ;  response received interrupt mask
USB_OTG_HCINTMSK_TXERRM                  EQU    (0x00000080)            ;  Transaction error mask
USB_OTG_HCINTMSK_BBERRM                  EQU    (0x00000100)            ;  Babble error mask
USB_OTG_HCINTMSK_FRMORM                  EQU    (0x00000200)            ;  Frame overrun mask
USB_OTG_HCINTMSK_DTERRM                  EQU    (0x00000400)            ;  Data toggle error mask

;********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************

USB_OTG_DIEPTSIZ_XFRSIZ                  EQU    (0x0007FFFF)            ;  Transfer size
USB_OTG_DIEPTSIZ_PKTCNT                  EQU    (0x1FF80000)            ;  Packet count
USB_OTG_DIEPTSIZ_MULCNT                  EQU    (0x60000000)            ;  Packet count
;********************  Bit definition forUSB_OTG_HCTSIZ register  ********************
USB_OTG_HCTSIZ_XFRSIZ                    EQU    (0x0007FFFF)            ;  Transfer size
USB_OTG_HCTSIZ_PKTCNT                    EQU    (0x1FF80000)            ;  Packet count
USB_OTG_HCTSIZ_DOPING                    EQU    (0x80000000)            ;  Do PING
USB_OTG_HCTSIZ_DPID                      EQU    (0x60000000)            ;  Data PID
USB_OTG_HCTSIZ_DPID_0                    EQU    (0x20000000)            ; Bit 0
USB_OTG_HCTSIZ_DPID_1                    EQU    (0x40000000)            ; Bit 1

;********************  Bit definition forUSB_OTG_DIEPDMA register  ********************
USB_OTG_DIEPDMA_DMAADDR                  EQU    (0xFFFFFFFF)            ;  DMA address

;********************  Bit definition forUSB_OTG_HCDMA register  ********************
USB_OTG_HCDMA_DMAADDR                    EQU    (0xFFFFFFFF)            ;  DMA address

;********************  Bit definition forUSB_OTG_DTXFSTS register  ********************
USB_OTG_DTXFSTS_INEPTFSAV                EQU    (0x0000FFFF)            ;  IN endpoint TxFIFO space avail

;********************  Bit definition forUSB_OTG_DIEPTXF register  ********************
USB_OTG_DIEPTXF_INEPTXSA                 EQU    (0x0000FFFF)            ;  IN endpoint FIFOx transmit RAM start address
USB_OTG_DIEPTXF_INEPTXFD                 EQU    (0xFFFF0000)            ;  IN endpoint TxFIFO depth

;********************  Bit definition forUSB_OTG_DOEPCTL register  ********************

USB_OTG_DOEPCTL_MPSIZ                     EQU    (0x000007FF)            ;  Maximum packet size          ; Bit 1
USB_OTG_DOEPCTL_USBAEP                    EQU    (0x00008000)            ;  USB active endpoint
USB_OTG_DOEPCTL_NAKSTS                    EQU    (0x00020000)            ;  NAK status
USB_OTG_DOEPCTL_SD0PID_SEVNFRM            EQU    (0x10000000)            ;  Set DATA0 PID
USB_OTG_DOEPCTL_SODDFRM                   EQU    (0x20000000)            ;  Set odd frame
USB_OTG_DOEPCTL_EPTYP                     EQU    (0x000C0000)            ;  Endpoint type
USB_OTG_DOEPCTL_EPTYP_0                   EQU    (0x00040000)            ;  Bit 0
USB_OTG_DOEPCTL_EPTYP_1                   EQU    (0x00080000)            ;  Bit 1
USB_OTG_DOEPCTL_SNPM                      EQU    (0x00100000)            ;  Snoop mode
USB_OTG_DOEPCTL_STALL                     EQU    (0x00200000)            ;  STALL handshake
USB_OTG_DOEPCTL_CNAK                      EQU    (0x04000000)            ;  Clear NAK
USB_OTG_DOEPCTL_SNAK                      EQU    (0x08000000)            ;  Set NAK
USB_OTG_DOEPCTL_EPDIS                     EQU    (0x40000000)            ;  Endpoint disable
USB_OTG_DOEPCTL_EPENA                     EQU    (0x80000000)            ;  Endpoint enable

;********************  Bit definition forUSB_OTG_DOEPINT register  ********************
USB_OTG_DOEPINT_XFRC                    EQU    (0x00000001)            ;  Transfer completed interrupt
USB_OTG_DOEPINT_EPDISD                  EQU    (0x00000002)            ;  Endpoint disabled interrupt
USB_OTG_DOEPINT_STUP                    EQU    (0x00000008)            ;  SETUP phase done
USB_OTG_DOEPINT_OTEPDIS                 EQU    (0x00000010)            ;  OUT token received when endpoint disabled
USB_OTG_DOEPINT_B2BSTUP                 EQU    (0x00000040)            ;  Back-to-back SETUP packets received
USB_OTG_DOEPINT_NYET                    EQU    (0x00004000)            ;  NYET interrupt

;********************  Bit definition forUSB_OTG_DOEPTSIZ register  ********************

USB_OTG_DOEPTSIZ_XFRSIZ                  EQU    (0x0007FFFF)            ;  Transfer size
USB_OTG_DOEPTSIZ_PKTCNT                  EQU    (0x1FF80000)            ;  Packet count

USB_OTG_DOEPTSIZ_STUPCNT                 EQU    (0x60000000)            ;  SETUP packet count
USB_OTG_DOEPTSIZ_STUPCNT_0               EQU    (0x20000000)            ;  Bit 0
USB_OTG_DOEPTSIZ_STUPCNT_1               EQU    (0x40000000)            ;  Bit 1

;********************  Bit definition for PCGCCTL register  ********************
USB_OTG_PCGCCTL_STOPCLK                 EQU    (0x00000001)            ;  SETUP packet count
USB_OTG_PCGCCTL_GATECLK                 EQU    (0x00000002)            ;  Bit 0
USB_OTG_PCGCCTL_PHYSUSP                 EQU    (0x00000010)            ;  Bit 1


    END



