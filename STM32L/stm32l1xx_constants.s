;******************** (C) Yifeng ZHU ******************************************************************
; @file    stm32l1xx_constants.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    April-22-2012
; @note    Modifed from stm32l1xx.h (C) 2010 STMicroelectronics
; @brief   Assembly version of CMSIS Cortex-M4 Device 
;          Peripheral Access Layer Header File for STM32L1xx devices
; @note
;          This code is for the book "Embedded Systems with ARM Cortex-M 
;          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;          ISBN-10: 0982692633.
; @attension
;          This code is provided for education purpose. The author shall not be 
;          held liable for any direct, indirect or consequential damages, for any 
;          reason whatever. More information can be found from book website: 
;          http://www.eece.maine.edu/~zhu/book
;******************************************************************************************************

 
HSE_VALUE                EQU    (8000000)    ; Value of the External High Speed oscillator (HSE) in Hz
HSE_STARTUP_TIMEOUT      EQU    (0x0500)     ; Time out for External High Speed oscillator (HSE) start up 
HSI_STARTUP_TIMEOUT      EQU    (0x0500)     ; Time out for Internal High Speed oscillator (HSI)  start up 
HSI_VALUE                EQU    (16000000)   ; Value of the Internal High Speed oscillator in Hz.
                                             ; The real value may vary depending on the variations
                                             ; in voltage and temperature.
LSI_VALUE                EQU    (37000)      ; Value of the Internal Low Speed oscillator in Hz
                                             ; The real value may vary depending on the variations
                                             ; in voltage and temperature.
LSE_VALUE                EQU    (32768)      ; Value of the External Low Speed oscillator in Hz 

; STM32L1xx Standard Peripheral Library version number
__STM32L1XX_STDPERIPH_VERSION_MAIN   EQU   (0x01) ; [31:24] main version                                   
__STM32L1XX_STDPERIPH_VERSION_SUB1   EQU   (0x00) ; [23:16] sub1 version 
__STM32L1XX_STDPERIPH_VERSION_SUB2   EQU   (0x00) ; [15:8]  sub2 version 
__STM32L1XX_STDPERIPH_VERSION_RC     EQU   (0x00) ; [7:0]  release candidate  




; Configuration_section_for_CMSIS

; STM32L1xx Interrupt Number Definition, according to the selected device 
; Library_configuration_section 
__MPU_PRESENT               EQU   1 ; STM32L provides MPU                        
__NVIC_PRIO_BITS            EQU   4 ; STM32 uses 4 Bits for the Priority Levels  
__Vendor_SysTickConfig      EQU   0 ; Set to 1 if different SysTick Config is used 

; Interrupt Number Definition (IRQn)
; ************* Cortex-M3 Processor Exceptions Numbers ****************************************
NonMaskableInt_IRQn         EQU -14    ; 2 Non Maskable Interrupt                              
MemoryManagement_IRQn       EQU -12    ; 4 Cortex-M3 Memory Management Interrupt               
BusFault_IRQn               EQU -11    ; 5 Cortex-M3 Bus Fault Interrupt                       
UsageFault_IRQn             EQU -10    ; 6 Cortex-M3 Usage Fault Interrupt                     
SVCall_IRQn                 EQU -5     ; 11 Cortex-M3 SV Call Interrupt                        
DebugMonitor_IRQn           EQU -4     ; 12 Cortex-M3 Debug Monitor Interrupt                  
PendSV_IRQn                 EQU -2     ; 14 Cortex-M3 Pend SV Interrupt                        
SysTick_IRQn                EQU -1     ; 15 Cortex-M3 System Tick Interrupt                    
  
; ************* STM32L specific Interrupt Numbers *********************************************
WWDG_IRQn                   EQU 0      ; Window WatchDog Interrupt                             
PVD_IRQn                    EQU 1      ; PVD through EXTI Line detection Interrupt             
TAMPER_STAMP_IRQn           EQU 2      ; Tamper and Time Stamp through EXTI Line Interrupts    
RTC_WKUP_IRQn               EQU 3      ; RTC Wakeup Timer through EXTI Line Interrupt          
FLASH_IRQn                  EQU 4      ; FLASH global Interrupt                                
RCC_IRQn                    EQU 5      ; RCC global Interrupt                                  
EXTI0_IRQn                  EQU 6      ; EXTI Line0 Interrupt                                  
EXTI1_IRQn                  EQU 7      ; EXTI Line1 Interrupt                                  
EXTI2_IRQn                  EQU 8      ; EXTI Line2 Interrupt                                  
EXTI3_IRQn                  EQU 9      ; EXTI Line3 Interrupt                                  
EXTI4_IRQn                  EQU 10     ; EXTI Line4 Interrupt                                  
DMA1_Channel1_IRQn          EQU 11     ; DMA1 Channel 1 global Interrupt                       
DMA1_Channel2_IRQn          EQU 12     ; DMA1 Channel 2 global Interrupt                       
DMA1_Channel3_IRQn          EQU 13     ; DMA1 Channel 3 global Interrupt                       
DMA1_Channel4_IRQn          EQU 14     ; DMA1 Channel 4 global Interrupt                       
DMA1_Channel5_IRQn          EQU 15     ; DMA1 Channel 5 global Interrupt                       
DMA1_Channel6_IRQn          EQU 16     ; DMA1 Channel 6 global Interrupt                       
DMA1_Channel7_IRQn          EQU 17     ; DMA1 Channel 7 global Interrupt                       
ADC1_IRQn                   EQU 18     ; ADC1 global Interrupt                                 
USB_HP_IRQn                 EQU 19     ; USB High Priority Interrupt                           
USB_LP_IRQn                 EQU 20     ; USB Low Priority Interrupt                            
DAC_IRQn                    EQU 21     ; DAC Interrupt                                         
COMP_IRQn                   EQU 22     ; Comparator through EXTI Line Interrupt                
EXTI9_5_IRQn                EQU 23     ; External Line[9:5] Interrupts                         
LCD_IRQn                    EQU 24     ; LCD Interrupt                                         
TIM9_IRQn                   EQU 25     ; TIM9 global Interrupt                                 
TIM10_IRQn                  EQU 26     ; TIM10 global Interrupt                                
TIM11_IRQn                  EQU 27     ; TIM11 global Interrupt                                
TIM2_IRQn                   EQU 28     ; TIM2 global Interrupt                                 
TIM3_IRQn                   EQU 29     ; TIM3 global Interrupt                                 
TIM4_IRQn                   EQU 30     ; TIM4 global Interrupt                                 
I2C1_EV_IRQn                EQU 31     ; I2C1 Event Interrupt                                  
I2C1_ER_IRQn                EQU 32     ; I2C1 Error Interrupt                                  
I2C2_EV_IRQn                EQU 33     ; I2C2 Event Interrupt                                  
I2C2_ER_IRQn                EQU 34     ; I2C2 Error Interrupt                                  
SPI1_IRQn                   EQU 35     ; SPI1 global Interrupt                                 
SPI2_IRQn                   EQU 36     ; SPI2 global Interrupt                                 
USART1_IRQn                 EQU 37     ; USART1 global Interrupt                               
USART2_IRQn                 EQU 38     ; USART2 global Interrupt                               
USART3_IRQn                 EQU 39     ; USART3 global Interrupt                               
EXTI15_10_IRQn              EQU 40     ; External Line[15:10] Interrupts                       
RTC_Alarm_IRQn              EQU 41     ; RTC Alarm through EXTI Line Interrupt                 
USB_FS_WKUP_IRQn            EQU 42     ; USB FS WakeUp from suspend through EXTI Line Interrupt
TIM6_IRQn                   EQU 43     ; TIM6 global Interrupt                                 
TIM7_IRQn                   EQU 44     ; TIM7 global Interrupt                                 



; Peripheral_registers_structures
    
;************ Analog to Digital Converter(ADC) ************
ADC_SR         EQU   0x00  ;
ADC_CR1        EQU   0x04  ;
ADC_CR2        EQU   0x08  ;
ADC_SMPR1      EQU   0x0C  ;
ADC_SMPR2      EQU   0x10  ;
ADC_SMPR3      EQU   0x14  ;
ADC_JOFR1      EQU   0x18  ;
ADC_JOFR2      EQU   0x1C  ;
ADC_JOFR3      EQU   0x20  ;
ADC_JOFR4      EQU   0x24  ;
ADC_HTR        EQU   0x28  ;
ADC_LTR        EQU   0x2C  ;
ADC_SQR1       EQU   0x30  ;
ADC_SQR2       EQU   0x34  ;
ADC_SQR3       EQU   0x38  ;
ADC_SQR4       EQU   0x3C  ;
ADC_SQR5       EQU   0x40  ;
ADC_JSQR       EQU   0x44  ;
ADC_JDR1       EQU   0x48  ;
ADC_JDR2       EQU   0x4C  ;
ADC_JDR3       EQU   0x50  ;
ADC_JDR4       EQU   0x54  ;
ADC_DR         EQU   0x58  ;

ADC_Common_CSR EQU   0x00  ;
ADC_Common_CCR EQU   0x04  ;


;************ Comparator ************
COMP_CSR       EQU   0x00;

 
;************ CRC calculation unit ************
CRC_DR         EQU   0x00  ;
CRC_IDR        EQU   0x04  ; uint8_t
CRC_RESERVED0  EQU   0x05  ; uint8_t
CRC_RESERVED1  EQU   0x06  ; uint16_t
CRC_CR         EQU   0x08  ;

 
;************ Digital to Analog Converter ************
DAC_CR             EQU   0x00  ;
DAC_SWTRIGR        EQU   0x04  ;
DAC_DHR12R1        EQU   0x08  ;
DAC_DHR12L1        EQU   0x0C  ;
DAC_DHR8R1         EQU   0x10  ;
DAC_DHR12R2        EQU   0x14  ;
DAC_DHR12L2        EQU   0x18  ;
DAC_DHR8R2         EQU   0x1C  ;
DAC_DHR12RD        EQU   0x20  ;
DAC_DHR12LD        EQU   0x24  ;
DAC_DHR8RD         EQU   0x28  ;
DAC_DOR1           EQU   0x2C  ;
DAC_DOR2           EQU   0x30  ;
DAC_SR             EQU   0x34  ;  

 
;************ Debug MCU ************
DBGMCU_IDCODE      EQU   0x00  ;
DBGMCU_CR          EQU   0x04  ;
DBGMCU_APB1FZ      EQU   0x08  ;
DBGMCU_APB2FZ      EQU   0x0C  ;

 
;************ DMA Controller ************
DMA_Channel_CCR    EQU   0x00  ;
DMA_Channel_CNDTR  EQU   0x04  ;
DMA_Channel_CPAR   EQU   0x08  ;
DMA_Channel_CMAR   EQU   0x0C  ;

DMA_ISR            EQU   0x00  ;
DMA_IFCR           EQU   0x04  ;


 
;************ External Interrupt/Event Controller ************
EXTI_IMR           EQU   0x00  ;
EXTI_EMR           EQU   0x04  ;
EXTI_RTSR          EQU   0x08  ;
EXTI_FTSR          EQU   0x0C  ;
EXTI_SWIER         EQU   0x10  ;
EXTI_PR            EQU   0x14  ;

 
;************ FLASH Registers ************
FLASH_ACR          EQU   0x00  
FLASH_PECR         EQU   0x04  
FLASH_PDKEYR       EQU   0x08  
FLASH_PEKEYR       EQU   0x0C  
FLASH_PRGKEYR      EQU   0x10  
FLASH_OPTKEYR      EQU   0x14  
FLASH_SR           EQU   0x18  
FLASH_OBR          EQU   0x1C  
FLASH_WRPR         EQU   0x20  

 
;************ Option Bytes Registers ************
OB_RDP             EQU   0x00
OB_USER            EQU   0x04
OB_WRP01           EQU   0x08
OB_WRP23           EQU   0x0C

 
;************ General Purpose IO ************
GPIO_MODER      EQU   0x00;
GPIO_OTYPER     EQU   0x04; uint16_t
GPIO_RESERVED0  EQU   0x06; uint16_t
GPIO_OSPEEDR    EQU   0x08;
GPIO_PUPDR      EQU   0x0C;
GPIO_IDR        EQU   0x10; uint16_t
GPIO_RESERVED1  EQU   0x12; uint16_t
GPIO_ODR        EQU   0x14; uint16_t
GPIO_RESERVED2  EQU   0x16; uint16_t
GPIO_BSRRL      EQU   0x18; uint16_t BSRR register is split to 2 * 16-bit fields BSRRL 
GPIO_BSRRH      EQU   0x1A; uint16_t BSRR register is split to 2 * 16-bit fields BSRRH 
GPIO_LCKR       EQU   0x1C;
GPIO_AFR0       EQU   0x20;  AFR[0]
GPIO_AFR1       EQU   0x24;  AFR[1]
GPIO_AFRL       EQU   0x20
GPIO_AFRH       EQU   0x24
  
 
;************ SysTem Configuration ************
SYSCFG_MEMRMP   EQU   0x00  ;
SYSCFG_PMC      EQU   0x04  ;
SYSCFG_EXTICR0  EQU   0x08  ;
SYSCFG_EXTICR1  EQU   0x0C  ;
SYSCFG_EXTICR2  EQU   0x10  ;
SYSCFG_EXTICR3  EQU   0x14  ;

 
;************ Inter-integrated Circuit Interface ************
I2C_CR1         EQU   0x00  ; uint16_t
I2C_RESERVED0   EQU   0x02  ; uint16_t
I2C_CR2         EQU   0x04  ; uint16_t
I2C_RESERVED1   EQU   0x06  ; uint16_t
I2C_OAR1        EQU   0x08  ; uint16_t
I2C_RESERVED2   EQU   0x0A  ; uint16_t
I2C_OAR2        EQU   0x0C  ; uint16_t
I2C_RESERVED3   EQU   0x0E  ; uint16_t
I2C_DR          EQU   0x10  ; uint16_t
I2C_RESERVED4   EQU   0x12  ; uint16_t
I2C_SR1         EQU   0x14  ; uint16_t
I2C_RESERVED5   EQU   0x16  ; uint16_t
I2C_SR2         EQU   0x18  ; uint16_t
I2C_RESERVED6   EQU   0x1A  ; uint16_t
I2C_CCR         EQU   0x1C  ; uint16_t
I2C_RESERVED7   EQU   0x1E  ; uint16_t
I2C_TRISE       EQU   0x20  ; uint16_t
I2C_RESERVED8   EQU   0x22  ; uint16_t

 
;************ Independent WATCHDOG ************
WDG_KR          EQU   0x00  
WDG_PR          EQU   0x04  
WDG_RLR         EQU   0x08  
WDG_SR          EQU   0x0C  


;************ LCD ************
LCD_CR          EQU   0x00
LCD_FCR         EQU   0x04
LCD_SR          EQU   0x08
LCD_CLR         EQU   0x0C
LCD_RESERVED    EQU   0x10
LCD_RAM0        EQU   0x14  ; LCD_RAM[0]
LCD_RAM1        EQU   0x18  ; LCD_RAM[1]
LCD_RAM2        EQU   0x1C  ; LCD_RAM[2]
LCD_RAM3        EQU   0x20  ; LCD_RAM[3]
LCD_RAM4        EQU   0x24  ; LCD_RAM[4]
LCD_RAM5        EQU   0x28  ; LCD_RAM[5]
LCD_RAM6        EQU   0x2C  ; LCD_RAM[6]
LCD_RAM7        EQU   0x30  ; LCD_RAM[7]
LCD_RAM8        EQU   0x34  ; LCD_RAM[8]
LCD_RAM9        EQU   0x38  ; LCD_RAM[9]
LCD_RAM10       EQU   0x3C  ; LCD_RAM[10]
LCD_RAM11       EQU   0x40  ; LCD_RAM[11]
LCD_RAM12       EQU   0x44  ; LCD_RAM[12]
LCD_RAM13       EQU   0x48  ; LCD_RAM[13]
LCD_RAM14       EQU   0x4C  ; LCD_RAM[14]
LCD_RAM15       EQU   0x50  ; LCD_RAM[15]

 
;************ Power Control ************
PWR_CR          EQU   0x00
PWR_CSR         EQU   0x04

 
;************ Reset and Clock Control ************
RCC_CR          EQU   0x00
RCC_ICSCR       EQU   0x04
RCC_CFGR        EQU   0x08
RCC_CIR         EQU   0x0C
RCC_AHBRSTR     EQU   0x10
RCC_APB2RSTR    EQU   0x14
RCC_APB1RSTR    EQU   0x18
RCC_AHBENR      EQU   0x1C
RCC_APB2ENR     EQU   0x20
RCC_APB1ENR     EQU   0x24
RCC_AHBLPENR    EQU   0x28
RCC_APB2LPENR   EQU   0x2C
RCC_APB1LPENR   EQU   0x30      
RCC_CSR         EQU   0x34    

 
;************ Routing Interface ************
RI_ICR          EQU   0x00
RI_ASCR1        EQU   0x04
RI_ASCR2        EQU   0x08
RI_HYSCR1       EQU   0x0C
RI_HYSCR2       EQU   0x10
RI_HYSCR3       EQU   0x14

 
;************ Real-Time Clock ************
RTC_TR          EQU   0x00
RTC_DR          EQU   0x04
RTC_CR          EQU   0x08
RTC_ISR         EQU   0x0C
RTC_PRER        EQU   0x10
RTC_WUTR        EQU   0x14
RTC_CALIBR      EQU   0x18
RTC_ALRMAR      EQU   0x1C
RTC_ALRMBR      EQU   0x20
RTC_WPR         EQU   0x24
RTC_RESERVED1   EQU   0x28
RTC_RESERVED2   EQU   0x2C
RTC_TSTR        EQU   0x30
RTC_TSDR        EQU   0x34
RTC_RESERVED3   EQU   0x38
RTC_RESERVED4   EQU   0x3C
RTC_TAFCR       EQU   0x40
RTC_RESERVED5   EQU   0x44
RTC_RESERVED6   EQU   0x48
RTC_RESERVED7   EQU   0x4C
RTC_BKP0R       EQU   0x50
RTC_BKP1R       EQU   0x54
RTC_BKP2R       EQU   0x58
RTC_BKP3R       EQU   0x5C
RTC_BKP4R       EQU   0x60
RTC_BKP5R       EQU   0x64
RTC_BKP6R       EQU   0x68
RTC_BKP7R       EQU   0x6C
RTC_BKP8R       EQU   0x70
RTC_BKP9R       EQU   0x74
RTC_BKP10R      EQU   0x78
RTC_BKP11R      EQU   0x7C
RTC_BKP12R      EQU   0x80
RTC_BKP13R      EQU   0x84
RTC_BKP14R      EQU   0x88
RTC_BKP15R      EQU   0x8C
RTC_BKP16R      EQU   0x90
RTC_BKP17R      EQU   0x94
RTC_BKP18R      EQU   0x98
RTC_BKP19R      EQU   0x9C

 
;************ Serial Peripheral Interface ************
SPI_CR1         EQU   0x00    ; uint16_t
SPI_RESERVED0   EQU   0x02    ; uint16_t
SPI_CR2         EQU   0x04    ; uint16_t
SPI_RESERVED1   EQU   0x06    ; uint16_t
SPI_SR          EQU   0x08    ; uint16_t
SPI_RESERVED2   EQU   0x0A    ; uint16_t
SPI_DR          EQU   0x0C    ; uint16_t
SPI_RESERVED3   EQU   0x0E    ; uint16_t
SPI_CRCPR       EQU   0x10    ; uint16_t
SPI_RESERVED4   EQU   0x12    ; uint16_t
SPI_RXCRCR      EQU   0x14    ; uint16_t
SPI_RESERVED5   EQU   0x16    ; uint16_t
SPI_TXCRCR      EQU   0x18    ; uint16_t
SPI_RESERVED6   EQU   0x1A    ; uint16_t

 
;************ TIM ************
TIM_CR1            EQU   0x00  ; uint16_t          
TIM_RESERVED0      EQU   0x02  ; uint16_t  
TIM_CR2            EQU   0x04  ; uint16_t  
TIM_RESERVED1      EQU   0x06  ; uint16_t  
TIM_SMCR           EQU   0x08  ; uint16_t  
TIM_RESERVED2      EQU   0x0A  ; uint16_t  
TIM_DIER           EQU   0x0C  ; uint16_t  
TIM_RESERVED3      EQU   0x0E  ; uint16_t  
TIM_SR             EQU   0x10  ; uint16_t  
TIM_RESERVED4      EQU   0x12  ; uint16_t  
TIM_EGR            EQU   0x14  ; uint16_t  
TIM_RESERVED5      EQU   0x16  ; uint16_t  
TIM_CCMR1          EQU   0x18  ; uint16_t  
TIM_RESERVED6      EQU   0x1A  ; uint16_t  
TIM_CCMR2          EQU   0x1C  ; uint16_t  
TIM_RESERVED7      EQU   0x1E  ; uint16_t  
TIM_CCER           EQU   0x20  ; uint16_t  
TIM_RESERVED8      EQU   0x22  ; uint16_t  
TIM_CNT            EQU   0x24  ; uint16_t  
TIM_RESERVED9      EQU   0x26  ; uint16_t  
TIM_PSC            EQU   0x28  ; uint16_t  
TIM_RESERVED10     EQU   0x2A  ; uint16_t  
TIM_ARR            EQU   0x2C  ; uint16_t  
TIM_RESERVED11     EQU   0x2E  ; uint16_t  
TIM_RESERVED12     EQU   0x30  ;    uint32_t  
TIM_CCR1           EQU   0x34  ; uint16_t  
TIM_RESERVED13     EQU   0x36  ; uint16_t  
TIM_CCR2           EQU   0x38  ; uint16_t  
TIM_RESERVED14     EQU   0x3A  ; uint16_t  
TIM_CCR3           EQU   0x3C  ; uint16_t  
TIM_RESERVED15     EQU   0x3E  ; uint16_t  
TIM_CCR4           EQU   0x40  ; uint16_t  
TIM_RESERVED16     EQU   0x42  ; uint16_t  
TIM_RESERVED17     EQU   0x44  ;    uint32_t  
TIM_DCR            EQU   0x48  ; uint16_t  
TIM_RESERVED18     EQU   0x4A  ; uint16_t  
TIM_DMAR           EQU   0x4C  ; uint16_t  
TIM_RESERVED19     EQU   0x4E  ; uint16_t  
TIM_OR             EQU   0x50  ; uint16_t  
TIM_RESERVED20     EQU   0x52  ; uint16_t  

 
;************ Universal Synchronous Asynchronous Receiver Transmitter ************
USART_SR           EQU   0x00 ;uint16_t
USART_RESERVED0    EQU   0x02 ;uint16_t
USART_DR           EQU   0x04 ;uint16_t
USART_RESERVED1    EQU   0x06 ;uint16_t
USART_BRR          EQU   0x08 ;uint16_t
USART_RESERVED2    EQU   0x0A ;uint16_t
USART_CR1          EQU   0x0C ;uint16_t
USART_RESERVED3    EQU   0x0E ;uint16_t
USART_CR2          EQU   0x10 ;uint16_t
USART_RESERVED4    EQU   0x12 ;uint16_t
USART_CR3          EQU   0x14 ;uint16_t
USART_RESERVED5    EQU   0x16 ;uint16_t
USART_GTPR         EQU   0x18 ;uint16_t
USART_RESERVED6    EQU   0x1A ;uint16_t


;************ Window WATCHDOG ************
WWDG_CR            EQU   0x00
WWDG_CFR           EQU   0x04
WWDG_SR            EQU   0x08


;**************************************************************************************
;
;      Peripheral Memory Map
;
;**************************************************************************************
FLASH_BASE            EQU   (0x08000000) ; FLASH base address in the alias region 
SRAM_BASE             EQU   (0x20000000) ; SRAM base address in the alias region 
PERIPH_BASE           EQU   (0x40000000) ; Peripheral base address in the alias region 
VECT_TAB_OFFSET       EQU   (0x0)        ; Vector Table base offset field

SRAM_BB_BASE          EQU   (0x22000000) ; SRAM base address in the bit-band region 
PERIPH_BB_BASE        EQU   (0x42000000) ; Peripheral base address in the bit-band region 

APB1PERIPH_BASE       EQU   (PERIPH_BASE)
APB2PERIPH_BASE       EQU   (PERIPH_BASE + 0x10000)
AHBPERIPH_BASE        EQU   (PERIPH_BASE + 0x20000)

TIM2_BASE             EQU   (APB1PERIPH_BASE + 0x0000)
TIM3_BASE             EQU   (APB1PERIPH_BASE + 0x0400)
TIM4_BASE             EQU   (APB1PERIPH_BASE + 0x0800)
TIM6_BASE             EQU   (APB1PERIPH_BASE + 0x1000)
TIM7_BASE             EQU   (APB1PERIPH_BASE + 0x1400)
LCD_BASE              EQU   (APB1PERIPH_BASE + 0x2400)
RTC_BASE              EQU   (APB1PERIPH_BASE + 0x2800)
WWDG_BASE             EQU   (APB1PERIPH_BASE + 0x2C00)
IWDG_BASE             EQU   (APB1PERIPH_BASE + 0x3000)
SPI2_BASE             EQU   (APB1PERIPH_BASE + 0x3800)
USART2_BASE           EQU   (APB1PERIPH_BASE + 0x4400)
USART3_BASE           EQU   (APB1PERIPH_BASE + 0x4800)
I2C1_BASE             EQU   (APB1PERIPH_BASE + 0x5400)
I2C2_BASE             EQU   (APB1PERIPH_BASE + 0x5800)
PWR_BASE              EQU   (APB1PERIPH_BASE + 0x7000)
DAC_BASE              EQU   (APB1PERIPH_BASE + 0x7400)
COMP_BASE             EQU   (APB1PERIPH_BASE + 0x7C00)
RI_BASE               EQU   (APB1PERIPH_BASE + 0x7C04)

SYSCFG_BASE           EQU   (APB2PERIPH_BASE + 0x0000)
EXTI_BASE             EQU   (APB2PERIPH_BASE + 0x0400)
TIM9_BASE             EQU   (APB2PERIPH_BASE + 0x0800)
TIM10_BASE            EQU   (APB2PERIPH_BASE + 0x0C00)
TIM11_BASE            EQU   (APB2PERIPH_BASE + 0x1000)
ADC1_BASE             EQU   (APB2PERIPH_BASE + 0x2400)
ADC_BASE              EQU   (APB2PERIPH_BASE + 0x2700)
SPI1_BASE             EQU   (APB2PERIPH_BASE + 0x3000)
USART1_BASE           EQU   (APB2PERIPH_BASE + 0x3800)

GPIOA_BASE            EQU   (AHBPERIPH_BASE + 0x0000)
GPIOB_BASE            EQU   (AHBPERIPH_BASE + 0x0400)
GPIOC_BASE            EQU   (AHBPERIPH_BASE + 0x0800)
GPIOD_BASE            EQU   (AHBPERIPH_BASE + 0x0C00)
GPIOE_BASE            EQU   (AHBPERIPH_BASE + 0x1000)
GPIOH_BASE            EQU   (AHBPERIPH_BASE + 0x1400)
CRC_BASE              EQU   (AHBPERIPH_BASE + 0x3000)
RCC_BASE              EQU   (AHBPERIPH_BASE + 0x3800)

FLASH_R_BASE          EQU   (AHBPERIPH_BASE + 0x3C00) ; FLASH registers base address 
OB_BASE               EQU   (0x1FF80000)              ; FLASH Option Bytes base address 

DMA1_BASE             EQU   (AHBPERIPH_BASE + 0x6000)
DMA1_Channel1_BASE    EQU   (DMA1_BASE + 0x0008)
DMA1_Channel2_BASE    EQU   (DMA1_BASE + 0x001C)
DMA1_Channel3_BASE    EQU   (DMA1_BASE + 0x0030)
DMA1_Channel4_BASE    EQU   (DMA1_BASE + 0x0044)
DMA1_Channel5_BASE    EQU   (DMA1_BASE + 0x0058)
DMA1_Channel6_BASE    EQU   (DMA1_BASE + 0x006C)
DMA1_Channel7_BASE    EQU   (DMA1_BASE + 0x0080)

DBGMCU_BASE           EQU   (0xE0042000) ; Debug MCU registers base address 


SysTick_CTRL          EQU    0x00  ; SysTick Control and Status Register 
SysTick_LOAD          EQU    0x04  ; SysTick Reload Value Register       
SysTick_VAL           EQU    0x08  ; SysTick Current Value Register      
SysTick_CALIB         EQU    0x0C  ; SysTick Calibration Register       

; Memory mapping of Cortex-M3 Hardware 
SCS_BASE              EQU    (0xE000E000)          ; System Control Space Base Address 
ITM_BASE              EQU    (0xE0000000)          ; ITM Base Address                  
CoreDebug_BASE        EQU    (0xE000EDF0)          ; Core Debug Base Address           
SysTick_BASE          EQU    (SCS_BASE +  0x0010)  ; SysTick Base Address           
   
SCB_BASE              EQU    (SCS_BASE +  0x0D00)  ; System Control Block Base Address 
SCB_CPUID             EQU     0x00  ; CPU ID Base Register                                  
SCB_ICSR              EQU     0x04  ; Interrupt Control State Register                      
SCB_VTOR              EQU     0x08  ; Vector Table Offset Register                          
SCB_AIRCR             EQU     0x0C  ; Application Interrupt / Reset Control Register        
SCB_SCR               EQU     0x10  ; System Control Register                               
SCB_CCR               EQU     0x14  ; Configuration Control Register                        
SCB_HP_1_12           EQU     0x18  ; System Handlers Priority Registers (4-7, 8-11, 12-15) 
SCB_SHCSR             EQU     0x24  ; System Handler Control and State Register             
SCB_CFSR              EQU     0x28  ; Configurable Fault Status Register                    
SCB_HFSR              EQU     0x2C  ; Hard Fault Status Register                            
SCB_DFSR              EQU     0x30  ; Debug Fault Status Register                           
SCB_MMFAR             EQU     0x34  ; Mem Manage Address Register                           
SCB_BFAR              EQU     0x38  ; Bus Fault Address Register                            
SCB_AFSR              EQU     0x3C  ; Auxiliary Fault Status Register                       
SCB_PFR_0             EQU     0x40  ; Processor Feature Register 1
SCB_PFR_1             EQU     0x44  ; Processor Feature Register 2                                                       
SCB_DFR               EQU     0x48  ; Debug Feature Register                                
SCB_ADR               EQU     0x4C  ; Auxiliary Feature Register                            
SCB_MMFR_0            EQU     0x50  ; Memory Model Feature Register 1          
SCB_MMFR_1            EQU     0x54  ; Memory Model Feature Register 2                        
SCB_MMFR_2            EQU     0x58  ; Memory Model Feature Register 3                        
SCB_MMFR_3            EQU     0x5C  ; Memory Model Feature Register 4                        
SCB_ISAR_0            EQU     0x60  ; ISA Feature Register 1  
SCB_ISAR_1            EQU     0x64  ; ISA Feature Register 2     
SCB_ISAR_2            EQU     0x68  ; ISA Feature Register 3     
SCB_ISAR_3            EQU     0x6C  ; ISA Feature Register 4     
SCB_ISAR_4            EQU     0x70  ; ISA Feature Register 5     
   
NVIC_BASE       EQU    (SCS_BASE + 0x0100)  ; NVIC Base Address   
NVIC_ISER0      EQU     0x000                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER1      EQU     0x004                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER2      EQU     0x008                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER3      EQU     0x00C                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER4      EQU     0x010                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER5      EQU     0x014                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER6      EQU     0x018                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)
NVIC_ISER7      EQU     0x01C                ; Interrupt Set-enable Registers (NVIC_ISER0-NVIC_ISER7)

NVIC_ICER0      EQU     0X080                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER1      EQU     0X084                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER2      EQU     0X088                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER3      EQU     0X08C                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER4      EQU     0X090                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER5      EQU     0X094                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER6      EQU     0X098                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)
NVIC_ICER7      EQU     0X09C                ; Interrupt Clear-enable Registers (NVIC_ICER0-NVIC_ICER7)

NVIC_ISPR0      EQU     0X100                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR1      EQU     0X104                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR2      EQU     0X108                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR3      EQU     0X10C                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR4      EQU     0X110                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR5      EQU     0X114                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR6      EQU     0X118                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)
NVIC_ISPR7      EQU     0X11C                ; Interrupt Set-pending Registers (NVIC_ISPR0-NVIC_ISPR7)

NVIC_ICPR0      EQU     0X180                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR1      EQU     0X184                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR2      EQU     0X188                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR3      EQU     0X18C                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR4      EQU     0X190                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR5      EQU     0X194                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR6      EQU     0X198                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)
NVIC_ICPR7      EQU     0X19C                ; Interrupt Clear-pending Registers (NVIC_ICPR0-NVIC_ICPR7)

NVIC_IABR0      EQU     0x200                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR1      EQU     0x204                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR2      EQU     0x208                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR3      EQU     0x20C                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR4      EQU     0x210                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR5      EQU     0x214                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR6      EQU     0x218                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)
NVIC_IABR7      EQU     0x21C                ; Interrupt Active Bit Registers (NVIC_IABR0-NVIC_IABR7)


NVIC_IPR0       EQU     0x300                ; Interrupt Priority Registers (NVIC_IPR0-NVIC_IPR59)
                                            ; an 8-bit priority field for each interrupt and each register holds four priority fields.

NVIC_STIR       EQU     0xE00                ; Software Trigger Interrupt Register (STIR)

  
; Peripheral_declaration
  
;TIM2                EQU   (TIM2_BASE)
;TIM3                EQU   (TIM3_BASE)
;TIM4                EQU   (TIM4_BASE)
;TIM6                EQU   (TIM6_BASE)
;TIM7                EQU   (TIM7_BASE)
;LCD                 EQU   (LCD_BASE)
;RTC                 EQU   (RTC_BASE)
;WWDG                EQU   (WWDG_BASE)
;IWDG                EQU   (IWDG_BASE)
;SPI2                EQU   (SPI2_BASE)
;USART2              EQU   (USART2_BASE)
;USART3              EQU   (USART3_BASE)
;I2C1                EQU   (I2C1_BASE)
;I2C2                EQU   (I2C2_BASE)
;PWR                 EQU   (PWR_BASE)
;DAC                 EQU   (DAC_BASE)
;COMP                EQU   (COMP_BASE)
;RI                  EQU   (RI_BASE)
;SYSCFG              EQU   (SYSCFG_BASE)
;EXTI                EQU   (EXTI_BASE)

;ADC1                EQU   (ADC1_BASE)
;ADC                 EQU   (ADC_BASE)
;TIM9                EQU   (TIM9_BASE)
;TIM10               EQU   (TIM10_BASE)
;TIM11               EQU   (TIM11_BASE)
;SPI1                EQU   (SPI1_BASE)
;USART1              EQU   (USART1_BASE)
;DMA1                EQU   (DMA1_BASE)
;DMA1_Channel1       EQU   (DMA1_Channel1_BASE)
;DMA1_Channel2       EQU   (DMA1_Channel2_BASE)
;DMA1_Channel3       EQU   (DMA1_Channel3_BASE)
;DMA1_Channel4       EQU   (DMA1_Channel4_BASE)
;DMA1_Channel5       EQU   (DMA1_Channel5_BASE)
;DMA1_Channel6       EQU   (DMA1_Channel6_BASE)
;DMA1_Channel7       EQU   (DMA1_Channel7_BASE)
;RCC                 EQU   (RCC_BASE)
;CRC                 EQU   (CRC_BASE)

;GPIOA               EQU   (GPIOA_BASE)
;GPIOB               EQU   (GPIOB_BASE)
;GPIOC               EQU   (GPIOC_BASE)
;GPIOD               EQU   (GPIOD_BASE)
;GPIOE               EQU   (GPIOE_BASE)
;GPIOH               EQU   (GPIOH_BASE)

;FLASH               EQU   (FLASH_R_BASE)
;OB                  EQU   (OB_BASE) 

;DBGMCU              EQU   (DBGMCU_BASE)




; Exported_constants

  
; Peripheral_Registers_Bits_Definition
    
;***************************************************************************
;                          Peripheral Registers Bits Definition             
;***************************************************************************

;***************************************************************************
;                                                                           
;                       Analog to Digital Converter (ADC)                   
;                                                                           
;***************************************************************************

;**************  Bit definitionfor ADC_SR register  *******************
ADC_SR_AWD                          EQU    (0x00000001)        ; Analog watchdog flag 
ADC_SR_EOC                          EQU    (0x00000002)        ; End of conversion 
ADC_SR_JEOC                         EQU    (0x00000004)        ; Injected channel end of conversion 
ADC_SR_JSTRT                        EQU    (0x00000008)        ; Injected channel Start flag 
ADC_SR_STRT                         EQU    (0x00000010)        ; Regular channel Start flag 
ADC_SR_OVR                          EQU    (0x00000020)        ; Overrun flag 
ADC_SR_ADONS                        EQU    (0x00000040)        ;ADC ON status 
ADC_SR_RCNR                         EQU    (0x00000100)        ; Regular channel not ready flag 
ADC_SR_JCNR                         EQU    (0x00000200)        ; Injected channel not ready flag 

;**************  Bit definitionfor ADC_CR1 register  *******************
ADC_CR1_AWDCH                       EQU    (0x0000001F)        ; AWDCH[4:0] bits (Analog watchdog channel select bits) 
ADC_CR1_AWDCH_0                     EQU    (0x00000001)        ; Bit 0 
ADC_CR1_AWDCH_1                     EQU    (0x00000002)        ; Bit 1 
ADC_CR1_AWDCH_2                     EQU    (0x00000004)        ; Bit 2 
ADC_CR1_AWDCH_3                     EQU    (0x00000008)        ; Bit 3 
ADC_CR1_AWDCH_4                     EQU    (0x00000010)        ; Bit 4 

ADC_CR1_EOCIE                       EQU    (0x00000020)        ; Interrupt enable for EOC 
ADC_CR1_AWDIE                       EQU    (0x00000040)        ; Analog Watchdog interrupt enable 
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

ADC_CR1_PDD                         EQU    (0x00010000)        ; Power Down during Delay phase 
ADC_CR1_PDI                         EQU    (0x00020000)        ; Power Down during Idle phase 

ADC_CR1_JAWDEN                      EQU    (0x00400000)        ; Analog watchdog enable on injected channels 
ADC_CR1_AWDEN                       EQU    (0x00800000)        ; Analog watchdog enable on regular channels 

ADC_CR1_RES                         EQU    (0x03000000)        ; RES[1:0] bits (Resolution) 
ADC_CR1_RES_0                       EQU    (0x01000000)        ; Bit 0 
ADC_CR1_RES_1                       EQU    (0x02000000)        ; Bit 1 

ADC_CR1_OVRIE                       EQU    (0x04000000)        ; Overrun interrupt enable 
  
;**************  Bit definitionfor ADC_CR2 register  *******************
ADC_CR2_ADON                        EQU    (0x00000001)        ; A/D Converter ON / OFF 
ADC_CR2_CONT                        EQU    (0x00000002)        ; Continuous Conversion 

ADC_CR2_DELS                        EQU    (0x00000070)        ; DELS[2:0] bits (Delay selection) 
ADC_CR2_DELS_0                      EQU    (0x00000010)        ; Bit 0 
ADC_CR2_DELS_1                      EQU    (0x00000020)        ; Bit 1 
ADC_CR2_DELS_2                      EQU    (0x00000040)        ; Bit 2 

ADC_CR2_DMA                         EQU    (0x00000100)        ; Direct Memory access mode 
ADC_CR2_DDS                         EQU    (0x00000200)        ; DMA disable selection (SingleADC) 
ADC_CR2_EOCS                        EQU    (0x00000400)        ; End of conversion selection 
ADC_CR2_ALIGN                       EQU    (0x00000800)        ; Data Alignment 

ADC_CR2_JEXTSEL                     EQU    (0x000F0000)        ; JEXTSEL[3:0] bits (External event select for injected group) 
ADC_CR2_JEXTSEL_0                   EQU    (0x00010000)        ; Bit 0 
ADC_CR2_JEXTSEL_1                   EQU    (0x00020000)        ; Bit 1 
ADC_CR2_JEXTSEL_2                   EQU    (0x00040000)        ; Bit 2 
ADC_CR2_JEXTSEL_3                   EQU    (0x00080000)        ; Bit 3 

ADC_CR2_JEXTEN                      EQU    (0x00300000)        ; JEXTEN[1:0] bits (External Trigger Conversion mode for injected channels) 
ADC_CR2_JEXTEN_0                    EQU    (0x00100000)        ; Bit 0 
ADC_CR2_JEXTEN_1                    EQU    (0x00200000)        ; Bit 1 

ADC_CR2_JSWSTART                    EQU    (0x00400000)        ; Start Conversion of injected channels 

ADC_CR2_EXTSEL                      EQU    (0x0F000000)        ; EXTSEL[3:0] bits (External Event Select for regular group) 
ADC_CR2_EXTSEL_0                    EQU    (0x01000000)        ; Bit 0 
ADC_CR2_EXTSEL_1                    EQU    (0x02000000)        ; Bit 1 
ADC_CR2_EXTSEL_2                    EQU    (0x04000000)        ; Bit 2 
ADC_CR2_EXTSEL_3                    EQU    (0x08000000)        ; Bit 3 

ADC_CR2_EXTEN                       EQU    (0x30000000)        ; EXTEN[1:0] bits (External Trigger Conversion mode for regular channels) 
ADC_CR2_EXTEN_0                     EQU    (0x10000000)        ; Bit 0 
ADC_CR2_EXTEN_1                     EQU    (0x20000000)        ; Bit 1 

ADC_CR2_SWSTART                     EQU    (0x40000000)        ; Start Conversion of regular channels 

;**************  Bit definitionfor ADC_SMPR1 register  ******************
ADC_SMPR1_SMP20                     EQU    (0x00000007)        ; SMP20[2:0] bits (Channel 20 Sample time selection) 
ADC_SMPR1_SMP20_0                   EQU    (0x00000001)        ; Bit 0 
ADC_SMPR1_SMP20_1                   EQU    (0x00000002)        ; Bit 1 
ADC_SMPR1_SMP20_2                   EQU    (0x00000004)        ; Bit 2 

ADC_SMPR1_SMP21                     EQU    (0x00000038)        ; SMP21[2:0] bits (Channel 21 Sample time selection) 
ADC_SMPR1_SMP21_0                   EQU    (0x00000008)        ; Bit 0 
ADC_SMPR1_SMP21_1                   EQU    (0x00000010)        ; Bit 1 
ADC_SMPR1_SMP21_2                   EQU    (0x00000020)        ; Bit 2 

ADC_SMPR1_SMP22                     EQU    (0x000001C0)        ; SMP22[2:0] bits (Channel 22 Sample time selection) 
ADC_SMPR1_SMP22_0                   EQU    (0x00000040)        ; Bit 0 
ADC_SMPR1_SMP22_1                   EQU    (0x00000080)        ; Bit 1 
ADC_SMPR1_SMP22_2                   EQU    (0x00000100)        ; Bit 2 

ADC_SMPR1_SMP23                     EQU    (0x00000E00)        ; SMP23[2:0] bits (Channel 23 Sample time selection) 
ADC_SMPR1_SMP23_0                   EQU    (0x00000200)        ; Bit 0 
ADC_SMPR1_SMP23_1                   EQU    (0x00000400)        ; Bit 1 
ADC_SMPR1_SMP23_2                   EQU    (0x00000800)        ; Bit 2 

ADC_SMPR1_SMP24                     EQU    (0x00007000)        ; SMP24[2:0] bits (Channel 24 Sample time selection) 
ADC_SMPR1_SMP24_0                   EQU    (0x00001000)        ; Bit 0 
ADC_SMPR1_SMP24_1                   EQU    (0x00002000)        ; Bit 1 
ADC_SMPR1_SMP24_2                   EQU    (0x00004000)        ; Bit 2 

ADC_SMPR1_SMP25                     EQU    (0x00038000)        ; SMP25[2:0] bits (Channel 25 Sample time selection) 
ADC_SMPR1_SMP25_0                   EQU    (0x00008000)        ; Bit 0 
ADC_SMPR1_SMP25_1                   EQU    (0x00010000)        ; Bit 1 
ADC_SMPR1_SMP25_2                   EQU    (0x00020000)        ; Bit 2 

;**************  Bit definitionfor ADC_SMPR2 register  ******************
ADC_SMPR2_SMP10                     EQU    (0x00000007)        ; SMP10[2:0] bits (Channel 10 Sample time selection) 
ADC_SMPR2_SMP10_0                   EQU    (0x00000001)        ; Bit 0 
ADC_SMPR2_SMP10_1                   EQU    (0x00000002)        ; Bit 1 
ADC_SMPR2_SMP10_2                   EQU    (0x00000004)        ; Bit 2 

ADC_SMPR2_SMP11                     EQU    (0x00000038)        ; SMP11[2:0] bits (Channel 11 Sample time selection) 
ADC_SMPR2_SMP11_0                   EQU    (0x00000008)        ; Bit 0 
ADC_SMPR2_SMP11_1                   EQU    (0x00000010)        ; Bit 1 
ADC_SMPR2_SMP11_2                   EQU    (0x00000020)        ; Bit 2 

ADC_SMPR2_SMP12                     EQU    (0x000001C0)        ; SMP12[2:0] bits (Channel 12 Sample time selection) 
ADC_SMPR2_SMP12_0                   EQU    (0x00000040)        ; Bit 0 
ADC_SMPR2_SMP12_1                   EQU    (0x00000080)        ; Bit 1 
ADC_SMPR2_SMP12_2                   EQU    (0x00000100)        ; Bit 2 

ADC_SMPR2_SMP13                     EQU    (0x00000E00)        ; SMP13[2:0] bits (Channel 13 Sample time selection) 
ADC_SMPR2_SMP13_0                   EQU    (0x00000200)        ; Bit 0 
ADC_SMPR2_SMP13_1                   EQU    (0x00000400)        ; Bit 1 
ADC_SMPR2_SMP13_2                   EQU    (0x00000800)        ; Bit 2 

ADC_SMPR2_SMP14                     EQU    (0x00007000)        ; SMP14[2:0] bits (Channel 14 Sample time selection) 
ADC_SMPR2_SMP14_0                   EQU    (0x00001000)        ; Bit 0 
ADC_SMPR2_SMP14_1                   EQU    (0x00002000)        ; Bit 1 
ADC_SMPR2_SMP14_2                   EQU    (0x00004000)        ; Bit 2 

ADC_SMPR2_SMP15                     EQU    (0x00038000)        ; SMP15[2:0] bits (Channel 5 Sample time selection) 
ADC_SMPR2_SMP15_0                   EQU    (0x00008000)        ; Bit 0 
ADC_SMPR2_SMP15_1                   EQU    (0x00010000)        ; Bit 1 
ADC_SMPR2_SMP15_2                   EQU    (0x00020000)        ; Bit 2 

ADC_SMPR2_SMP16                     EQU    (0x001C0000)        ; SMP16[2:0] bits (Channel 16 Sample time selection) 
ADC_SMPR2_SMP16_0                   EQU    (0x00040000)        ; Bit 0 
ADC_SMPR2_SMP16_1                   EQU    (0x00080000)        ; Bit 1 
ADC_SMPR2_SMP16_2                   EQU    (0x00100000)        ; Bit 2 

ADC_SMPR2_SMP17                     EQU    (0x00E00000)        ; SMP17[2:0] bits (Channel 17 Sample time selection) 
ADC_SMPR2_SMP17_0                   EQU    (0x00200000)        ; Bit 0 
ADC_SMPR2_SMP17_1                   EQU    (0x00400000)        ; Bit 1 
ADC_SMPR2_SMP17_2                   EQU    (0x00800000)        ; Bit 2 

ADC_SMPR2_SMP18                     EQU    (0x07000000)        ; SMP18[2:0] bits (Channel 18 Sample time selection) 
ADC_SMPR2_SMP18_0                   EQU    (0x01000000)        ; Bit 0 
ADC_SMPR2_SMP18_1                   EQU    (0x02000000)        ; Bit 1 
ADC_SMPR2_SMP18_2                   EQU    (0x04000000)        ; Bit 2 

ADC_SMPR2_SMP19                     EQU    (0x38000000)        ; SMP19[2:0] bits (Channel 19 Sample time selection) 
ADC_SMPR2_SMP19_0                   EQU    (0x08000000)        ; Bit 0 
ADC_SMPR2_SMP19_1                   EQU    (0x10000000)        ; Bit 1 
ADC_SMPR2_SMP19_2                   EQU    (0x20000000)        ; Bit 2 

;**************  Bit definitionfor ADC_SMPR3 register  ******************
ADC_SMPR3_SMP0                      EQU    (0x00000007)        ; SMP0[2:0] bits (Channel 0 Sample time selection) 
ADC_SMPR3_SMP0_0                    EQU    (0x00000001)        ; Bit 0 
ADC_SMPR3_SMP0_1                    EQU    (0x00000002)        ; Bit 1 
ADC_SMPR3_SMP0_2                    EQU    (0x00000004)        ; Bit 2 
 
ADC_SMPR3_SMP1                      EQU    (0x00000038)        ; SMP1[2:0] bits (Channel 1 Sample time selection) 
ADC_SMPR3_SMP1_0                    EQU    (0x00000008)        ; Bit 0 
ADC_SMPR3_SMP1_1                    EQU    (0x00000010)        ; Bit 1 
ADC_SMPR3_SMP1_2                    EQU    (0x00000020)        ; Bit 2 

ADC_SMPR3_SMP2                      EQU    (0x000001C0)        ; SMP2[2:0] bits (Channel 2 Sample time selection) 
ADC_SMPR3_SMP2_0                    EQU    (0x00000040)        ; Bit 0 
ADC_SMPR3_SMP2_1                    EQU    (0x00000080)        ; Bit 1 
ADC_SMPR3_SMP2_2                    EQU    (0x00000100)        ; Bit 2 

ADC_SMPR3_SMP3                      EQU    (0x00000E00)        ; SMP3[2:0] bits (Channel 3 Sample time selection) 
ADC_SMPR3_SMP3_0                    EQU    (0x00000200)        ; Bit 0 
ADC_SMPR3_SMP3_1                    EQU    (0x00000400)        ; Bit 1 
ADC_SMPR3_SMP3_2                    EQU    (0x00000800)        ; Bit 2 

ADC_SMPR3_SMP4                      EQU    (0x00007000)        ; SMP4[2:0] bits (Channel 4 Sample time selection) 
ADC_SMPR3_SMP4_0                    EQU    (0x00001000)        ; Bit 0 
ADC_SMPR3_SMP4_1                    EQU    (0x00002000)        ; Bit 1 
ADC_SMPR3_SMP4_2                    EQU    (0x00004000)        ; Bit 2 

ADC_SMPR3_SMP5                      EQU    (0x00038000)        ; SMP5[2:0] bits (Channel 5 Sample time selection) 
ADC_SMPR3_SMP5_0                    EQU    (0x00008000)        ; Bit 0 
ADC_SMPR3_SMP5_1                    EQU    (0x00010000)        ; Bit 1 
ADC_SMPR3_SMP5_2                    EQU    (0x00020000)        ; Bit 2 

ADC_SMPR3_SMP6                      EQU    (0x001C0000)        ; SMP6[2:0] bits (Channel 6 Sample time selection) 
ADC_SMPR3_SMP6_0                    EQU    (0x00040000)        ; Bit 0 
ADC_SMPR3_SMP6_1                    EQU    (0x00080000)        ; Bit 1 
ADC_SMPR3_SMP6_2                    EQU    (0x00100000)        ; Bit 2 

ADC_SMPR3_SMP7                      EQU    (0x00E00000)        ; SMP7[2:0] bits (Channel 7 Sample time selection) 
ADC_SMPR3_SMP7_0                    EQU    (0x00200000)        ; Bit 0 
ADC_SMPR3_SMP7_1                    EQU    (0x00400000)        ; Bit 1 
ADC_SMPR3_SMP7_2                    EQU    (0x00800000)        ; Bit 2 

ADC_SMPR3_SMP8                      EQU    (0x07000000)        ; SMP8[2:0] bits (Channel 8 Sample time selection) 
ADC_SMPR3_SMP8_0                    EQU    (0x01000000)        ; Bit 0 
ADC_SMPR3_SMP8_1                    EQU    (0x02000000)        ; Bit 1 
ADC_SMPR3_SMP8_2                    EQU    (0x04000000)        ; Bit 2 

ADC_SMPR3_SMP9                      EQU    (0x38000000)        ; SMP9[2:0] bits (Channel 9 Sample time selection) 
ADC_SMPR3_SMP9_0                    EQU    (0x08000000)        ; Bit 0 
ADC_SMPR3_SMP9_1                    EQU    (0x10000000)        ; Bit 1 
ADC_SMPR3_SMP9_2                    EQU    (0x20000000)        ; Bit 2 


;**************  Bit definitionfor ADC_JOFR1 register  ******************
ADC_JOFR1_JOFFSET1                  EQU    (0x00000FFF)        ; Data offset for injected channel 1 

;**************  Bit definitionfor ADC_JOFR2 register  ******************
ADC_JOFR2_JOFFSET2                  EQU    (0x00000FFF)        ; Data offset for injected channel 2 

;**************  Bit definitionfor ADC_JOFR3 register  ******************
ADC_JOFR3_JOFFSET3                  EQU    (0x00000FFF)        ; Data offset for injected channel 3 

;**************  Bit definitionfor ADC_JOFR4 register  ******************
ADC_JOFR4_JOFFSET4                  EQU    (0x00000FFF)        ; Data offset for injected channel 4 

;**************  Bit definitionfor ADC_HTR register  *******************
ADC_HTR_HT                          EQU    (0x00000FFF)        ; Analog watchdog high threshold 

;**************  Bit definitionfor ADC_LTR register  *******************
ADC_LTR_LT                          EQU    (0x00000FFF)         ; Analog watchdog low threshold 

;**************  Bit definitionfor ADC_SQR1 register  ******************
ADC_SQR1_SQ25                       EQU    (0x0000001F)        ; SQ25[4:0] bits (25th conversion in regular sequence) 
ADC_SQR1_SQ25_0                     EQU    (0x00000001)        ; Bit 0 
ADC_SQR1_SQ25_1                     EQU    (0x00000002)        ; Bit 1 
ADC_SQR1_SQ25_2                     EQU    (0x00000004)        ; Bit 2 
ADC_SQR1_SQ25_3                     EQU    (0x00000008)        ; Bit 3 
ADC_SQR1_SQ25_4                     EQU    (0x00000010)        ; Bit 4 

ADC_SQR1_SQ26                       EQU    (0x000003E0)        ; SQ26[4:0] bits (26th conversion in regular sequence) 
ADC_SQR1_SQ26_0                     EQU    (0x00000020)        ; Bit 0 
ADC_SQR1_SQ26_1                     EQU    (0x00000040)        ; Bit 1 
ADC_SQR1_SQ26_2                     EQU    (0x00000080)        ; Bit 2 
ADC_SQR1_SQ26_3                     EQU    (0x00000100)        ; Bit 3 
ADC_SQR1_SQ26_4                     EQU    (0x00000200)        ; Bit 4 

ADC_SQR1_SQ27                       EQU    (0x00007C00)        ; SQ27[4:0] bits (27th conversion in regular sequence) 
ADC_SQR1_SQ27_0                     EQU    (0x00000400)        ; Bit 0 
ADC_SQR1_SQ27_1                     EQU    (0x00000800)        ; Bit 1 
ADC_SQR1_SQ27_2                     EQU    (0x00001000)        ; Bit 2 
ADC_SQR1_SQ27_3                     EQU    (0x00002000)        ; Bit 3 
ADC_SQR1_SQ27_4                     EQU    (0x00004000)        ; Bit 4 

ADC_SQR1_L                          EQU    (0x00F00000)        ; L[3:0] bits (Regular channel sequence length) 
ADC_SQR1_L_0                        EQU    (0x00100000)        ; Bit 0 
ADC_SQR1_L_1                        EQU    (0x00200000)        ; Bit 1 
ADC_SQR1_L_2                        EQU    (0x00400000)        ; Bit 2 
ADC_SQR1_L_3                        EQU    (0x00800000)        ; Bit 3 

;**************  Bit definitionfor ADC_SQR2 register  ******************
ADC_SQR2_SQ19                       EQU    (0x0000001F)        ; SQ19[4:0] bits (19th conversion in regular sequence) 
ADC_SQR2_SQ19_0                     EQU    (0x00000001)        ; Bit 0 
ADC_SQR2_SQ19_1                     EQU    (0x00000002)        ; Bit 1 
ADC_SQR2_SQ19_2                     EQU    (0x00000004)        ; Bit 2 
ADC_SQR2_SQ19_3                     EQU    (0x00000008)        ; Bit 3 
ADC_SQR2_SQ19_4                     EQU    (0x00000010)        ; Bit 4 

ADC_SQR2_SQ20                       EQU    (0x000003E0)        ; SQ20[4:0] bits (20th conversion in regular sequence) 
ADC_SQR2_SQ20_0                     EQU    (0x00000020)        ; Bit 0 
ADC_SQR2_SQ20_1                     EQU    (0x00000040)        ; Bit 1 
ADC_SQR2_SQ20_2                     EQU    (0x00000080)        ; Bit 2 
ADC_SQR2_SQ20_3                     EQU    (0x00000100)        ; Bit 3 
ADC_SQR2_SQ20_4                     EQU    (0x00000200)        ; Bit 4 

ADC_SQR2_SQ21                       EQU    (0x00007C00)        ; SQ21[4:0] bits (21th conversion in regular sequence) 
ADC_SQR2_SQ21_0                     EQU    (0x00000400)        ; Bit 0 
ADC_SQR2_SQ21_1                     EQU    (0x00000800)        ; Bit 1 
ADC_SQR2_SQ21_2                     EQU    (0x00001000)        ; Bit 2 
ADC_SQR2_SQ21_3                     EQU    (0x00002000)        ; Bit 3 
ADC_SQR2_SQ21_4                     EQU    (0x00004000)        ; Bit 4 

ADC_SQR2_SQ22                       EQU    (0x000F8000)        ; SQ22[4:0] bits (22th conversion in regular sequence) 
ADC_SQR2_SQ22_0                     EQU    (0x00008000)        ; Bit 0 
ADC_SQR2_SQ22_1                     EQU    (0x00010000)        ; Bit 1 
ADC_SQR2_SQ22_2                     EQU    (0x00020000)        ; Bit 2 
ADC_SQR2_SQ22_3                     EQU    (0x00040000)        ; Bit 3 
ADC_SQR2_SQ22_4                     EQU    (0x00080000)        ; Bit 4 

ADC_SQR2_SQ23                       EQU    (0x01F00000)        ; SQ23[4:0] bits (23th conversion in regular sequence) 
ADC_SQR2_SQ23_0                     EQU    (0x00100000)        ; Bit 0 
ADC_SQR2_SQ23_1                     EQU    (0x00200000)        ; Bit 1 
ADC_SQR2_SQ23_2                     EQU    (0x00400000)        ; Bit 2 
ADC_SQR2_SQ23_3                     EQU    (0x00800000)        ; Bit 3 
ADC_SQR2_SQ23_4                     EQU    (0x01000000)        ; Bit 4 

ADC_SQR2_SQ24                       EQU    (0x3E000000)        ; SQ24[4:0] bits (24th conversion in regular sequence) 
ADC_SQR2_SQ24_0                     EQU    (0x02000000)        ; Bit 0 
ADC_SQR2_SQ24_1                     EQU    (0x04000000)        ; Bit 1 
ADC_SQR2_SQ24_2                     EQU    (0x08000000)        ; Bit 2 
ADC_SQR2_SQ24_3                     EQU    (0x10000000)        ; Bit 3 
ADC_SQR2_SQ24_4                     EQU    (0x20000000)        ; Bit 4 

;**************  Bit definitionfor ADC_SQR3 register  ******************
ADC_SQR3_SQ13                       EQU    (0x0000001F)        ; SQ13[4:0] bits (13th conversion in regular sequence) 
ADC_SQR3_SQ13_0                     EQU    (0x00000001)        ; Bit 0 
ADC_SQR3_SQ13_1                     EQU    (0x00000002)        ; Bit 1 
ADC_SQR3_SQ13_2                     EQU    (0x00000004)        ; Bit 2 
ADC_SQR3_SQ13_3                     EQU    (0x00000008)        ; Bit 3 
ADC_SQR3_SQ13_4                     EQU    (0x00000010)        ; Bit 4 

ADC_SQR3_SQ14                       EQU    (0x000003E0)        ; SQ14[4:0] bits (14th conversion in regular sequence) 
ADC_SQR3_SQ14_0                     EQU    (0x00000020)        ; Bit 0 
ADC_SQR3_SQ14_1                     EQU    (0x00000040)        ; Bit 1 
ADC_SQR3_SQ14_2                     EQU    (0x00000080)        ; Bit 2 
ADC_SQR3_SQ14_3                     EQU    (0x00000100)        ; Bit 3 
ADC_SQR3_SQ14_4                     EQU    (0x00000200)        ; Bit 4 

ADC_SQR3_SQ15                       EQU    (0x00007C00)        ; SQ15[4:0] bits (15th conversion in regular sequence) 
ADC_SQR3_SQ15_0                     EQU    (0x00000400)        ; Bit 0 
ADC_SQR3_SQ15_1                     EQU    (0x00000800)        ; Bit 1 
ADC_SQR3_SQ15_2                     EQU    (0x00001000)        ; Bit 2 
ADC_SQR3_SQ15_3                     EQU    (0x00002000)        ; Bit 3 
ADC_SQR3_SQ15_4                     EQU    (0x00004000)        ; Bit 4 

ADC_SQR3_SQ16                       EQU    (0x000F8000)        ; SQ16[4:0] bits (16th conversion in regular sequence) 
ADC_SQR3_SQ16_0                     EQU    (0x00008000)        ; Bit 0 
ADC_SQR3_SQ16_1                     EQU    (0x00010000)        ; Bit 1 
ADC_SQR3_SQ16_2                     EQU    (0x00020000)        ; Bit 2 
ADC_SQR3_SQ16_3                     EQU    (0x00040000)        ; Bit 3 
ADC_SQR3_SQ16_4                     EQU    (0x00080000)        ; Bit 4 

ADC_SQR3_SQ17                       EQU    (0x01F00000)        ; SQ17[4:0] bits (17th conversion in regular sequence) 
ADC_SQR3_SQ17_0                     EQU    (0x00100000)        ; Bit 0 
ADC_SQR3_SQ17_1                     EQU    (0x00200000)        ; Bit 1 
ADC_SQR3_SQ17_2                     EQU    (0x00400000)        ; Bit 2 
ADC_SQR3_SQ17_3                     EQU    (0x00800000)        ; Bit 3 
ADC_SQR3_SQ17_4                     EQU    (0x01000000)        ; Bit 4 

ADC_SQR3_SQ18                       EQU    (0x3E000000)        ; SQ18[4:0] bits (18th conversion in regular sequence) 
ADC_SQR3_SQ18_0                     EQU    (0x02000000)        ; Bit 0 
ADC_SQR3_SQ18_1                     EQU    (0x04000000)        ; Bit 1 
ADC_SQR3_SQ18_2                     EQU    (0x08000000)        ; Bit 2 
ADC_SQR3_SQ18_3                     EQU    (0x10000000)        ; Bit 3 
ADC_SQR3_SQ18_4                     EQU    (0x20000000)        ; Bit 4 

;**************  Bit definitionfor ADC_SQR4 register  ******************
ADC_SQR4_SQ7                        EQU    (0x0000001F)        ; SQ7[4:0] bits (7th conversion in regular sequence) 
ADC_SQR4_SQ7_0                      EQU    (0x00000001)        ; Bit 0 
ADC_SQR4_SQ7_1                      EQU    (0x00000002)        ; Bit 1 
ADC_SQR4_SQ7_2                      EQU    (0x00000004)        ; Bit 2 
ADC_SQR4_SQ7_3                      EQU    (0x00000008)        ; Bit 3 
ADC_SQR4_SQ7_4                      EQU    (0x00000010)        ; Bit 4 

ADC_SQR4_SQ8                        EQU    (0x000003E0)        ; SQ8[4:0] bits (8th conversion in regular sequence) 
ADC_SQR4_SQ8_0                      EQU    (0x00000020)        ; Bit 0 
ADC_SQR4_SQ8_1                      EQU    (0x00000040)        ; Bit 1 
ADC_SQR4_SQ8_2                      EQU    (0x00000080)        ; Bit 2 
ADC_SQR4_SQ8_3                      EQU    (0x00000100)        ; Bit 3 
ADC_SQR4_SQ8_4                      EQU    (0x00000200)        ; Bit 4 

ADC_SQR4_SQ9                        EQU    (0x00007C00)        ; SQ9[4:0] bits (9th conversion in regular sequence) 
ADC_SQR4_SQ9_0                      EQU    (0x00000400)        ; Bit 0 
ADC_SQR4_SQ9_1                      EQU    (0x00000800)        ; Bit 1 
ADC_SQR4_SQ9_2                      EQU    (0x00001000)        ; Bit 2 
ADC_SQR4_SQ9_3                      EQU    (0x00002000)        ; Bit 3 
ADC_SQR4_SQ9_4                      EQU    (0x00004000)        ; Bit 4 

ADC_SQR4_SQ10                       EQU    (0x000F8000)        ; SQ10[4:0] bits (10th conversion in regular sequence) 
ADC_SQR4_SQ10_0                     EQU    (0x00008000)        ; Bit 0 
ADC_SQR4_SQ10_1                     EQU    (0x00010000)        ; Bit 1 
ADC_SQR4_SQ10_2                     EQU    (0x00020000)        ; Bit 2 
ADC_SQR4_SQ10_3                     EQU    (0x00040000)        ; Bit 3 
ADC_SQR4_SQ10_4                     EQU    (0x00080000)        ; Bit 4 

ADC_SQR4_SQ11                       EQU    (0x01F00000)        ; SQ11[4:0] bits (11th conversion in regular sequence) 
ADC_SQR4_SQ11_0                     EQU    (0x00100000)        ; Bit 0 
ADC_SQR4_SQ11_1                     EQU    (0x00200000)        ; Bit 1 
ADC_SQR4_SQ11_2                     EQU    (0x00400000)        ; Bit 2 
ADC_SQR4_SQ11_3                     EQU    (0x00800000)        ; Bit 3 
ADC_SQR4_SQ11_4                     EQU    (0x01000000)        ; Bit 4 

ADC_SQR4_SQ12                       EQU    (0x3E000000)        ; SQ12[4:0] bits (12th conversion in regular sequence) 
ADC_SQR4_SQ12_0                     EQU    (0x02000000)        ; Bit 0 
ADC_SQR4_SQ12_1                     EQU    (0x04000000)        ; Bit 1 
ADC_SQR4_SQ12_2                     EQU    (0x08000000)        ; Bit 2 
ADC_SQR4_SQ12_3                     EQU    (0x10000000)        ; Bit 3 
ADC_SQR4_SQ12_4                     EQU    (0x20000000)        ; Bit 4 

;**************  Bit definitionfor ADC_SQR5 register  ******************
ADC_SQR5_SQ1                        EQU    (0x0000001F)        ; SQ1[4:0] bits (1st conversion in regular sequence) 
ADC_SQR5_SQ1_0                      EQU    (0x00000001)        ; Bit 0 
ADC_SQR5_SQ1_1                      EQU    (0x00000002)        ; Bit 1 
ADC_SQR5_SQ1_2                      EQU    (0x00000004)        ; Bit 2 
ADC_SQR5_SQ1_3                      EQU    (0x00000008)        ; Bit 3 
ADC_SQR5_SQ1_4                      EQU    (0x00000010)        ; Bit 4 

ADC_SQR5_SQ2                        EQU    (0x000003E0)        ; SQ2[4:0] bits (2nd conversion in regular sequence) 
ADC_SQR5_SQ2_0                      EQU    (0x00000020)        ; Bit 0 
ADC_SQR5_SQ2_1                      EQU    (0x00000040)        ; Bit 1 
ADC_SQR5_SQ2_2                      EQU    (0x00000080)        ; Bit 2 
ADC_SQR5_SQ2_3                      EQU    (0x00000100)        ; Bit 3 
ADC_SQR5_SQ2_4                      EQU    (0x00000200)        ; Bit 4 

ADC_SQR5_SQ3                        EQU    (0x00007C00)        ; SQ3[4:0] bits (3rd conversion in regular sequence) 
ADC_SQR5_SQ3_0                      EQU    (0x00000400)        ; Bit 0 
ADC_SQR5_SQ3_1                      EQU    (0x00000800)        ; Bit 1 
ADC_SQR5_SQ3_2                      EQU    (0x00001000)        ; Bit 2 
ADC_SQR5_SQ3_3                      EQU    (0x00002000)        ; Bit 3 
ADC_SQR5_SQ3_4                      EQU    (0x00004000)        ; Bit 4 

ADC_SQR5_SQ4                        EQU    (0x000F8000)        ; SQ4[4:0] bits (4th conversion in regular sequence) 
ADC_SQR5_SQ4_0                      EQU    (0x00008000)        ; Bit 0 
ADC_SQR5_SQ4_1                      EQU    (0x00010000)        ; Bit 1 
ADC_SQR5_SQ4_2                      EQU    (0x00020000)        ; Bit 2 
ADC_SQR5_SQ4_3                      EQU    (0x00040000)        ; Bit 3 
ADC_SQR5_SQ4_4                      EQU    (0x00080000)        ; Bit 4 

ADC_SQR5_SQ5                        EQU    (0x01F00000)        ; SQ5[4:0] bits (5th conversion in regular sequence) 
ADC_SQR5_SQ5_0                      EQU    (0x00100000)        ; Bit 0 
ADC_SQR5_SQ5_1                      EQU    (0x00200000)        ; Bit 1 
ADC_SQR5_SQ5_2                      EQU    (0x00400000)        ; Bit 2 
ADC_SQR5_SQ5_3                      EQU    (0x00800000)        ; Bit 3 
ADC_SQR5_SQ5_4                      EQU    (0x01000000)        ; Bit 4 

ADC_SQR5_SQ6                        EQU    (0x3E000000)        ; SQ6[4:0] bits (6th conversion in regular sequence) 
ADC_SQR5_SQ6_0                      EQU    (0x02000000)        ; Bit 0 
ADC_SQR5_SQ6_1                      EQU    (0x04000000)        ; Bit 1 
ADC_SQR5_SQ6_2                      EQU    (0x08000000)        ; Bit 2 
ADC_SQR5_SQ6_3                      EQU    (0x10000000)        ; Bit 3 
ADC_SQR5_SQ6_4                      EQU    (0x20000000)        ; Bit 4 


;**************  Bit definitionfor ADC_JSQR register  ******************
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

;**************  Bit definitionfor ADC_JDR1 register  ******************
ADC_JDR1_JDATA                      EQU    (0x0000FFFF)        ; Injected data 

;**************  Bit definitionfor ADC_JDR2 register  ******************
ADC_JDR2_JDATA                      EQU    (0x0000FFFF)        ; Injected data 

;**************  Bit definitionfor ADC_JDR3 register  ******************
ADC_JDR3_JDATA                      EQU    (0x0000FFFF)        ; Injected data 

;**************  Bit definitionfor ADC_JDR4 register  ******************
ADC_JDR4_JDATA                      EQU    (0x0000FFFF)        ; Injected data 

*;**************  Bit definitionfor ADC_DR register  *******************
ADC_DR_DATA                         EQU    (0x0000FFFF)        ; Regular data 


;**************  Bit definitionfor ADC_CSR register  *******************
ADC_CSR_AWD1                        EQU    (0x00000001)        ;ADC1 Analog watchdog flag 
ADC_CSR_EOC1                        EQU    (0x00000002)        ;ADC1 End of conversion 
ADC_CSR_JEOC1                       EQU    (0x00000004)        ;ADC1 Injected channel end of conversion 
ADC_CSR_JSTRT1                      EQU    (0x00000008)        ;ADC1 Injected channel Start flag 
ADC_CSR_STRT1                       EQU    (0x00000010)        ;ADC1 Regular channel Start flag 
ADC_CSR_OVR1                        EQU    (0x00000020)        ;ADC1 overrun  flag 
ADC_CSR_ADONS1                      EQU    (0x00000040)        ; ADON status ofADC1 

;**************  Bit definitionfor ADC_CCR register  *******************
ADC_CCR_ADCPRE                      EQU    (0x00030000)        ;ADC prescaler
ADC_CCR_ADCPRE_0                    EQU    (0x00010000)        ; Bit 0 
ADC_CCR_ADCPRE_1                    EQU    (0x00020000)        ; Bit 1  
ADC_ADC_CCR_TSVREFE                 EQU    (0x00800000)        ; Temperature Sensor and VREFINT Enable 



;***************************************************************************
;                                                                           
;                       Analog Comparators (COMP)                           
;                                                                           
;***************************************************************************

;**************  Bit definition for COMP_CSR register  *******************
COMP_CSR_10KPU                      EQU    (0x00000001)        ; 10K pull-up resistor 
COMP_CSR_400KPU                     EQU    (0x00000002)        ; 400K pull-up resistor 
COMP_CSR_10KPD                      EQU    (0x00000004)        ; 10K pull-down resistor 
COMP_CSR_400KPD                     EQU    (0x00000008)        ; 400K pull-down resistor 

COMP_CSR_CMP1EN                     EQU    (0x00000010)        ; Comparator 1 enable 
COMP_CSR_CMP1OUT                    EQU    (0x00000080)        ; Comparator 1 output 

COMP_CSR_SPEED                      EQU    (0x00001000)        ; Comparator 2 speed 
COMP_CSR_CMP2OUT                    EQU    (0x00002000)        ; Comparator 2 ouput 

COMP_CSR_VREFOUTEN                  EQU    (0x00010000)        ; Comparator Vref Enable 
COMP_CSR_WNDWE                      EQU    (0x00020000)        ; Window mode enable 

COMP_CSR_INSEL                      EQU    (0x001C0000)        ; INSEL[2:0] Inversion input Selection 
COMP_CSR_INSEL_0                    EQU    (0x00040000)        ; Bit 0 
COMP_CSR_INSEL_1                    EQU    (0x00080000)        ; Bit 1 
COMP_CSR_INSEL_2                    EQU    (0x00100000)        ; Bit 2 

COMP_CSR_OUTSEL                     EQU    (0x00E00000)        ; OUTSEL[2:0] comparator 2 output redirection 
COMP_CSR_OUTSEL_0                   EQU    (0x00200000)        ; Bit 0 
COMP_CSR_OUTSEL_1                   EQU    (0x00400000)        ; Bit 1 
COMP_CSR_OUTSEL_2                   EQU    (0x00800000)        ; Bit 2 



;***************************************************************************
;                                                                           
;                        CRC calculation unit (CRC)                         
;                                                                           
;***************************************************************************

;**************  Bit definition for CRC_DR register  ********************
CRC_DR_DR                           EQU    (0xFFFFFFFF) ; Data register bits 

;**************  Bit definition for CRC_IDR register  *******************
CRC_IDR_IDR                         EQU    (0xFF)       ; General-purpose 8-bit data register bits 

;**************  Bit definition for CRC_CR register  *******************
CRC_CR_RESET                        EQU    (0x00000001) ; RESET bit 



;***************************************************************************
;                                                                           
;                     Digital to Analog Converter (DAC)                     
;                                                                           
;***************************************************************************

;**************  Bit definition for DAC_CR register  *******************
DAC_CR_EN1                          EQU    (0x00000001)        ; DAC channel1 enable 
DAC_CR_BOFF1                        EQU    (0x00000002)        ; DAC channel1 output buffer disable 
DAC_CR_TEN1                         EQU    (0x00000004)        ; DAC channel1 Trigger enable 

DAC_CR_TSEL1                        EQU    (0x00000038)        ; TSEL1[2:0] (DAC channel1 Trigger selection) 
DAC_CR_TSEL1_0                      EQU    (0x00000008)        ; Bit 0 
DAC_CR_TSEL1_1                      EQU    (0x00000010)        ; Bit 1 
DAC_CR_TSEL1_2                      EQU    (0x00000020)        ; Bit 2 

DAC_CR_WAVE1                        EQU    (0x000000C0)        ; WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) 
DAC_CR_WAVE1_0                      EQU    (0x00000040)        ; Bit 0 
DAC_CR_WAVE1_1                      EQU    (0x00000080)        ; Bit 1 

DAC_CR_MAMP1                        EQU    (0x00000F00)        ; MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) 
DAC_CR_MAMP1_0                      EQU    (0x00000100)        ; Bit 0 
DAC_CR_MAMP1_1                      EQU    (0x00000200)        ; Bit 1 
DAC_CR_MAMP1_2                      EQU    (0x00000400)        ; Bit 2 
DAC_CR_MAMP1_3                      EQU    (0x00000800)        ; Bit 3 

DAC_CR_DMAEN1                       EQU    (0x00001000)        ; DAC channel1 DMA enable 
DAC_CR_EN2                          EQU    (0x00010000)        ; DAC channel2 enable 
DAC_CR_BOFF2                        EQU    (0x00020000)        ; DAC channel2 output buffer disable 
DAC_CR_TEN2                         EQU    (0x00040000)        ; DAC channel2 Trigger enable 

DAC_CR_TSEL2                        EQU    (0x00380000)        ; TSEL2[2:0] (DAC channel2 Trigger selection) 
DAC_CR_TSEL2_0                      EQU    (0x00080000)        ; Bit 0 
DAC_CR_TSEL2_1                      EQU    (0x00100000)        ; Bit 1 
DAC_CR_TSEL2_2                      EQU    (0x00200000)        ; Bit 2 

DAC_CR_WAVE2                        EQU    (0x00C00000)        ; WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) 
DAC_CR_WAVE2_0                      EQU    (0x00400000)        ; Bit 0 
DAC_CR_WAVE2_1                      EQU    (0x00800000)        ; Bit 1 

DAC_CR_MAMP2                        EQU    (0x0F000000)        ; MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) 
DAC_CR_MAMP2_0                      EQU    (0x01000000)        ; Bit 0 
DAC_CR_MAMP2_1                      EQU    (0x02000000)        ; Bit 1 
DAC_CR_MAMP2_2                      EQU    (0x04000000)        ; Bit 2 
DAC_CR_MAMP2_3                      EQU    (0x08000000)        ; Bit 3 

DAC_CR_DMAEN2                       EQU    (0x10000000)        ; DAC channel2 DMA enabled 

;**************  Bit definition for DAC_SWTRIGR register  *****************
DAC_SWTRIGR_SWTRIG1                 EQU    (0x01)               ; DAC channel1 software trigger 
DAC_SWTRIGR_SWTRIG2                 EQU    (0x02)               ; DAC channel2 software trigger 

;**************  Bit definition for DAC_DHR12R1 register  *****************
DAC_DHR12R1_DACC1DHR                EQU    (0x0FFF)            ; DAC channel1 12-bit Right aligned data 

;**************  Bit definition for DAC_DHR12L1 register  *****************
DAC_DHR12L1_DACC1DHR                EQU    (0xFFF0)            ; DAC channel1 12-bit Left aligned data 

;**************  Bit definition for DAC_DHR8R1 register  *****************
DAC_DHR8R1_DACC1DHR                 EQU    (0xFF)               ; DAC channel1 8-bit Right aligned data 

;**************  Bit definition for DAC_DHR12R2 register  *****************
DAC_DHR12R2_DACC2DHR                EQU    (0x0FFF)            ; DAC channel2 12-bit Right aligned data 

;**************  Bit definition for DAC_DHR12L2 register  *****************
DAC_DHR12L2_DACC2DHR                EQU    (0xFFF0)            ; DAC channel2 12-bit Left aligned data 

;**************  Bit definition for DAC_DHR8R2 register  *****************
DAC_DHR8R2_DACC2DHR                 EQU    (0xFF)               ; DAC channel2 8-bit Right aligned data 

;**************  Bit definition for DAC_DHR12RD register  *****************
DAC_DHR12RD_DACC1DHR                EQU    (0x00000FFF)        ; DAC channel1 12-bit Right aligned data 
DAC_DHR12RD_DACC2DHR                EQU    (0x0FFF0000)        ; DAC channel2 12-bit Right aligned data 

;**************  Bit definition for DAC_DHR12LD register  *****************
DAC_DHR12LD_DACC1DHR                EQU    (0x0000FFF0)        ; DAC channel1 12-bit Left aligned data 
DAC_DHR12LD_DACC2DHR                EQU    (0xFFF00000)        ; DAC channel2 12-bit Left aligned data 

;**************  Bit definition for DAC_DHR8RD register  *****************
DAC_DHR8RD_DACC1DHR                 EQU    (0x00FF)            ; DAC channel1 8-bit Right aligned data 
DAC_DHR8RD_DACC2DHR                 EQU    (0xFF00)            ; DAC channel2 8-bit Right aligned data 

;**************  Bit definition for DAC_DOR1 register  ******************
DAC_DOR1_DACC1DOR                   EQU    (0x0FFF)            ; DAC channel1 data output 

;**************  Bit definition for DAC_DOR2 register  ******************
DAC_DOR2_DACC2DOR                   EQU    (0x0FFF)            ; DAC channel2 data output 

;**************  Bit definition for DAC_SR register  *******************
DAC_SR_DMAUDR1                      EQU    (0x00002000)        ; DAC channel1 DMA underrun flag 
DAC_SR_DMAUDR2                      EQU    (0x20000000)        ; DAC channel2 DMA underrun flag 



;***************************************************************************
;                                                                           
;                            Debug MCU (DBGMCU)                             
;                                                                           
;***************************************************************************

;**************  Bit definition for DBGMCU_IDCODE register  ****************
DBGMCU_IDCODE_DEV_ID                EQU    (0x00000FFF)        ; Device Identifier 

DBGMCU_IDCODE_REV_ID                EQU    (0xFFFF0000)        ; REV_ID[15:0] bits (Revision Identifier) 
DBGMCU_IDCODE_REV_ID_0              EQU    (0x00010000)        ; Bit 0 
DBGMCU_IDCODE_REV_ID_1              EQU    (0x00020000)        ; Bit 1 
DBGMCU_IDCODE_REV_ID_2              EQU    (0x00040000)        ; Bit 2 
DBGMCU_IDCODE_REV_ID_3              EQU    (0x00080000)        ; Bit 3 
DBGMCU_IDCODE_REV_ID_4              EQU    (0x00100000)        ; Bit 4 
DBGMCU_IDCODE_REV_ID_5              EQU    (0x00200000)        ; Bit 5 
DBGMCU_IDCODE_REV_ID_6              EQU    (0x00400000)        ; Bit 6 
DBGMCU_IDCODE_REV_ID_7              EQU    (0x00800000)        ; Bit 7 
DBGMCU_IDCODE_REV_ID_8              EQU    (0x01000000)        ; Bit 8 
DBGMCU_IDCODE_REV_ID_9              EQU    (0x02000000)        ; Bit 9 
DBGMCU_IDCODE_REV_ID_10             EQU    (0x04000000)        ; Bit 10 
DBGMCU_IDCODE_REV_ID_11             EQU    (0x08000000)        ; Bit 11 
DBGMCU_IDCODE_REV_ID_12             EQU    (0x10000000)        ; Bit 12 
DBGMCU_IDCODE_REV_ID_13             EQU    (0x20000000)        ; Bit 13 
DBGMCU_IDCODE_REV_ID_14             EQU    (0x40000000)        ; Bit 14 
DBGMCU_IDCODE_REV_ID_15             EQU    (0x80000000)        ; Bit 15 

;**************  Bit definition for DBGMCU_CR register  ******************
DBGMCU_CR_DBG_SLEEP                 EQU    (0x00000001)        ; Debug Sleep Mode 
DBGMCU_CR_DBG_STOP                  EQU    (0x00000002)        ; Debug Stop Mode 
DBGMCU_CR_DBG_STANDBY               EQU    (0x00000004)        ; Debug Standby mode 
DBGMCU_CR_TRACE_IOEN                EQU    (0x00000020)        ; Trace Pin Assignment Control 

DBGMCU_CR_TRACE_MODE                EQU    (0x000000C0)        ; TRACE_MODE[1:0] bits (Trace Pin Assignment Control) 
DBGMCU_CR_TRACE_MODE_0              EQU    (0x00000040)        ; Bit 0 
DBGMCU_CR_TRACE_MODE_1              EQU    (0x00000080)        ; Bit 1 

;**************  Bit definition for DBGMCU_APB1_FZ register  *************

DBGMCU_APB1_FZ_DBG_TIM2_STOP             EQU    (0x00000001)        ; TIM2 counter stopped when core is halted 
DBGMCU_APB1_FZ_DBG_TIM3_STOP             EQU    (0x00000002)        ; TIM3 counter stopped when core is halted 
DBGMCU_APB1_FZ_DBG_TIM4_STOP             EQU    (0x00000004)        ; TIM4 counter stopped when core is halted 
DBGMCU_APB1_FZ_DBG_TIM6_STOP             EQU    (0x00000010)        ; TIM6 counter stopped when core is halted 
DBGMCU_APB1_FZ_DBG_TIM7_STOP             EQU    (0x00000020)        ; TIM7 counter stopped when core is halted 
DBGMCU_APB1_FZ_DBG_WWDG_STOP             EQU    (0x00000800)        ; Debug Window Watchdog stopped when Core is halted 
DBGMCU_APB1_FZ_DBG_IWDG_STOP             EQU    (0x00001000)        ; Debug Independent Watchdog stopped when Core is halted 
DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT    EQU    (0x00200000)        ; SMBUS timeout mode stopped when Core is halted 
DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT    EQU    (0x00400000)        ; SMBUS timeout mode stopped when Core is halted 

;**************  Bit definition for DBGMCU_APB2_FZ register  *************

DBGMCU_APB2_FZ_DBG_TIM9_STOP             EQU    (0x00000004)        ; TIM9 counter stopped when core is halted 
DBGMCU_APB2_FZ_DBG_TIM10_STOP            EQU    (0x00000008)        ; TIM10 counter stopped when core is halted 
DBGMCU_APB2_FZ_DBG_TIM11_STOP            EQU    (0x00000010)        ; TIM11 counter stopped when core is halted 



;***************************************************************************
;                                                                           
;                            DMA Controller (DMA)                           
;                                                                           
;***************************************************************************

;**************  Bit definition for DMA_ISR register  *******************
DMA_ISR_GIF1                        EQU    (0x00000001)        ; Channel 1 Global interrupt flag 
DMA_ISR_TCIF1                       EQU    (0x00000002)        ; Channel 1 Transfer Complete flag 
DMA_ISR_HTIF1                       EQU    (0x00000004)        ; Channel 1 Half Transfer flag 
DMA_ISR_TEIF1                       EQU    (0x00000008)        ; Channel 1 Transfer Error flag 
DMA_ISR_GIF2                        EQU    (0x00000010)        ; Channel 2 Global interrupt flag 
DMA_ISR_TCIF2                       EQU    (0x00000020)        ; Channel 2 Transfer Complete flag 
DMA_ISR_HTIF2                       EQU    (0x00000040)        ; Channel 2 Half Transfer flag 
DMA_ISR_TEIF2                       EQU    (0x00000080)        ; Channel 2 Transfer Error flag 
DMA_ISR_GIF3                        EQU    (0x00000100)        ; Channel 3 Global interrupt flag 
DMA_ISR_TCIF3                       EQU    (0x00000200)        ; Channel 3 Transfer Complete flag 
DMA_ISR_HTIF3                       EQU    (0x00000400)        ; Channel 3 Half Transfer flag 
DMA_ISR_TEIF3                       EQU    (0x00000800)        ; Channel 3 Transfer Error flag 
DMA_ISR_GIF4                        EQU    (0x00001000)        ; Channel 4 Global interrupt flag 
DMA_ISR_TCIF4                       EQU    (0x00002000)        ; Channel 4 Transfer Complete flag 
DMA_ISR_HTIF4                       EQU    (0x00004000)        ; Channel 4 Half Transfer flag 
DMA_ISR_TEIF4                       EQU    (0x00008000)        ; Channel 4 Transfer Error flag 
DMA_ISR_GIF5                        EQU    (0x00010000)        ; Channel 5 Global interrupt flag 
DMA_ISR_TCIF5                       EQU    (0x00020000)        ; Channel 5 Transfer Complete flag 
DMA_ISR_HTIF5                       EQU    (0x00040000)        ; Channel 5 Half Transfer flag 
DMA_ISR_TEIF5                       EQU    (0x00080000)        ; Channel 5 Transfer Error flag 
DMA_ISR_GIF6                        EQU    (0x00100000)        ; Channel 6 Global interrupt flag 
DMA_ISR_TCIF6                       EQU    (0x00200000)        ; Channel 6 Transfer Complete flag 
DMA_ISR_HTIF6                       EQU    (0x00400000)        ; Channel 6 Half Transfer flag 
DMA_ISR_TEIF6                       EQU    (0x00800000)        ; Channel 6 Transfer Error flag 
DMA_ISR_GIF7                        EQU    (0x01000000)        ; Channel 7 Global interrupt flag 
DMA_ISR_TCIF7                       EQU    (0x02000000)        ; Channel 7 Transfer Complete flag 
DMA_ISR_HTIF7                       EQU    (0x04000000)        ; Channel 7 Half Transfer flag 
DMA_ISR_TEIF7                       EQU    (0x08000000)        ; Channel 7 Transfer Error flag 

;**************  Bit definition for DMA_IFCR register  ******************
DMA_IFCR_CGIF1                      EQU    (0x00000001)        ; Channel 1 Global interrupt clearr 
DMA_IFCR_CTCIF1                     EQU    (0x00000002)        ; Channel 1 Transfer Complete clear 
DMA_IFCR_CHTIF1                     EQU    (0x00000004)        ; Channel 1 Half Transfer clear 
DMA_IFCR_CTEIF1                     EQU    (0x00000008)        ; Channel 1 Transfer Error clear 
DMA_IFCR_CGIF2                      EQU    (0x00000010)        ; Channel 2 Global interrupt clear 
DMA_IFCR_CTCIF2                     EQU    (0x00000020)        ; Channel 2 Transfer Complete clear 
DMA_IFCR_CHTIF2                     EQU    (0x00000040)        ; Channel 2 Half Transfer clear 
DMA_IFCR_CTEIF2                     EQU    (0x00000080)        ; Channel 2 Transfer Error clear 
DMA_IFCR_CGIF3                      EQU    (0x00000100)        ; Channel 3 Global interrupt clear 
DMA_IFCR_CTCIF3                     EQU    (0x00000200)        ; Channel 3 Transfer Complete clear 
DMA_IFCR_CHTIF3                     EQU    (0x00000400)        ; Channel 3 Half Transfer clear 
DMA_IFCR_CTEIF3                     EQU    (0x00000800)        ; Channel 3 Transfer Error clear 
DMA_IFCR_CGIF4                      EQU    (0x00001000)        ; Channel 4 Global interrupt clear 
DMA_IFCR_CTCIF4                     EQU    (0x00002000)        ; Channel 4 Transfer Complete clear 
DMA_IFCR_CHTIF4                     EQU    (0x00004000)        ; Channel 4 Half Transfer clear 
DMA_IFCR_CTEIF4                     EQU    (0x00008000)        ; Channel 4 Transfer Error clear 
DMA_IFCR_CGIF5                      EQU    (0x00010000)        ; Channel 5 Global interrupt clear 
DMA_IFCR_CTCIF5                     EQU    (0x00020000)        ; Channel 5 Transfer Complete clear 
DMA_IFCR_CHTIF5                     EQU    (0x00040000)        ; Channel 5 Half Transfer clear 
DMA_IFCR_CTEIF5                     EQU    (0x00080000)        ; Channel 5 Transfer Error clear 
DMA_IFCR_CGIF6                      EQU    (0x00100000)        ; Channel 6 Global interrupt clear 
DMA_IFCR_CTCIF6                     EQU    (0x00200000)        ; Channel 6 Transfer Complete clear 
DMA_IFCR_CHTIF6                     EQU    (0x00400000)        ; Channel 6 Half Transfer clear 
DMA_IFCR_CTEIF6                     EQU    (0x00800000)        ; Channel 6 Transfer Error clear 
DMA_IFCR_CGIF7                      EQU    (0x01000000)        ; Channel 7 Global interrupt clear 
DMA_IFCR_CTCIF7                     EQU    (0x02000000)        ; Channel 7 Transfer Complete clear 
DMA_IFCR_CHTIF7                     EQU    (0x04000000)        ; Channel 7 Half Transfer clear 
DMA_IFCR_CTEIF7                     EQU    (0x08000000)        ; Channel 7 Transfer Error clear 

;**************  Bit definition for DMA_CCR1 register  ******************
DMA_CCR1_EN                         EQU    (0x0001)            ; Channel enable
DMA_CCR1_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR1_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR1_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR1_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR1_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR1_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR1_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR1_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR1_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR1_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR1_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR1_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR1_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR1_PL                         EQU    (0x3000)            ; PL[1:0] bits(Channel Priority level) 
DMA_CCR1_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR1_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR1_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode 

;**************  Bit definition for DMA_CCR2 register  ******************
DMA_CCR2_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR2_TCIE                       EQU    (0x0002)            ; ransfer complete interrupt enable 
DMA_CCR2_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR2_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR2_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR2_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR2_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR2_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR2_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR2_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR2_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR2_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR2_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR2_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR2_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR2_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR2_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR2_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode 

;**************  Bit definition for DMA_CCR3 register  ******************
DMA_CCR3_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR3_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR3_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR3_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR3_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR3_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR3_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR3_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR3_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR3_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR3_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR3_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR3_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR3_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR3_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR3_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR3_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR3_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode 

;**************  Bit definition for DMA_CCR4 register  ******************
DMA_CCR4_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR4_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR4_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR4_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR4_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR4_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR4_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR4_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR4_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR4_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR4_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR4_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR4_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR4_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR4_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR4_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR4_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR4_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode 

;**************  Bit definition for DMA_CCR5 register  ******************
DMA_CCR5_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR5_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR5_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR5_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR5_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR5_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR5_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR5_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR5_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR5_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR5_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR5_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR5_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR5_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR5_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR5_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR5_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR5_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode enable 

;**************  Bit definition for DMA_CCR6 register  ******************
DMA_CCR6_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR6_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR6_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR6_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR6_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR6_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR6_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR6_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR6_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR6_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR6_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR6_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR6_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR6_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR6_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR6_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR6_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR6_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode 

;**************  Bit definition for DMA_CCR7 register  ******************
DMA_CCR7_EN                         EQU    (0x0001)            ; Channel enable 
DMA_CCR7_TCIE                       EQU    (0x0002)            ; Transfer complete interrupt enable 
DMA_CCR7_HTIE                       EQU    (0x0004)            ; Half Transfer interrupt enable 
DMA_CCR7_TEIE                       EQU    (0x0008)            ; Transfer error interrupt enable 
DMA_CCR7_DIR                        EQU    (0x0010)            ; Data transfer direction 
DMA_CCR7_CIRC                       EQU    (0x0020)            ; Circular mode 
DMA_CCR7_PINC                       EQU    (0x0040)            ; Peripheral increment mode 
DMA_CCR7_MINC                       EQU    (0x0080)            ; Memory increment mode 

DMA_CCR7_PSIZE                      EQU    (0x0300)            ; PSIZE[1:0] bits (Peripheral size) 
DMA_CCR7_PSIZE_0                    EQU    (0x0100)            ; Bit 0 
DMA_CCR7_PSIZE_1                    EQU    (0x0200)            ; Bit 1 

DMA_CCR7_MSIZE                      EQU    (0x0C00)            ; MSIZE[1:0] bits (Memory size) 
DMA_CCR7_MSIZE_0                    EQU    (0x0400)            ; Bit 0 
DMA_CCR7_MSIZE_1                    EQU    (0x0800)            ; Bit 1 

DMA_CCR7_PL                         EQU    (0x3000)            ; PL[1:0] bits (Channel Priority level) 
DMA_CCR7_PL_0                       EQU    (0x1000)            ; Bit 0 
DMA_CCR7_PL_1                       EQU    (0x2000)            ; Bit 1 

DMA_CCR7_MEM2MEM                    EQU    (0x4000)            ; Memory to memory mode enable 

;**************  Bit definition for DMA_CNDTR1 register  *****************
DMA_CNDTR1_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR2 register  *****************
DMA_CNDTR2_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR3 register  *****************
DMA_CNDTR3_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR4 register  *****************
DMA_CNDTR4_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR5 register  *****************
DMA_CNDTR5_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR6 register  *****************
DMA_CNDTR6_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CNDTR7 register  *****************
DMA_CNDTR7_NDT                      EQU    (0xFFFF)            ; Number of data to Transfer 

;**************  Bit definition for DMA_CPAR1 register  ******************
DMA_CPAR1_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 

;**************  Bit definition for DMA_CPAR2 register  ******************
DMA_CPAR2_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 

;**************  Bit definition for DMA_CPAR3 register  ******************
DMA_CPAR3_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 


;**************  Bit definition for DMA_CPAR4 register  ******************
DMA_CPAR4_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 

;**************  Bit definition for DMA_CPAR5 register  ******************
DMA_CPAR5_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 

;**************  Bit definition for DMA_CPAR6 register  ******************
DMA_CPAR6_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 


;**************  Bit definition for DMA_CPAR7 register  ******************
DMA_CPAR7_PA                        EQU    (0xFFFFFFFF)        ; Peripheral Address 

;**************  Bit definition for DMA_CMAR1 register  ******************
DMA_CMAR1_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 

;**************  Bit definition for DMA_CMAR2 register  ******************
DMA_CMAR2_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 

;**************  Bit definition for DMA_CMAR3 register  ******************
DMA_CMAR3_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 


;**************  Bit definition for DMA_CMAR4 register  ******************
DMA_CMAR4_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 

;**************  Bit definition for DMA_CMAR5 register  ******************
DMA_CMAR5_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 

;**************  Bit definition for DMA_CMAR6 register  ******************
DMA_CMAR6_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 

;**************  Bit definition for DMA_CMAR7 register  ******************
DMA_CMAR7_MA                        EQU    (0xFFFFFFFF)        ; Memory Address 



;***************************************************************************
;                                                                           
;                   External Interrupt/Event Controller (EXTI)              
;                                                                           
;***************************************************************************

;**************  Bit definition for EXTI_IMR register  ******************
EXTI_IMR_MR0                        EQU    (0x00000001)        ; Interrupt Mask on line 0 
EXTI_IMR_MR1                        EQU    (0x00000002)        ; Interrupt Mask on line 1 
EXTI_IMR_MR2                        EQU    (0x00000004)        ; Interrupt Mask on line 2 
EXTI_IMR_MR3                        EQU    (0x00000008)        ; Interrupt Mask on line 3 
EXTI_IMR_MR4                        EQU    (0x00000010)        ; Interrupt Mask on line 4 
EXTI_IMR_MR5                        EQU    (0x00000020)        ; Interrupt Mask on line 5 
EXTI_IMR_MR6                        EQU    (0x00000040)        ; Interrupt Mask on line 6 
EXTI_IMR_MR7                        EQU    (0x00000080)        ; Interrupt Mask on line 7 
EXTI_IMR_MR8                        EQU    (0x00000100)        ; Interrupt Mask on line 8 
EXTI_IMR_MR9                        EQU    (0x00000200)        ; Interrupt Mask on line 9 
EXTI_IMR_MR10                       EQU    (0x00000400)        ; Interrupt Mask on line 10 
EXTI_IMR_MR11                       EQU    (0x00000800)        ; Interrupt Mask on line 11 
EXTI_IMR_MR12                       EQU    (0x00001000)        ; Interrupt Mask on line 12 
EXTI_IMR_MR13                       EQU    (0x00002000)        ; Interrupt Mask on line 13 
EXTI_IMR_MR14                       EQU    (0x00004000)        ; Interrupt Mask on line 14 
EXTI_IMR_MR15                       EQU    (0x00008000)        ; Interrupt Mask on line 15 
EXTI_IMR_MR16                       EQU    (0x00010000)        ; Interrupt Mask on line 16 
EXTI_IMR_MR17                       EQU    (0x00020000)        ; Interrupt Mask on line 17 
EXTI_IMR_MR18                       EQU    (0x00040000)        ; Interrupt Mask on line 18 
EXTI_IMR_MR19                       EQU    (0x00080000)        ; Interrupt Mask on line 19 
EXTI_IMR_MR20                       EQU    (0x00100000)        ; Interrupt Mask on line 20 
EXTI_IMR_MR21                       EQU    (0x00200000)        ; Interrupt Mask on line 21 
EXTI_IMR_MR22                       EQU    (0x00400000)        ; Interrupt Mask on line 22 

;**************  Bit definition for EXTI_EMR register  ******************
EXTI_EMR_MR0                        EQU    (0x00000001)        ; Event Mask on line 0 
EXTI_EMR_MR1                        EQU    (0x00000002)        ; Event Mask on line 1 
EXTI_EMR_MR2                        EQU    (0x00000004)        ; Event Mask on line 2 
EXTI_EMR_MR3                        EQU    (0x00000008)        ; Event Mask on line 3 
EXTI_EMR_MR4                        EQU    (0x00000010)        ; Event Mask on line 4 
EXTI_EMR_MR5                        EQU    (0x00000020)        ; Event Mask on line 5 
EXTI_EMR_MR6                        EQU    (0x00000040)        ; Event Mask on line 6 
EXTI_EMR_MR7                        EQU    (0x00000080)        ; Event Mask on line 7 
EXTI_EMR_MR8                        EQU    (0x00000100)        ; Event Mask on line 8 
EXTI_EMR_MR9                        EQU    (0x00000200)        ; Event Mask on line 9 
EXTI_EMR_MR10                       EQU    (0x00000400)        ; Event Mask on line 10 
EXTI_EMR_MR11                       EQU    (0x00000800)        ; Event Mask on line 11 
EXTI_EMR_MR12                       EQU    (0x00001000)        ; Event Mask on line 12 
EXTI_EMR_MR13                       EQU    (0x00002000)        ; Event Mask on line 13 
EXTI_EMR_MR14                       EQU    (0x00004000)        ; Event Mask on line 14 
EXTI_EMR_MR15                       EQU    (0x00008000)        ; Event Mask on line 15 
EXTI_EMR_MR16                       EQU    (0x00010000)        ; Event Mask on line 16 
EXTI_EMR_MR17                       EQU    (0x00020000)        ; Event Mask on line 17 
EXTI_EMR_MR18                       EQU    (0x00040000)        ; Event Mask on line 18 
EXTI_EMR_MR19                       EQU    (0x00080000)        ; Event Mask on line 19 
EXTI_EMR_MR20                       EQU    (0x00100000)        ; Event Mask on line 20 
EXTI_EMR_MR21                       EQU    (0x00200000)        ; Event Mask on line 21 
EXTI_EMR_MR22                       EQU    (0x00400000)        ; Event Mask on line 22 

;**************  Bit definition for EXTI_RTSR register  ******************
EXTI_RTSR_TR0                       EQU    (0x00000001)        ; Rising trigger event configuration bit of line 0 
EXTI_RTSR_TR1                       EQU    (0x00000002)        ; Rising trigger event configuration bit of line 1 
EXTI_RTSR_TR2                       EQU    (0x00000004)        ; Rising trigger event configuration bit of line 2 
EXTI_RTSR_TR3                       EQU    (0x00000008)        ; Rising trigger event configuration bit of line 3 
EXTI_RTSR_TR4                       EQU    (0x00000010)        ; Rising trigger event configuration bit of line 4 
EXTI_RTSR_TR5                       EQU    (0x00000020)        ; Rising trigger event configuration bit of line 5 
EXTI_RTSR_TR6                       EQU    (0x00000040)        ; Rising trigger event configuration bit of line 6 
EXTI_RTSR_TR7                       EQU    (0x00000080)        ; Rising trigger event configuration bit of line 7 
EXTI_RTSR_TR8                       EQU    (0x00000100)        ; Rising trigger event configuration bit of line 8 
EXTI_RTSR_TR9                       EQU    (0x00000200)        ; Rising trigger event configuration bit of line 9 
EXTI_RTSR_TR10                      EQU    (0x00000400)        ; Rising trigger event configuration bit of line 10 
EXTI_RTSR_TR11                      EQU    (0x00000800)        ; Rising trigger event configuration bit of line 11 
EXTI_RTSR_TR12                      EQU    (0x00001000)        ; Rising trigger event configuration bit of line 12 
EXTI_RTSR_TR13                      EQU    (0x00002000)        ; Rising trigger event configuration bit of line 13 
EXTI_RTSR_TR14                      EQU    (0x00004000)        ; Rising trigger event configuration bit of line 14 
EXTI_RTSR_TR15                      EQU    (0x00008000)        ; Rising trigger event configuration bit of line 15 
EXTI_RTSR_TR16                      EQU    (0x00010000)        ; Rising trigger event configuration bit of line 16 
EXTI_RTSR_TR17                      EQU    (0x00020000)        ; Rising trigger event configuration bit of line 17 
EXTI_RTSR_TR18                      EQU    (0x00040000)        ; Rising trigger event configuration bit of line 18 
EXTI_RTSR_TR19                      EQU    (0x00080000)        ; Rising trigger event configuration bit of line 19 
EXTI_RTSR_TR20                      EQU    (0x00100000)        ; Rising trigger event configuration bit of line 20 
EXTI_RTSR_TR21                      EQU    (0x00200000)        ; Rising trigger event configuration bit of line 21 
EXTI_RTSR_TR22                      EQU    (0x00400000)        ; Rising trigger event configuration bit of line 22 

;**************  Bit definition for EXTI_FTSR register  ******************
EXTI_FTSR_TR0                       EQU    (0x00000001)        ; Falling trigger event configuration bit of line 0 
EXTI_FTSR_TR1                       EQU    (0x00000002)        ; Falling trigger event configuration bit of line 1 
EXTI_FTSR_TR2                       EQU    (0x00000004)        ; Falling trigger event configuration bit of line 2 
EXTI_FTSR_TR3                       EQU    (0x00000008)        ; Falling trigger event configuration bit of line 3 
EXTI_FTSR_TR4                       EQU    (0x00000010)        ; Falling trigger event configuration bit of line 4 
EXTI_FTSR_TR5                       EQU    (0x00000020)        ; Falling trigger event configuration bit of line 5 
EXTI_FTSR_TR6                       EQU    (0x00000040)        ; Falling trigger event configuration bit of line 6 
EXTI_FTSR_TR7                       EQU    (0x00000080)        ; Falling trigger event configuration bit of line 7 
EXTI_FTSR_TR8                       EQU    (0x00000100)        ; Falling trigger event configuration bit of line 8 
EXTI_FTSR_TR9                       EQU    (0x00000200)        ; Falling trigger event configuration bit of line 9 
EXTI_FTSR_TR10                      EQU    (0x00000400)        ; Falling trigger event configuration bit of line 10 
EXTI_FTSR_TR11                      EQU    (0x00000800)        ; Falling trigger event configuration bit of line 11 
EXTI_FTSR_TR12                      EQU    (0x00001000)        ; Falling trigger event configuration bit of line 12 
EXTI_FTSR_TR13                      EQU    (0x00002000)        ; Falling trigger event configuration bit of line 13 
EXTI_FTSR_TR14                      EQU    (0x00004000)        ; Falling trigger event configuration bit of line 14 
EXTI_FTSR_TR15                      EQU    (0x00008000)        ; Falling trigger event configuration bit of line 15 
EXTI_FTSR_TR16                      EQU    (0x00010000)        ; Falling trigger event configuration bit of line 16 
EXTI_FTSR_TR17                      EQU    (0x00020000)        ; Falling trigger event configuration bit of line 17 
EXTI_FTSR_TR18                      EQU    (0x00040000)        ; Falling trigger event configuration bit of line 18 
EXTI_FTSR_TR19                      EQU    (0x00080000)        ; Falling trigger event configuration bit of line 19 
EXTI_FTSR_TR20                      EQU    (0x00100000)        ; Falling trigger event configuration bit of line 20 
EXTI_FTSR_TR21                      EQU    (0x00200000)        ; Falling trigger event configuration bit of line 21 
EXTI_FTSR_TR22                      EQU    (0x00400000)        ; Falling trigger event configuration bit of line 22 

;**************  Bit definition for EXTI_SWIER register  *****************
EXTI_SWIER_SWIER0                   EQU    (0x00000001)        ; Software Interrupt on line 0 
EXTI_SWIER_SWIER1                   EQU    (0x00000002)        ; Software Interrupt on line 1 
EXTI_SWIER_SWIER2                   EQU    (0x00000004)        ; Software Interrupt on line 2 
EXTI_SWIER_SWIER3                   EQU    (0x00000008)        ; Software Interrupt on line 3 
EXTI_SWIER_SWIER4                   EQU    (0x00000010)        ; Software Interrupt on line 4 
EXTI_SWIER_SWIER5                   EQU    (0x00000020)        ; Software Interrupt on line 5 
EXTI_SWIER_SWIER6                   EQU    (0x00000040)        ; Software Interrupt on line 6 
EXTI_SWIER_SWIER7                   EQU    (0x00000080)        ; Software Interrupt on line 7 
EXTI_SWIER_SWIER8                   EQU    (0x00000100)        ; Software Interrupt on line 8 
EXTI_SWIER_SWIER9                   EQU    (0x00000200)        ; Software Interrupt on line 9 
EXTI_SWIER_SWIER10                  EQU    (0x00000400)        ; Software Interrupt on line 10 
EXTI_SWIER_SWIER11                  EQU    (0x00000800)        ; Software Interrupt on line 11 
EXTI_SWIER_SWIER12                  EQU    (0x00001000)        ; Software Interrupt on line 12 
EXTI_SWIER_SWIER13                  EQU    (0x00002000)        ; Software Interrupt on line 13 
EXTI_SWIER_SWIER14                  EQU    (0x00004000)        ; Software Interrupt on line 14 
EXTI_SWIER_SWIER15                  EQU    (0x00008000)        ; Software Interrupt on line 15 
EXTI_SWIER_SWIER16                  EQU    (0x00010000)        ; Software Interrupt on line 16 
EXTI_SWIER_SWIER17                  EQU    (0x00020000)        ; Software Interrupt on line 17 
EXTI_SWIER_SWIER18                  EQU    (0x00040000)        ; Software Interrupt on line 18 
EXTI_SWIER_SWIER19                  EQU    (0x00080000)        ; Software Interrupt on line 19 
EXTI_SWIER_SWIER20                  EQU    (0x00100000)        ; Software Interrupt on line 20 
EXTI_SWIER_SWIER21                  EQU    (0x00200000)        ; Software Interrupt on line 21 
EXTI_SWIER_SWIER22                  EQU    (0x00400000)        ; Software Interrupt on line 22 

;**************  Bit definition for EXTI_PR register  *******************
EXTI_PR_PR0                         EQU    (0x00000001)        ; Pending bit 0 
EXTI_PR_PR1                         EQU    (0x00000002)        ; Pending bit 1 
EXTI_PR_PR2                         EQU    (0x00000004)        ; Pending bit 2 
EXTI_PR_PR3                         EQU    (0x00000008)        ; Pending bit 3 
EXTI_PR_PR4                         EQU    (0x00000010)        ; Pending bit 4 
EXTI_PR_PR5                         EQU    (0x00000020)        ; Pending bit 5 
EXTI_PR_PR6                         EQU    (0x00000040)        ; Pending bit 6 
EXTI_PR_PR7                         EQU    (0x00000080)        ; Pending bit 7 
EXTI_PR_PR8                         EQU    (0x00000100)        ; Pending bit 8 
EXTI_PR_PR9                         EQU    (0x00000200)        ; Pending bit 9 
EXTI_PR_PR10                        EQU    (0x00000400)        ; Pending bit 10 
EXTI_PR_PR11                        EQU    (0x00000800)        ; Pending bit 11 
EXTI_PR_PR12                        EQU    (0x00001000)        ; Pending bit 12 
EXTI_PR_PR13                        EQU    (0x00002000)        ; Pending bit 13 
EXTI_PR_PR14                        EQU    (0x00004000)        ; Pending bit 14 
EXTI_PR_PR15                        EQU    (0x00008000)        ; Pending bit 15 
EXTI_PR_PR16                        EQU    (0x00010000)        ; Pending bit 16 
EXTI_PR_PR17                        EQU    (0x00020000)        ; Pending bit 17 
EXTI_PR_PR18                        EQU    (0x00040000)        ; Pending bit 18 
EXTI_PR_PR19                        EQU    (0x00080000)        ; Pending bit 19 
EXTI_PR_PR20                        EQU    (0x00100000)        ; Pending bit 20 
EXTI_PR_PR21                        EQU    (0x00200000)        ; Pending bit 21 
EXTI_EXTI_PR_PR22                   EQU    (0x00400000)        ; Pending bit 22 



;***************************************************************************
;                                                                           
;                 FLASH, DATA EEPROM and Option Bytes Registers             
;                         (FLASH, DATA_EEPROM, OB)                          
;                                                                           
;***************************************************************************

;**************  Bit definition for FLASH_ACR register  *****************
FLASH_ACR_LATENCY                   EQU    (0x00000001)        ; Latency 
FLASH_ACR_PRFTEN                    EQU    (0x00000002)        ; Prefetch Buffer Enable 
FLASH_ACR_ACC64                     EQU    (0x00000004)        ; Access 64 bits 
FLASH_ACR_SLEEP_PD                  EQU    (0x00000008)        ; Flash mode during sleep mode 
FLASH_ACR_RUN_PD                    EQU    (0x00000010)        ; Flash mode during RUN mode 

;**************  Bit definition for FLASH_PECR register  *****************
FLASH_PECR_PELOCK                   EQU    (0x00000001)        ;FLASH_PECR and Flash data Lock 
FLASH_PECR_PRGLOCK                  EQU    (0x00000002)        ; Program matrix Lock 
FLASH_PECR_OPTLOCK                  EQU    (0x00000004)        ; Option byte matrix Lock 
FLASH_PECR_PROG                     EQU    (0x00000008)        ; Program matrix selection 
FLASH_PECR_DATA                     EQU    (0x00000010)        ; Data matrix selection 
FLASH_PECR_FTDW                     EQU    (0x00000100)        ; Fixed Time Data write for Word/Half Word/Byte programming 
FLASH_PECR_ERASE                    EQU    (0x00000200)        ; Page erasing mode 
FLASH_PECR_FPRG                     EQU    (0x00000400)        ; Fast Page/Half Page programming mode 
FLASH_PECR_EOPIE                    EQU    (0x00010000)        ; End of programming interrupt  
FLASH_PECR_ERRIE                    EQU    (0x00020000)        ; Error interrupt  
FLASH_PECR_OBL_LAUNCH               EQU    (0x00040000)        ; Launch the option byte loading  

;**************  Bit definition for FLASH_PDKEYR register  *****************
FLASH_PDKEYR_PDKEYR                 EQU    (0xFFFFFFFF)       ;FLASH_PEC and data matrix Key 

;**************  Bit definition for FLASH_PEKEYR register  *****************
FLASH_PEKEYR_PEKEYR                 EQU    (0xFFFFFFFF)       ;FLASH_PEC and data matrix Key 

;**************  Bit definition for FLASH_PRGKEYR register  *****************
FLASH_PRGKEYR_PRGKEYR               EQU    (0xFFFFFFFF)        ; Program matrix Key 

;**************  Bit definition for FLASH_OPTKEYR register  *****************
FLASH_OPTKEYR_OPTKEYR               EQU    (0xFFFFFFFF)        ; Option bytes matrix Key 

;**************  Bit definition for FLASH_SR register  ******************
FLASH_SR_BSY                        EQU    (0x00000001)        ; Busy 
FLASH_SR_EOP                        EQU    (0x00000002)        ; End Of Programming
FLASH_SR_ENHV                       EQU    (0x00000004)        ; End of high voltage 
FLASH_SR_READY                      EQU    (0x00000008)        ; Flash ready after low power mode 

FLASH_SR_WRPERR                     EQU    (0x00000100)        ; Write protected error 
FLASH_SR_PGAERR                     EQU    (0x00000200)        ; Programming Alignment Error 
FLASH_SR_SIZERR                     EQU    (0x00000400)        ; Size error 
FLASH_SR_OPTVERR                    EQU    (0x00000800)        ; Option validity error 

;**************  Bit definition for FLASH_OBR register  ******************
FLASH_OBR_RDPRT                     EQU    (0x000000AA)        ; Read Protection 
FLASH_OBR_BOR_LEV                   EQU    (0x000F0000)        ; BOR_LEV[3:0] Brown Out Reset Threshold Level
FLASH_OBR_USER                      EQU    (0x00700000)        ; User Option Bytes 
FLASH_OBR_IWDG_SW                   EQU    (0x00100000)        ; IWDG_SW 
FLASH_OBR_nRST_STOP                 EQU    (0x00200000)        ; nRST_STOP 
FLASH_OBR_nRST_STDBY                EQU    (0x00400000)        ; nRST_STDBY 

;**************  Bit definition for FLASH_WRPR register  *****************
FLASH_WRPR_WRP                      EQU    (0xFFFFFFFF)        ; Write Protect 



;***************************************************************************
;                                                                           
;                       General Purpose IOs (GPIO)                          
;                                                                           
;***************************************************************************

;**************  Bit definition for GPIO_MODER register  ****************  
GPIO_MODER_MODER0          EQU    (0x00000003)
GPIO_MODER_MODER0_0        EQU    (0x00000001)
GPIO_MODER_MODER0_1        EQU    (0x00000002)
GPIO_MODER_MODER1          EQU    (0x0000000C)
GPIO_MODER_MODER1_0        EQU    (0x00000004)
GPIO_MODER_MODER1_1        EQU    (0x00000008)
GPIO_MODER_MODER2          EQU    (0x00000030)
GPIO_MODER_MODER2_0        EQU    (0x00000010)
GPIO_MODER_MODER2_1        EQU    (0x00000020)
GPIO_MODER_MODER3          EQU    (0x000000C0)
GPIO_MODER_MODER3_0        EQU    (0x00000040)
GPIO_MODER_MODER3_1        EQU    (0x00000080)
GPIO_MODER_MODER4          EQU    (0x00000300)
GPIO_MODER_MODER4_0        EQU    (0x00000100)
GPIO_MODER_MODER4_1        EQU    (0x00000200)
GPIO_MODER_MODER5          EQU    (0x00000C00)
GPIO_MODER_MODER5_0        EQU    (0x00000400)
GPIO_MODER_MODER5_1        EQU    (0x00000800)
GPIO_MODER_MODER6          EQU    (0x00003000)
GPIO_MODER_MODER6_0        EQU    (0x00001000)
GPIO_MODER_MODER6_1        EQU    (0x00002000)
GPIO_MODER_MODER7          EQU    (0x0000C000)
GPIO_MODER_MODER7_0        EQU    (0x00004000)
GPIO_MODER_MODER7_1        EQU    (0x00008000)
GPIO_MODER_MODER8          EQU    (0x00030000)
GPIO_MODER_MODER8_0        EQU    (0x00010000)
GPIO_MODER_MODER8_1        EQU    (0x00020000)
GPIO_MODER_MODER9          EQU    (0x000C0000)
GPIO_MODER_MODER9_0        EQU    (0x00040000)
GPIO_MODER_MODER9_1        EQU    (0x00080000)
GPIO_MODER_MODER10         EQU    (0x00300000)
GPIO_MODER_MODER10_0       EQU    (0x00100000)
GPIO_MODER_MODER10_1       EQU    (0x00200000)
GPIO_MODER_MODER11         EQU    (0x00C00000)
GPIO_MODER_MODER11_0       EQU    (0x00400000)
GPIO_MODER_MODER11_1       EQU    (0x00800000)
GPIO_MODER_MODER12         EQU    (0x03000000)
GPIO_MODER_MODER12_0       EQU    (0x01000000)
GPIO_MODER_MODER12_1       EQU    (0x02000000)
GPIO_MODER_MODER13         EQU    (0x0C000000)
GPIO_MODER_MODER13_0       EQU    (0x04000000)
GPIO_MODER_MODER13_1       EQU    (0x08000000)
GPIO_MODER_MODER14         EQU    (0x30000000)
GPIO_MODER_MODER14_0       EQU    (0x10000000)
GPIO_MODER_MODER14_1       EQU    (0x20000000)
GPIO_MODER_MODER15         EQU    (0xC0000000)
GPIO_MODER_MODER15_0       EQU    (0x40000000)
GPIO_MODER_MODER15_1       EQU    (0x80000000)

;**************  Bit definition for GPIO_OTYPER register  ***************   
GPIO_OTYPER_OT_0           EQU    (0x00000001)
GPIO_OTYPER_OT_1           EQU    (0x00000002)
GPIO_OTYPER_OT_2           EQU    (0x00000004)
GPIO_OTYPER_OT_3           EQU    (0x00000008)
GPIO_OTYPER_OT_4           EQU    (0x00000010)
GPIO_OTYPER_OT_5           EQU    (0x00000020)
GPIO_OTYPER_OT_6           EQU    (0x00000040)
GPIO_OTYPER_OT_7           EQU    (0x00000080)
GPIO_OTYPER_OT_8           EQU    (0x00000100)
GPIO_OTYPER_OT_9           EQU    (0x00000200)
GPIO_OTYPER_OT_10          EQU    (0x00000400)
GPIO_OTYPER_OT_11          EQU    (0x00000800)
GPIO_OTYPER_OT_12          EQU    (0x00001000)
GPIO_OTYPER_OT_13          EQU    (0x00002000)
GPIO_OTYPER_OT_14          EQU    (0x00004000)
GPIO_OTYPER_OT_15          EQU    (0x00008000)

;**************  Bit definition for GPIO_OSPEEDR register  **************  
GPIO_OSPEEDER_OSPEEDR0     EQU    (0x00000003)
GPIO_OSPEEDER_OSPEEDR0_0   EQU    (0x00000001)
GPIO_OSPEEDER_OSPEEDR0_1   EQU    (0x00000002)
GPIO_OSPEEDER_OSPEEDR1     EQU    (0x0000000C)
GPIO_OSPEEDER_OSPEEDR1_0   EQU    (0x00000004)
GPIO_OSPEEDER_OSPEEDR1_1   EQU    (0x00000008)
GPIO_OSPEEDER_OSPEEDR2     EQU    (0x00000030)
GPIO_OSPEEDER_OSPEEDR2_0   EQU    (0x00000010)
GPIO_OSPEEDER_OSPEEDR2_1   EQU    (0x00000020)
GPIO_OSPEEDER_OSPEEDR3     EQU    (0x000000C0)
GPIO_OSPEEDER_OSPEEDR3_0   EQU    (0x00000040)
GPIO_OSPEEDER_OSPEEDR3_1   EQU    (0x00000080)
GPIO_OSPEEDER_OSPEEDR4     EQU    (0x00000300)
GPIO_OSPEEDER_OSPEEDR4_0   EQU    (0x00000100)
GPIO_OSPEEDER_OSPEEDR4_1   EQU    (0x00000200)
GPIO_OSPEEDER_OSPEEDR5     EQU    (0x00000C00)
GPIO_OSPEEDER_OSPEEDR5_0   EQU    (0x00000400)
GPIO_OSPEEDER_OSPEEDR5_1   EQU    (0x00000800)
GPIO_OSPEEDER_OSPEEDR6     EQU    (0x00003000)
GPIO_OSPEEDER_OSPEEDR6_0   EQU    (0x00001000)
GPIO_OSPEEDER_OSPEEDR6_1   EQU    (0x00002000)
GPIO_OSPEEDER_OSPEEDR7     EQU    (0x0000C000)
GPIO_OSPEEDER_OSPEEDR7_0   EQU    (0x00004000)
GPIO_OSPEEDER_OSPEEDR7_1   EQU    (0x00008000)
GPIO_OSPEEDER_OSPEEDR8     EQU    (0x00030000)
GPIO_OSPEEDER_OSPEEDR8_0   EQU    (0x00010000)
GPIO_OSPEEDER_OSPEEDR8_1   EQU    (0x00020000)
GPIO_OSPEEDER_OSPEEDR9     EQU    (0x000C0000)
GPIO_OSPEEDER_OSPEEDR9_0   EQU    (0x00040000)
GPIO_OSPEEDER_OSPEEDR9_1   EQU    (0x00080000)
GPIO_OSPEEDER_OSPEEDR10    EQU    (0x00300000)
GPIO_OSPEEDER_OSPEEDR10_0  EQU    (0x00100000)
GPIO_OSPEEDER_OSPEEDR10_1  EQU    (0x00200000)
GPIO_OSPEEDER_OSPEEDR11    EQU    (0x00C00000)
GPIO_OSPEEDER_OSPEEDR11_0  EQU    (0x00400000)
GPIO_OSPEEDER_OSPEEDR11_1  EQU    (0x00800000)
GPIO_OSPEEDER_OSPEEDR12    EQU    (0x03000000)
GPIO_OSPEEDER_OSPEEDR12_0  EQU    (0x01000000)
GPIO_OSPEEDER_OSPEEDR12_1  EQU    (0x02000000)
GPIO_OSPEEDER_OSPEEDR13    EQU    (0x0C000000)
GPIO_OSPEEDER_OSPEEDR13_0  EQU    (0x04000000)
GPIO_OSPEEDER_OSPEEDR13_1  EQU    (0x08000000)
GPIO_OSPEEDER_OSPEEDR14    EQU    (0x30000000)
GPIO_OSPEEDER_OSPEEDR14_0  EQU    (0x10000000)
GPIO_OSPEEDER_OSPEEDR14_1  EQU    (0x20000000)
GPIO_OSPEEDER_OSPEEDR15    EQU    (0xC0000000)
GPIO_OSPEEDER_OSPEEDR15_0  EQU    (0x40000000)
GPIO_OSPEEDER_OSPEEDR15_1  EQU    (0x80000000)

;**************  Bit definition for GPIO_PUPDR register  ****************  
GPIO_PUPDR_PUPDR0          EQU    (0x00000003)
GPIO_PUPDR_PUPDR0_0        EQU    (0x00000001)
GPIO_PUPDR_PUPDR0_1        EQU    (0x00000002)
GPIO_PUPDR_PUPDR1          EQU    (0x0000000C)
GPIO_PUPDR_PUPDR1_0        EQU    (0x00000004)
GPIO_PUPDR_PUPDR1_1        EQU    (0x00000008)
GPIO_PUPDR_PUPDR2          EQU    (0x00000030)
GPIO_PUPDR_PUPDR2_0        EQU    (0x00000010)
GPIO_PUPDR_PUPDR2_1        EQU    (0x00000020)
GPIO_PUPDR_PUPDR3          EQU    (0x000000C0)
GPIO_PUPDR_PUPDR3_0        EQU    (0x00000040)
GPIO_PUPDR_PUPDR3_1        EQU    (0x00000080)
GPIO_PUPDR_PUPDR4          EQU    (0x00000300)
GPIO_PUPDR_PUPDR4_0        EQU    (0x00000100)
GPIO_PUPDR_PUPDR4_1        EQU    (0x00000200)
GPIO_PUPDR_PUPDR5          EQU    (0x00000C00)
GPIO_PUPDR_PUPDR5_0        EQU    (0x00000400)
GPIO_PUPDR_PUPDR5_1        EQU    (0x00000800)
GPIO_PUPDR_PUPDR6          EQU    (0x00003000)
GPIO_PUPDR_PUPDR6_0        EQU    (0x00001000)
GPIO_PUPDR_PUPDR6_1        EQU    (0x00002000)
GPIO_PUPDR_PUPDR7          EQU    (0x0000C000)
GPIO_PUPDR_PUPDR7_0        EQU    (0x00004000)
GPIO_PUPDR_PUPDR7_1        EQU    (0x00008000)
GPIO_PUPDR_PUPDR8          EQU    (0x00030000)
GPIO_PUPDR_PUPDR8_0        EQU    (0x00010000)
GPIO_PUPDR_PUPDR8_1        EQU    (0x00020000)
GPIO_PUPDR_PUPDR9          EQU    (0x000C0000)
GPIO_PUPDR_PUPDR9_0        EQU    (0x00040000)
GPIO_PUPDR_PUPDR9_1        EQU    (0x00080000)
GPIO_PUPDR_PUPDR10         EQU    (0x00300000)
GPIO_PUPDR_PUPDR10_0       EQU    (0x00100000)
GPIO_PUPDR_PUPDR10_1       EQU    (0x00200000)
GPIO_PUPDR_PUPDR11         EQU    (0x00C00000)
GPIO_PUPDR_PUPDR11_0       EQU    (0x00400000)
GPIO_PUPDR_PUPDR11_1       EQU    (0x00800000)
GPIO_PUPDR_PUPDR12         EQU    (0x03000000)
GPIO_PUPDR_PUPDR12_0       EQU    (0x01000000)
GPIO_PUPDR_PUPDR12_1       EQU    (0x02000000)
GPIO_PUPDR_PUPDR13         EQU    (0x0C000000)
GPIO_PUPDR_PUPDR13_0       EQU    (0x04000000)
GPIO_PUPDR_PUPDR13_1       EQU    (0x08000000)
GPIO_PUPDR_PUPDR14         EQU    (0x30000000)
GPIO_PUPDR_PUPDR14_0       EQU    (0x10000000)
GPIO_PUPDR_PUPDR14_1       EQU    (0x20000000)
GPIO_PUPDR_PUPDR15         EQU    (0xC0000000)
GPIO_PUPDR_PUPDR15_0       EQU    (0x40000000)
GPIO_PUPDR_PUPDR15_1       EQU    (0x80000000)

;**************  Bit definition for GPIO_IDR register  ******************  
GPIO_OTYPER_IDR_0          EQU    (0x00000001)
GPIO_OTYPER_IDR_1          EQU    (0x00000002)
GPIO_OTYPER_IDR_2          EQU    (0x00000004)
GPIO_OTYPER_IDR_3          EQU    (0x00000008)
GPIO_OTYPER_IDR_4          EQU    (0x00000010)
GPIO_OTYPER_IDR_5          EQU    (0x00000020)
GPIO_OTYPER_IDR_6          EQU    (0x00000040)
GPIO_OTYPER_IDR_7          EQU    (0x00000080)
GPIO_OTYPER_IDR_8          EQU    (0x00000100)
GPIO_OTYPER_IDR_9          EQU    (0x00000200)
GPIO_OTYPER_IDR_10         EQU    (0x00000400)
GPIO_OTYPER_IDR_11         EQU    (0x00000800)
GPIO_OTYPER_IDR_12         EQU    (0x00001000)
GPIO_OTYPER_IDR_13         EQU    (0x00002000)
GPIO_OTYPER_IDR_14         EQU    (0x00004000)
GPIO_OTYPER_IDR_15         EQU    (0x00008000)

;**************  Bit definition for GPIO_ODR register  ******************   
GPIO_OTYPER_ODR_0          EQU    (0x00000001)
GPIO_OTYPER_ODR_1          EQU    (0x00000002)
GPIO_OTYPER_ODR_2          EQU    (0x00000004)
GPIO_OTYPER_ODR_3          EQU    (0x00000008)
GPIO_OTYPER_ODR_4          EQU    (0x00000010)
GPIO_OTYPER_ODR_5          EQU    (0x00000020)
GPIO_OTYPER_ODR_6          EQU    (0x00000040)
GPIO_OTYPER_ODR_7          EQU    (0x00000080)
GPIO_OTYPER_ODR_8          EQU    (0x00000100)
GPIO_OTYPER_ODR_9          EQU    (0x00000200)
GPIO_OTYPER_ODR_10         EQU    (0x00000400)
GPIO_OTYPER_ODR_11         EQU    (0x00000800)
GPIO_OTYPER_ODR_12         EQU    (0x00001000)
GPIO_OTYPER_ODR_13         EQU    (0x00002000)
GPIO_OTYPER_ODR_14         EQU    (0x00004000)
GPIO_OTYPER_ODR_15         EQU    (0x00008000)

;**************  Bit definition for GPIO_BSRR register  *****************  
GPIO_BSRR_BS_0             EQU    (0x00000001)
GPIO_BSRR_BS_1             EQU    (0x00000002)
GPIO_BSRR_BS_2             EQU    (0x00000004)
GPIO_BSRR_BS_3             EQU    (0x00000008)
GPIO_BSRR_BS_4             EQU    (0x00000010)
GPIO_BSRR_BS_5             EQU    (0x00000020)
GPIO_BSRR_BS_6             EQU    (0x00000040)
GPIO_BSRR_BS_7             EQU    (0x00000080)
GPIO_BSRR_BS_8             EQU    (0x00000100)
GPIO_BSRR_BS_9             EQU    (0x00000200)
GPIO_BSRR_BS_10            EQU    (0x00000400)
GPIO_BSRR_BS_11            EQU    (0x00000800)
GPIO_BSRR_BS_12            EQU    (0x00001000)
GPIO_BSRR_BS_13            EQU    (0x00002000)
GPIO_BSRR_BS_14            EQU    (0x00004000)
GPIO_BSRR_BS_15            EQU    (0x00008000)
GPIO_BSRR_BR_0             EQU    (0x00010000)
GPIO_BSRR_BR_1             EQU    (0x00020000)
GPIO_BSRR_BR_2             EQU    (0x00040000)
GPIO_BSRR_BR_3             EQU    (0x00080000)
GPIO_BSRR_BR_4             EQU    (0x00100000)
GPIO_BSRR_BR_5             EQU    (0x00200000)
GPIO_BSRR_BR_6             EQU    (0x00400000)
GPIO_BSRR_BR_7             EQU    (0x00800000)
GPIO_BSRR_BR_8             EQU    (0x01000000)
GPIO_BSRR_BR_9             EQU    (0x02000000)
GPIO_BSRR_BR_10            EQU    (0x04000000)
GPIO_BSRR_BR_11            EQU    (0x08000000)
GPIO_BSRR_BR_12            EQU    (0x10000000)
GPIO_BSRR_BR_13            EQU    (0x20000000)
GPIO_BSRR_BR_14            EQU    (0x40000000)
GPIO_BSRR_BR_15            EQU    (0x80000000)

;**************  Bit definition for GPIO_LCKR register  *****************
GPIO_LCKR_LCK0             EQU    (0x00000001)
GPIO_LCKR_LCK1             EQU    (0x00000002)
GPIO_LCKR_LCK2             EQU    (0x00000004)
GPIO_LCKR_LCK3             EQU    (0x00000008)
GPIO_LCKR_LCK4             EQU    (0x00000010)
GPIO_LCKR_LCK5             EQU    (0x00000020)
GPIO_LCKR_LCK6             EQU    (0x00000040)
GPIO_LCKR_LCK7             EQU    (0x00000080)
GPIO_LCKR_LCK8             EQU    (0x00000100)
GPIO_LCKR_LCK9             EQU    (0x00000200)
GPIO_LCKR_LCK10            EQU    (0x00000400)
GPIO_LCKR_LCK11            EQU    (0x00000800)
GPIO_LCKR_LCK12            EQU    (0x00001000)
GPIO_LCKR_LCK13            EQU    (0x00002000)
GPIO_LCKR_LCK14            EQU    (0x00004000)
GPIO_LCKR_LCK15            EQU    (0x00008000)
GPIO_LCKR_LCKK             EQU    (0x00010000)

;**************  Bit definition for GPIO_AFRL register  *****************
GPIO_AFRL_AFRL0            EQU    (0x0000000F)
GPIO_AFRL_AFRL1            EQU    (0x000000F0)
GPIO_AFRL_AFRL2            EQU    (0x00000F00)
GPIO_AFRL_AFRL3            EQU    (0x0000F000)
GPIO_AFRL_AFRL4            EQU    (0x000F0000)
GPIO_AFRL_AFRL5            EQU    (0x00F00000)
GPIO_AFRL_AFRL6            EQU    (0x0F000000)
GPIO_AFRL_AFRL7            EQU    (0xF0000000)

;**************  Bit definition for GPIO_AFRH register  *****************
GPIO_AFRH_AFRH8            EQU    (0x0000000F)
GPIO_AFRH_AFRH9            EQU    (0x000000F0)
GPIO_AFRH_AFRH10           EQU    (0x00000F00)
GPIO_AFRH_AFRH11           EQU    (0x0000F000)
GPIO_AFRH_AFRH12           EQU    (0x000F0000)
GPIO_AFRH_AFRH13           EQU    (0x00F00000)
GPIO_AFRH_AFRH14           EQU    (0x0F000000)
GPIO_AFRH_AFRH15           EQU    (0xF0000000)



;***************************************************************************
;                                                                           
;                    Inter-integrated Circuit Interface (I2C)               
;                                                                           
;***************************************************************************

;**************  Bit definition for I2C_CR1 register  *******************
I2C_CR1_PE                          EQU    (0x0001)            ; Peripheral Enable 
I2C_CR1_SMBUS                       EQU    (0x0002)            ; SMBus Mode 
I2C_CR1_SMBTYPE                     EQU    (0x0008)            ; SMBus Type 
I2C_CR1_ENARP                       EQU    (0x0010)            ; ARP Enable 
I2C_CR1_ENPEC                       EQU    (0x0020)            ; PEC Enable 
I2C_CR1_ENGC                        EQU    (0x0040)            ; General Call Enable 
I2C_CR1_NOSTRETCH                   EQU    (0x0080)            ; Clock Stretching Disable (Slave mode) 
I2C_CR1_START                       EQU    (0x0100)            ; Start Generation 
I2C_CR1_STOP                        EQU    (0x0200)            ; Stop Generation 
I2C_CR1_ACK                         EQU    (0x0400)            ; Acknowledge Enable 
I2C_CR1_POS                         EQU    (0x0800)            ; Acknowledge/PEC Position (for data reception) 
I2C_CR1_PEC                         EQU    (0x1000)            ; Packet Error Checking 
I2C_CR1_ALERT                       EQU    (0x2000)            ; SMBus Alert 
I2C_CR1_SWRST                       EQU    (0x8000)            ; Software Reset 

;**************  Bit definition for I2C_CR2 register  *******************
I2C_CR2_FREQ                        EQU    (0x003F)            ; FREQ[5:0] bits (Peripheral Clock Frequency) 
I2C_CR2_FREQ_0                      EQU    (0x0001)            ; Bit 0 
I2C_CR2_FREQ_1                      EQU    (0x0002)            ; Bit 1 
I2C_CR2_FREQ_2                      EQU    (0x0004)            ; Bit 2 
I2C_CR2_FREQ_3                      EQU    (0x0008)            ; Bit 3 
I2C_CR2_FREQ_4                      EQU    (0x0010)            ; Bit 4 
I2C_CR2_FREQ_5                      EQU    (0x0020)            ; Bit 5 

I2C_CR2_ITERREN                     EQU    (0x0100)            ; Error Interrupt Enable 
I2C_CR2_ITEVTEN                     EQU    (0x0200)            ; Event Interrupt Enable 
I2C_CR2_ITBUFEN                     EQU    (0x0400)            ; Buffer Interrupt Enable 
I2C_CR2_DMAEN                       EQU    (0x0800)            ; DMA Requests Enable 
I2C_CR2_LAST                        EQU    (0x1000)            ; DMA Last Transfer 

;**************  Bit definition for I2C_OAR1 register  ******************
I2C_OAR1_ADD1_7                     EQU    (0x00FE)            ; Interface Address 
I2C_OAR1_ADD8_9                     EQU    (0x0300)            ; Interface Address 

I2C_OAR1_ADD0                       EQU    (0x0001)            ; Bit 0 
I2C_OAR1_ADD1                       EQU    (0x0002)            ; Bit 1 
I2C_OAR1_ADD2                       EQU    (0x0004)            ; Bit 2 
I2C_OAR1_ADD3                       EQU    (0x0008)            ; Bit 3 
I2C_OAR1_ADD4                       EQU    (0x0010)            ; Bit 4 
I2C_OAR1_ADD5                       EQU    (0x0020)            ; Bit 5 
I2C_OAR1_ADD6                       EQU    (0x0040)            ; Bit 6 
I2C_OAR1_ADD7                       EQU    (0x0080)            ; Bit 7 
I2C_OAR1_ADD8                       EQU    (0x0100)            ; Bit 8 
I2C_OAR1_ADD9                       EQU    (0x0200)            ; Bit 9 

I2C_OAR1_ADDMODE                    EQU    (0x8000)            ; Addressing Mode (Slave mode) 

;**************  Bit definition for I2C_OAR2 register  ******************
I2C_OAR2_ENDUAL                     EQU    (0x01)               ; Dual addressing mode enable 
I2C_OAR2_ADD2                       EQU    (0xFE)               ; Interface address 

;**************  Bit definition for I2C_DR register  *******************
I2C_DR_DR                           EQU    (0xFF)               ; 8-bit Data Register 

;**************  Bit definition for I2C_SR1 register  *******************
I2C_SR1_SB                          EQU    (0x0001)            ; Start Bit (Master mode) 
I2C_SR1_ADDR                        EQU    (0x0002)            ; Address sent (master mode)/matched (slave mode) 
I2C_SR1_BTF                         EQU    (0x0004)            ; Byte Transfer Finished 
I2C_SR1_ADD10                       EQU    (0x0008)            ; 10-bit header sent (Master mode) 
I2C_SR1_STOPF                       EQU    (0x0010)            ; Stop detection (Slave mode) 
I2C_SR1_RXNE                        EQU    (0x0040)            ; Data Register not Empty (receivers) 
I2C_SR1_TXE                         EQU    (0x0080)            ; Data Register Empty (transmitters) 
I2C_SR1_BERR                        EQU    (0x0100)            ; Bus Error 
I2C_SR1_ARLO                        EQU    (0x0200)            ; Arbitration Lost (master mode) 
I2C_SR1_AF                          EQU    (0x0400)            ; Acknowledge Failure 
I2C_SR1_OVR                         EQU    (0x0800)            ; Overrun/Underrun 
I2C_SR1_PECERR                      EQU    (0x1000)            ; PEC Error in reception 
I2C_SR1_TIMEOUT                     EQU    (0x4000)            ; Timeout or Tlow Error 
I2C_SR1_SMBALERT                    EQU    (0x8000)            ; SMBus Alert 

;**************  Bit definition for I2C_SR2 register  *******************
I2C_SR2_MSL                         EQU    (0x0001)            ; Master/Slave 
I2C_SR2_BUSY                        EQU    (0x0002)            ; Bus Busy 
I2C_SR2_TRA                         EQU    (0x0004)            ; Transmitter/Receiver 
I2C_SR2_GENCALL                     EQU    (0x0010)            ; General Call Address (Slave mode) 
I2C_SR2_SMBDEFAULT                  EQU    (0x0020)            ; SMBus Device Default Address (Slave mode) 
I2C_SR2_SMBHOST                     EQU    (0x0040)            ; SMBus Host Header (Slave mode) 
I2C_SR2_DUALF                       EQU    (0x0080)            ; Dual Flag (Slave mode) 
I2C_SR2_PEC                         EQU    (0xFF00)            ; Packet Error Checking Register 

;**************  Bit definition for I2C_CCR register  *******************
I2C_CCR_CCR                         EQU    (0x0FFF)            ; Clock Control Register in Fast/Standard mode (Master mode) 
I2C_CCR_DUTY                        EQU    (0x4000)            ; Fast Mode Duty Cycle 
I2C_CCR_FS                          EQU    (0x8000)            ; I2C Master Mode Selection 

;**************  Bit definition for I2C_TRISE register  ******************
I2C_TRISE_TRISE                     EQU    (0x3F)               ; Maximum Rise Time in Fast/Standard mode (Master mode) 



;***************************************************************************
;                                                                           
;                         Independent WATCHDOG (IWDG)                       
;                                                                           
;***************************************************************************

;**************  Bit definition for IWDG_KR register  *******************
IWDG_KR_KEY                         EQU    (0xFFFF)            ; Key value (write only, read 0000h) 

;**************  Bit definition for IWDG_PR register  *******************
IWDG_PR_PR                          EQU    (0x07)               ; PR[2:0] (Prescaler divider) 
IWDG_PR_PR_0                        EQU    (0x01)               ; Bit 0 
IWDG_PR_PR_1                        EQU    (0x02)               ; Bit 1 
IWDG_PR_PR_2                        EQU    (0x04)               ; Bit 2 

;**************  Bit definition for IWDG_RLR register  ******************
IWDG_RLR_RL                         EQU    (0x0FFF)            ; Watchdog counter reload value 

;**************  Bit definition for IWDG_SR register  *******************
IWDG_SR_PVU                         EQU    (0x01)               ; Watchdog prescaler value update 
IWDG_SR_RVU                         EQU    (0x02)               ; Watchdog counter reload value update 



;***************************************************************************
;                                                                           
;                           LCD Controller (LCD)                            
;                                                                           
;***************************************************************************

;**************  Bit definition for LCD_CR register  ********************
LCD_CR_LCDEN               EQU    (0x00000001)     ; LCD Enable Bit 
LCD_CR_VSEL                EQU    (0x00000002)     ; Voltage source selector Bit 

LCD_CR_DUTY                EQU    (0x0000001C)     ; DUTY[2:0] bits (Duty selector) 
LCD_CR_DUTY_0              EQU    (0x00000004)     ; Duty selector Bit 0 
LCD_CR_DUTY_1              EQU    (0x00000008)     ; Duty selector Bit 1 
LCD_CR_DUTY_2              EQU    (0x00000010)     ; Duty selector Bit 2 

LCD_CR_BIAS                EQU    (0x00000060)     ; BIAS[1:0] bits (Bias selector) 
LCD_CR_BIAS_0              EQU    (0x00000020)     ; Bias selector Bit 0 
LCD_CR_BIAS_1              EQU    (0x00000040)     ; Bias selector Bit 1 

LCD_CR_MUX_SEG             EQU    (0x00000080)     ; Mux Segment Enable Bit 

;**************  Bit definition for LCD_FCR register  *******************
LCD_FCR_HD                 EQU    (0x00000001)     ; High Drive Enable Bit 
LCD_FCR_SOFIE              EQU    (0x00000002)     ; Start of Frame Interrupt Enable Bit 
LCD_FCR_UDDIE              EQU    (0x00000008)     ; Update Display Done Interrupt Enable Bit 

LCD_FCR_PON                EQU    (0x00000070)     ; PON[2:0] bits (Puls ON Duration) 
LCD_FCR_PON_0              EQU    (0x00000010)     ; Bit 0 
LCD_FCR_PON_1              EQU    (0x00000020)     ; Bit 1 
LCD_FCR_PON_2              EQU    (0x00000040)     ; Bit 2 

LCD_FCR_DEAD               EQU    (0x00000380)     ; DEAD[2:0] bits (DEAD Time) 
LCD_FCR_DEAD_0             EQU    (0x00000080)     ; Bit 0 
LCD_FCR_DEAD_1             EQU    (0x00000100)     ; Bit 1 
LCD_FCR_DEAD_2             EQU    (0x00000200)     ; Bit 2 

LCD_FCR_CC                 EQU    (0x00001C00)     ; CC[2:0] bits (Contrast Control) 
LCD_FCR_CC_0               EQU    (0x00000400)     ; Bit 0 
LCD_FCR_CC_1               EQU    (0x00000800)     ; Bit 1 
LCD_FCR_CC_2               EQU    (0x00001000)     ; Bit 2 

LCD_FCR_BLINKF             EQU    (0x0000E000)     ; BLINKF[2:0] bits (Blink Frequency) 
LCD_FCR_BLINKF_0           EQU    (0x00002000)     ; Bit 0 
LCD_FCR_BLINKF_1           EQU    (0x00004000)     ; Bit 1 
LCD_FCR_BLINKF_2           EQU    (0x00008000)     ; Bit 2 

LCD_FCR_BLINK              EQU    (0x00030000)     ; BLINK[1:0] bits (Blink Enable) 
LCD_FCR_BLINK_0            EQU    (0x00010000)     ; Bit 0 
LCD_FCR_BLINK_1            EQU    (0x00020000)     ; Bit 1 

LCD_FCR_DIV                EQU    (0x003C0000)     ; DIV[3:0] bits (Divider) 
LCD_FCR_PS                 EQU    (0x03C00000)     ; PS[3:0] bits (Prescaler) 

;**************  Bit definition for LCD_SR register  ********************
LCD_SR_ENS                 EQU    (0x00000001)     ; LCD Enabled Bit 
LCD_SR_SOF                 EQU    (0x00000002)     ; Start Of Frame Flag Bit 
LCD_SR_UDR                 EQU    (0x00000004)     ; Update Display Request Bit 
LCD_SR_UDD                 EQU    (0x00000008)     ; Update Display Done Flag Bit 
LCD_SR_RDY                 EQU    (0x00000010)     ; Ready Flag Bit 
LCD_SR_FCRSR               EQU    (0x00000020)     ; LCD FCR Register Synchronization Flag Bit 

;**************  Bit definition for LCD_CLR register  *******************
LCD_CLR_SOFC               EQU    (0x00000002)     ; Start Of Frame Flag Clear Bit 
LCD_CLR_UDDC               EQU    (0x00000008)     ; Update Display Done Flag Clear Bit 

;**************  Bit definition for LCD_RAM register  *******************
LCD_RAM_SEGMENT_DATA       EQU    (0xFFFFFFFF)     ; Segment Data Bits 



;***************************************************************************
;                                                                           
;                           Power Control (PWR)                             
;                                                                           
;***************************************************************************

;**************  Bit definition for PWR_CR register  *******************
PWR_CR_LPSDSR                       EQU    (0x0001)     ; Low-power deepsleep/sleep/low power run 
PWR_CR_PDDS                         EQU    (0x0002)     ; Power Down Deepsleep 
PWR_CR_CWUF                         EQU    (0x0004)     ; Clear Wakeup Flag 
PWR_CR_CSBF                         EQU    (0x0008)     ; Clear Standby Flag 
PWR_CR_PVDE                         EQU    (0x0010)     ; Power Voltage Detector Enable 

PWR_CR_PLS                          EQU    (0x00E0)     ; PLS[2:0] bits (PVD Level Selection) 
PWR_CR_PLS_0                        EQU    (0x0020)     ; Bit 0 
PWR_CR_PLS_1                        EQU    (0x0040)     ; Bit 1 
PWR_CR_PLS_2                        EQU    (0x0080)     ; Bit 2 

; PVD level configuration 
PWR_CR_PLS_LEV0                     EQU    (0x0000)     ; PVD level 0 
PWR_CR_PLS_LEV1                     EQU    (0x0020)     ; PVD level 1 
PWR_CR_PLS_LEV2                     EQU    (0x0040)     ; PVD level 2 
PWR_CR_PLS_LEV3                     EQU    (0x0060)     ; PVD level 3 
PWR_CR_PLS_LEV4                     EQU    (0x0080)     ; PVD level 4 
PWR_CR_PLS_LEV5                     EQU    (0x00A0)     ; PVD level 5 
PWR_CR_PLS_LEV6                     EQU    (0x00C0)     ; PVD level 6 
PWR_CR_PLS_LEV7                     EQU    (0x00E0)     ; PVD level 7 

PWR_CR_DBP                          EQU    (0x0100)     ; Disable Backup Domain write protection 
PWR_CR_ULP                          EQU    (0x0200)     ; Ultra Low Power mode 
PWR_CR_FWU                          EQU    (0x0400)     ; Fast wakeup 

PWR_CR_VOS                          EQU    (0x1800)     ; VOS[1:0] bits (Voltage scaling range selection) 
PWR_CR_VOS_0                        EQU    (0x0800)     ; Bit 0 
PWR_CR_VOS_1                        EQU    (0x1000)     ; Bit 1 
PWR_CR_LPRUN                        EQU    (0x4000)     ; Low power run mode 

;**************  Bit definition for PWR_CSR register  *******************
PWR_CSR_WUF                         EQU    (0x0001)     ; Wakeup Flag 
PWR_CSR_SBF                         EQU    (0x0002)     ; Standby Flag 
PWR_CSR_PVDO                        EQU    (0x0004)     ; PVD Output 
PWR_CSR_VREFINTRDYF                 EQU    (0x0008)     ; Internal voltage reference (VREFINT) ready flag 
PWR_CSR_VOSF                        EQU    (0x0010)     ; Voltage Scaling select flag 
PWR_CSR_REGLPF                      EQU    (0x0020)     ; Regulator LP flag 

PWR_CSR_EWUP1                       EQU    (0x0100)     ; Enable WKUP pin 1 
PWR_CSR_EWUP2                       EQU    (0x0200)     ; Enable WKUP pin 2 
PWR_CSR_EWUP3                       EQU    (0x0400)     ; Enable WKUP pin 3 



;***************************************************************************
;                                                                           
;                       Reset and Clock Control (RCC)                       
;                                                                           
;***************************************************************************
;**************  Bit definition for RCC_CR register  *******************
RCC_CR_HSION                        EQU    (0x00000001)        ; Internal High Speed clock enable 
RCC_CR_HSIRDY                       EQU    (0x00000002)        ; Internal High Speed clock ready flag 

RCC_CR_MSION                        EQU    (0x00000100)        ; Internal Multi Speed clock enable 
RCC_CR_MSIRDY                       EQU    (0x00000200)        ; Internal Multi Speed clock ready flag 

RCC_CR_HSEON                        EQU    (0x00010000)        ; External High Speed clock enable 
RCC_CR_HSERDY                       EQU    (0x00020000)        ; External High Speed clock ready flag 
RCC_CR_HSEBYP                       EQU    (0x00040000)        ; External High Speed clock Bypass 

RCC_CR_PLLON                        EQU    (0x01000000)        ; PLL enable 
RCC_CR_PLLRDY                       EQU    (0x02000000)        ; PLL clock ready flag 
RCC_CR_CSSON                        EQU    (0x10000000)        ; Clock Security System enable 

RCC_CR_RTCPRE                       EQU    (0x60000000)        ; RTC/LCD Prescaler 
RCC_CR_RTCPRE_0                     EQU    (0x20000000)        ; Bit0 
RCC_CR_RTCPRE_1                     EQU    (0x40000000)        ; Bit1 

;**************  Bit definition for RCC_ICSCR register  ****************
RCC_ICSCR_HSICAL                    EQU    (0x000000FF)        ; Internal High Speed clock Calibration 
RCC_ICSCR_HSITRIM                   EQU    (0x00001F00)        ; Internal High Speed clock trimming 

RCC_ICSCR_MSIRANGE                  EQU    (0x0000E000)        ; Internal Multi Speed clock Range 
RCC_ICSCR_MSIRANGE_0                EQU    (0x00000000)        ; Internal Multi Speed clock Range 65.536 KHz 
RCC_ICSCR_MSIRANGE_1                EQU    (0x00002000)        ; Internal Multi Speed clock Range 131.072 KHz 
RCC_ICSCR_MSIRANGE_2                EQU    (0x00004000)        ; Internal Multi Speed clock Range 262.144 KHz 
RCC_ICSCR_MSIRANGE_3                EQU    (0x00006000)        ; Internal Multi Speed clock Range 524.288 KHz 
RCC_ICSCR_MSIRANGE_4                EQU    (0x00008000)        ; Internal Multi Speed clock Range 1.048 MHz 
RCC_ICSCR_MSIRANGE_5                EQU    (0x0000A000)        ; Internal Multi Speed clock Range 2.097 MHz 
RCC_ICSCR_MSIRANGE_6                EQU    (0x0000C000)        ; Internal Multi Speed clock Range 4.194 MHz 
RCC_ICSCR_MSICAL                    EQU    (0x00FF0000)        ; Internal Multi Speed clock Calibration 
RCC_ICSCR_MSITRIM                   EQU    (0xFF000000)        ; Internal Multi Speed clock trimming 

;**************  Bit definition for RCC_CFGR register  *****************
RCC_CFGR_SW                         EQU    (0x00000003)        ; SW[1:0] bits (System clock Switch) 
RCC_CFGR_SW_0                       EQU    (0x00000001)        ; Bit 0 
RCC_CFGR_SW_1                       EQU    (0x00000002)        ; Bit 1 

; SW configuration 
RCC_CFGR_SW_MSI                     EQU    (0x00000000)        ; MSI selected as system clock 
RCC_CFGR_SW_HSI                     EQU    (0x00000001)        ; HSI selected as system clock 
RCC_CFGR_SW_HSE                     EQU    (0x00000002)        ; HSE selected as system clock 
RCC_CFGR_SW_PLL                     EQU    (0x00000003)        ; PLL selected as system clock 

RCC_CFGR_SWS                        EQU    (0x0000000C)        ; SWS[1:0] bits (System Clock Switch Status) 
RCC_CFGR_SWS_0                      EQU    (0x00000004)        ; Bit 0 
RCC_CFGR_SWS_1                      EQU    (0x00000008)        ; Bit 1 

; SWS configuration 
RCC_CFGR_SWS_MSI                    EQU    (0x00000000)        ; MSI oscillator used as system clock 
RCC_CFGR_SWS_HSI                    EQU    (0x00000004)        ; HSI oscillator used as system clock 
RCC_CFGR_SWS_HSE                    EQU    (0x00000008)        ; HSE oscillator used as system clock 
RCC_CFGR_SWS_PLL                    EQU    (0x0000000C)        ; PLL used as system clock 

RCC_CFGR_HPRE                       EQU    (0x000000F0)        ; HPRE[3:0] bits (AHB prescaler) 
RCC_CFGR_HPRE_0                     EQU    (0x00000010)        ; Bit 0 
RCC_CFGR_HPRE_1                     EQU    (0x00000020)        ; Bit 1 
RCC_CFGR_HPRE_2                     EQU    (0x00000040)        ; Bit 2 
RCC_CFGR_HPRE_3                     EQU    (0x00000080)        ; Bit 3 

; HPRE configuration 
RCC_CFGR_HPRE_DIV1                  EQU    (0x00000000)        ; SYSCLK not divided 
RCC_CFGR_HPRE_DIV2                  EQU    (0x00000080)        ; SYSCLK divided by 2 
RCC_CFGR_HPRE_DIV4                  EQU    (0x00000090)        ; SYSCLK divided by 4 
RCC_CFGR_HPRE_DIV8                  EQU    (0x000000A0)        ; SYSCLK divided by 8 
RCC_CFGR_HPRE_DIV16                 EQU    (0x000000B0)        ; SYSCLK divided by 16 
RCC_CFGR_HPRE_DIV64                 EQU    (0x000000C0)        ; SYSCLK divided by 64 
RCC_CFGR_HPRE_DIV128                EQU    (0x000000D0)        ; SYSCLK divided by 128 
RCC_CFGR_HPRE_DIV256                EQU    (0x000000E0)        ; SYSCLK divided by 256 
RCC_CFGR_HPRE_DIV512                EQU    (0x000000F0)        ; SYSCLK divided by 512 

RCC_CFGR_PPRE1                      EQU    (0x00000700)        ; PRE1[2:0] bits (APB1 prescaler) 
RCC_CFGR_PPRE1_0                    EQU    (0x00000100)        ; Bit 0 
RCC_CFGR_PPRE1_1                    EQU    (0x00000200)        ; Bit 1 
RCC_CFGR_PPRE1_2                    EQU    (0x00000400)        ; Bit 2 

; PPRE1 configuration 
RCC_CFGR_PPRE1_DIV1                 EQU    (0x00000000)        ; HCLK not divided 
RCC_CFGR_PPRE1_DIV2                 EQU    (0x00000400)        ; HCLK divided by 2 
RCC_CFGR_PPRE1_DIV4                 EQU    (0x00000500)        ; HCLK divided by 4 
RCC_CFGR_PPRE1_DIV8                 EQU    (0x00000600)        ; HCLK divided by 8 
RCC_CFGR_PPRE1_DIV16                EQU    (0x00000700)        ; HCLK divided by 16 

RCC_CFGR_PPRE2                      EQU    (0x00003800)        ; PRE2[2:0] bits (APB2 prescaler) 
RCC_CFGR_PPRE2_0                    EQU    (0x00000800)        ; Bit 0 
RCC_CFGR_PPRE2_1                    EQU    (0x00001000)        ; Bit 1 
RCC_CFGR_PPRE2_2                    EQU    (0x00002000)        ; Bit 2 

; PPRE2 configuration 
RCC_CFGR_PPRE2_DIV1                 EQU    (0x00000000)        ; HCLK not divided 
RCC_CFGR_PPRE2_DIV2                 EQU    (0x00002000)        ; HCLK divided by 2 
RCC_CFGR_PPRE2_DIV4                 EQU    (0x00002800)        ; HCLK divided by 4 
RCC_CFGR_PPRE2_DIV8                 EQU    (0x00003000)        ; HCLK divided by 8 
RCC_CFGR_PPRE2_DIV16                EQU    (0x00003800)        ; HCLK divided by 16 

; PLL entry clock source
RCC_CFGR_PLLSRC                     EQU    (0x00010000)        ; PLL entry clock source 

RCC_CFGR_PLLSRC_HSI                 EQU    (0x00000000)        ; HSI as PLL entry clock source 
RCC_CFGR_PLLSRC_HSE                 EQU    (0x00010000)        ; HSE as PLL entry clock source 


RCC_CFGR_PLLMUL                     EQU    (0x003C0000)        ; PLLMUL[3:0] bits (PLL multiplication factor) 
RCC_CFGR_PLLMUL_0                   EQU    (0x00040000)        ; Bit 0 
RCC_CFGR_PLLMUL_1                   EQU    (0x00080000)        ; Bit 1 
RCC_CFGR_PLLMUL_2                   EQU    (0x00100000)        ; Bit 2 
RCC_CFGR_PLLMUL_3                   EQU    (0x00200000)        ; Bit 3 

; PLLMUL configuration 
RCC_CFGR_PLLMUL3                    EQU    (0x00000000)        ; PLL input clock * 3 
RCC_CFGR_PLLMUL4                    EQU    (0x00040000)        ; PLL input clock * 4 
RCC_CFGR_PLLMUL6                    EQU    (0x00080000)        ; PLL input clock * 6 
RCC_CFGR_PLLMUL8                    EQU    (0x000C0000)        ; PLL input clock * 8 
RCC_CFGR_PLLMUL12                   EQU    (0x00100000)        ; PLL input clock * 12 
RCC_CFGR_PLLMUL16                   EQU    (0x00140000)        ; PLL input clock * 16 
RCC_CFGR_PLLMUL24                   EQU    (0x00180000)        ; PLL input clock * 24 
RCC_CFGR_PLLMUL32                   EQU    (0x001C0000)        ; PLL input clock * 32 
RCC_CFGR_PLLMUL48                   EQU    (0x00200000)        ; PLL input clock * 48 

; PLLDIV configuration 
RCC_CFGR_PLLDIV                     EQU    (0x00C00000)        ; PLLDIV[1:0] bits (PLL Output Division) 
RCC_CFGR_PLLDIV_0                   EQU    (0x00400000)        ; Bit0 
RCC_CFGR_PLLDIV_1                   EQU    (0x00800000)        ; Bit1 


; PLLDIV configuration 
RCC_CFGR_PLLDIV1                    EQU    (0x00000000)        ; PLL clock output EQU CKVCO / 1 
RCC_CFGR_PLLDIV2                    EQU    (0x00400000)        ; PLL clock output EQU CKVCO / 2 
RCC_CFGR_PLLDIV3                    EQU    (0x00800000)        ; PLL clock output EQU CKVCO / 3 
RCC_CFGR_PLLDIV4                    EQU    (0x00C00000)        ; PLL clock output EQU CKVCO / 4 


RCC_CFGR_MCOSEL                     EQU    (0x07000000)        ; MCO[2:0] bits (Microcontroller Clock Output) 
RCC_CFGR_MCOSEL_0                   EQU    (0x01000000)        ; Bit 0 
RCC_CFGR_MCOSEL_1                   EQU    (0x02000000)        ; Bit 1 
RCC_CFGR_MCOSEL_2                   EQU    (0x04000000)        ; Bit 2 

; MCO configuration 
RCC_CFGR_MCO_NOCLOCK                EQU    (0x00000000)        ; No clock 
RCC_CFGR_MCO_SYSCLK                 EQU    (0x01000000)        ; System clock selected 
RCC_CFGR_MCO_HSI                    EQU    (0x02000000)        ; Internal 16 MHz RC oscillator clock selected 
RCC_CFGR_MCO_MSI                    EQU    (0x03000000)        ; Internal Medium Speed RC oscillator clock selected 
RCC_CFGR_MCO_HSE                    EQU    (0x04000000)        ; External 1-25 MHz oscillator clock selected 
RCC_CFGR_MCO_PLL                    EQU    (0x05000000)        ; PLL clock divided 
RCC_CFGR_MCO_LSI                    EQU    (0x06000000)        ; LSI selected 
RCC_CFGR_MCO_LSE                    EQU    (0x07000000)        ; LSE selected 

RCC_CFGR_MCOPRE                     EQU    (0x70000000)        ; MCOPRE[2:0] bits (Microcontroller Clock Output Prescaler) 
RCC_CFGR_MCOPRE_0                   EQU    (0x10000000)        ; Bit 0 
RCC_CFGR_MCOPRE_1                   EQU    (0x20000000)        ; Bit 1 
RCC_CFGR_MCOPRE_2                   EQU    (0x40000000)        ; Bit 2 

; MCO Prescaler configuration 
RCC_CFGR_MCO_DIV1                   EQU    (0x00000000)        ; MCO Clock divided by 1 
RCC_CFGR_MCO_DIV2                   EQU    (0x10000000)        ; MCO Clock divided by 2 
RCC_CFGR_MCO_DIV4                   EQU    (0x20000000)        ; MCO Clock divided by 4 
RCC_CFGR_MCO_DIV8                   EQU    (0x30000000)        ; MCO Clock divided by 8 
RCC_CFGR_MCO_DIV16                  EQU    (0x40000000)        ; MCO Clock divided by 16 

;**************  Bit definition for RCC_CIR register  *******************
RCC_CIR_LSIRDYF                     EQU    (0x00000001)        ; LSI Ready Interrupt flag 
RCC_CIR_LSERDYF                     EQU    (0x00000002)        ; LSE Ready Interrupt flag 
RCC_CIR_HSIRDYF                     EQU    (0x00000004)        ; HSI Ready Interrupt flag 
RCC_CIR_HSERDYF                     EQU    (0x00000008)        ; HSE Ready Interrupt flag 
RCC_CIR_PLLRDYF                     EQU    (0x00000010)        ; PLL Ready Interrupt flag 
RCC_CIR_MSIRDYF                     EQU    (0x00000020)        ; MSI Ready Interrupt flag 
RCC_CIR_CSSF                        EQU    (0x00000080)        ; Clock Security System Interrupt flag 

RCC_CIR_LSIRDYIE                    EQU    (0x00000100)        ; LSI Ready Interrupt Enable 
RCC_CIR_LSERDYIE                    EQU    (0x00000200)        ; LSE Ready Interrupt Enable 
RCC_CIR_HSIRDYIE                    EQU    (0x00000400)        ; HSI Ready Interrupt Enable 
RCC_CIR_HSERDYIE                    EQU    (0x00000800)        ; HSE Ready Interrupt Enable 
RCC_CIR_PLLRDYIE                    EQU    (0x00001000)        ; PLL Ready Interrupt Enable 
RCC_CIR_MSIRDYIE                    EQU    (0x00002000)        ; MSI Ready Interrupt Enable 

RCC_CIR_LSIRDYC                     EQU    (0x00010000)        ; LSI Ready Interrupt Clear 
RCC_CIR_LSERDYC                     EQU    (0x00020000)        ; LSE Ready Interrupt Clear 
RCC_CIR_HSIRDYC                     EQU    (0x00040000)        ; HSI Ready Interrupt Clear 
RCC_CIR_HSERDYC                     EQU    (0x00080000)        ; HSE Ready Interrupt Clear 
RCC_CIR_PLLRDYC                     EQU    (0x00100000)        ; PLL Ready Interrupt Clear 
RCC_CIR_MSIRDYC                     EQU    (0x00200000)        ; MSI Ready Interrupt Clear 
RCC_CIR_CSSC                        EQU    (0x00800000)        ; Clock Security System Interrupt Clear 


;**************  Bit definition for RCC_AHBRSTR register  *****************
RCC_AHBRSTR_GPIOARST                EQU    (0x00000001)        ; GPIO port A reset 
RCC_AHBRSTR_GPIOBRST                EQU    (0x00000002)        ; GPIO port B reset 
RCC_AHBRSTR_GPIOCRST                EQU    (0x00000004)        ; GPIO port C reset 
RCC_AHBRSTR_GPIODRST                EQU    (0x00000008)        ; GPIO port D reset 
RCC_AHBRSTR_GPIOERST                EQU    (0x00000010)        ; GPIO port E reset 
RCC_AHBRSTR_GPIOHRST                EQU    (0x00000020)        ; GPIO port H reset 
RCC_AHBRSTR_CRCRST                  EQU    (0x00001000)        ; CRC reset 
RCC_AHBRSTR_FLITFRST                EQU    (0x00008000)        ; FLITF reset 
RCC_AHBRSTR_DMA1RST                 EQU    (0x01000000)        ; DMA1 reset 
 
;**************  Bit definition for RCC_APB2RSTR register  ****************
RCC_APB2RSTR_SYSCFGRST              EQU    (0x00000001)        ; System Configuration SYSCFG reset 
RCC_APB2RSTR_TIM9RST                EQU    (0x00000004)        ; TIM9 reset 
RCC_APB2RSTR_TIM10RST               EQU    (0x00000008)        ; TIM10 reset 
RCC_APB2RSTR_TIM11RST               EQU    (0x00000010)        ; TIM11 reset 
RCC_APB2RSTR_ADC1RST                EQU    (0x00000200)        ;ADC1 reset 
RCC_APB2RSTR_SPI1RST                EQU    (0x00001000)        ; SPI1 reset 
RCC_APB2RSTR_USART1RST              EQU    (0x00004000)        ; USART1 reset 

;**************  Bit definition for RCC_APB1RSTR register  ****************
RCC_APB1RSTR_TIM2RST                EQU    (0x00000001)        ; Timer 2 reset 
RCC_APB1RSTR_TIM3RST                EQU    (0x00000002)        ; Timer 3 reset 
RCC_APB1RSTR_TIM4RST                EQU    (0x00000004)        ; Timer 4 reset 
RCC_APB1RSTR_TIM6RST                EQU    (0x00000010)        ; Timer 6 reset 
RCC_APB1RSTR_TIM7RST                EQU    (0x00000020)        ; Timer 7 reset 
RCC_APB1RSTR_LCDRST                 EQU    (0x00000200)        ; LCD reset 
RCC_APB1RSTR_WWDGRST                EQU    (0x00000800)        ; Window Watchdog reset 
RCC_APB1RSTR_SPI2RST                EQU    (0x00004000)        ; SPI 2 reset 
RCC_APB1RSTR_USART2RST              EQU    (0x00020000)        ; USART 2 reset 
RCC_APB1RSTR_USART3RST              EQU    (0x00040000)        ; RUSART 3 reset 
RCC_APB1RSTR_I2C1RST                EQU    (0x00200000)        ; I2C 1 reset 
RCC_APB1RSTR_I2C2RST                EQU    (0x00400000)        ; I2C 2 reset 
RCC_APB1RSTR_USBRST                 EQU    (0x00800000)        ; USB reset 
RCC_APB1RSTR_PWRRST                 EQU    (0x10000000)        ; Power interface reset 
RCC_APB1RSTR_DACRST                 EQU    (0x20000000)        ; DAC interface reset 
RCC_APB1RSTR_COMPRST                EQU    (0x80000000)        ; Comparator interface reset 

;**************  Bit definition for RCC_AHBENR register  *****************
RCC_AHBENR_GPIOAEN                  EQU    (0x00000001)        ; GPIO port A clock enable 
RCC_AHBENR_GPIOBEN                  EQU    (0x00000002)        ; GPIO port B clock enable 
RCC_AHBENR_GPIOCEN                  EQU    (0x00000004)        ; GPIO port C clock enable 
RCC_AHBENR_GPIODEN                  EQU    (0x00000008)        ; GPIO port D clock enable 
RCC_AHBENR_GPIOEEN                  EQU    (0x00000010)        ; GPIO port E clock enable 
RCC_AHBENR_GPIOHEN                  EQU    (0x00000020)        ; GPIO port H clock enable 
RCC_AHBENR_CRCEN                    EQU    (0x00001000)        ; CRC clock enable 
RCC_AHBENR_FLITFEN                  EQU    (0x00008000)        ; FLITF clock enable (has effect only when
                                                               ;   the Flash memory is in power down mode) 
RCC_AHBENR_DMA1EN                   EQU    (0x01000000)        ; DMA1 clock enable 


;**************  Bit definition for RCC_APB2ENR register  ****************
RCC_APB2ENR_SYSCFGEN                EQU    (0x00000001)         ; System Configuration SYSCFG clock enable 
RCC_APB2ENR_TIM9EN                  EQU    (0x00000004)         ; TIM9 interface clock enable 
RCC_APB2ENR_TIM10EN                 EQU    (0x00000008)         ; TIM10 interface clock enable 
RCC_APB2ENR_TIM11EN                 EQU    (0x00000010)         ; TIM11 Timer clock enable 
RCC_APB2ENR_ADC1EN                  EQU    (0x00000200)         ; ADC1 clock enable 
RCC_APB2ENR_SPI1EN                  EQU    (0x00001000)         ; SPI1 clock enable 
RCC_APB2ENR_USART1EN                EQU    (0x00004000)         ; USART1 clock enable 


;**************  Bit definition for RCC_APB1ENR register  *****************
RCC_APB1ENR_TIM2EN                  EQU    (0x00000001)        ; Timer 2 clock enabled
RCC_APB1ENR_TIM3EN                  EQU    (0x00000002)        ; Timer 3 clock enable 
RCC_APB1ENR_TIM4EN                  EQU    (0x00000004)        ; Timer 4 clock enable 
RCC_APB1ENR_TIM6EN                  EQU    (0x00000010)        ; Timer 6 clock enable 
RCC_APB1ENR_TIM7EN                  EQU    (0x00000020)        ; Timer 7 clock enable 
RCC_APB1ENR_LCDEN                   EQU    (0x00000200)        ; LCD clock enable 
RCC_APB1ENR_WWDGEN                  EQU    (0x00000800)        ; Window Watchdog clock enable 
RCC_APB1ENR_SPI2EN                  EQU    (0x00004000)        ; SPI 2 clock enable 
RCC_APB1ENR_USART2EN                EQU    (0x00020000)        ; USART 2 clock enable 
RCC_APB1ENR_USART3EN                EQU    (0x00040000)        ; USART 3 clock enable 
RCC_APB1ENR_I2C1EN                  EQU    (0x00200000)        ; I2C 1 clock enable 
RCC_APB1ENR_I2C2EN                  EQU    (0x00400000)        ; I2C 2 clock enable 
RCC_APB1ENR_USBEN                   EQU    (0x00800000)        ; USB clock enable 
RCC_APB1ENR_PWREN                   EQU    (0x10000000)        ; Power interface clock enable 
RCC_APB1ENR_DACEN                   EQU    (0x20000000)        ; DAC interface clock enable 
RCC_APB1ENR_COMPEN                  EQU    (0x80000000)        ; Comparator interface clock enable 

;**************  Bit definition for RCC_AHBLPENR register  ***************
RCC_AHBLPENR_GPIOALPEN              EQU    (0x00000001)        ; GPIO port A clock enabled in sleep mode 
RCC_AHBLPENR_GPIOBLPEN              EQU    (0x00000002)        ; GPIO port B clock enabled in sleep mode 
RCC_AHBLPENR_GPIOCLPEN              EQU    (0x00000004)        ; GPIO port C clock enabled in sleep mode 
RCC_AHBLPENR_GPIODLPEN              EQU    (0x00000008)        ; GPIO port D clock enabled in sleep mode 
RCC_AHBLPENR_GPIOELPEN              EQU    (0x00000010)        ; GPIO port E clock enabled in sleep mode 
RCC_AHBLPENR_GPIOHLPEN              EQU    (0x00000020)        ; GPIO port H clock enabled in sleep mode 
RCC_AHBLPENR_CRCLPEN                EQU    (0x00001000)        ; CRC clock enabled in sleep mode 
RCC_AHBLPENR_FLITFLPEN              EQU    (0x00008000)        ; Flash Interface clock enabled in sleep mode
                                                               ;   (has effect only when the Flash memory is
                                                               ;   in power down mode) 
RCC_AHBLPENR_SRAMLPEN               EQU    (0x00010000)        ; SRAM clock enabled in sleep mode 
RCC_AHBLPENR_DMA1LPEN               EQU    (0x01000000)        ; DMA1 clock enabled in sleep mode 

;**************  Bit definition for RCC_APB2LPENR register  **************
RCC_APB2LPENR_SYSCFGLPEN            EQU    (0x00000001)         ; System Configuration SYSCFG clock enabled in sleep mode 
RCC_APB2LPENR_TIM9LPEN              EQU    (0x00000004)         ; TIM9 interface clock enabled in sleep mode 
RCC_APB2LPENR_TIM10LPEN             EQU    (0x00000008)         ; TIM10 interface clock enabled in sleep mode 
RCC_APB2LPENR_TIM11LPEN             EQU    (0x00000010)         ; TIM11 Timer clock enabled in sleep mode 
RCC_APB2LPENR_ADC1LPEN              EQU    (0x00000200)         ; ADC1 clock enabled in sleep mode 
RCC_APB2LPENR_SPI1LPEN              EQU    (0x00001000)         ; SPI1 clock enabled in sleep mode 
RCC_APB2LPENR_USART1LPEN            EQU    (0x00004000)         ; USART1 clock enabled in sleep mode 

;**************  Bit definition for RCC_APB1LPENR register  ***************
RCC_APB1LPENR_TIM2LPEN              EQU    (0x00000001)        ; Timer 2 clock enabled in sleep mode 
RCC_APB1LPENR_TIM3LPEN              EQU    (0x00000002)        ; Timer 3 clock enabled in sleep mode 
RCC_APB1LPENR_TIM4LPEN              EQU    (0x00000004)        ; Timer 4 clock enabled in sleep mode 
RCC_APB1LPENR_TIM6LPEN              EQU    (0x00000010)        ; Timer 6 clock enabled in sleep mode 
RCC_APB1LPENR_TIM7LPEN              EQU    (0x00000020)        ; Timer 7 clock enabled in sleep mode 
RCC_APB1LPENR_LCDLPEN               EQU    (0x00000200)        ; LCD clock enabled in sleep mode 
RCC_APB1LPENR_WWDGLPEN              EQU    (0x00000800)        ; Window Watchdog clock enabled in sleep mode 
RCC_APB1LPENR_SPI2LPEN              EQU    (0x00004000)        ; SPI 2 clock enabled in sleep mode 
RCC_APB1LPENR_USART2LPEN            EQU    (0x00020000)        ; USART 2 clock enabled in sleep mode 
RCC_APB1LPENR_USART3LPEN            EQU    (0x00040000)        ; USART 3 clock enabled in sleep mode 
RCC_APB1LPENR_I2C1LPEN              EQU    (0x00200000)        ; I2C 1 clock enabled in sleep mode 
RCC_APB1LPENR_I2C2LPEN              EQU    (0x00400000)        ; I2C 2 clock enabled in sleep mode 
RCC_APB1LPENR_USBLPEN               EQU    (0x00800000)        ; USB clock enabled in sleep mode 
RCC_APB1LPENR_PWRLPEN               EQU    (0x10000000)        ; Power interface clock enabled in sleep mode 
RCC_APB1LPENR_DACLPEN               EQU    (0x20000000)        ; DAC interface clock enabled in sleep mode 
RCC_APB1LPENR_COMPLPEN              EQU    (0x80000000)        ; Comparator interface clock enabled in sleep mode

;**************  Bit definition for RCC_CSR register  *******************
RCC_CSR_LSION                      EQU    (0x00000001)        ; Internal Low Speed oscillator enable 
RCC_CSR_LSIRDY                     EQU    (0x00000002)        ; Internal Low Speed oscillator Ready 

RCC_CSR_LSEON                      EQU    (0x00000100)        ; External Low Speed oscillator enable 
RCC_CSR_LSERDY                     EQU    (0x00000200)        ; External Low Speed oscillator Ready 
RCC_CSR_LSEBYP                     EQU    (0x00000400)        ; External Low Speed oscillator Bypass 

RCC_CSR_RTCSEL                     EQU    (0x00030000)        ; RTCSEL[1:0] bits (RTC clock source selection) 
RCC_CSR_RTCSEL_0                   EQU    (0x00010000)        ; Bit 0 
RCC_CSR_RTCSEL_1                   EQU    (0x00020000)        ; Bit 1 

; RTC congiguration 
RCC_CSR_RTCSEL_NOCLOCK             EQU    (0x00000000)        ; No clock 
RCC_CSR_RTCSEL_LSE                 EQU    (0x00010000)        ; LSE oscillator clock used as RTC clock 
RCC_CSR_RTCSEL_LSI                 EQU    (0x00020000)        ; LSI oscillator clock used as RTC clock 
RCC_CSR_RTCSEL_HSE                 EQU    (0x00030000)        ; HSE oscillator clock divided by 2, 4, 8 or 16 by RTCPRE used as RTC clock 

RCC_CSR_RTCEN                      EQU    (0x00400000)        ; RTC clock enable 
RCC_CSR_RTCRST                     EQU    (0x00800000)        ; RTC reset
 
RCC_CSR_RMVF                       EQU    (0x01000000)        ; Remove reset flag 
RCC_CSR_OBLRSTF                    EQU    (0x02000000)        ; Option Bytes Loader reset flag 
RCC_CSR_PINRSTF                    EQU    (0x04000000)        ; PIN reset flag 
RCC_CSR_PORRSTF                    EQU    (0x08000000)        ; POR/PDR reset flag 
RCC_CSR_SFTRSTF                    EQU    (0x10000000)        ; Software Reset flag 
RCC_CSR_IWDGRSTF                   EQU    (0x20000000)        ; Independent Watchdog reset flag 
RCC_CSR_WWDGRSTF                   EQU    (0x40000000)        ; Window watchdog reset flag 
RCC_CSR_LPWRRSTF                   EQU    (0x80000000)        ; Low-Power reset flag 


;***************************************************************************
;                                                                           
;                            Real-Time Clock (RTC)                          
;                                                                           
;***************************************************************************
;**;***************  Bits definition for RTC_TR register  ******************
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

;**;***************  Bits definition for RTC_DR register  ******************
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

;**;***************  Bits definition for RTC_CR register  ******************
RTC_CR_COE                           EQU    (0x00800000)
RTC_CR_OSEL                          EQU    (0x00600000)
RTC_CR_OSEL_0                        EQU    (0x00200000)
RTC_CR_OSEL_1                        EQU    (0x00400000)
RTC_CR_POL                           EQU    (0x00100000)
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
RTC_CR_REFCKON                       EQU    (0x00000010)
RTC_CR_TSEDGE                        EQU    (0x00000008)
RTC_CR_WUCKSEL                       EQU    (0x00000007)
RTC_CR_WUCKSEL_0                     EQU    (0x00000001)
RTC_CR_WUCKSEL_1                     EQU    (0x00000002)
RTC_CR_WUCKSEL_2                     EQU    (0x00000004)

;**;***************  Bits definition for RTC_ISR register  *****************
RTC_ISR_TAMP1F                       EQU    (0x00002000)
RTC_ISR_TSOVF                        EQU    (0x00001000)
RTC_ISR_TSF                          EQU    (0x00000800)
RTC_ISR_WUTF                         EQU    (0x00000400)
RTC_ISR_ALRBF                        EQU    (0x00000200)
RTC_ISR_ALRAF                        EQU    (0x00000100)
RTC_ISR_INIT                         EQU    (0x00000080)
RTC_ISR_INITF                        EQU    (0x00000040)
RTC_ISR_RSF                          EQU    (0x00000020)
RTC_ISR_INITS                        EQU    (0x00000010)
RTC_ISR_WUTWF                        EQU    (0x00000004)
RTC_ISR_ALRBWF                       EQU    (0x00000002)
RTC_ISR_ALRAWF                       EQU    (0x00000001)

;**;***************  Bits definition for RTC_PRER register  ****************
RTC_PRER_PREDIV_A                    EQU    (0x007F0000)
RTC_PRER_PREDIV_S                    EQU    (0x00001FFF)

;**;***************  Bits definition for RTC_WUTR register  ****************
RTC_WUTR_WUT                         EQU    (0x0000FFFF)

;**;***************  Bits definition for RTC_CALIBR register  **************
RTC_CALIBR_DCS                       EQU    (0x00000080)
RTC_CALIBR_DC                        EQU    (0x0000001F)

;**;***************  Bits definition for RTC_ALRMAR register  **************
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

;**;***************  Bits definition for RTC_ALRMBR register  **************
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

;**;***************  Bits definition for RTC_WPR register  *****************
RTC_WPR_KEY                          EQU    (0x000000FF)

;**;***************  Bits definition for RTC_TSTR register  ****************
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

;**;***************  Bits definition for RTC_TSDR register  ****************
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

;**;***************  Bits definition for RTC_TAFCR register  ***************
RTC_TAFCR_ALARMOUTTYPE               EQU    (0x00040000)
RTC_TAFCR_TAMPIE                     EQU    (0x00000004)
RTC_TAFCR_TAMP1TRG                   EQU    (0x00000002)
RTC_TAFCR_TAMP1E                     EQU    (0x00000001)

;**;***************  Bits definition for RTC_BKP0R register  ***************
;RTC_BKP0R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP1R register  ***************
;RTC_BKP1R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP2R register  ***************
;RTC_BKP2R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP3R register  ***************
;RTC_BKP3R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP4R register  ***************
;RTC_BKP4R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP5R register  ***************
;RTC_BKP5R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP6R register  ***************
;RTC_BKP6R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP7R register  ***************
;RTC_BKP7R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP8R register  ***************
;RTC_BKP8R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP9R register  ***************
;RTC_BKP9R                            EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP10R register  **************
;RTC_BKP10R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP11R register  **************
;RTC_BKP11R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP12R register  **************
;RTC_BKP12R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP13R register  **************
;RTC_BKP13R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP14R register  **************
;RTC_BKP14R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP15R register  **************
;RTC_BKP15R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP16R register  **************
;RTC_BKP16R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP17R register  **************
;RTC_BKP17R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP18R register  **************
;RTC_BKP18R                           EQU    (0xFFFFFFFF)

;**;***************  Bits definition for RTC_BKP19R register  **************
;RTC_BKP19R                           EQU    (0xFFFFFFFF)


;***************************************************************************
;                                                                           
;                      Serial Peripheral Interface (SPI)                    
;                                                                           
;***************************************************************************

;**************  Bit definition for SPI_CR1 register  *******************
SPI_CR1_CPHA                        EQU    (0x0001)            ; Clock Phase 
SPI_CR1_CPOL                        EQU    (0x0002)            ; Clock Polarity 
SPI_CR1_MSTR                        EQU    (0x0004)            ; Master Selection 

SPI_CR1_BR                          EQU    (0x0038)            ; BR[2:0] bits (Baud Rate Control) 
SPI_CR1_BR_0                        EQU    (0x0008)            ; Bit 0 
SPI_CR1_BR_1                        EQU    (0x0010)            ; Bit 1 
SPI_CR1_BR_2                        EQU    (0x0020)            ; Bit 2 

SPI_CR1_SPE                         EQU    (0x0040)            ; SPI Enable 
SPI_CR1_LSBFIRST                    EQU    (0x0080)            ; Frame Format 
SPI_CR1_SSI                         EQU    (0x0100)            ; Internal slave select 
SPI_CR1_SSM                         EQU    (0x0200)            ; Software slave management 
SPI_CR1_RXONLY                      EQU    (0x0400)            ; Receive only 
SPI_CR1_DFF                         EQU    (0x0800)            ; Data Frame Format 
SPI_CR1_CRCNEXT                     EQU    (0x1000)            ; Transmit CRC next 
SPI_CR1_CRCEN                       EQU    (0x2000)            ; Hardware CRC calculation enable 
SPI_CR1_BIDIOE                      EQU    (0x4000)            ; Output enable in bidirectional mode 
SPI_CR1_BIDIMODE                    EQU    (0x8000)            ; Bidirectional data mode enable 

;**************  Bit definition for SPI_CR2 register  *******************
SPI_CR2_RXDMAEN                     EQU    (0x01)               ; Rx Buffer DMA Enable 
SPI_CR2_TXDMAEN                     EQU    (0x02)               ; Tx Buffer DMA Enable 
SPI_CR2_SSOE                        EQU    (0x04)               ; SS Output Enable 
SPI_CR2_ERRIE                       EQU    (0x20)               ; Error Interrupt Enable 
SPI_CR2_RXNEIE                      EQU    (0x40)               ; RX buffer Not Empty Interrupt Enable 
SPI_CR2_TXEIE                       EQU    (0x80)               ; Tx buffer Empty Interrupt Enable 

;**************  Bit definition for SPI_SR register  *******************
SPI_SR_RXNE                         EQU    (0x01)               ; Receive buffer Not Empty 
SPI_SR_TXE                          EQU    (0x02)               ; Transmit buffer Empty 
SPI_SR_CRCERR                       EQU    (0x10)               ; CRC Error flag 
SPI_SR_MODF                         EQU    (0x20)               ; Mode fault 
SPI_SR_OVR                          EQU    (0x40)               ; Overrun flag 
SPI_SR_BSY                          EQU    (0x80)               ; Busy flag 

;**************  Bit definition for SPI_DR register  *******************
SPI_DR_DR                           EQU    (0xFFFF)            ; Data Register 

;**************  Bit definition for SPI_CRCPR register  *****************
SPI_CRCPR_CRCPOLY                   EQU    (0xFFFF)            ; CRC polynomial register 

;**************  Bit definition for SPI_RXCRCR register  *****************
SPI_RXCRCR_RXCRC                    EQU    (0xFFFF)            ; Rx CRC Register 

;**************  Bit definition for SPI_TXCRCR register  *****************
SPI_TXCRCR_TXCRC                    EQU    (0xFFFF)            ; Tx CRC Register 

;***************************************************************************
;                                                                           
;                        System Configuration (SYSCFG)                      
;                                                                           
;***************************************************************************
;**************  Bit definition for SYSCFG_MEMRMP register  ***************
SYSCFG_MEMRMP_MEM_MODE          EQU    (0x00000003) ; SYSCFG_Memory Remap Config 
SYSCFG_MEMRMP_MEM_MODE_0        EQU    (0x00000001) ; Bit 0 
SYSCFG_MEMRMP_MEM_MODE_1        EQU    (0x00000002) ; Bit 1 

;**************  Bit definition for SYSCFG_PMC register  ******************
SYSCFG_PMC_USB_PU               EQU    (0x00000001) ; SYSCFG PMC 

;**************  Bit definition for SYSCFG_EXTICR1 register  **************
SYSCFG_EXTICR1_EXTI0            EQU    (0x000F) ; EXTI 0 configuration 
SYSCFG_EXTICR1_EXTI1            EQU    (0x00F0) ; EXTI 1 configuration 
SYSCFG_EXTICR1_EXTI2            EQU    (0x0F00) ; EXTI 2 configuration 
SYSCFG_EXTICR1_EXTI3            EQU    (0xF000) ; EXTI 3 configuration 

 
;  EXTI0 configuration  
 
SYSCFG_EXTICR1_EXTI0_PA         EQU    (0x0000) ; PA[0] pin 
SYSCFG_EXTICR1_EXTI0_PB         EQU    (0x0001) ; PB[0] pin 
SYSCFG_EXTICR1_EXTI0_PC         EQU    (0x0002) ; PC[0] pin 
SYSCFG_EXTICR1_EXTI0_PD         EQU    (0x0003) ; PD[0] pin 
SYSCFG_EXTICR1_EXTI0_PE         EQU    (0x0004) ; PE[0] pin 
SYSCFG_EXTICR1_EXTI0_PH         EQU    (0x0005) ; PH[0] pin 

 
;  EXTI1 configuration  
 
SYSCFG_EXTICR1_EXTI1_PA         EQU    (0x0000) ; PA[1] pin 
SYSCFG_EXTICR1_EXTI1_PB         EQU    (0x0010) ; PB[1] pin 
SYSCFG_EXTICR1_EXTI1_PC         EQU    (0x0020) ; PC[1] pin 
SYSCFG_EXTICR1_EXTI1_PD         EQU    (0x0030) ; PD[1] pin 
SYSCFG_EXTICR1_EXTI1_PE         EQU    (0x0040) ; PE[1] pin 
SYSCFG_EXTICR1_EXTI1_PH         EQU    (0x0050) ; PH[1] pin 

 
;  EXTI2 configuration  
 
SYSCFG_EXTICR1_EXTI2_PA         EQU    (0x0000) ; PA[2] pin 
SYSCFG_EXTICR1_EXTI2_PB         EQU    (0x0100) ; PB[2] pin 
SYSCFG_EXTICR1_EXTI2_PC         EQU    (0x0200) ; PC[2] pin 
SYSCFG_EXTICR1_EXTI2_PD         EQU    (0x0300) ; PD[2] pin 
SYSCFG_EXTICR1_EXTI2_PE         EQU    (0x0400) ; PE[2] pin 
SYSCFG_EXTICR1_EXTI2_PH         EQU    (0x0500) ; PH[2] pin 

 
;  EXTI3 configuration  
 
SYSCFG_EXTICR1_EXTI3_PA         EQU    (0x0000) ; PA[3] pin 
SYSCFG_EXTICR1_EXTI3_PB         EQU    (0x1000) ; PB[3] pin 
SYSCFG_EXTICR1_EXTI3_PC         EQU    (0x2000) ; PC[3] pin 
SYSCFG_EXTICR1_EXTI3_PD         EQU    (0x3000) ; PD[3] pin 
SYSCFG_EXTICR1_EXTI3_PE         EQU    (0x4000) ; PE[3] pin 

;**************  Bit definition for SYSCFG_EXTICR2 register  ****************
SYSCFG_EXTICR2_EXTI4            EQU    (0x000F) ; EXTI 4 configuration 
SYSCFG_EXTICR2_EXTI5            EQU    (0x00F0) ; EXTI 5 configuration 
SYSCFG_EXTICR2_EXTI6            EQU    (0x0F00) ; EXTI 6 configuration 
SYSCFG_EXTICR2_EXTI7            EQU    (0xF000) ; EXTI 7 configuration 

 
;  EXTI4 configuration  
 
SYSCFG_EXTICR2_EXTI4_PA         EQU    (0x0000) ; PA[4] pin 
SYSCFG_EXTICR2_EXTI4_PB         EQU    (0x0001) ; PB[4] pin 
SYSCFG_EXTICR2_EXTI4_PC         EQU    (0x0002) ; PC[4] pin 
SYSCFG_EXTICR2_EXTI4_PD         EQU    (0x0003) ; PD[4] pin 
SYSCFG_EXTICR2_EXTI4_PE         EQU    (0x0004) ; PE[4] pin 

 
;  EXTI5 configuration  
 
SYSCFG_EXTICR2_EXTI5_PA         EQU    (0x0000) ; PA[5] pin 
SYSCFG_EXTICR2_EXTI5_PB         EQU    (0x0010) ; PB[5] pin 
SYSCFG_EXTICR2_EXTI5_PC         EQU    (0x0020) ; PC[5] pin 
SYSCFG_EXTICR2_EXTI5_PD         EQU    (0x0030) ; PD[5] pin 
SYSCFG_EXTICR2_EXTI5_PE         EQU    (0x0040) ; PE[5] pin 

 
;  EXTI6 configuration  
 
SYSCFG_EXTICR2_EXTI6_PA         EQU    (0x0000) ; PA[6] pin 
SYSCFG_EXTICR2_EXTI6_PB         EQU    (0x0100) ; PB[6] pin 
SYSCFG_EXTICR2_EXTI6_PC         EQU    (0x0200) ; PC[6] pin 
SYSCFG_EXTICR2_EXTI6_PD         EQU    (0x0300) ; PD[6] pin 
SYSCFG_EXTICR2_EXTI6_PE         EQU    (0x0400) ; PE[6] pin 

 
;  EXTI7 configuration  
 
SYSCFG_EXTICR2_EXTI7_PA         EQU    (0x0000) ; PA[7] pin 
SYSCFG_EXTICR2_EXTI7_PB         EQU    (0x1000) ; PB[7] pin 
SYSCFG_EXTICR2_EXTI7_PC         EQU    (0x2000) ; PC[7] pin 
SYSCFG_EXTICR2_EXTI7_PD         EQU    (0x3000) ; PD[7] pin 
SYSCFG_EXTICR2_EXTI7_PE         EQU    (0x4000) ; PE[7] pin 

;**************  Bit definition for SYSCFG_EXTICR3 register  ****************
SYSCFG_EXTICR3_EXTI8            EQU    (0x000F) ; EXTI 8 configuration 
SYSCFG_EXTICR3_EXTI9            EQU    (0x00F0) ; EXTI 9 configuration 
SYSCFG_EXTICR3_EXTI10           EQU    (0x0F00) ; EXTI 10 configuration 
SYSCFG_EXTICR3_EXTI11           EQU    (0xF000) ; EXTI 11 configuration 
                                      
 
;  EXTI8 configuration  
 
SYSCFG_EXTICR3_EXTI8_PA         EQU    (0x0000) ; PA[8] pin 
SYSCFG_EXTICR3_EXTI8_PB         EQU    (0x0001) ; PB[8] pin 
SYSCFG_EXTICR3_EXTI8_PC         EQU    (0x0002) ; PC[8] pin 
SYSCFG_EXTICR3_EXTI8_PD         EQU    (0x0003) ; PD[8] pin 
SYSCFG_EXTICR3_EXTI8_PE         EQU    (0x0004) ; PE[8] pin 

 
;  EXTI9 configuration  
 
SYSCFG_EXTICR3_EXTI9_PA         EQU    (0x0000) ; PA[9] pin 
SYSCFG_EXTICR3_EXTI9_PB         EQU    (0x0010) ; PB[9] pin 
SYSCFG_EXTICR3_EXTI9_PC         EQU    (0x0020) ; PC[9] pin 
SYSCFG_EXTICR3_EXTI9_PD         EQU    (0x0030) ; PD[9] pin 
SYSCFG_EXTICR3_EXTI9_PE         EQU    (0x0040) ; PE[9] pin 

 
;  EXTI10 configuration  
 
SYSCFG_EXTICR3_EXTI10_PA        EQU    (0x0000) ; PA[10] pin 
SYSCFG_EXTICR3_EXTI10_PB        EQU    (0x0100) ; PB[10] pin 
SYSCFG_EXTICR3_EXTI10_PC        EQU    (0x0200) ; PC[10] pin 
SYSCFG_EXTICR3_EXTI10_PD        EQU    (0x0300) ; PD[10] pin 
SYSCFG_EXTICR3_EXTI10_PE        EQU    (0x0400) ; PE[10] pin 

 
;  EXTI11 configuration  
 
SYSCFG_EXTICR3_EXTI11_PA        EQU    (0x0000) ; PA[11] pin 
SYSCFG_EXTICR3_EXTI11_PB        EQU    (0x1000) ; PB[11] pin 
SYSCFG_EXTICR3_EXTI11_PC        EQU    (0x2000) ; PC[11] pin 
SYSCFG_EXTICR3_EXTI11_PD        EQU    (0x3000) ; PD[11] pin 
SYSCFG_EXTICR3_EXTI11_PE        EQU    (0x4000) ; PE[11] pin 

;**************  Bit definition for SYSCFG_EXTICR4 register  ****************
SYSCFG_EXTICR4_EXTI12           EQU    (0x000F) ; EXTI 12 configuration 
SYSCFG_EXTICR4_EXTI13           EQU    (0x00F0) ; EXTI 13 configuration 
SYSCFG_EXTICR4_EXTI14           EQU    (0x0F00) ; EXTI 14 configuration 
SYSCFG_EXTICR4_EXTI15           EQU    (0xF000) ; EXTI 15 configuration 

 
;  EXTI12 configuration  
 
SYSCFG_EXTICR4_EXTI12_PA        EQU    (0x0000) ; PA[12] pin 
SYSCFG_EXTICR4_EXTI12_PB        EQU    (0x0001) ; PB[12] pin 
SYSCFG_EXTICR4_EXTI12_PC        EQU    (0x0002) ; PC[12] pin 
SYSCFG_EXTICR4_EXTI12_PD        EQU    (0x0003) ; PD[12] pin 
SYSCFG_EXTICR4_EXTI12_PE        EQU    (0x0004) ; PE[12] pin 

 
;  EXTI13 configuration  
 
SYSCFG_EXTICR4_EXTI13_PA        EQU    (0x0000) ; PA[13] pin 
SYSCFG_EXTICR4_EXTI13_PB        EQU    (0x0010) ; PB[13] pin 
SYSCFG_EXTICR4_EXTI13_PC        EQU    (0x0020) ; PC[13] pin 
SYSCFG_EXTICR4_EXTI13_PD        EQU    (0x0030) ; PD[13] pin 
SYSCFG_EXTICR4_EXTI13_PE        EQU    (0x0040) ; PE[13] pin 

 
;  EXTI14 configuration  
 
SYSCFG_EXTICR4_EXTI14_PA        EQU    (0x0000) ; PA[14] pin 
SYSCFG_EXTICR4_EXTI14_PB        EQU    (0x0100) ; PB[14] pin 
SYSCFG_EXTICR4_EXTI14_PC        EQU    (0x0200) ; PC[14] pin 
SYSCFG_EXTICR4_EXTI14_PD        EQU    (0x0300) ; PD[14] pin 
SYSCFG_EXTICR4_EXTI14_PE        EQU    (0x0400) ; PE[14] pin 

 
;  EXTI15 configuration  
 
SYSCFG_EXTICR4_EXTI15_PA        EQU    (0x0000) ; PA[15] pin 
SYSCFG_EXTICR4_EXTI15_PB        EQU    (0x1000) ; PB[15] pin 
SYSCFG_EXTICR4_EXTI15_PC        EQU    (0x2000) ; PC[15] pin 
SYSCFG_EXTICR4_EXTI15_PD        EQU    (0x3000) ; PD[15] pin 
SYSCFG_EXTICR4_EXTI15_PE        EQU    (0x4000) ; PE[15] pin 
 
 
;***************************************************************************
;                                                                           
;                        Routing Interface (RI)                             
;                                                                           
;***************************************************************************

;**************  Bit definition for RI_ICR register  *******************
RI_ICR_IC1Z                    EQU    (0x0000000F) ; IC1Z[3:0] bits (Input Capture 1 select bits) 
RI_ICR_IC1Z_0                  EQU    (0x00000001) ; Bit 0 
RI_ICR_IC1Z_1                  EQU    (0x00000002) ; Bit 1 
RI_ICR_IC1Z_2                  EQU    (0x00000004) ; Bit 2 
RI_ICR_IC1Z_3                  EQU    (0x00000008) ; Bit 3 

RI_ICR_IC2Z                    EQU    (0x000000F0) ; IC2Z[3:0] bits (Input Capture 2 select bits) 
RI_ICR_IC2Z_0                  EQU    (0x00000010) ; Bit 0 
RI_ICR_IC2Z_1                  EQU    (0x00000020) ; Bit 1 
RI_ICR_IC2Z_2                  EQU    (0x00000040) ; Bit 2 
RI_ICR_IC2Z_3                  EQU    (0x00000080) ; Bit 3 

RI_ICR_IC3Z                    EQU    (0x00000F00) ; IC3Z[3:0] bits (Input Capture 3 select bits) 
RI_ICR_IC3Z_0                  EQU    (0x00000100) ; Bit 0 
RI_ICR_IC3Z_1                  EQU    (0x00000200) ; Bit 1 
RI_ICR_IC3Z_2                  EQU    (0x00000400) ; Bit 2 
RI_ICR_IC3Z_3                  EQU    (0x00000800) ; Bit 3 

RI_ICR_IC4Z                    EQU    (0x0000F000) ; IC4Z[3:0] bits (Input Capture 4 select bits) 
RI_ICR_IC4Z_0                  EQU    (0x00001000) ; Bit 0 
RI_ICR_IC4Z_1                  EQU    (0x00002000) ; Bit 1 
RI_ICR_IC4Z_2                  EQU    (0x00004000) ; Bit 2 
RI_ICR_IC4Z_3                  EQU    (0x00008000) ; Bit 3 

RI_ICR_TIM                     EQU    (0x00030000) ; TIM[3:0] bits (Timers select bits) 
RI_ICR_TIM_0                   EQU    (0x00010000) ; Bit 0 
RI_ICR_TIM_1                   EQU    (0x00020000) ; Bit 1 

RI_ICR_IC1                     EQU    (0x00040000) ; Input capture 1 
RI_ICR_IC2                     EQU    (0x00080000) ; Input capture 2 
RI_ICR_IC3                     EQU    (0x00100000) ; Input capture 3 
RI_ICR_IC4                     EQU    (0x00200000) ; Input capture 4 

;**************  Bit definition for RI_ASCR1 register  *******************
RI_ASCR1_CH                    EQU    (0x03FCFFFF) ; AS_CH[25:18] & AS_CH[15:0] bits ( Analog switches selection bits) 
RI_ASCR1_CH_0                  EQU    (0x00000001) ; Bit 0 
RI_ASCR1_CH_1                  EQU    (0x00000002) ; Bit 1 
RI_ASCR1_CH_2                  EQU    (0x00000004) ; Bit 2 
RI_ASCR1_CH_3                  EQU    (0x00000008) ; Bit 3 
RI_ASCR1_CH_4                  EQU    (0x00000010) ; Bit 4 
RI_ASCR1_CH_5                  EQU    (0x00000020) ; Bit 5 
RI_ASCR1_CH_6                  EQU    (0x00000040) ; Bit 6 
RI_ASCR1_CH_7                  EQU    (0x00000080) ; Bit 7 
RI_ASCR1_CH_8                  EQU    (0x00000100) ; Bit 8 
RI_ASCR1_CH_9                  EQU    (0x00000200) ; Bit 9 
RI_ASCR1_CH_10                 EQU    (0x00000400) ; Bit 10 
RI_ASCR1_CH_11                 EQU    (0x00000800) ; Bit 11 
RI_ASCR1_CH_12                 EQU    (0x00001000) ; Bit 12 
RI_ASCR1_CH_13                 EQU    (0x00002000) ; Bit 13 
RI_ASCR1_CH_14                 EQU    (0x00004000) ; Bit 14 
RI_ASCR1_CH_15                 EQU    (0x00008000) ; Bit 15 
RI_ASCR1_CH_18                 EQU    (0x00040000) ; Bit 18 
RI_ASCR1_CH_19                 EQU    (0x00080000) ; Bit 19 
RI_ASCR1_CH_20                 EQU    (0x00100000) ; Bit 20 
RI_ASCR1_CH_21                 EQU    (0x00200000) ; Bit 21 
RI_ASCR1_CH_22                 EQU    (0x00400000) ; Bit 22 
RI_ASCR1_CH_23                 EQU    (0x00800000) ; Bit 23 
RI_ASCR1_CH_24                 EQU    (0x01000000) ; Bit 24 
RI_ASCR1_CH_25                 EQU    (0x02000000) ; Bit 25 

RI_ASCR1_VCOMP                 EQU    (0x04000000) ; ADC analog switch selection for internal node to COMP1 
RI_ASCR1_SCM                   EQU    (0x80000000) ; I/O Switch control mode 

;**************  Bit definition for RI_ASCR2 register  *******************
RI_ASCR2_GR10_1                EQU    (0x00000001) ; GR10-1 selection bit 
RI_ASCR2_GR10_2                EQU    (0x00000002) ; GR10-2 selection bit 
RI_ASCR2_GR10_3                EQU    (0x00000004) ; GR10-3 selection bit 
RI_ASCR2_GR10_4                EQU    (0x00000008) ; GR10-4 selection bit 
RI_ASCR2_GR6_1                 EQU    (0x00000010) ; GR6-1 selection bit 
RI_ASCR2_GR6_2                 EQU    (0x00000020) ; GR6-2 selection bit 
RI_ASCR2_GR5_1                 EQU    (0x00000040) ; GR5-1 selection bit 
RI_ASCR2_GR5_2                 EQU    (0x00000080) ; GR5-2 selection bit 
RI_ASCR2_GR5_3                 EQU    (0x00000100) ; GR5-3 selection bit 
RI_ASCR2_GR4_1                 EQU    (0x00000200) ; GR4-1 selection bit 
RI_ASCR2_GR4_2                 EQU    (0x00000400) ; GR4-2 selection bit 
RI_ASCR2_GR4_3                 EQU    (0x00000800) ; GR4-3 selection bit 


;**************  Bit definition for RI_HYSCR1 register  *******************
RI_HYSCR1_PA                   EQU    (0x0000FFFF) ; PA[15:0] Port A Hysteresis selection 
RI_HYSCR1_PA_0                 EQU    (0x00000001) ; Bit 0 
RI_HYSCR1_PA_1                 EQU    (0x00000002) ; Bit 1 
RI_HYSCR1_PA_2                 EQU    (0x00000004) ; Bit 2 
RI_HYSCR1_PA_3                 EQU    (0x00000008) ; Bit 3 
RI_HYSCR1_PA_4                 EQU    (0x00000010) ; Bit 4 
RI_HYSCR1_PA_5                 EQU    (0x00000020) ; Bit 5 
RI_HYSCR1_PA_6                 EQU    (0x00000040) ; Bit 6 
RI_HYSCR1_PA_7                 EQU    (0x00000080) ; Bit 7 
RI_HYSCR1_PA_8                 EQU    (0x00000100) ; Bit 8 
RI_HYSCR1_PA_9                 EQU    (0x00000200) ; Bit 9 
RI_HYSCR1_PA_10                EQU    (0x00000400) ; Bit 10 
RI_HYSCR1_PA_11                EQU    (0x00000800) ; Bit 11 
RI_HYSCR1_PA_12                EQU    (0x00001000) ; Bit 12 
RI_HYSCR1_PA_13                EQU    (0x00002000) ; Bit 13 
RI_HYSCR1_PA_14                EQU    (0x00004000) ; Bit 14 
RI_HYSCR1_PA_15                EQU    (0x00008000) ; Bit 15 

RI_HYSCR1_PB                   EQU    (0xFFFF0000) ; PB[15:0] Port B Hysteresis selection 
RI_HYSCR1_PB_0                 EQU    (0x00010000) ; Bit 0 
RI_HYSCR1_PB_1                 EQU    (0x00020000) ; Bit 1 
RI_HYSCR1_PB_2                 EQU    (0x00040000) ; Bit 2 
RI_HYSCR1_PB_3                 EQU    (0x00080000) ; Bit 3 
RI_HYSCR1_PB_4                 EQU    (0x00100000) ; Bit 4 
RI_HYSCR1_PB_5                 EQU    (0x00200000) ; Bit 5 
RI_HYSCR1_PB_6                 EQU    (0x00400000) ; Bit 6 
RI_HYSCR1_PB_7                 EQU    (0x00800000) ; Bit 7 
RI_HYSCR1_PB_8                 EQU    (0x01000000) ; Bit 8 
RI_HYSCR1_PB_9                 EQU    (0x02000000) ; Bit 9 
RI_HYSCR1_PB_10                EQU    (0x04000000) ; Bit 10 
RI_HYSCR1_PB_11                EQU    (0x08000000) ; Bit 11 
RI_HYSCR1_PB_12                EQU    (0x10000000) ; Bit 12 
RI_HYSCR1_PB_13                EQU    (0x20000000) ; Bit 13 
RI_HYSCR1_PB_14                EQU    (0x40000000) ; Bit 14 
RI_HYSCR1_PB_15                EQU    (0x80000000) ; Bit 15 

;**************  Bit definition for RI_HYSCR2 register  *******************
RI_HYSCR2_PC                   EQU    (0x0000FFFF) ; PC[15:0] Port C Hysteresis selection 
RI_HYSCR2_PC_0                 EQU    (0x00000001) ; Bit 0 
RI_HYSCR2_PC_1                 EQU    (0x00000002) ; Bit 1 
RI_HYSCR2_PC_2                 EQU    (0x00000004) ; Bit 2 
RI_HYSCR2_PC_3                 EQU    (0x00000008) ; Bit 3 
RI_HYSCR2_PC_4                 EQU    (0x00000010) ; Bit 4 
RI_HYSCR2_PC_5                 EQU    (0x00000020) ; Bit 5 
RI_HYSCR2_PC_6                 EQU    (0x00000040) ; Bit 6 
RI_HYSCR2_PC_7                 EQU    (0x00000080) ; Bit 7 
RI_HYSCR2_PC_8                 EQU    (0x00000100) ; Bit 8 
RI_HYSCR2_PC_9                 EQU    (0x00000200) ; Bit 9 
RI_HYSCR2_PC_10                EQU    (0x00000400) ; Bit 10 
RI_HYSCR2_PC_11                EQU    (0x00000800) ; Bit 11 
RI_HYSCR2_PC_12                EQU    (0x00001000) ; Bit 12 
RI_HYSCR2_PC_13                EQU    (0x00002000) ; Bit 13 
RI_HYSCR2_PC_14                EQU    (0x00004000) ; Bit 14 
RI_HYSCR2_PC_15                EQU    (0x00008000) ; Bit 15 

RI_HYSCR2_PD                   EQU    (0xFFFF0000) ; PD[15:0] Port D Hysteresis selection 
RI_HYSCR2_PD_0                 EQU    (0x00010000) ; Bit 0 
RI_HYSCR2_PD_1                 EQU    (0x00020000) ; Bit 1 
RI_HYSCR2_PD_2                 EQU    (0x00040000) ; Bit 2 
RI_HYSCR2_PD_3                 EQU    (0x00080000) ; Bit 3 
RI_HYSCR2_PD_4                 EQU    (0x00100000) ; Bit 4 
RI_HYSCR2_PD_5                 EQU    (0x00200000) ; Bit 5 
RI_HYSCR2_PD_6                 EQU    (0x00400000) ; Bit 6 
RI_HYSCR2_PD_7                 EQU    (0x00800000) ; Bit 7 
RI_HYSCR2_PD_8                 EQU    (0x01000000) ; Bit 8 
RI_HYSCR2_PD_9                 EQU    (0x02000000) ; Bit 9 
RI_HYSCR2_PD_10                EQU    (0x04000000) ; Bit 10 
RI_HYSCR2_PD_11                EQU    (0x08000000) ; Bit 11 
RI_HYSCR2_PD_12                EQU    (0x10000000) ; Bit 12 
RI_HYSCR2_PD_13                EQU    (0x20000000) ; Bit 13 
RI_HYSCR2_PD_14                EQU    (0x40000000) ; Bit 14 
RI_HYSCR2_PD_15                EQU    (0x80000000) ; Bit 15 

;**************  Bit definition for RI_HYSCR3 register  *******************
RI_HYSCR2_PE                   EQU    (0x0000FFFF) ; PE[15:0] Port E Hysteresis selection 
RI_HYSCR2_PE_0                 EQU    (0x00000001) ; Bit 0 
RI_HYSCR2_PE_1                 EQU    (0x00000002) ; Bit 1 
RI_HYSCR2_PE_2                 EQU    (0x00000004) ; Bit 2 
RI_HYSCR2_PE_3                 EQU    (0x00000008) ; Bit 3 
RI_HYSCR2_PE_4                 EQU    (0x00000010) ; Bit 4 
RI_HYSCR2_PE_5                 EQU    (0x00000020) ; Bit 5 
RI_HYSCR2_PE_6                 EQU    (0x00000040) ; Bit 6 
RI_HYSCR2_PE_7                 EQU    (0x00000080) ; Bit 7 
RI_HYSCR2_PE_8                 EQU    (0x00000100) ; Bit 8 
RI_HYSCR2_PE_9                 EQU    (0x00000200) ; Bit 9 
RI_HYSCR2_PE_10                EQU    (0x00000400) ; Bit 10 
RI_HYSCR2_PE_11                EQU    (0x00000800) ; Bit 11 
RI_HYSCR2_PE_12                EQU    (0x00001000) ; Bit 12 
RI_HYSCR2_PE_13                EQU    (0x00002000) ; Bit 13 
RI_HYSCR2_PE_14                EQU    (0x00004000) ; Bit 14 
RI_HYSCR2_PE_15                EQU    (0x00008000) ; Bit 15 


;***************************************************************************
;                                                                           
;                                Timers (TIM)                               
;                                                                           
;***************************************************************************

;**************  Bit definition for TIM_CR1 register  *******************
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

;**************  Bit definition for TIM_CR2 register  *******************
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

;**************  Bit definition for TIM_SMCR register  ******************
TIM_SMCR_SMS                        EQU    (0x0007)            ; SMS[2:0] bits (Slave mode selection) 
TIM_SMCR_SMS_0                      EQU    (0x0001)            ; Bit 0 
TIM_SMCR_SMS_1                      EQU    (0x0002)            ; Bit 1 
TIM_SMCR_SMS_2                      EQU    (0x0004)            ; Bit 2 

TIM_SMCR_OCCS                       EQU    (0x0008)            ; OCCS bits (OCref Clear Selection) 

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

;**************  Bit definition for TIM_DIER register  ******************
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

;**************  Bit definition for TIM_SR register  *******************
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

;**************  Bit definition for TIM_EGR register  *******************
TIM_EGR_UG                          EQU    (0x01)               ; Update Generation 
TIM_EGR_CC1G                        EQU    (0x02)               ; Capture/Compare 1 Generation 
TIM_EGR_CC2G                        EQU    (0x04)               ; Capture/Compare 2 Generation 
TIM_EGR_CC3G                        EQU    (0x08)               ; Capture/Compare 3 Generation 
TIM_EGR_CC4G                        EQU    (0x10)               ; Capture/Compare 4 Generation 
TIM_EGR_COMG                        EQU    (0x20)               ; Capture/Compare Control Update Generation 
TIM_EGR_TG                          EQU    (0x40)               ; Trigger Generation 
TIM_EGR_BG                          EQU    (0x80)               ; Break Generation 

;**************  Bit definition for TIM_CCMR1 register  ******************
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

; ----------------------------------------------------------------------------

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

;**************  Bit definition for TIM_CCMR2 register  ******************
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

; ----------------------------------------------------------------------------

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

;**************  Bit definition for TIM_CCER register  ******************
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

;**************  Bit definition for TIM_CNT register  *******************
TIM_CNT_CNT                         EQU    (0xFFFF)            ; Counter Value 

;**************  Bit definition for TIM_PSC register  *******************
TIM_PSC_PSC                         EQU    (0xFFFF)            ; Prescaler Value 

;**************  Bit definition for TIM_ARR register  *******************
TIM_ARR_ARR                         EQU    (0xFFFF)            ; actual auto-reload Value 

;**************  Bit definition for TIM_RCR register  *******************
TIM_RCR_REP                         EQU    (0xFF)               ; Repetition Counter Value 

;**************  Bit definition for TIM_CCR1 register  ******************
TIM_CCR1_CCR1                       EQU    (0xFFFF)            ; Capture/Compare 1 Value 

;**************  Bit definition for TIM_CCR2 register  ******************
TIM_CCR2_CCR2                       EQU    (0xFFFF)            ; Capture/Compare 2 Value 

;**************  Bit definition for TIM_CCR3 register  ******************
TIM_CCR3_CCR3                       EQU    (0xFFFF)            ; Capture/Compare 3 Value 

;**************  Bit definition for TIM_CCR4 register  ******************
TIM_CCR4_CCR4                       EQU    (0xFFFF)            ; Capture/Compare 4 Value 

;**************  Bit definition for TIM_DCR register  *******************
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

;**************  Bit definition for TIM_DMAR register  ******************
TIM_DMAR_DMAB                       EQU    (0xFFFF)            ; DMA register for burst accesses 

;**************  Bit definition for TIM_OR register  ********************
TIM_OR_TI1RMP                       EQU    (0x0003)            ; Option register for TI1 Remapping 
TIM_OR_TI1RMP_0                     EQU    (0x0001)            ; Bit 0 
TIM_OR_TI1RMP_1                     EQU    (0x0002)            ; Bit 1 



;***************************************************************************
;                                                                           
;       Universal Synchronous Asynchronous Receiver Transmitter (USART)     
;                                                                           
;***************************************************************************

;**************  Bit definition for USART_SR register  ******************
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

;**************  Bit definition for USART_DR register  ******************
USART_DR_DR                         EQU    (0x01FF)            ; Data value 

;**************  Bit definition for USART_BRR register  ******************
USART_BRR_DIV_FRACTION              EQU    (0x000F)            ; Fraction of USARTDIV 
USART_BRR_DIV_MANTISSA              EQU    (0xFFF0)            ; Mantissa of USARTDIV 

;**************  Bit definition for USART_CR1 register  ******************
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
USART_CR1_OVER8                     EQU    (0x8000)            ; Oversampling by 8-bit mode 

;**************  Bit definition for USART_CR2 register  ******************
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

;**************  Bit definition for USART_CR3 register  ******************
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
USART_CR3_ONEBIT                    EQU    (0x0800)            ; One sample bit method enable 

;**************  Bit definition for USART_GTPR register  *****************
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


;***************************************************************************
;                                                                           
;                      Universal Serial Bus (USB)                           
;                                                                           
;***************************************************************************

; Endpoint-specific registers 
;**************  Bit definition for USB_EP0R register  ******************
USB_EP0R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP0R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP0R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP0R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP0R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP0R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP0R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP0R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP0R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP0R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP0R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP0R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP0R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP0R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP0R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP0R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP1R register  ******************
USB_EP1R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP1R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP1R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP1R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP1R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP1R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP1R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP1R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP1R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP1R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP1R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP1R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP1R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP1R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP1R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP1R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP2R register  ******************
USB_EP2R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP2R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP2R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP2R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP2R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP2R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP2R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP2R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP2R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP2R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP2R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP2R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP2R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP2R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP2R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP2R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP3R register  ******************
USB_EP3R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP3R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP3R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP3R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP3R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP3R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP3R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP3R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP3R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP3R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP3R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP3R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP3R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP3R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP3R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP3R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP4R register  ******************
USB_EP4R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP4R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP4R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP4R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP4R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP4R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP4R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP4R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP4R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP4R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP4R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP4R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP4R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP4R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP4R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP4R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP5R register  ******************
USB_EP5R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP5R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP5R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP5R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP5R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP5R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP5R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP5R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP5R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP5R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP5R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP5R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP5R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP5R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP5R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP5R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP6R register  ******************
USB_EP6R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP6R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP6R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP6R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP6R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP6R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP6R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP6R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP6R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP6R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP6R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP6R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP6R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP6R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP6R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP6R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

;**************  Bit definition for USB_EP7R register  ******************
USB_EP7R_EA                         EQU    (0x000F)            ; Endpoint Address 

USB_EP7R_STAT_TX                    EQU    (0x0030)            ; STAT_TX[1:0] bits (Status bits, for transmission transfers) 
USB_EP7R_STAT_TX_0                  EQU    (0x0010)            ; Bit 0 
USB_EP7R_STAT_TX_1                  EQU    (0x0020)            ; Bit 1 

USB_EP7R_DTOG_TX                    EQU    (0x0040)            ; Data Toggle, for transmission transfers 
USB_EP7R_CTR_TX                     EQU    (0x0080)            ; Correct Transfer for transmission 
USB_EP7R_EP_KIND                    EQU    (0x0100)            ; Endpoint Kind 

USB_EP7R_EP_TYPE                    EQU    (0x0600)            ; EP_TYPE[1:0] bits (Endpoint type) 
USB_EP7R_EP_TYPE_0                  EQU    (0x0200)            ; Bit 0 
USB_EP7R_EP_TYPE_1                  EQU    (0x0400)            ; Bit 1 

USB_EP7R_SETUP                      EQU    (0x0800)            ; Setup transaction completed 

USB_EP7R_STAT_RX                    EQU    (0x3000)            ; STAT_RX[1:0] bits (Status bits, for reception transfers) 
USB_EP7R_STAT_RX_0                  EQU    (0x1000)            ; Bit 0 
USB_EP7R_STAT_RX_1                  EQU    (0x2000)            ; Bit 1 

USB_EP7R_DTOG_RX                    EQU    (0x4000)            ; Data Toggle, for reception transfers 
USB_EP7R_CTR_RX                     EQU    (0x8000)            ; Correct Transfer for reception 

; Common registers 
;**************  Bit definition for USB_CNTR register  ******************
USB_CNTR_FRES                       EQU    (0x0001)            ; Force USB Reset 
USB_CNTR_PDWN                       EQU    (0x0002)            ; Power down 
USB_CNTR_LP_MODE                    EQU    (0x0004)            ; Low-power mode 
USB_CNTR_FSUSP                      EQU    (0x0008)            ; Force suspend 
USB_CNTR_RESUME                     EQU    (0x0010)            ; Resume request 
USB_CNTR_ESOFM                      EQU    (0x0100)            ; Expected Start Of Frame Interrupt Mask 
USB_CNTR_SOFM                       EQU    (0x0200)            ; Start Of Frame Interrupt Mask 
USB_CNTR_RESETM                     EQU    (0x0400)            ; RESET Interrupt Mask 
USB_CNTR_SUSPM                      EQU    (0x0800)            ; Suspend mode Interrupt Mask 
USB_CNTR_WKUPM                      EQU    (0x1000)            ; Wakeup Interrupt Mask 
USB_CNTR_ERRM                       EQU    (0x2000)            ; Error Interrupt Mask 
USB_CNTR_PMAOVRM                    EQU    (0x4000)            ; Packet Memory Area Over / Underrun Interrupt Mask 
USB_CNTR_CTRM                       EQU    (0x8000)            ; Correct Transfer Interrupt Mask 

;**************  Bit definition for USB_ISTR register  ******************
USB_ISTR_EP_ID                      EQU    (0x000F)            ; Endpoint Identifier 
USB_ISTR_DIR                        EQU    (0x0010)            ; Direction of transaction 
USB_ISTR_ESOF                       EQU    (0x0100)            ; Expected Start Of Frame 
USB_ISTR_SOF                        EQU    (0x0200)            ; Start Of Frame 
USB_ISTR_RESET                      EQU    (0x0400)            ; USB RESET request 
USB_ISTR_SUSP                       EQU    (0x0800)            ; Suspend mode request 
USB_ISTR_WKUP                       EQU    (0x1000)            ; Wake up 
USB_ISTR_ERR                        EQU    (0x2000)            ; Error 
USB_ISTR_PMAOVR                     EQU    (0x4000)            ; Packet Memory Area Over / Underrun 
USB_ISTR_CTR                        EQU    (0x8000)            ; Correct Transfer 

;**************  Bit definition for USB_FNR register  *******************
USB_FNR_FN                          EQU    (0x07FF)            ; Frame Number 
USB_FNR_LSOF                        EQU    (0x1800)            ; Lost SOF 
USB_FNR_LCK                         EQU    (0x2000)            ; Locked 
USB_FNR_RXDM                        EQU    (0x4000)            ; Receive Data - Line Status 
USB_FNR_RXDP                        EQU    (0x8000)            ; Receive Data + Line Status 

;**************  Bit definition for USB_DADDR register  ******************
USB_DADDR_ADD                       EQU    (0x7F)               ; ADD[6:0] bits (Device Address) 
USB_DADDR_ADD0                      EQU    (0x01)               ; Bit 0 
USB_DADDR_ADD1                      EQU    (0x02)               ; Bit 1 
USB_DADDR_ADD2                      EQU    (0x04)               ; Bit 2 
USB_DADDR_ADD3                      EQU    (0x08)               ; Bit 3 
USB_DADDR_ADD4                      EQU    (0x10)               ; Bit 4 
USB_DADDR_ADD5                      EQU    (0x20)               ; Bit 5 
USB_DADDR_ADD6                      EQU    (0x40)               ; Bit 6 

USB_DADDR_EF                        EQU    (0x80)               ; Enable Function 

;**************  Bit definition for USB_BTABLE register  *****************    
USB_BTABLE_BTABLE                   EQU    (0xFFF8)            ; Buffer Table 

; Buffer descriptor table 
;**************  Bit definition for USB_ADDR0_TX register  ****************
USB_ADDR0_TX_ADDR0_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 0 

;**************  Bit definition for USB_ADDR1_TX register  ****************
USB_ADDR1_TX_ADDR1_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 1 

;**************  Bit definition for USB_ADDR2_TX register  ****************
USB_ADDR2_TX_ADDR2_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 2 

;**************  Bit definition for USB_ADDR3_TX register  ****************
USB_ADDR3_TX_ADDR3_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 3 

;**************  Bit definition for USB_ADDR4_TX register  ****************
USB_ADDR4_TX_ADDR4_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 4 

;**************  Bit definition for USB_ADDR5_TX register  ****************
USB_ADDR5_TX_ADDR5_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 5 

;**************  Bit definition for USB_ADDR6_TX register  ****************
USB_ADDR6_TX_ADDR6_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 6 

;**************  Bit definition for USB_ADDR7_TX register  ****************
USB_ADDR7_TX_ADDR7_TX               EQU    (0xFFFE)            ; Transmission Buffer Address 7 

; ----------------------------------------------------------------------------

;**************  Bit definition for USB_COUNT0_TX register  ***************
USB_COUNT0_TX_COUNT0_TX             EQU    (0x03FF)            ; Transmission Byte Count 0 

;**************  Bit definition for USB_COUNT1_TX register  ***************
USB_COUNT1_TX_COUNT1_TX             EQU    (0x03FF)            ; Transmission Byte Count 1 

;**************  Bit definition for USB_COUNT2_TX register  ***************
USB_COUNT2_TX_COUNT2_TX             EQU    (0x03FF)            ; Transmission Byte Count 2 

;**************  Bit definition for USB_COUNT3_TX register  ***************
USB_COUNT3_TX_COUNT3_TX             EQU    (0x03FF)            ; Transmission Byte Count 3 

;**************  Bit definition for USB_COUNT4_TX register  ***************
USB_COUNT4_TX_COUNT4_TX             EQU    (0x03FF)            ; Transmission Byte Count 4 

;**************  Bit definition for USB_COUNT5_TX register  ***************
USB_COUNT5_TX_COUNT5_TX             EQU    (0x03FF)            ; Transmission Byte Count 5 

;**************  Bit definition for USB_COUNT6_TX register  ***************
USB_COUNT6_TX_COUNT6_TX             EQU    (0x03FF)            ; Transmission Byte Count 6 

;**************  Bit definition for USB_COUNT7_TX register  ***************
USB_COUNT7_TX_COUNT7_TX             EQU    (0x03FF)            ; Transmission Byte Count 7 

; ----------------------------------------------------------------------------

;**************  Bit definition for USB_COUNT0_TX_0 register  **************
USB_COUNT0_TX_0_COUNT0_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 0 (low) 

;**************  Bit definition for USB_COUNT0_TX_1 register  **************
USB_COUNT0_TX_1_COUNT0_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 0 (high) 

;**************  Bit definition for USB_COUNT1_TX_0 register  **************
USB_COUNT1_TX_0_COUNT1_TX_0          EQU    (0x000003FF)        ; Transmission Byte Count 1 (low) 

;**************  Bit definition for USB_COUNT1_TX_1 register  **************
USB_COUNT1_TX_1_COUNT1_TX_1          EQU    (0x03FF0000)        ; Transmission Byte Count 1 (high) 

;**************  Bit definition for USB_COUNT2_TX_0 register  **************
USB_COUNT2_TX_0_COUNT2_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 2 (low) 

;**************  Bit definition for USB_COUNT2_TX_1 register  **************
USB_COUNT2_TX_1_COUNT2_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 2 (high) 

;**************  Bit definition for USB_COUNT3_TX_0 register  **************
USB_COUNT3_TX_0_COUNT3_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 3 (low) 

;**************  Bit definition for USB_COUNT3_TX_1 register  **************
USB_COUNT3_TX_1_COUNT3_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 3 (high) 

;**************  Bit definition for USB_COUNT4_TX_0 register  **************
USB_COUNT4_TX_0_COUNT4_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 4 (low) 

;**************  Bit definition for USB_COUNT4_TX_1 register  **************
USB_COUNT4_TX_1_COUNT4_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 4 (high) 

;**************  Bit definition for USB_COUNT5_TX_0 register  **************
USB_COUNT5_TX_0_COUNT5_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 5 (low) 

;**************  Bit definition for USB_COUNT5_TX_1 register  **************
USB_COUNT5_TX_1_COUNT5_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 5 (high) 

;**************  Bit definition for USB_COUNT6_TX_0 register  **************
USB_COUNT6_TX_0_COUNT6_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 6 (low) 

;**************  Bit definition for USB_COUNT6_TX_1 register  **************
USB_COUNT6_TX_1_COUNT6_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 6 (high) 

;**************  Bit definition for USB_COUNT7_TX_0 register  **************
USB_COUNT7_TX_0_COUNT7_TX_0         EQU    (0x000003FF)        ; Transmission Byte Count 7 (low) 

;**************  Bit definition for USB_COUNT7_TX_1 register  **************
USB_COUNT7_TX_1_COUNT7_TX_1         EQU    (0x03FF0000)        ; Transmission Byte Count 7 (high) 

; ----------------------------------------------------------------------------

;**************  Bit definition for USB_ADDR0_RX register  ****************
USB_ADDR0_RX_ADDR0_RX               EQU    (0xFFFE)            ; Reception Buffer Address 0 

;**************  Bit definition for USB_ADDR1_RX register  ****************
USB_ADDR1_RX_ADDR1_RX               EQU    (0xFFFE)            ; Reception Buffer Address 1 

;**************  Bit definition for USB_ADDR2_RX register  ****************
USB_ADDR2_RX_ADDR2_RX               EQU    (0xFFFE)            ; Reception Buffer Address 2 

;**************  Bit definition for USB_ADDR3_RX register  ****************
USB_ADDR3_RX_ADDR3_RX               EQU    (0xFFFE)            ; Reception Buffer Address 3 

;**************  Bit definition for USB_ADDR4_RX register  ****************
USB_ADDR4_RX_ADDR4_RX               EQU    (0xFFFE)            ; Reception Buffer Address 4 

;**************  Bit definition for USB_ADDR5_RX register  ****************
USB_ADDR5_RX_ADDR5_RX               EQU    (0xFFFE)            ; Reception Buffer Address 5 

;**************  Bit definition for USB_ADDR6_RX register  ****************
USB_ADDR6_RX_ADDR6_RX               EQU    (0xFFFE)            ; Reception Buffer Address 6 

;**************  Bit definition for USB_ADDR7_RX register  ****************
USB_ADDR7_RX_ADDR7_RX               EQU    (0xFFFE)            ; Reception Buffer Address 7 

; ----------------------------------------------------------------------------

;**************  Bit definition for USB_COUNT0_RX register  ***************
USB_COUNT0_RX_COUNT0_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT0_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT0_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT0_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT0_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT0_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT0_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT0_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT1_RX register  ***************
USB_COUNT1_RX_COUNT1_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT1_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT1_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT1_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT1_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT1_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT1_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT1_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT2_RX register  ***************
USB_COUNT2_RX_COUNT2_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT2_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT2_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT2_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT2_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT2_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT2_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT2_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT3_RX register  ***************
USB_COUNT3_RX_COUNT3_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT3_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT3_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT3_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT3_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT3_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT3_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT3_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT4_RX register  ***************
USB_COUNT4_RX_COUNT4_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT4_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT4_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT4_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT4_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT4_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT4_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT4_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT5_RX register  ***************
USB_COUNT5_RX_COUNT5_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT5_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT5_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT5_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT5_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT5_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT5_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT5_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT6_RX register  ***************
USB_COUNT6_RX_COUNT6_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT6_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT6_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT6_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT6_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT6_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT6_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT6_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

;**************  Bit definition for USB_COUNT7_RX register  ***************
USB_COUNT7_RX_COUNT7_RX             EQU    (0x03FF)            ; Reception Byte Count 

USB_COUNT7_RX_NUM_BLOCK             EQU    (0x7C00)            ; NUM_BLOCK[4:0] bits (Number of blocks) 
USB_COUNT7_RX_NUM_BLOCK_0           EQU    (0x0400)            ; Bit 0 
USB_COUNT7_RX_NUM_BLOCK_1           EQU    (0x0800)            ; Bit 1 
USB_COUNT7_RX_NUM_BLOCK_2           EQU    (0x1000)            ; Bit 2 
USB_COUNT7_RX_NUM_BLOCK_3           EQU    (0x2000)            ; Bit 3 
USB_COUNT7_RX_NUM_BLOCK_4           EQU    (0x4000)            ; Bit 4 

USB_COUNT7_RX_BLSIZE                EQU    (0x8000)            ; BLock SIZE 

; ----------------------------------------------------------------------------

;**************  Bit definition for USB_COUNT0_RX_0 register  **************
USB_COUNT0_RX_0_COUNT0_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT0_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT0_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT0_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT0_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT0_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT0_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT0_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT0_RX_1 register  **************
USB_COUNT0_RX_1_COUNT0_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT0_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT0_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 1 
USB_COUNT0_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT0_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT0_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT0_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT0_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;**************  Bit definition for USB_COUNT1_RX_0 register  **************
USB_COUNT1_RX_0_COUNT1_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT1_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT1_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT1_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT1_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT1_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT1_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT1_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT1_RX_1 register  **************
USB_COUNT1_RX_1_COUNT1_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT1_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT1_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT1_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT1_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT1_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT1_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT1_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;**************  Bit definition for USB_COUNT2_RX_0 register  **************
USB_COUNT2_RX_0_COUNT2_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT2_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT2_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT2_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT2_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT2_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT2_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT2_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT2_RX_1 register  **************
USB_COUNT2_RX_1_COUNT2_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT2_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT2_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT2_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT2_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT2_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT2_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT2_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;**************  Bit definition for USB_COUNT3_RX_0 register  **************
USB_COUNT3_RX_0_COUNT3_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT3_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT3_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT3_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT3_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT3_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT3_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT3_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT3_RX_1 register  **************
USB_COUNT3_RX_1_COUNT3_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT3_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT3_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT3_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT3_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT3_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT3_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT3_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;**************  Bit definition for USB_COUNT4_RX_0 register  **************
USB_COUNT4_RX_0_COUNT4_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT4_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT4_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT4_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT4_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT4_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT4_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT4_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT4_RX_1 register  **************
USB_COUNT4_RX_1_COUNT4_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT4_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT4_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT4_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT4_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT4_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT4_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT4_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;**************  Bit definition for USB_COUNT5_RX_0 register  **************
USB_COUNT5_RX_0_COUNT5_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT5_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT5_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT5_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT5_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT5_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT5_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT5_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT5_RX_1 register  **************
USB_COUNT5_RX_1_COUNT5_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT5_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT5_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT5_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT5_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT5_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT5_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT5_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;*************  Bit definition for USB_COUNT6_RX_0  register  **************
USB_COUNT6_RX_0_COUNT6_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT6_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT6_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT6_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT6_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT6_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT6_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT6_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;**************  Bit definition for USB_COUNT6_RX_1 register  **************
USB_COUNT6_RX_1_COUNT6_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT6_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT6_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT6_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT6_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT6_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT6_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT6_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 

;*************  Bit definition for USB_COUNT7_RX_0 register  ***************
USB_COUNT7_RX_0_COUNT7_RX_0         EQU    (0x000003FF)        ; Reception Byte Count (low) 

USB_COUNT7_RX_0_NUM_BLOCK_0         EQU    (0x00007C00)        ; NUM_BLOCK_0[4:0] bits (Number of blocks) (low) 
USB_COUNT7_RX_0_NUM_BLOCK_0_0       EQU    (0x00000400)        ; Bit 0 
USB_COUNT7_RX_0_NUM_BLOCK_0_1       EQU    (0x00000800)        ; Bit 1 
USB_COUNT7_RX_0_NUM_BLOCK_0_2       EQU    (0x00001000)        ; Bit 2 
USB_COUNT7_RX_0_NUM_BLOCK_0_3       EQU    (0x00002000)        ; Bit 3 
USB_COUNT7_RX_0_NUM_BLOCK_0_4       EQU    (0x00004000)        ; Bit 4 

USB_COUNT7_RX_0_BLSIZE_0            EQU    (0x00008000)        ; BLock SIZE (low) 

;*************  Bit definition for USB_COUNT7_RX_1 register  ***************
USB_COUNT7_RX_1_COUNT7_RX_1         EQU    (0x03FF0000)        ; Reception Byte Count (high) 

USB_COUNT7_RX_1_NUM_BLOCK_1         EQU    (0x7C000000)        ; NUM_BLOCK_1[4:0] bits (Number of blocks) (high) 
USB_COUNT7_RX_1_NUM_BLOCK_1_0       EQU    (0x04000000)        ; Bit 0 
USB_COUNT7_RX_1_NUM_BLOCK_1_1       EQU    (0x08000000)        ; Bit 1 
USB_COUNT7_RX_1_NUM_BLOCK_1_2       EQU    (0x10000000)        ; Bit 2 
USB_COUNT7_RX_1_NUM_BLOCK_1_3       EQU    (0x20000000)        ; Bit 3 
USB_COUNT7_RX_1_NUM_BLOCK_1_4       EQU    (0x40000000)        ; Bit 4 

USB_COUNT7_RX_1_BLSIZE_1            EQU    (0x80000000)        ; BLock SIZE (high) 



;***************************************************************************
;                                                                           
;                          Window WATCHDOG (WWDG)                           
;                                                                           
;***************************************************************************

;**************  Bit definition for WWDG_CR register  *******************
WWDG_CR_T                           EQU    (0x7F)               ; T[6:0] bits (7-Bit counter (MSB to LSB)) 
WWDG_CR_T0                          EQU    (0x01)               ; Bit 0 
WWDG_CR_T1                          EQU    (0x02)               ; Bit 1 
WWDG_CR_T2                          EQU    (0x04)               ; Bit 2 
WWDG_CR_T3                          EQU    (0x08)               ; Bit 3 
WWDG_CR_T4                          EQU    (0x10)               ; Bit 4 
WWDG_CR_T5                          EQU    (0x20)               ; Bit 5 
WWDG_CR_T6                          EQU    (0x40)               ; Bit 6 

WWDG_CR_WDGA                        EQU    (0x80)               ; Activation bit 

;**************  Bit definition for WWDG_CFR register  ******************
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

;**************  Bit definition for WWDG_SR register  *******************
WWDG_SR_EWIF                        EQU    (0x01)               ; Early Wakeup Interrupt Flag 



;***************************************************************************
;                                                                           
;                         SystemTick (SysTick)                              
;                                                                           
;***************************************************************************

;**************  Bit definition forSysTick_CTRL register  ****************
SysTick_CTRL_ENABLE                 EQU    (0x00000001)        ; Counter enable 
SysTick_CTRL_TICKINT                EQU    (0x00000002)        ; Counting down to 0 pends the SysTick handler 
SysTick_CTRL_CLKSOURCE              EQU    (0x00000004)        ; Clock source 
SysTick_CTRL_COUNTFLAG              EQU    (0x00010000)        ; Count Flag 

;**************  Bit definition forSysTick_LOAD register  ****************
SysTick_LOAD_RELOAD                 EQU    (0x00FFFFFF)        ; Value to load into the SysTick Current Value Register when the counter reaches 0 

;**************  Bit definition forSysTick_VAL register  *****************
SysTick_VAL_CURRENT                 EQU    (0x00FFFFFF)        ; Current value at the time the register is accessed 

;**************  Bit definition forSysTick_CALIB register  ***************
SysTick_CALIB_TENMS                 EQU    (0x00FFFFFF)        ; Reload value to use for 10ms timing 
SysTick_CALIB_SKEW                  EQU    (0x40000000)        ; Calibration value is not exactly 10 ms 
SysTick_CALIB_NOREF                 EQU    (0x80000000)        ; The reference clock is not provided 



;***************************************************************************
;                                                                           
;                Nested Vectored Interrupt Controller (NVIC)                
;                                                                           
;***************************************************************************

;**************  Bit definition for NVIC_ISER register  ******************
NVIC_ISER_SETENA                    EQU    (0xFFFFFFFF)        ; Interrupt set enable bits 
NVIC_ISER_SETENA_0                  EQU    (0x00000001)        ; bit 0 
NVIC_ISER_SETENA_1                  EQU    (0x00000002)        ; bit 1 
NVIC_ISER_SETENA_2                  EQU    (0x00000004)        ; bit 2 
NVIC_ISER_SETENA_3                  EQU    (0x00000008)        ; bit 3 
NVIC_ISER_SETENA_4                  EQU    (0x00000010)        ; bit 4 
NVIC_ISER_SETENA_5                  EQU    (0x00000020)        ; bit 5 
NVIC_ISER_SETENA_6                  EQU    (0x00000040)        ; bit 6 
NVIC_ISER_SETENA_7                  EQU    (0x00000080)        ; bit 7 
NVIC_ISER_SETENA_8                  EQU    (0x00000100)        ; bit 8 
NVIC_ISER_SETENA_9                  EQU    (0x00000200)        ; bit 9 
NVIC_ISER_SETENA_10                 EQU    (0x00000400)        ; bit 10 
NVIC_ISER_SETENA_11                 EQU    (0x00000800)        ; bit 11 
NVIC_ISER_SETENA_12                 EQU    (0x00001000)        ; bit 12 
NVIC_ISER_SETENA_13                 EQU    (0x00002000)        ; bit 13 
NVIC_ISER_SETENA_14                 EQU    (0x00004000)        ; bit 14 
NVIC_ISER_SETENA_15                 EQU    (0x00008000)        ; bit 15 
NVIC_ISER_SETENA_16                 EQU    (0x00010000)        ; bit 16 
NVIC_ISER_SETENA_17                 EQU    (0x00020000)        ; bit 17 
NVIC_ISER_SETENA_18                 EQU    (0x00040000)        ; bit 18 
NVIC_ISER_SETENA_19                 EQU    (0x00080000)        ; bit 19 
NVIC_ISER_SETENA_20                 EQU    (0x00100000)        ; bit 20 
NVIC_ISER_SETENA_21                 EQU    (0x00200000)        ; bit 21 
NVIC_ISER_SETENA_22                 EQU    (0x00400000)        ; bit 22 
NVIC_ISER_SETENA_23                 EQU    (0x00800000)        ; bit 23 
NVIC_ISER_SETENA_24                 EQU    (0x01000000)        ; bit 24 
NVIC_ISER_SETENA_25                 EQU    (0x02000000)        ; bit 25 
NVIC_ISER_SETENA_26                 EQU    (0x04000000)        ; bit 26 
NVIC_ISER_SETENA_27                 EQU    (0x08000000)        ; bit 27 
NVIC_ISER_SETENA_28                 EQU    (0x10000000)        ; bit 28 
NVIC_ISER_SETENA_29                 EQU    (0x20000000)        ; bit 29 
NVIC_ISER_SETENA_30                 EQU    (0x40000000)        ; bit 30 
NVIC_ISER_SETENA_31                 EQU    (0x80000000)        ; bit 31 

;**************  Bit definition for NVIC_ICER register  ******************
NVIC_ICER_CLRENA                    EQU    (0xFFFFFFFF)        ; Interrupt clear-enable bits 
NVIC_ICER_CLRENA_0                  EQU    (0x00000001)        ; bit 0 
NVIC_ICER_CLRENA_1                  EQU    (0x00000002)        ; bit 1 
NVIC_ICER_CLRENA_2                  EQU    (0x00000004)        ; bit 2 
NVIC_ICER_CLRENA_3                  EQU    (0x00000008)        ; bit 3 
NVIC_ICER_CLRENA_4                  EQU    (0x00000010)        ; bit 4 
NVIC_ICER_CLRENA_5                  EQU    (0x00000020)        ; bit 5 
NVIC_ICER_CLRENA_6                  EQU    (0x00000040)        ; bit 6 
NVIC_ICER_CLRENA_7                  EQU    (0x00000080)        ; bit 7 
NVIC_ICER_CLRENA_8                  EQU    (0x00000100)        ; bit 8 
NVIC_ICER_CLRENA_9                  EQU    (0x00000200)        ; bit 9 
NVIC_ICER_CLRENA_10                 EQU    (0x00000400)        ; bit 10 
NVIC_ICER_CLRENA_11                 EQU    (0x00000800)        ; bit 11 
NVIC_ICER_CLRENA_12                 EQU    (0x00001000)        ; bit 12 
NVIC_ICER_CLRENA_13                 EQU    (0x00002000)        ; bit 13 
NVIC_ICER_CLRENA_14                 EQU    (0x00004000)        ; bit 14 
NVIC_ICER_CLRENA_15                 EQU    (0x00008000)        ; bit 15 
NVIC_ICER_CLRENA_16                 EQU    (0x00010000)        ; bit 16 
NVIC_ICER_CLRENA_17                 EQU    (0x00020000)        ; bit 17 
NVIC_ICER_CLRENA_18                 EQU    (0x00040000)        ; bit 18 
NVIC_ICER_CLRENA_19                 EQU    (0x00080000)        ; bit 19 
NVIC_ICER_CLRENA_20                 EQU    (0x00100000)        ; bit 20 
NVIC_ICER_CLRENA_21                 EQU    (0x00200000)        ; bit 21 
NVIC_ICER_CLRENA_22                 EQU    (0x00400000)        ; bit 22 
NVIC_ICER_CLRENA_23                 EQU    (0x00800000)        ; bit 23 
NVIC_ICER_CLRENA_24                 EQU    (0x01000000)        ; bit 24 
NVIC_ICER_CLRENA_25                 EQU    (0x02000000)        ; bit 25 
NVIC_ICER_CLRENA_26                 EQU    (0x04000000)        ; bit 26 
NVIC_ICER_CLRENA_27                 EQU    (0x08000000)        ; bit 27 
NVIC_ICER_CLRENA_28                 EQU    (0x10000000)        ; bit 28 
NVIC_ICER_CLRENA_29                 EQU    (0x20000000)        ; bit 29 
NVIC_ICER_CLRENA_30                 EQU    (0x40000000)        ; bit 30 
NVIC_ICER_CLRENA_31                 EQU    (0x80000000)        ; bit 31 

;**************  Bit definition for NVIC_ISPR register  ******************
NVIC_ISPR_SETPEND                   EQU    (0xFFFFFFFF)        ; Interrupt set-pending bits 
NVIC_ISPR_SETPEND_0                 EQU    (0x00000001)        ; bit 0 
NVIC_ISPR_SETPEND_1                 EQU    (0x00000002)        ; bit 1 
NVIC_ISPR_SETPEND_2                 EQU    (0x00000004)        ; bit 2 
NVIC_ISPR_SETPEND_3                 EQU    (0x00000008)        ; bit 3 
NVIC_ISPR_SETPEND_4                 EQU    (0x00000010)        ; bit 4 
NVIC_ISPR_SETPEND_5                 EQU    (0x00000020)        ; bit 5 
NVIC_ISPR_SETPEND_6                 EQU    (0x00000040)        ; bit 6 
NVIC_ISPR_SETPEND_7                 EQU    (0x00000080)        ; bit 7 
NVIC_ISPR_SETPEND_8                 EQU    (0x00000100)        ; bit 8 
NVIC_ISPR_SETPEND_9                 EQU    (0x00000200)        ; bit 9 
NVIC_ISPR_SETPEND_10                EQU    (0x00000400)        ; bit 10 
NVIC_ISPR_SETPEND_11                EQU    (0x00000800)        ; bit 11 
NVIC_ISPR_SETPEND_12                EQU    (0x00001000)        ; bit 12 
NVIC_ISPR_SETPEND_13                EQU    (0x00002000)        ; bit 13 
NVIC_ISPR_SETPEND_14                EQU    (0x00004000)        ; bit 14 
NVIC_ISPR_SETPEND_15                EQU    (0x00008000)        ; bit 15 
NVIC_ISPR_SETPEND_16                EQU    (0x00010000)        ; bit 16 
NVIC_ISPR_SETPEND_17                EQU    (0x00020000)        ; bit 17 
NVIC_ISPR_SETPEND_18                EQU    (0x00040000)        ; bit 18 
NVIC_ISPR_SETPEND_19                EQU    (0x00080000)        ; bit 19 
NVIC_ISPR_SETPEND_20                EQU    (0x00100000)        ; bit 20 
NVIC_ISPR_SETPEND_21                EQU    (0x00200000)        ; bit 21 
NVIC_ISPR_SETPEND_22                EQU    (0x00400000)        ; bit 22 
NVIC_ISPR_SETPEND_23                EQU    (0x00800000)        ; bit 23 
NVIC_ISPR_SETPEND_24                EQU    (0x01000000)        ; bit 24 
NVIC_ISPR_SETPEND_25                EQU    (0x02000000)        ; bit 25 
NVIC_ISPR_SETPEND_26                EQU    (0x04000000)        ; bit 26 
NVIC_ISPR_SETPEND_27                EQU    (0x08000000)        ; bit 27 
NVIC_ISPR_SETPEND_28                EQU    (0x10000000)        ; bit 28 
NVIC_ISPR_SETPEND_29                EQU    (0x20000000)        ; bit 29 
NVIC_ISPR_SETPEND_30                EQU    (0x40000000)        ; bit 30 
NVIC_ISPR_SETPEND_31                EQU    (0x80000000)        ; bit 31 

;**************  Bit definition for NVIC_ICPR register  ******************
NVIC_ICPR_CLRPEND                   EQU    (0xFFFFFFFF)        ; Interrupt clear-pending bits 
NVIC_ICPR_CLRPEND_0                 EQU    (0x00000001)        ; bit 0 
NVIC_ICPR_CLRPEND_1                 EQU    (0x00000002)        ; bit 1 
NVIC_ICPR_CLRPEND_2                 EQU    (0x00000004)        ; bit 2 
NVIC_ICPR_CLRPEND_3                 EQU    (0x00000008)        ; bit 3 
NVIC_ICPR_CLRPEND_4                 EQU    (0x00000010)        ; bit 4 
NVIC_ICPR_CLRPEND_5                 EQU    (0x00000020)        ; bit 5 
NVIC_ICPR_CLRPEND_6                 EQU    (0x00000040)        ; bit 6 
NVIC_ICPR_CLRPEND_7                 EQU    (0x00000080)        ; bit 7 
NVIC_ICPR_CLRPEND_8                 EQU    (0x00000100)        ; bit 8 
NVIC_ICPR_CLRPEND_9                 EQU    (0x00000200)        ; bit 9 
NVIC_ICPR_CLRPEND_10                EQU    (0x00000400)        ; bit 10 
NVIC_ICPR_CLRPEND_11                EQU    (0x00000800)        ; bit 11 
NVIC_ICPR_CLRPEND_12                EQU    (0x00001000)        ; bit 12 
NVIC_ICPR_CLRPEND_13                EQU    (0x00002000)        ; bit 13 
NVIC_ICPR_CLRPEND_14                EQU    (0x00004000)        ; bit 14 
NVIC_ICPR_CLRPEND_15                EQU    (0x00008000)        ; bit 15 
NVIC_ICPR_CLRPEND_16                EQU    (0x00010000)        ; bit 16 
NVIC_ICPR_CLRPEND_17                EQU    (0x00020000)        ; bit 17 
NVIC_ICPR_CLRPEND_18                EQU    (0x00040000)        ; bit 18 
NVIC_ICPR_CLRPEND_19                EQU    (0x00080000)        ; bit 19 
NVIC_ICPR_CLRPEND_20                EQU    (0x00100000)        ; bit 20 
NVIC_ICPR_CLRPEND_21                EQU    (0x00200000)        ; bit 21 
NVIC_ICPR_CLRPEND_22                EQU    (0x00400000)        ; bit 22 
NVIC_ICPR_CLRPEND_23                EQU    (0x00800000)        ; bit 23 
NVIC_ICPR_CLRPEND_24                EQU    (0x01000000)        ; bit 24 
NVIC_ICPR_CLRPEND_25                EQU    (0x02000000)        ; bit 25 
NVIC_ICPR_CLRPEND_26                EQU    (0x04000000)        ; bit 26 
NVIC_ICPR_CLRPEND_27                EQU    (0x08000000)        ; bit 27 
NVIC_ICPR_CLRPEND_28                EQU    (0x10000000)        ; bit 28 
NVIC_ICPR_CLRPEND_29                EQU    (0x20000000)        ; bit 29 
NVIC_ICPR_CLRPEND_30                EQU    (0x40000000)        ; bit 30 
NVIC_ICPR_CLRPEND_31                EQU    (0x80000000)        ; bit 31 

;**************  Bit definition for NVIC_IABR register  ******************
NVIC_IABR_ACTIVE                    EQU    (0xFFFFFFFF)        ; Interrupt active flags 
NVIC_IABR_ACTIVE_0                  EQU    (0x00000001)        ; bit 0 
NVIC_IABR_ACTIVE_1                  EQU    (0x00000002)        ; bit 1 
NVIC_IABR_ACTIVE_2                  EQU    (0x00000004)        ; bit 2 
NVIC_IABR_ACTIVE_3                  EQU    (0x00000008)        ; bit 3 
NVIC_IABR_ACTIVE_4                  EQU    (0x00000010)        ; bit 4 
NVIC_IABR_ACTIVE_5                  EQU    (0x00000020)        ; bit 5 
NVIC_IABR_ACTIVE_6                  EQU    (0x00000040)        ; bit 6 
NVIC_IABR_ACTIVE_7                  EQU    (0x00000080)        ; bit 7 
NVIC_IABR_ACTIVE_8                  EQU    (0x00000100)        ; bit 8 
NVIC_IABR_ACTIVE_9                  EQU    (0x00000200)        ; bit 9 
NVIC_IABR_ACTIVE_10                 EQU    (0x00000400)        ; bit 10 
NVIC_IABR_ACTIVE_11                 EQU    (0x00000800)        ; bit 11 
NVIC_IABR_ACTIVE_12                 EQU    (0x00001000)        ; bit 12 
NVIC_IABR_ACTIVE_13                 EQU    (0x00002000)        ; bit 13 
NVIC_IABR_ACTIVE_14                 EQU    (0x00004000)        ; bit 14 
NVIC_IABR_ACTIVE_15                 EQU    (0x00008000)        ; bit 15 
NVIC_IABR_ACTIVE_16                 EQU    (0x00010000)        ; bit 16 
NVIC_IABR_ACTIVE_17                 EQU    (0x00020000)        ; bit 17 
NVIC_IABR_ACTIVE_18                 EQU    (0x00040000)        ; bit 18 
NVIC_IABR_ACTIVE_19                 EQU    (0x00080000)        ; bit 19 
NVIC_IABR_ACTIVE_20                 EQU    (0x00100000)        ; bit 20 
NVIC_IABR_ACTIVE_21                 EQU    (0x00200000)        ; bit 21 
NVIC_IABR_ACTIVE_22                 EQU    (0x00400000)        ; bit 22 
NVIC_IABR_ACTIVE_23                 EQU    (0x00800000)        ; bit 23 
NVIC_IABR_ACTIVE_24                 EQU    (0x01000000)        ; bit 24 
NVIC_IABR_ACTIVE_25                 EQU    (0x02000000)        ; bit 25 
NVIC_IABR_ACTIVE_26                 EQU    (0x04000000)        ; bit 26 
NVIC_IABR_ACTIVE_27                 EQU    (0x08000000)        ; bit 27 
NVIC_IABR_ACTIVE_28                 EQU    (0x10000000)        ; bit 28 
NVIC_IABR_ACTIVE_29                 EQU    (0x20000000)        ; bit 29 
NVIC_IABR_ACTIVE_30                 EQU    (0x40000000)        ; bit 30 
NVIC_IABR_ACTIVE_31                 EQU    (0x80000000)        ; bit 31 

;**************  Bit definition for NVIC_PRI0 register  ******************
NVIC_IPR0_PRI_0                     EQU    (0x000000FF)        ; Priority of interrupt 0 
NVIC_IPR0_PRI_1                     EQU    (0x0000FF00)        ; Priority of interrupt 1 
NVIC_IPR0_PRI_2                     EQU    (0x00FF0000)        ; Priority of interrupt 2 
NVIC_IPR0_PRI_3                     EQU    (0xFF000000)        ; Priority of interrupt 3 

;**************  Bit definition for NVIC_PRI1 register  ******************
NVIC_IPR1_PRI_4                     EQU    (0x000000FF)        ; Priority of interrupt 4 
NVIC_IPR1_PRI_5                     EQU    (0x0000FF00)        ; Priority of interrupt 5 
NVIC_IPR1_PRI_6                     EQU    (0x00FF0000)        ; Priority of interrupt 6 
NVIC_IPR1_PRI_7                     EQU    (0xFF000000)        ; Priority of interrupt 7 

;**************  Bit definition for NVIC_PRI2 register  ******************
NVIC_IPR2_PRI_8                     EQU    (0x000000FF)        ; Priority of interrupt 8 
NVIC_IPR2_PRI_9                     EQU    (0x0000FF00)        ; Priority of interrupt 9 
NVIC_IPR2_PRI_10                    EQU    (0x00FF0000)        ; Priority of interrupt 10 
NVIC_IPR2_PRI_11                    EQU    (0xFF000000)        ; Priority of interrupt 11 

;**************  Bit definition for NVIC_PRI3 register  ******************
NVIC_IPR3_PRI_12                    EQU    (0x000000FF)        ; Priority of interrupt 12 
NVIC_IPR3_PRI_13                    EQU    (0x0000FF00)        ; Priority of interrupt 13 
NVIC_IPR3_PRI_14                    EQU    (0x00FF0000)        ; Priority of interrupt 14 
NVIC_IPR3_PRI_15                    EQU    (0xFF000000)        ; Priority of interrupt 15 

;**************  Bit definition for NVIC_PRI4 register  ******************
NVIC_IPR4_PRI_16                    EQU    (0x000000FF)        ; Priority of interrupt 16 
NVIC_IPR4_PRI_17                    EQU    (0x0000FF00)        ; Priority of interrupt 17 
NVIC_IPR4_PRI_18                    EQU    (0x00FF0000)        ; Priority of interrupt 18 
NVIC_IPR4_PRI_19                    EQU    (0xFF000000)        ; Priority of interrupt 19 

;**************  Bit definition for NVIC_PRI5 register  ******************
NVIC_IPR5_PRI_20                    EQU    (0x000000FF)        ; Priority of interrupt 20 
NVIC_IPR5_PRI_21                    EQU    (0x0000FF00)        ; Priority of interrupt 21 
NVIC_IPR5_PRI_22                    EQU    (0x00FF0000)        ; Priority of interrupt 22 
NVIC_IPR5_PRI_23                    EQU    (0xFF000000)        ; Priority of interrupt 23 

;**************  Bit definition for NVIC_PRI6 register  ******************
NVIC_IPR6_PRI_24                    EQU    (0x000000FF)        ; Priority of interrupt 24 
NVIC_IPR6_PRI_25                    EQU    (0x0000FF00)        ; Priority of interrupt 25 
NVIC_IPR6_PRI_26                    EQU    (0x00FF0000)        ; Priority of interrupt 26 
NVIC_IPR6_PRI_27                    EQU    (0xFF000000)        ; Priority of interrupt 27 

;**************  Bit definition for NVIC_PRI7 register  ******************
NVIC_IPR7_PRI_28                    EQU    (0x000000FF)        ; Priority of interrupt 28 
NVIC_IPR7_PRI_29                    EQU    (0x0000FF00)        ; Priority of interrupt 29 
NVIC_IPR7_PRI_30                    EQU    (0x00FF0000)        ; Priority of interrupt 30 
NVIC_IPR7_PRI_31                    EQU    (0xFF000000)        ; Priority of interrupt 31 

;**************  Bit definition for SCB_CPUID register  ******************
SCB_CPUID_REVISION                  EQU    (0x0000000F)        ; Implementation defined revision number 
SCB_CPUID_PARTNO                    EQU    (0x0000FFF0)        ; Number of processor within family 
SCB_CPUID_Constant                  EQU    (0x000F0000)        ; Reads as 0x0F 
SCB_CPUID_VARIANT                   EQU    (0x00F00000)        ; Implementation defined variant number 
SCB_CPUID_IMPLEMENTER               EQU    (0xFF000000)        ; Implementer code. ARM is 0x41 

;**************  Bit definition for SCB_ICSR register  ******************
SCB_ICSR_VECTACTIVE                 EQU    (0x000001FF)        ; Active ISR number field 
SCB_ICSR_RETTOBASE                  EQU    (0x00000800)        ; All active exceptions minus the IPSR_current_exception yields the empty set 
SCB_ICSR_VECTPENDING                EQU    (0x003FF000)        ; Pending ISR number field 
SCB_ICSR_ISRPENDING                 EQU    (0x00400000)        ; Interrupt pending flag 
SCB_ICSR_ISRPREEMPT                 EQU    (0x00800000)        ; It indicates that a pending interrupt becomes active in the next running cycle 
SCB_ICSR_PENDSTCLR                  EQU    (0x02000000)        ; Clear pending SysTick bit 
SCB_ICSR_PENDSTSET                  EQU    (0x04000000)        ; Set pending SysTick bit 
SCB_ICSR_PENDSVCLR                  EQU    (0x08000000)        ; Clear pending pendSV bit 
SCB_ICSR_PENDSVSET                  EQU    (0x10000000)        ; Set pending pendSV bit 
SCB_ICSR_NMIPENDSET                 EQU    (0x80000000)        ; Set pending NMI bit 

;**************  Bit definition for SCB_VTOR register  ******************
SCB_VTOR_TBLOFF                     EQU    (0x1FFFFF80)        ; Vector table base offset field 
SCB_VTOR_TBLBASE                    EQU    (0x20000000)        ; Table base in code(0) or RAM(1) 

;**************  Bit definition for SCB_AIRCR register  ******************
SCB_AIRCR_VECTRESET                 EQU    (0x00000001)        ; System Reset bit 
SCB_AIRCR_VECTCLRACTIVE             EQU    (0x00000002)        ; Clear active vector bit 
SCB_AIRCR_SYSRESETREQ               EQU    (0x00000004)        ; Requests chip control logic to generate a reset 

SCB_AIRCR_PRIGROUP                  EQU    (0x00000700)        ; PRIGROUP[2:0] bits (Priority group) 
SCB_AIRCR_PRIGROUP_0                EQU    (0x00000100)        ; Bit 0 
SCB_AIRCR_PRIGROUP_1                EQU    (0x00000200)        ; Bit 1 
SCB_AIRCR_PRIGROUP_2                EQU    (0x00000400)        ; Bit 2

;  prority group configuration 
SCB_AIRCR_PRIGROUP0                 EQU    (0x00000000)        ; Priority groupEQU0 (7 bits of pre-emption priority, 1 bit of subpriority) 
SCB_AIRCR_PRIGROUP1                 EQU    (0x00000100)        ; Priority groupEQU1 (6 bits of pre-emption priority, 2 bits of subpriority) 
SCB_AIRCR_PRIGROUP2                 EQU    (0x00000200)        ; Priority groupEQU2 (5 bits of pre-emption priority, 3 bits of subpriority) 
SCB_AIRCR_PRIGROUP3                 EQU    (0x00000300)        ; Priority groupEQU3 (4 bits of pre-emption priority, 4 bits of subpriority) 
SCB_AIRCR_PRIGROUP4                 EQU    (0x00000400)        ; Priority groupEQU4 (3 bits of pre-emption priority, 5 bits of subpriority) 
SCB_AIRCR_PRIGROUP5                 EQU    (0x00000500)        ; Priority groupEQU5 (2 bits of pre-emption priority, 6 bits of subpriority) 
SCB_AIRCR_PRIGROUP6                 EQU    (0x00000600)        ; Priority groupEQU6 (1 bit of pre-emption priority, 7 bits of subpriority) 
SCB_AIRCR_PRIGROUP7                 EQU    (0x00000700)        ; Priority groupEQU7 (no pre-emption priority, 8 bits of subpriority) 

SCB_AIRCR_ENDIANESS                 EQU    (0x00008000)        ; Data endianness bit 
SCB_AIRCR_VECTKEY                   EQU    (0xFFFF0000)        ; Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) 

;**************  Bit definition for SCB_SCR register  *******************
SCB_SCR_SLEEPONEXIT                 EQU    (0x02)               ; Sleep on exit bit 
SCB_SCR_SLEEPDEEP                   EQU    (0x04)               ; Sleep deep bit 
SCB_SCR_SEVONPEND                   EQU    (0x10)               ; Wake up from WFE 

;**************  Bit definition for SCB_CCR register  ******************
SCB_CCR_NONBASETHRDENA              EQU    (0x0001)            ; Thread mode can be entered from any level in Handler mode by controlled return value 
SCB_CCR_USERSETMPEND                EQU    (0x0002)            ; Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception 
SCB_CCR_UNALIGN_TRP                 EQU    (0x0008)            ; Trap for unaligned access 
SCB_CCR_DIV_0_TRP                   EQU    (0x0010)            ; Trap on Divide by 0 
SCB_CCR_BFHFNMIGN                   EQU    (0x0100)            ; Handlers running at priority -1 and -2 
SCB_CCR_STKALIGN                    EQU    (0x0200)            ; On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned 

;**************  Bit definition for SCB_SHPR register *******************
SCB_SHPR_PRI_N                      EQU    (0x000000FF)        ; Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor 
SCB_SHPR_PRI_N1                     EQU    (0x0000FF00)        ; Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved 
SCB_SHPR_PRI_N2                     EQU    (0x00FF0000)        ; Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV 
SCB_SHPR_PRI_N3                     EQU    (0xFF000000)        ; Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick 

;**************  Bit definition for SCB_SHCSR register  ******************
SCB_SHCSR_MEMFAULTACT               EQU    (0x00000001)        ; MemManage is active 
SCB_SHCSR_BUSFAULTACT               EQU    (0x00000002)        ; BusFault is active 
SCB_SHCSR_USGFAULTACT               EQU    (0x00000008)        ; UsageFault is active 
SCB_SHCSR_SVCALLACT                 EQU    (0x00000080)        ; SVCall is active 
SCB_SHCSR_MONITORACT                EQU    (0x00000100)        ; Monitor is active 
SCB_SHCSR_PENDSVACT                 EQU    (0x00000400)        ; PendSV is active 
SCB_SHCSR_SYSTICKACT                EQU    (0x00000800)        ; SysTick is active 
SCB_SHCSR_USGFAULTPENDED            EQU    (0x00001000)        ; Usage Fault is pended 
SCB_SHCSR_MEMFAULTPENDED            EQU    (0x00002000)        ; MemManage is pended 
SCB_SHCSR_BUSFAULTPENDED            EQU    (0x00004000)        ; Bus Fault is pended 
SCB_SHCSR_SVCALLPENDED              EQU    (0x00008000)        ; SVCall is pended 
SCB_SHCSR_MEMFAULTENA               EQU    (0x00010000)        ; MemManage enable 
SCB_SHCSR_BUSFAULTENA               EQU    (0x00020000)        ; Bus Fault enable 
SCB_SHCSR_USGFAULTENA               EQU    (0x00040000)        ; UsageFault enable 

;**************  Bit definition for SCB_CFSR register  ******************
; MFSR 
SCB_CFSR_IACCVIOL                   EQU    (0x00000001)        ; Instruction access violation 
SCB_CFSR_DACCVIOL                   EQU    (0x00000002)        ; Data access violation 
SCB_CFSR_MUNSTKERR                  EQU    (0x00000008)        ; Unstacking error 
SCB_CFSR_MSTKERR                    EQU    (0x00000010)        ; Stacking error 
SCB_CFSR_MMARVALID                  EQU    (0x00000080)        ; Memory Manage Address Register address valid flag 
; BFSR 
SCB_CFSR_IBUSERR                    EQU    (0x00000100)        ; Instruction bus error flag 
SCB_CFSR_PRECISERR                  EQU    (0x00000200)        ; Precise data bus error 
SCB_CFSR_IMPRECISERR                EQU    (0x00000400)        ; Imprecise data bus error 
SCB_CFSR_UNSTKERR                   EQU    (0x00000800)        ; Unstacking error 
SCB_CFSR_STKERR                     EQU    (0x00001000)        ; Stacking error 
SCB_CFSR_BFARVALID                  EQU    (0x00008000)        ; Bus Fault Address Register address valid flag 
; UFSR 
SCB_CFSR_UNDEFINSTR                 EQU    (0x00010000)        ; The processor attempt to excecute an undefined instruction 
SCB_CFSR_INVSTATE                   EQU    (0x00020000)        ; Invalid combination of EPSR and instruction 
SCB_CFSR_INVPC                      EQU    (0x00040000)        ; Attempt to load EXC_RETURN into pc illegally 
SCB_CFSR_NOCP                       EQU    (0x00080000)        ; Attempt to use a coprocessor instruction 
SCB_CFSR_UNALIGNED                  EQU    (0x01000000)        ; Fault occurs when there is an attempt to make an unaligned memory access 
SCB_CFSR_DIVBYZERO                  EQU    (0x02000000)        ; Fault occurs when SDIV or DIV instruction is used with a divisor of 0 

;**************  Bit definition for SCB_HFSR register  ******************
SCB_HFSR_VECTTBL                    EQU    (0x00000002)        ; Fault occures because of vector table read on exception processing 
SCB_HFSR_FORCED                     EQU    (0x40000000)        ; Hard Fault activated when a configurable Fault was received and cannot activate 
SCB_HFSR_DEBUGEVT                   EQU    (0x80000000)        ; Fault related to debug 

;**************  Bit definition for SCB_DFSR register  ******************
SCB_DFSR_HALTED                     EQU    (0x01)              ; Halt request flag 
SCB_DFSR_BKPT                       EQU    (0x02)              ; BKPT flag 
SCB_DFSR_DWTTRAP                    EQU    (0x04)              ; Data Watchpoint and Trace (DWT) flag 
SCB_DFSR_VCATCH                     EQU    (0x08)              ; Vector catch flag 
SCB_DFSR_EXTERNAL                   EQU    (0x10)              ; External debug request flag 

;**************  Bit definition for SCB_MMFAR register  *****************
SCB_MMFAR_ADDRESS                   EQU    (0xFFFFFFFF)        ; Mem Manage fault address field 

;**************  Bit definition for SCB_BFAR register  ******************
SCB_BFAR_ADDRESS                    EQU    (0xFFFFFFFF)        ; Bus fault address field 

;**************  Bit definition for SCB_afsr register  ******************
SCB_AFSR_IMPDEF                     EQU    (0xFFFFFFFF)        ; Implementation defined 

; This following is added to remove the compiler warning.
    AREA    __DEFINES_STM32L1xx_DUMMY, CODE, READONLY
    END
