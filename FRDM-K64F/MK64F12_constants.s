; @file    stm32f401xc_constant.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    May-17-2015
; @note    Modifed from MK54F12.h (C) 2010 Freescale
; @brief   Assembly version of Cortex M3 core
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


; This following is added to remove the compiler warning.
    AREA    __DEFINES_MK64F12_xx_DUMMY, CODE, READONLY
		
; Memory map major version (memory maps with equal major version number are compatible)
MCU_MEM_MAP_VERSION       EQU  0x0200

; Memory map minor version
MCU_MEM_MAP_VERSION_MINOR EQU  0x0003

; ----------------------------------------------------------------------------
; -- Peripheral Instance Base Addresses
; ----------------------------------------------------------------------------
ADC0_BASE              EQU  0x4003B000
ADC1_BASE              EQU  0x400BB000
AIPS0_BASE             EQU  0x40000000
AIPS1_BASE             EQU  0x40080000
AXBS_BASE              EQU  0x40004000
CAN0_BASE              EQU  0x40024000
CAU_BASE               EQU  0xE0081000
CMP0_BASE              EQU  0x40073000
CMP1_BASE              EQU  0x40073008
CMP2_BASE              EQU  0x40073010
CMT_BASE               EQU  0x40062000
CRC_BASE               EQU  0x40032000
DAC0_BASE              EQU  0x400CC000
DAC1_BASE              EQU  0x400CD000
DMA_BASE               EQU  0x40008000
DMAMUX_BASE            EQU  0x40021000
ENET_BASE              EQU  0x400C0000
EWM_BASE               EQU  0x40061000
FB_BASE                EQU  0x4000C000
FMC_BASE               EQU  0x4001F000
FTFE_BASE              EQU  0x40020000
FTM0_BASE              EQU  0x40038000
FTM1_BASE              EQU  0x40039000
FTM2_BASE              EQU  0x4003A000
FTM3_BASE              EQU  0x400B9000
GPIOA_BASE             EQU  0x400FF000
GPIOB_BASE             EQU  0x400FF040
GPIOC_BASE             EQU  0x400FF080
GPIOD_BASE             EQU  0x400FF0C0
GPIOE_BASE             EQU  0x400FF100
I2C0_BASE              EQU  0x40066000
I2C1_BASE              EQU  0x40067000
I2C2_BASE              EQU  0x400E6000
I2S0_BASE              EQU  0x4002F000
LLWU_BASE              EQU  0x4007C000
LPTMR0_BASE            EQU  0x40040000
MCG_BASE               EQU  0x40064000
MCM_BASE               EQU  0xE0080000
MPU_BASE               EQU  0x4000D000
FTFE_FlashConfig_BASE  EQU  0x400
OSC_BASE               EQU  0x40065000
PDB0_BASE              EQU  0x40036000
PIT_BASE               EQU  0x40037000
PMC_BASE               EQU  0x4007D000
PORTA_BASE             EQU  0x40049000
PORTB_BASE             EQU  0x4004A000
PORTC_BASE             EQU  0x4004B000
PORTD_BASE             EQU  0x4004C000
PORTE_BASE             EQU  0x4004D000
PTA_BASE               EQU  0x400FF000
PTB_BASE               EQU  0x400FF040
PTC_BASE               EQU  0x400FF080
PTD_BASE               EQU  0x400FF0C0
PTE_BASE               EQU  0x400FF100
RCM_BASE               EQU  0x4007F000
RFSYS_BASE             EQU  0x40041000
RFVBAT_BASE            EQU  0x4003E000
RNG_BASE               EQU  0x40029000
RTC_BASE               EQU  0x4003D000
SDHC_BASE              EQU  0x400B1000
SIM_BASE               EQU  0x40047000
SMC_BASE               EQU  0x4007E000
SPI0_BASE              EQU  0x4002C000
SPI1_BASE              EQU  0x4002D000
SPI2_BASE              EQU  0x400AC000
UART0_BASE             EQU  0x4006A000
UART1_BASE             EQU  0x4006B000
UART2_BASE             EQU  0x4006C000
UART3_BASE             EQU  0x4006D000
UART4_BASE             EQU  0x400EA000
UART5_BASE             EQU  0x400EB000
USB0_BASE              EQU  0x40072000
USBDCD_BASE            EQU  0x40035000
VREF_BASE              EQU  0x40074000
WDOG_BASE              EQU  0x40052000

; ----------------------------------------------------------------------------
; -- Interrupt vector numbers
; ----------------------------------------------------------------------------

; Interrupt_vector_numbers Interrupt vector numbers

; Interrupt Number Definitions
; Core interrupts
NonMaskableInt_IRQn           EQU -14   ; Non Maskable Interrupt
HardFault_IRQn                EQU -13   ; Cortex-M4 SV Hard Fault Interrupt
MemoryManagement_IRQn         EQU -12   ; Cortex-M4 Memory Management Interrupt
BusFault_IRQn                 EQU -11   ; Cortex-M4 Bus Fault Interrupt
UsageFault_IRQn               EQU -10   ; Cortex-M4 Usage Fault Interrupt
SVCall_IRQn                   EQU -5    ; Cortex-M4 SV Call Interrupt
DebugMonitor_IRQn             EQU -4    ; Cortex-M4 Debug Monitor Interrupt
PendSV_IRQn                   EQU -2    ; Cortex-M4 Pend SV Interrupt
SysTick_IRQn                  EQU -1    ; Cortex-M4 System Tick Interrupt

; Device specific interrupts
DMA0_IRQn                     EQU  0     ; DMA Channel 0 Transfer Complete
DMA1_IRQn                     EQU  1     ; DMA Channel 1 Transfer Complete
DMA2_IRQn                     EQU  2     ; DMA Channel 2 Transfer Complete
DMA3_IRQn                     EQU  3     ; DMA Channel 3 Transfer Complete
DMA4_IRQn                     EQU  4     ; DMA Channel 4 Transfer Complete
DMA5_IRQn                     EQU  5     ; DMA Channel 5 Transfer Complete
DMA6_IRQn                     EQU  6     ; DMA Channel 6 Transfer Complete
DMA7_IRQn                     EQU  7     ; DMA Channel 7 Transfer Complete
DMA8_IRQn                     EQU  8     ; DMA Channel 8 Transfer Complete
DMA9_IRQn                     EQU  9     ; DMA Channel 9 Transfer Complete
DMA10_IRQn                    EQU  10    ; DMA Channel 10 Transfer Complete
DMA11_IRQn                    EQU  11    ; DMA Channel 11 Transfer Complete
DMA12_IRQn                    EQU  12    ; DMA Channel 12 Transfer Complete
DMA13_IRQn                    EQU  13    ; DMA Channel 13 Transfer Complete
DMA14_IRQn                    EQU  14    ; DMA Channel 14 Transfer Complete
DMA15_IRQn                    EQU  15    ; DMA Channel 15 Transfer Complete
DMA_Error_IRQn                EQU  16    ; DMA Error Interrupt
MCM_IRQn                      EQU  17    ; Normal Interrupt
FTFE_IRQn                     EQU  18    ; FTFE Command complete interrupt
Read_Collision_IRQn           EQU  19    ; Read Collision Interrupt
LVD_LVW_IRQn                  EQU  20    ; Low Voltage Detect, Low Voltage Warning
LLW_IRQn                      EQU  21    ; Low Leakage Wakeup
Watchdog_IRQn                 EQU  22    ; WDOG Interrupt
RNG_IRQn                      EQU  23    ; RNG Interrupt
I2C0_IRQn                     EQU  24    ; I2C0 interrupt
I2C1_IRQn                     EQU  25    ; I2C1 interrupt
SPI0_IRQn                     EQU  26    ; SPI0 Interrupt
SPI1_IRQn                     EQU  27    ; SPI1 Interrupt
I2S0_Tx_IRQn                  EQU  28    ; I2S0 transmit interrupt
I2S0_Rx_IRQn                  EQU  29    ; I2S0 receive interrupt
UART0_LON_IRQn                EQU  30    ; UART0 LON interrupt
UART0_RX_TX_IRQn              EQU  31    ; UART0 Receive/Transmit interrupt
UART0_ERR_IRQn                EQU  32    ; UART0 Error interrupt
UART1_RX_TX_IRQn              EQU  33    ; UART1 Receive/Transmit interrupt
UART1_ERR_IRQn                EQU  34    ; UART1 Error interrupt
UART2_RX_TX_IRQn              EQU  35    ; UART2 Receive/Transmit interrupt
UART2_ERR_IRQn                EQU  36    ; UART2 Error interrupt
UART3_RX_TX_IRQn              EQU  37    ; UART3 Receive/Transmit interrupt
UART3_ERR_IRQn                EQU  38    ; UART3 Error interrupt
ADC0_IRQn                     EQU  39    ; ADC0 interrupt
CMP0_IRQn                     EQU  40    ; CMP0 interrupt
CMP1_IRQn                     EQU  41    ; CMP1 interrupt
FTM0_IRQn                     EQU  42    ; FTM0 fault, overflow and channels interrupt
FTM1_IRQn                     EQU  43    ; FTM1 fault, overflow and channels interrupt
FTM2_IRQn                     EQU  44    ; FTM2 fault, overflow and channels interrupt
CMT_IRQn                      EQU  45    ; CMT interrupt
RTC_IRQn                      EQU  46    ; RTC interrupt
RTC_Seconds_IRQn              EQU  47    ; RTC seconds interrupt
PIT0_IRQn                     EQU  48    ; PIT timer channel 0 interrupt
PIT1_IRQn                     EQU  49    ; PIT timer channel 1 interrupt
PIT2_IRQn                     EQU  50    ; PIT timer channel 2 interrupt
PIT3_IRQn                     EQU  51    ; PIT timer channel 3 interrupt
PDB0_IRQn                     EQU  52    ; PDB0 Interrupt
USB0_IRQn                     EQU  53    ; USB0 interrupt
USBDCD_IRQn                   EQU  54    ; USBDCD Interrupt
Reserved71_IRQn               EQU  55    ; Reserved interrupt 71
DAC0_IRQn                     EQU  56    ; DAC0 interrupt
MCG_IRQn                      EQU  57    ; MCG Interrupt
LPTimer_IRQn                  EQU  58    ; LPTimer interrupt
PORTA_IRQn                    EQU  59    ; Port A interrupt
PORTB_IRQn                    EQU  60    ; Port B interrupt
PORTC_IRQn                    EQU  61    ; Port C interrupt
PORTD_IRQn                    EQU  62    ; Port D interrupt
PORTE_IRQn                    EQU  63    ; Port E interrupt
SWI_IRQn                      EQU  64    ; Software interrupt
SPI2_IRQn                     EQU  65    ; SPI2 Interrupt
UART4_RX_TX_IRQn              EQU  66    ; UART4 Receive/Transmit interrupt
UART4_ERR_IRQn                EQU  67    ; UART4 Error interrupt
UART5_RX_TX_IRQn              EQU  68    ; UART5 Receive/Transmit interrupt
UART5_ERR_IRQn                EQU  69    ; UART5 Error interrupt
CMP2_IRQn                     EQU  70    ; CMP2 interrupt
FTM3_IRQn                     EQU  71    ; FTM3 fault, overflow and channels interrupt
DAC1_IRQn                     EQU  72    ; DAC1 interrupt
ADC1_IRQn                     EQU  73    ; ADC1 interrupt
I2C2_IRQn                     EQU  74    ; I2C2 interrupt
CAN0_ORed_Message_buffer_IRQn EQU  75    ; CAN0 OR'd message buffers interrupt
CAN0_Bus_Off_IRQn             EQU  76    ; CAN0 bus off interrupt
CAN0_Error_IRQn               EQU  77    ; CAN0 error interrupt
CAN0_Tx_Warning_IRQn          EQU  78    ; CAN0 Tx warning interrupt
CAN0_Rx_Warning_IRQn          EQU  79    ; CAN0 Rx warning interrupt
CAN0_Wake_Up_IRQn             EQU  80    ; CAN0 wake up interrupt
SDHC_IRQn                     EQU  81    ; SDHC interrupt
ENET_1588_Timer_IRQn          EQU  82    ; Ethernet MAC IEEE 1588 Timer Interrupt
ENET_Transmit_IRQn            EQU  83    ; Ethernet MAC Transmit Interrupt
ENET_Receive_IRQn             EQU  84    ; Ethernet MAC Receive Interrupt
ENET_Error_IRQn               EQU  85    ; Ethernet MAC Error and miscellaneous Interrupt

; ----------------------------------------------------------------------------
; -- Cortex M4 Core Configuration
; ----------------------------------------------------------------------------

; Cortex_Core_Configuration Cortex M4 Core Configuration
__MPU_PRESENT                 EQU  0   ; Defines if an MPU is present or not
__NVIC_PRIO_BITS              EQU  4   ; Number of priority bits implemented in the NVIC
__Vendor_SysTickConfig        EQU  0   ; Vendor specific implementation of SysTickConfig is defined
__FPU_PRESENT                 EQU  1   ; Defines if an FPU is present or not

; ----------------------------------------------------------------------------
; -- ADC Peripheral Access Layer
; ----------------------------------------------------------------------------

ADC_SC1_0    EQU  0x00  ; ADC Status and Control Registers 1, array offset: 0x0, array step: 0x4
ADC_SC1_1    EQU  0x04  ; ADC Status and Control Registers 2
ADC_CFG1     EQU  0x08  ; ADC Configuration Register 1, offset: 0x8
ADC_CFG2     EQU  0x0C  ; ADC Configuration Register 2, offset: 0xC
ADC_R1       EQU  0x10  ; ADC Data Result Register, array offset: 0x10, array step: 0x4
ADC_R2       EQU  0x14  ; ADC Data Result Register
ADC_CV1      EQU  0x18  ; Compare Value Registers, offset: 0x18
ADC_CV2      EQU  0x1C  ; Compare Value Registers, offset: 0x1C
ADC_SC2      EQU  0x20  ; Status and Control Register 2, offset: 0x20
ADC_SC3      EQU  0x24  ; Status and Control Register 3, offset: 0x24
ADC_OFS      EQU  0x28  ; ADC Offset Correction Register, offset: 0x28
ADC_PG       EQU  0x2C  ; ADC Plus-Side Gain Register, offset: 0x2C
ADC_MG       EQU  0x30  ; ADC Minus-Side Gain Register, offset: 0x30
ADC_CLPD     EQU  0x34  ; ADC Plus-Side General Calibration Value Register, offset: 0x34
ADC_CLPS     EQU  0x38  ; ADC Plus-Side General Calibration Value Register, offset: 0x38
ADC_CLP4     EQU  0x3C  ; ADC Plus-Side General Calibration Value Register, offset: 0x3C
ADC_CLP3     EQU  0x40  ; ADC Plus-Side General Calibration Value Register, offset: 0x40
ADC_CLP2     EQU  0x44  ; ADC Plus-Side General Calibration Value Register, offset: 0x44
ADC_CLP1     EQU  0x48  ; ADC Plus-Side General Calibration Value Register, offset: 0x48
ADC_CLP0     EQU  0x4C  ; ADC Plus-Side General Calibration Value Register, offset: 0x4C
ADC_CLMD     EQU  0x54  ; ADC Minus-Side General Calibration Value Register, offset: 0x54
ADC_CLMS     EQU  0x58  ; ADC Minus-Side General Calibration Value Register, offset: 0x58
ADC_CLM4     EQU  0x5C  ; ADC Minus-Side General Calibration Value Register, offset: 0x5C
ADC_CLM3     EQU  0x60  ; ADC Minus-Side General Calibration Value Register, offset: 0x60
ADC_CLM2     EQU  0x64  ; ADC Minus-Side General Calibration Value Register, offset: 0x64
ADC_CLM1     EQU  0x68  ; ADC Minus-Side General Calibration Value Register, offset: 0x68
ADC_CLM0     EQU  0x6C  ; ADC Minus-Side General Calibration Value Register, offset: 0x6C

; ----------------------------------------------------------------------------
; -- ADC Register Masks
; ----------------------------------------------------------------------------

; SC1 Bit Fields
ADC_SC1_ADCH_MASK        EQU  0x1F
ADC_SC1_ADCH_SHIFT       EQU  0
ADC_SC1_DIFF_MASK        EQU  0x20
ADC_SC1_DIFF_SHIFT       EQU  5
ADC_SC1_AIEN_MASK        EQU  0x40
ADC_SC1_AIEN_SHIFT       EQU  6
ADC_SC1_COCO_MASK        EQU  0x80
ADC_SC1_COCO_SHIFT       EQU  7

; CFG1 Bit Fields
ADC_CFG1_ADICLK_MASK     EQU  0x3
ADC_CFG1_ADICLK_SHIFT    EQU  0
ADC_CFG1_MODE_MASK       EQU  0xC
ADC_CFG1_MODE_SHIFT      EQU  2
ADC_CFG1_ADLSMP_MASK     EQU  0x10
ADC_CFG1_ADLSMP_SHIFT    EQU  4
ADC_CFG1_ADIV_MASK       EQU  0x60
ADC_CFG1_ADIV_SHIFT      EQU  5
ADC_CFG1_ADLPC_MASK      EQU  0x80
ADC_CFG1_ADLPC_SHIFT     EQU  7

; CFG2 Bit Fields
ADC_CFG2_ADLSTS_MASK     EQU  0x3
ADC_CFG2_ADLSTS_SHIFT    EQU  0
ADC_CFG2_ADHSC_MASK      EQU  0x4
ADC_CFG2_ADHSC_SHIFT     EQU  2
ADC_CFG2_ADACKEN_MASK    EQU  0x8
ADC_CFG2_ADACKEN_SHIFT   EQU  3
ADC_CFG2_MUXSEL_MASK     EQU  0x10
ADC_CFG2_MUXSEL_SHIFT    EQU  4

; R Bit Fields
ADC_R_D_MASK             EQU  0xFFFF
ADC_R_D_SHIFT            EQU  0

; CV1 Bit Fields
ADC_CV1_CV_MASK          EQU  0xFFFF
ADC_CV1_CV_SHIFT         EQU  0

; CV2 Bit Fields
ADC_CV2_CV_MASK          EQU  0xFFFF
ADC_CV2_CV_SHIFT         EQU  0

; SC2 Bit Fields
ADC_SC2_REFSEL_MASK      EQU  0x3
ADC_SC2_REFSEL_SHIFT     EQU  0
ADC_SC2_DMAEN_MASK       EQU  0x4
ADC_SC2_DMAEN_SHIFT      EQU  2
ADC_SC2_ACREN_MASK       EQU  0x8
ADC_SC2_ACREN_SHIFT      EQU  3
ADC_SC2_ACFGT_MASK       EQU  0x10
ADC_SC2_ACFGT_SHIFT      EQU  4
ADC_SC2_ACFE_MASK        EQU  0x20
ADC_SC2_ACFE_SHIFT       EQU  5
ADC_SC2_ADTRG_MASK       EQU  0x40
ADC_SC2_ADTRG_SHIFT      EQU  6
ADC_SC2_ADACT_MASK       EQU  0x80
ADC_SC2_ADACT_SHIFT      EQU  7

; SC3 Bit Fields
ADC_SC3_AVGS_MASK        EQU  0x3
ADC_SC3_AVGS_SHIFT       EQU  0
ADC_SC3_AVGE_MASK        EQU  0x4
ADC_SC3_AVGE_SHIFT       EQU  2
ADC_SC3_ADCO_MASK        EQU  0x8
ADC_SC3_ADCO_SHIFT       EQU  3
ADC_SC3_CALF_MASK        EQU  0x40
ADC_SC3_CALF_SHIFT       EQU  6
ADC_SC3_CAL_MASK         EQU  0x80
ADC_SC3_CAL_SHIFT        EQU  7

; OFS Bit Fields
ADC_OFS_OFS_MASK         EQU  0xFFFF
ADC_OFS_OFS_SHIFT        EQU  0

; PG Bit Fields
ADC_PG_PG_MASK           EQU  0xFFFF
ADC_PG_PG_SHIFT          EQU  0

; MG Bit Fields
ADC_MG_MG_MASK           EQU  0xFFFF
ADC_MG_MG_SHIFT          EQU  0

; CLPD Bit Fields
ADC_CLPD_CLPD_MASK       EQU  0x3F
ADC_CLPD_CLPD_SHIFT      EQU  0

; CLPS Bit Fields
ADC_CLPS_CLPS_MASK       EQU  0x3F
ADC_CLPS_CLPS_SHIFT      EQU  0

; CLP4 Bit Fields
ADC_CLP4_CLP4_MASK       EQU  0x3FF
ADC_CLP4_CLP4_SHIFT      EQU  0

; CLP3 Bit Fields
ADC_CLP3_CLP3_MASK       EQU  0x1FF
ADC_CLP3_CLP3_SHIFT      EQU  0

; CLP2 Bit Fields
ADC_CLP2_CLP2_MASK       EQU  0xFF
ADC_CLP2_CLP2_SHIFT      EQU  0

; CLP1 Bit Fields
ADC_CLP1_CLP1_MASK       EQU  0x7F
ADC_CLP1_CLP1_SHIFT      EQU  0

; CLP0 Bit Fields
ADC_CLP0_CLP0_MASK       EQU  0x3F
ADC_CLP0_CLP0_SHIFT      EQU  0

; CLMD Bit Fields
ADC_CLMD_CLMD_MASK       EQU  0x3F
ADC_CLMD_CLMD_SHIFT      EQU  0

; CLMS Bit Fields
ADC_CLMS_CLMS_MASK       EQU  0x3F
ADC_CLMS_CLMS_SHIFT      EQU  0

; CLM4 Bit Fields
ADC_CLM4_CLM4_MASK       EQU  0x3FF
ADC_CLM4_CLM4_SHIFT      EQU  0

; CLM3 Bit Fields
ADC_CLM3_CLM3_MASK       EQU  0x1FF
ADC_CLM3_CLM3_SHIFT      EQU  0

; CLM2 Bit Fields
ADC_CLM2_CLM2_MASK       EQU  0xFF
ADC_CLM2_CLM2_SHIFT      EQU  0

; CLM1 Bit Fields
ADC_CLM1_CLM1_MASK       EQU  0x7F
ADC_CLM1_CLM1_SHIFT      EQU  0

; CLM0 Bit Fields
ADC_CLM0_CLM0_MASK       EQU  0x3F
ADC_CLM0_CLM0_SHIFT      EQU  0

; ----------------------------------------------------------------------------
; -- AIPS Peripheral Access Layer
; ----------------------------------------------------------------------------

AIPS_MPRA    EQU  0x00  ; Master Privilege Register A, offset: 0x0
AIPS_PACRA   EQU  0x20  ; Peripheral Access Control Register, offset: 0x20
AIPS_PACRB   EQU  0x24  ; Peripheral Access Control Register, offset: 0x24
AIPS_PACRC   EQU  0x28  ; Peripheral Access Control Register, offset: 0x28
AIPS_PACRD   EQU  0x2C  ; Peripheral Access Control Register, offset: 0x2C
AIPS_PACRE   EQU  0x40  ; Peripheral Access Control Register, offset: 0x40
AIPS_PACRF   EQU  0x44  ; Peripheral Access Control Register, offset: 0x44
AIPS_PACRG   EQU  0x48  ; Peripheral Access Control Register, offset: 0x48
AIPS_PACRH   EQU  0x4C  ; Peripheral Access Control Register, offset: 0x4C
AIPS_PACRI   EQU  0x50  ; Peripheral Access Control Register, offset: 0x50
AIPS_PACRJ   EQU  0x54  ; Peripheral Access Control Register, offset: 0x54
AIPS_PACRK   EQU  0x58  ; Peripheral Access Control Register, offset: 0x58
AIPS_PACRL   EQU  0x5C  ; Peripheral Access Control Register, offset: 0x5C
AIPS_PACRM   EQU  0x60  ; Peripheral Access Control Register, offset: 0x60
AIPS_PACRN   EQU  0x64  ; Peripheral Access Control Register, offset: 0x64
AIPS_PACRO   EQU  0x68  ; Peripheral Access Control Register, offset: 0x68
AIPS_PACRP   EQU  0x6C  ; Peripheral Access Control Register, offset: 0x6C
AIPS_PACRU   EQU  0x80  ; Peripheral Access Control Register, offset: 0x80

; ----------------------------------------------------------------------------
; -- AIPS Register Masks
; ----------------------------------------------------------------------------

; MPRA Bit Fields
AIPS_MPRA_MPL5_MASK      EQU  0x100
AIPS_MPRA_MPL5_SHIFT     EQU  8
AIPS_MPRA_MTW5_MASK      EQU  0x200
AIPS_MPRA_MTW5_SHIFT     EQU  9
AIPS_MPRA_MTR5_MASK      EQU  0x400
AIPS_MPRA_MTR5_SHIFT     EQU  10
AIPS_MPRA_MPL4_MASK      EQU  0x1000
AIPS_MPRA_MPL4_SHIFT     EQU  12
AIPS_MPRA_MTW4_MASK      EQU  0x2000
AIPS_MPRA_MTW4_SHIFT     EQU  13
AIPS_MPRA_MTR4_MASK      EQU  0x4000
AIPS_MPRA_MTR4_SHIFT     EQU  14
AIPS_MPRA_MPL3_MASK      EQU  0x10000
AIPS_MPRA_MPL3_SHIFT     EQU  16
AIPS_MPRA_MTW3_MASK      EQU  0x20000
AIPS_MPRA_MTW3_SHIFT     EQU  17
AIPS_MPRA_MTR3_MASK      EQU  0x40000
AIPS_MPRA_MTR3_SHIFT     EQU  18
AIPS_MPRA_MPL2_MASK      EQU  0x100000
AIPS_MPRA_MPL2_SHIFT     EQU  20
AIPS_MPRA_MTW2_MASK      EQU  0x200000
AIPS_MPRA_MTW2_SHIFT     EQU  21
AIPS_MPRA_MTR2_MASK      EQU  0x400000
AIPS_MPRA_MTR2_SHIFT     EQU  22
AIPS_MPRA_MPL1_MASK      EQU  0x1000000
AIPS_MPRA_MPL1_SHIFT     EQU  24
AIPS_MPRA_MTW1_MASK      EQU  0x2000000
AIPS_MPRA_MTW1_SHIFT     EQU  25
AIPS_MPRA_MTR1_MASK      EQU  0x4000000
AIPS_MPRA_MTR1_SHIFT     EQU  26
AIPS_MPRA_MPL0_MASK      EQU  0x10000000
AIPS_MPRA_MPL0_SHIFT     EQU  28
AIPS_MPRA_MTW0_MASK      EQU  0x20000000
AIPS_MPRA_MTW0_SHIFT     EQU  29
AIPS_MPRA_MTR0_MASK      EQU  0x40000000
AIPS_MPRA_MTR0_SHIFT     EQU  30

; PACRA Bit Fields
AIPS_PACRA_TP7_MASK      EQU  0x1
AIPS_PACRA_TP7_SHIFT     EQU  0
AIPS_PACRA_WP7_MASK      EQU  0x2
AIPS_PACRA_WP7_SHIFT     EQU  1
AIPS_PACRA_SP7_MASK      EQU  0x4
AIPS_PACRA_SP7_SHIFT     EQU  2
AIPS_PACRA_TP6_MASK      EQU  0x10
AIPS_PACRA_TP6_SHIFT     EQU  4
AIPS_PACRA_WP6_MASK      EQU  0x20
AIPS_PACRA_WP6_SHIFT     EQU  5
AIPS_PACRA_SP6_MASK      EQU  0x40
AIPS_PACRA_SP6_SHIFT     EQU  6
AIPS_PACRA_TP5_MASK      EQU  0x100
AIPS_PACRA_TP5_SHIFT     EQU  8
AIPS_PACRA_WP5_MASK      EQU  0x200
AIPS_PACRA_WP5_SHIFT     EQU  9
AIPS_PACRA_SP5_MASK      EQU  0x400
AIPS_PACRA_SP5_SHIFT     EQU  10
AIPS_PACRA_TP4_MASK      EQU  0x1000
AIPS_PACRA_TP4_SHIFT     EQU  12
AIPS_PACRA_WP4_MASK      EQU  0x2000
AIPS_PACRA_WP4_SHIFT     EQU  13
AIPS_PACRA_SP4_MASK      EQU  0x4000
AIPS_PACRA_SP4_SHIFT     EQU  14
AIPS_PACRA_TP3_MASK      EQU  0x10000
AIPS_PACRA_TP3_SHIFT     EQU  16
AIPS_PACRA_WP3_MASK      EQU  0x20000
AIPS_PACRA_WP3_SHIFT     EQU  17
AIPS_PACRA_SP3_MASK      EQU  0x40000
AIPS_PACRA_SP3_SHIFT     EQU  18
AIPS_PACRA_TP2_MASK      EQU  0x100000
AIPS_PACRA_TP2_SHIFT     EQU  20
AIPS_PACRA_WP2_MASK      EQU  0x200000
AIPS_PACRA_WP2_SHIFT     EQU  21
AIPS_PACRA_SP2_MASK      EQU  0x400000
AIPS_PACRA_SP2_SHIFT     EQU  22
AIPS_PACRA_TP1_MASK      EQU  0x1000000
AIPS_PACRA_TP1_SHIFT     EQU  24
AIPS_PACRA_WP1_MASK      EQU  0x2000000
AIPS_PACRA_WP1_SHIFT     EQU  25
AIPS_PACRA_SP1_MASK      EQU  0x4000000
AIPS_PACRA_SP1_SHIFT     EQU  26
AIPS_PACRA_TP0_MASK      EQU  0x10000000
AIPS_PACRA_TP0_SHIFT     EQU  28
AIPS_PACRA_WP0_MASK      EQU  0x20000000
AIPS_PACRA_WP0_SHIFT     EQU  29
AIPS_PACRA_SP0_MASK      EQU  0x40000000
AIPS_PACRA_SP0_SHIFT     EQU  30

; PACRB Bit Fields
AIPS_PACRB_TP7_MASK      EQU  0x1
AIPS_PACRB_TP7_SHIFT     EQU  0
AIPS_PACRB_WP7_MASK      EQU  0x2
AIPS_PACRB_WP7_SHIFT     EQU  1
AIPS_PACRB_SP7_MASK      EQU  0x4
AIPS_PACRB_SP7_SHIFT     EQU  2
AIPS_PACRB_TP6_MASK      EQU  0x10
AIPS_PACRB_TP6_SHIFT     EQU  4
AIPS_PACRB_WP6_MASK      EQU  0x20
AIPS_PACRB_WP6_SHIFT     EQU  5
AIPS_PACRB_SP6_MASK      EQU  0x40
AIPS_PACRB_SP6_SHIFT     EQU  6
AIPS_PACRB_TP5_MASK      EQU  0x100
AIPS_PACRB_TP5_SHIFT     EQU  8
AIPS_PACRB_WP5_MASK      EQU  0x200
AIPS_PACRB_WP5_SHIFT     EQU  9
AIPS_PACRB_SP5_MASK      EQU  0x400
AIPS_PACRB_SP5_SHIFT     EQU  10
AIPS_PACRB_TP4_MASK      EQU  0x1000
AIPS_PACRB_TP4_SHIFT     EQU  12
AIPS_PACRB_WP4_MASK      EQU  0x2000
AIPS_PACRB_WP4_SHIFT     EQU  13
AIPS_PACRB_SP4_MASK      EQU  0x4000
AIPS_PACRB_SP4_SHIFT     EQU  14
AIPS_PACRB_TP3_MASK      EQU  0x10000
AIPS_PACRB_TP3_SHIFT     EQU  16
AIPS_PACRB_WP3_MASK      EQU  0x20000
AIPS_PACRB_WP3_SHIFT     EQU  17
AIPS_PACRB_SP3_MASK      EQU  0x40000
AIPS_PACRB_SP3_SHIFT     EQU  18
AIPS_PACRB_TP2_MASK      EQU  0x100000
AIPS_PACRB_TP2_SHIFT     EQU  20
AIPS_PACRB_WP2_MASK      EQU  0x200000
AIPS_PACRB_WP2_SHIFT     EQU  21
AIPS_PACRB_SP2_MASK      EQU  0x400000
AIPS_PACRB_SP2_SHIFT     EQU  22
AIPS_PACRB_TP1_MASK      EQU  0x1000000
AIPS_PACRB_TP1_SHIFT     EQU  24
AIPS_PACRB_WP1_MASK      EQU  0x2000000
AIPS_PACRB_WP1_SHIFT     EQU  25
AIPS_PACRB_SP1_MASK      EQU  0x4000000
AIPS_PACRB_SP1_SHIFT     EQU  26
AIPS_PACRB_TP0_MASK      EQU  0x10000000
AIPS_PACRB_TP0_SHIFT     EQU  28
AIPS_PACRB_WP0_MASK      EQU  0x20000000
AIPS_PACRB_WP0_SHIFT     EQU  29
AIPS_PACRB_SP0_MASK      EQU  0x40000000
AIPS_PACRB_SP0_SHIFT     EQU  30

; PACRC Bit Fields
AIPS_PACRC_TP7_MASK      EQU  0x1
AIPS_PACRC_TP7_SHIFT     EQU  0
AIPS_PACRC_WP7_MASK      EQU  0x2
AIPS_PACRC_WP7_SHIFT     EQU  1
AIPS_PACRC_SP7_MASK      EQU  0x4
AIPS_PACRC_SP7_SHIFT     EQU  2
AIPS_PACRC_TP6_MASK      EQU  0x10
AIPS_PACRC_TP6_SHIFT     EQU  4
AIPS_PACRC_WP6_MASK      EQU  0x20
AIPS_PACRC_WP6_SHIFT     EQU  5
AIPS_PACRC_SP6_MASK      EQU  0x40
AIPS_PACRC_SP6_SHIFT     EQU  6
AIPS_PACRC_TP5_MASK      EQU  0x100
AIPS_PACRC_TP5_SHIFT     EQU  8
AIPS_PACRC_WP5_MASK      EQU  0x200
AIPS_PACRC_WP5_SHIFT     EQU  9
AIPS_PACRC_SP5_MASK      EQU  0x400
AIPS_PACRC_SP5_SHIFT     EQU  10
AIPS_PACRC_TP4_MASK      EQU  0x1000
AIPS_PACRC_TP4_SHIFT     EQU  12
AIPS_PACRC_WP4_MASK      EQU  0x2000
AIPS_PACRC_WP4_SHIFT     EQU  13
AIPS_PACRC_SP4_MASK      EQU  0x4000
AIPS_PACRC_SP4_SHIFT     EQU  14
AIPS_PACRC_TP3_MASK      EQU  0x10000
AIPS_PACRC_TP3_SHIFT     EQU  16
AIPS_PACRC_WP3_MASK      EQU  0x20000
AIPS_PACRC_WP3_SHIFT     EQU  17
AIPS_PACRC_SP3_MASK      EQU  0x40000
AIPS_PACRC_SP3_SHIFT     EQU  18
AIPS_PACRC_TP2_MASK      EQU  0x100000
AIPS_PACRC_TP2_SHIFT     EQU  20
AIPS_PACRC_WP2_MASK      EQU  0x200000
AIPS_PACRC_WP2_SHIFT     EQU  21
AIPS_PACRC_SP2_MASK      EQU  0x400000
AIPS_PACRC_SP2_SHIFT     EQU  22
AIPS_PACRC_TP1_MASK      EQU  0x1000000
AIPS_PACRC_TP1_SHIFT     EQU  24
AIPS_PACRC_WP1_MASK      EQU  0x2000000
AIPS_PACRC_WP1_SHIFT     EQU  25
AIPS_PACRC_SP1_MASK      EQU  0x4000000
AIPS_PACRC_SP1_SHIFT     EQU  26
AIPS_PACRC_TP0_MASK      EQU  0x10000000
AIPS_PACRC_TP0_SHIFT     EQU  28
AIPS_PACRC_WP0_MASK      EQU  0x20000000
AIPS_PACRC_WP0_SHIFT     EQU  29
AIPS_PACRC_SP0_MASK      EQU  0x40000000
AIPS_PACRC_SP0_SHIFT     EQU  30

; PACRD Bit Fields
AIPS_PACRD_TP7_MASK      EQU  0x1
AIPS_PACRD_TP7_SHIFT     EQU  0
AIPS_PACRD_WP7_MASK      EQU  0x2
AIPS_PACRD_WP7_SHIFT     EQU  1
AIPS_PACRD_SP7_MASK      EQU  0x4
AIPS_PACRD_SP7_SHIFT     EQU  2
AIPS_PACRD_TP6_MASK      EQU  0x10
AIPS_PACRD_TP6_SHIFT     EQU  4
AIPS_PACRD_WP6_MASK      EQU  0x20
AIPS_PACRD_WP6_SHIFT     EQU  5
AIPS_PACRD_SP6_MASK      EQU  0x40
AIPS_PACRD_SP6_SHIFT     EQU  6
AIPS_PACRD_TP5_MASK      EQU  0x100
AIPS_PACRD_TP5_SHIFT     EQU  8
AIPS_PACRD_WP5_MASK      EQU  0x200
AIPS_PACRD_WP5_SHIFT     EQU  9
AIPS_PACRD_SP5_MASK      EQU  0x400
AIPS_PACRD_SP5_SHIFT     EQU  10
AIPS_PACRD_TP4_MASK      EQU  0x1000
AIPS_PACRD_TP4_SHIFT     EQU  12
AIPS_PACRD_WP4_MASK      EQU  0x2000
AIPS_PACRD_WP4_SHIFT     EQU  13
AIPS_PACRD_SP4_MASK      EQU  0x4000
AIPS_PACRD_SP4_SHIFT     EQU  14
AIPS_PACRD_TP3_MASK      EQU  0x10000
AIPS_PACRD_TP3_SHIFT     EQU  16
AIPS_PACRD_WP3_MASK      EQU  0x20000
AIPS_PACRD_WP3_SHIFT     EQU  17
AIPS_PACRD_SP3_MASK      EQU  0x40000
AIPS_PACRD_SP3_SHIFT     EQU  18
AIPS_PACRD_TP2_MASK      EQU  0x100000
AIPS_PACRD_TP2_SHIFT     EQU  20
AIPS_PACRD_WP2_MASK      EQU  0x200000
AIPS_PACRD_WP2_SHIFT     EQU  21
AIPS_PACRD_SP2_MASK      EQU  0x400000
AIPS_PACRD_SP2_SHIFT     EQU  22
AIPS_PACRD_TP1_MASK      EQU  0x1000000
AIPS_PACRD_TP1_SHIFT     EQU  24
AIPS_PACRD_WP1_MASK      EQU  0x2000000
AIPS_PACRD_WP1_SHIFT     EQU  25
AIPS_PACRD_SP1_MASK      EQU  0x4000000
AIPS_PACRD_SP1_SHIFT     EQU  26
AIPS_PACRD_TP0_MASK      EQU  0x10000000
AIPS_PACRD_TP0_SHIFT     EQU  28
AIPS_PACRD_WP0_MASK      EQU  0x20000000
AIPS_PACRD_WP0_SHIFT     EQU  29
AIPS_PACRD_SP0_MASK      EQU  0x40000000
AIPS_PACRD_SP0_SHIFT     EQU  30

; PACRE Bit Fields
AIPS_PACRE_TP7_MASK      EQU  0x1
AIPS_PACRE_TP7_SHIFT     EQU  0
AIPS_PACRE_WP7_MASK      EQU  0x2
AIPS_PACRE_WP7_SHIFT     EQU  1
AIPS_PACRE_SP7_MASK      EQU  0x4
AIPS_PACRE_SP7_SHIFT     EQU  2
AIPS_PACRE_TP6_MASK      EQU  0x10
AIPS_PACRE_TP6_SHIFT     EQU  4
AIPS_PACRE_WP6_MASK      EQU  0x20
AIPS_PACRE_WP6_SHIFT     EQU  5
AIPS_PACRE_SP6_MASK      EQU  0x40
AIPS_PACRE_SP6_SHIFT     EQU  6
AIPS_PACRE_TP5_MASK      EQU  0x100
AIPS_PACRE_TP5_SHIFT     EQU  8
AIPS_PACRE_WP5_MASK      EQU  0x200
AIPS_PACRE_WP5_SHIFT     EQU  9
AIPS_PACRE_SP5_MASK      EQU  0x400
AIPS_PACRE_SP5_SHIFT     EQU  10
AIPS_PACRE_TP4_MASK      EQU  0x1000
AIPS_PACRE_TP4_SHIFT     EQU  12
AIPS_PACRE_WP4_MASK      EQU  0x2000
AIPS_PACRE_WP4_SHIFT     EQU  13
AIPS_PACRE_SP4_MASK      EQU  0x4000
AIPS_PACRE_SP4_SHIFT     EQU  14
AIPS_PACRE_TP3_MASK      EQU  0x10000
AIPS_PACRE_TP3_SHIFT     EQU  16
AIPS_PACRE_WP3_MASK      EQU  0x20000
AIPS_PACRE_WP3_SHIFT     EQU  17
AIPS_PACRE_SP3_MASK      EQU  0x40000
AIPS_PACRE_SP3_SHIFT     EQU  18
AIPS_PACRE_TP2_MASK      EQU  0x100000
AIPS_PACRE_TP2_SHIFT     EQU  20
AIPS_PACRE_WP2_MASK      EQU  0x200000
AIPS_PACRE_WP2_SHIFT     EQU  21
AIPS_PACRE_SP2_MASK      EQU  0x400000
AIPS_PACRE_SP2_SHIFT     EQU  22
AIPS_PACRE_TP1_MASK      EQU  0x1000000
AIPS_PACRE_TP1_SHIFT     EQU  24
AIPS_PACRE_WP1_MASK      EQU  0x2000000
AIPS_PACRE_WP1_SHIFT     EQU  25
AIPS_PACRE_SP1_MASK      EQU  0x4000000
AIPS_PACRE_SP1_SHIFT     EQU  26
AIPS_PACRE_TP0_MASK      EQU  0x10000000
AIPS_PACRE_TP0_SHIFT     EQU  28
AIPS_PACRE_WP0_MASK      EQU  0x20000000
AIPS_PACRE_WP0_SHIFT     EQU  29
AIPS_PACRE_SP0_MASK      EQU  0x40000000
AIPS_PACRE_SP0_SHIFT     EQU  30

; PACRF Bit Fields
AIPS_PACRF_TP7_MASK      EQU  0x1
AIPS_PACRF_TP7_SHIFT     EQU  0
AIPS_PACRF_WP7_MASK      EQU  0x2
AIPS_PACRF_WP7_SHIFT     EQU  1
AIPS_PACRF_SP7_MASK      EQU  0x4
AIPS_PACRF_SP7_SHIFT     EQU  2
AIPS_PACRF_TP6_MASK      EQU  0x10
AIPS_PACRF_TP6_SHIFT     EQU  4
AIPS_PACRF_WP6_MASK      EQU  0x20
AIPS_PACRF_WP6_SHIFT     EQU  5
AIPS_PACRF_SP6_MASK      EQU  0x40
AIPS_PACRF_SP6_SHIFT     EQU  6
AIPS_PACRF_TP5_MASK      EQU  0x100
AIPS_PACRF_TP5_SHIFT     EQU  8
AIPS_PACRF_WP5_MASK      EQU  0x200
AIPS_PACRF_WP5_SHIFT     EQU  9
AIPS_PACRF_SP5_MASK      EQU  0x400
AIPS_PACRF_SP5_SHIFT     EQU  10
AIPS_PACRF_TP4_MASK      EQU  0x1000
AIPS_PACRF_TP4_SHIFT     EQU  12
AIPS_PACRF_WP4_MASK      EQU  0x2000
AIPS_PACRF_WP4_SHIFT     EQU  13
AIPS_PACRF_SP4_MASK      EQU  0x4000
AIPS_PACRF_SP4_SHIFT     EQU  14
AIPS_PACRF_TP3_MASK      EQU  0x10000
AIPS_PACRF_TP3_SHIFT     EQU  16
AIPS_PACRF_WP3_MASK      EQU  0x20000
AIPS_PACRF_WP3_SHIFT     EQU  17
AIPS_PACRF_SP3_MASK      EQU  0x40000
AIPS_PACRF_SP3_SHIFT     EQU  18
AIPS_PACRF_TP2_MASK      EQU  0x100000
AIPS_PACRF_TP2_SHIFT     EQU  20
AIPS_PACRF_WP2_MASK      EQU  0x200000
AIPS_PACRF_WP2_SHIFT     EQU  21
AIPS_PACRF_SP2_MASK      EQU  0x400000
AIPS_PACRF_SP2_SHIFT     EQU  22
AIPS_PACRF_TP1_MASK      EQU  0x1000000
AIPS_PACRF_TP1_SHIFT     EQU  24
AIPS_PACRF_WP1_MASK      EQU  0x2000000
AIPS_PACRF_WP1_SHIFT     EQU  25
AIPS_PACRF_SP1_MASK      EQU  0x4000000
AIPS_PACRF_SP1_SHIFT     EQU  26
AIPS_PACRF_TP0_MASK      EQU  0x10000000
AIPS_PACRF_TP0_SHIFT     EQU  28
AIPS_PACRF_WP0_MASK      EQU  0x20000000
AIPS_PACRF_WP0_SHIFT     EQU  29
AIPS_PACRF_SP0_MASK      EQU  0x40000000
AIPS_PACRF_SP0_SHIFT     EQU  30

; PACRG Bit Fields
AIPS_PACRG_TP7_MASK      EQU  0x1
AIPS_PACRG_TP7_SHIFT     EQU  0
AIPS_PACRG_WP7_MASK      EQU  0x2
AIPS_PACRG_WP7_SHIFT     EQU  1
AIPS_PACRG_SP7_MASK      EQU  0x4
AIPS_PACRG_SP7_SHIFT     EQU  2
AIPS_PACRG_TP6_MASK      EQU  0x10
AIPS_PACRG_TP6_SHIFT     EQU  4
AIPS_PACRG_WP6_MASK      EQU  0x20
AIPS_PACRG_WP6_SHIFT     EQU  5
AIPS_PACRG_SP6_MASK      EQU  0x40
AIPS_PACRG_SP6_SHIFT     EQU  6
AIPS_PACRG_TP5_MASK      EQU  0x100
AIPS_PACRG_TP5_SHIFT     EQU  8
AIPS_PACRG_WP5_MASK      EQU  0x200
AIPS_PACRG_WP5_SHIFT     EQU  9
AIPS_PACRG_SP5_MASK      EQU  0x400
AIPS_PACRG_SP5_SHIFT     EQU  10
AIPS_PACRG_TP4_MASK      EQU  0x1000
AIPS_PACRG_TP4_SHIFT     EQU  12
AIPS_PACRG_WP4_MASK      EQU  0x2000
AIPS_PACRG_WP4_SHIFT     EQU  13
AIPS_PACRG_SP4_MASK      EQU  0x4000
AIPS_PACRG_SP4_SHIFT     EQU  14
AIPS_PACRG_TP3_MASK      EQU  0x10000
AIPS_PACRG_TP3_SHIFT     EQU  16
AIPS_PACRG_WP3_MASK      EQU  0x20000
AIPS_PACRG_WP3_SHIFT     EQU  17
AIPS_PACRG_SP3_MASK      EQU  0x40000
AIPS_PACRG_SP3_SHIFT     EQU  18
AIPS_PACRG_TP2_MASK      EQU  0x100000
AIPS_PACRG_TP2_SHIFT     EQU  20
AIPS_PACRG_WP2_MASK      EQU  0x200000
AIPS_PACRG_WP2_SHIFT     EQU  21
AIPS_PACRG_SP2_MASK      EQU  0x400000
AIPS_PACRG_SP2_SHIFT     EQU  22
AIPS_PACRG_TP1_MASK      EQU  0x1000000
AIPS_PACRG_TP1_SHIFT     EQU  24
AIPS_PACRG_WP1_MASK      EQU  0x2000000
AIPS_PACRG_WP1_SHIFT     EQU  25
AIPS_PACRG_SP1_MASK      EQU  0x4000000
AIPS_PACRG_SP1_SHIFT     EQU  26
AIPS_PACRG_TP0_MASK      EQU  0x10000000
AIPS_PACRG_TP0_SHIFT     EQU  28
AIPS_PACRG_WP0_MASK      EQU  0x20000000
AIPS_PACRG_WP0_SHIFT     EQU  29
AIPS_PACRG_SP0_MASK      EQU  0x40000000
AIPS_PACRG_SP0_SHIFT     EQU  30

; PACRH Bit Fields
AIPS_PACRH_TP7_MASK      EQU  0x1
AIPS_PACRH_TP7_SHIFT     EQU  0
AIPS_PACRH_WP7_MASK      EQU  0x2
AIPS_PACRH_WP7_SHIFT     EQU  1
AIPS_PACRH_SP7_MASK      EQU  0x4
AIPS_PACRH_SP7_SHIFT     EQU  2
AIPS_PACRH_TP6_MASK      EQU  0x10
AIPS_PACRH_TP6_SHIFT     EQU  4
AIPS_PACRH_WP6_MASK      EQU  0x20
AIPS_PACRH_WP6_SHIFT     EQU  5
AIPS_PACRH_SP6_MASK      EQU  0x40
AIPS_PACRH_SP6_SHIFT     EQU  6
AIPS_PACRH_TP5_MASK      EQU  0x100
AIPS_PACRH_TP5_SHIFT     EQU  8
AIPS_PACRH_WP5_MASK      EQU  0x200
AIPS_PACRH_WP5_SHIFT     EQU  9
AIPS_PACRH_SP5_MASK      EQU  0x400
AIPS_PACRH_SP5_SHIFT     EQU  10
AIPS_PACRH_TP4_MASK      EQU  0x1000
AIPS_PACRH_TP4_SHIFT     EQU  12
AIPS_PACRH_WP4_MASK      EQU  0x2000
AIPS_PACRH_WP4_SHIFT     EQU  13
AIPS_PACRH_SP4_MASK      EQU  0x4000
AIPS_PACRH_SP4_SHIFT     EQU  14
AIPS_PACRH_TP3_MASK      EQU  0x10000
AIPS_PACRH_TP3_SHIFT     EQU  16
AIPS_PACRH_WP3_MASK      EQU  0x20000
AIPS_PACRH_WP3_SHIFT     EQU  17
AIPS_PACRH_SP3_MASK      EQU  0x40000
AIPS_PACRH_SP3_SHIFT     EQU  18
AIPS_PACRH_TP2_MASK      EQU  0x100000
AIPS_PACRH_TP2_SHIFT     EQU  20
AIPS_PACRH_WP2_MASK      EQU  0x200000
AIPS_PACRH_WP2_SHIFT     EQU  21
AIPS_PACRH_SP2_MASK      EQU  0x400000
AIPS_PACRH_SP2_SHIFT     EQU  22
AIPS_PACRH_TP1_MASK      EQU  0x1000000
AIPS_PACRH_TP1_SHIFT     EQU  24
AIPS_PACRH_WP1_MASK      EQU  0x2000000
AIPS_PACRH_WP1_SHIFT     EQU  25
AIPS_PACRH_SP1_MASK      EQU  0x4000000
AIPS_PACRH_SP1_SHIFT     EQU  26
AIPS_PACRH_TP0_MASK      EQU  0x10000000
AIPS_PACRH_TP0_SHIFT     EQU  28
AIPS_PACRH_WP0_MASK      EQU  0x20000000
AIPS_PACRH_WP0_SHIFT     EQU  29
AIPS_PACRH_SP0_MASK      EQU  0x40000000
AIPS_PACRH_SP0_SHIFT     EQU  30

; PACRI Bit Fields
AIPS_PACRI_TP7_MASK      EQU  0x1
AIPS_PACRI_TP7_SHIFT     EQU  0
AIPS_PACRI_WP7_MASK      EQU  0x2
AIPS_PACRI_WP7_SHIFT     EQU  1
AIPS_PACRI_SP7_MASK      EQU  0x4
AIPS_PACRI_SP7_SHIFT     EQU  2
AIPS_PACRI_TP6_MASK      EQU  0x10
AIPS_PACRI_TP6_SHIFT     EQU  4
AIPS_PACRI_WP6_MASK      EQU  0x20
AIPS_PACRI_WP6_SHIFT     EQU  5
AIPS_PACRI_SP6_MASK      EQU  0x40
AIPS_PACRI_SP6_SHIFT     EQU  6
AIPS_PACRI_TP5_MASK      EQU  0x100
AIPS_PACRI_TP5_SHIFT     EQU  8
AIPS_PACRI_WP5_MASK      EQU  0x200
AIPS_PACRI_WP5_SHIFT     EQU  9
AIPS_PACRI_SP5_MASK      EQU  0x400
AIPS_PACRI_SP5_SHIFT     EQU  10
AIPS_PACRI_TP4_MASK      EQU  0x1000
AIPS_PACRI_TP4_SHIFT     EQU  12
AIPS_PACRI_WP4_MASK      EQU  0x2000
AIPS_PACRI_WP4_SHIFT     EQU  13
AIPS_PACRI_SP4_MASK      EQU  0x4000
AIPS_PACRI_SP4_SHIFT     EQU  14
AIPS_PACRI_TP3_MASK      EQU  0x10000
AIPS_PACRI_TP3_SHIFT     EQU  16
AIPS_PACRI_WP3_MASK      EQU  0x20000
AIPS_PACRI_WP3_SHIFT     EQU  17
AIPS_PACRI_SP3_MASK      EQU  0x40000
AIPS_PACRI_SP3_SHIFT     EQU  18
AIPS_PACRI_TP2_MASK      EQU  0x100000
AIPS_PACRI_TP2_SHIFT     EQU  20
AIPS_PACRI_WP2_MASK      EQU  0x200000
AIPS_PACRI_WP2_SHIFT     EQU  21
AIPS_PACRI_SP2_MASK      EQU  0x400000
AIPS_PACRI_SP2_SHIFT     EQU  22
AIPS_PACRI_TP1_MASK      EQU  0x1000000
AIPS_PACRI_TP1_SHIFT     EQU  24
AIPS_PACRI_WP1_MASK      EQU  0x2000000
AIPS_PACRI_WP1_SHIFT     EQU  25
AIPS_PACRI_SP1_MASK      EQU  0x4000000
AIPS_PACRI_SP1_SHIFT     EQU  26
AIPS_PACRI_TP0_MASK      EQU  0x10000000
AIPS_PACRI_TP0_SHIFT     EQU  28
AIPS_PACRI_WP0_MASK      EQU  0x20000000
AIPS_PACRI_WP0_SHIFT     EQU  29
AIPS_PACRI_SP0_MASK      EQU  0x40000000
AIPS_PACRI_SP0_SHIFT     EQU  30

; PACRJ Bit Fields
AIPS_PACRJ_TP7_MASK      EQU  0x1
AIPS_PACRJ_TP7_SHIFT     EQU  0
AIPS_PACRJ_WP7_MASK      EQU  0x2
AIPS_PACRJ_WP7_SHIFT     EQU  1
AIPS_PACRJ_SP7_MASK      EQU  0x4
AIPS_PACRJ_SP7_SHIFT     EQU  2
AIPS_PACRJ_TP6_MASK      EQU  0x10
AIPS_PACRJ_TP6_SHIFT     EQU  4
AIPS_PACRJ_WP6_MASK      EQU  0x20
AIPS_PACRJ_WP6_SHIFT     EQU  5
AIPS_PACRJ_SP6_MASK      EQU  0x40
AIPS_PACRJ_SP6_SHIFT     EQU  6
AIPS_PACRJ_TP5_MASK      EQU  0x100
AIPS_PACRJ_TP5_SHIFT     EQU  8
AIPS_PACRJ_WP5_MASK      EQU  0x200
AIPS_PACRJ_WP5_SHIFT     EQU  9
AIPS_PACRJ_SP5_MASK      EQU  0x400
AIPS_PACRJ_SP5_SHIFT     EQU  10
AIPS_PACRJ_TP4_MASK      EQU  0x1000
AIPS_PACRJ_TP4_SHIFT     EQU  12
AIPS_PACRJ_WP4_MASK      EQU  0x2000
AIPS_PACRJ_WP4_SHIFT     EQU  13
AIPS_PACRJ_SP4_MASK      EQU  0x4000
AIPS_PACRJ_SP4_SHIFT     EQU  14
AIPS_PACRJ_TP3_MASK      EQU  0x10000
AIPS_PACRJ_TP3_SHIFT     EQU  16
AIPS_PACRJ_WP3_MASK      EQU  0x20000
AIPS_PACRJ_WP3_SHIFT     EQU  17
AIPS_PACRJ_SP3_MASK      EQU  0x40000
AIPS_PACRJ_SP3_SHIFT     EQU  18
AIPS_PACRJ_TP2_MASK      EQU  0x100000
AIPS_PACRJ_TP2_SHIFT     EQU  20
AIPS_PACRJ_WP2_MASK      EQU  0x200000
AIPS_PACRJ_WP2_SHIFT     EQU  21
AIPS_PACRJ_SP2_MASK      EQU  0x400000
AIPS_PACRJ_SP2_SHIFT     EQU  22
AIPS_PACRJ_TP1_MASK      EQU  0x1000000
AIPS_PACRJ_TP1_SHIFT     EQU  24
AIPS_PACRJ_WP1_MASK      EQU  0x2000000
AIPS_PACRJ_WP1_SHIFT     EQU  25
AIPS_PACRJ_SP1_MASK      EQU  0x4000000
AIPS_PACRJ_SP1_SHIFT     EQU  26
AIPS_PACRJ_TP0_MASK      EQU  0x10000000
AIPS_PACRJ_TP0_SHIFT     EQU  28
AIPS_PACRJ_WP0_MASK      EQU  0x20000000
AIPS_PACRJ_WP0_SHIFT     EQU  29
AIPS_PACRJ_SP0_MASK      EQU  0x40000000
AIPS_PACRJ_SP0_SHIFT     EQU  30

; PACRK Bit Fields
AIPS_PACRK_TP7_MASK      EQU  0x1
AIPS_PACRK_TP7_SHIFT     EQU  0
AIPS_PACRK_WP7_MASK      EQU  0x2
AIPS_PACRK_WP7_SHIFT     EQU  1
AIPS_PACRK_SP7_MASK      EQU  0x4
AIPS_PACRK_SP7_SHIFT     EQU  2
AIPS_PACRK_TP6_MASK      EQU  0x10
AIPS_PACRK_TP6_SHIFT     EQU  4
AIPS_PACRK_WP6_MASK      EQU  0x20
AIPS_PACRK_WP6_SHIFT     EQU  5
AIPS_PACRK_SP6_MASK      EQU  0x40
AIPS_PACRK_SP6_SHIFT     EQU  6
AIPS_PACRK_TP5_MASK      EQU  0x100
AIPS_PACRK_TP5_SHIFT     EQU  8
AIPS_PACRK_WP5_MASK      EQU  0x200
AIPS_PACRK_WP5_SHIFT     EQU  9
AIPS_PACRK_SP5_MASK      EQU  0x400
AIPS_PACRK_SP5_SHIFT     EQU  10
AIPS_PACRK_TP4_MASK      EQU  0x1000
AIPS_PACRK_TP4_SHIFT     EQU  12
AIPS_PACRK_WP4_MASK      EQU  0x2000
AIPS_PACRK_WP4_SHIFT     EQU  13
AIPS_PACRK_SP4_MASK      EQU  0x4000
AIPS_PACRK_SP4_SHIFT     EQU  14
AIPS_PACRK_TP3_MASK      EQU  0x10000
AIPS_PACRK_TP3_SHIFT     EQU  16
AIPS_PACRK_WP3_MASK      EQU  0x20000
AIPS_PACRK_WP3_SHIFT     EQU  17
AIPS_PACRK_SP3_MASK      EQU  0x40000
AIPS_PACRK_SP3_SHIFT     EQU  18
AIPS_PACRK_TP2_MASK      EQU  0x100000
AIPS_PACRK_TP2_SHIFT     EQU  20
AIPS_PACRK_WP2_MASK      EQU  0x200000
AIPS_PACRK_WP2_SHIFT     EQU  21
AIPS_PACRK_SP2_MASK      EQU  0x400000
AIPS_PACRK_SP2_SHIFT     EQU  22
AIPS_PACRK_TP1_MASK      EQU  0x1000000
AIPS_PACRK_TP1_SHIFT     EQU  24
AIPS_PACRK_WP1_MASK      EQU  0x2000000
AIPS_PACRK_WP1_SHIFT     EQU  25
AIPS_PACRK_SP1_MASK      EQU  0x4000000
AIPS_PACRK_SP1_SHIFT     EQU  26
AIPS_PACRK_TP0_MASK      EQU  0x10000000
AIPS_PACRK_TP0_SHIFT     EQU  28
AIPS_PACRK_WP0_MASK      EQU  0x20000000
AIPS_PACRK_WP0_SHIFT     EQU  29
AIPS_PACRK_SP0_MASK      EQU  0x40000000
AIPS_PACRK_SP0_SHIFT     EQU  30

; PACRL Bit Fields
AIPS_PACRL_TP7_MASK      EQU  0x1
AIPS_PACRL_TP7_SHIFT     EQU  0
AIPS_PACRL_WP7_MASK      EQU  0x2
AIPS_PACRL_WP7_SHIFT     EQU  1
AIPS_PACRL_SP7_MASK      EQU  0x4
AIPS_PACRL_SP7_SHIFT     EQU  2
AIPS_PACRL_TP6_MASK      EQU  0x10
AIPS_PACRL_TP6_SHIFT     EQU  4
AIPS_PACRL_WP6_MASK      EQU  0x20
AIPS_PACRL_WP6_SHIFT     EQU  5
AIPS_PACRL_SP6_MASK      EQU  0x40
AIPS_PACRL_SP6_SHIFT     EQU  6
AIPS_PACRL_TP5_MASK      EQU  0x100
AIPS_PACRL_TP5_SHIFT     EQU  8
AIPS_PACRL_WP5_MASK      EQU  0x200
AIPS_PACRL_WP5_SHIFT     EQU  9
AIPS_PACRL_SP5_MASK      EQU  0x400
AIPS_PACRL_SP5_SHIFT     EQU  10
AIPS_PACRL_TP4_MASK      EQU  0x1000
AIPS_PACRL_TP4_SHIFT     EQU  12
AIPS_PACRL_WP4_MASK      EQU  0x2000
AIPS_PACRL_WP4_SHIFT     EQU  13
AIPS_PACRL_SP4_MASK      EQU  0x4000
AIPS_PACRL_SP4_SHIFT     EQU  14
AIPS_PACRL_TP3_MASK      EQU  0x10000
AIPS_PACRL_TP3_SHIFT     EQU  16
AIPS_PACRL_WP3_MASK      EQU  0x20000
AIPS_PACRL_WP3_SHIFT     EQU  17
AIPS_PACRL_SP3_MASK      EQU  0x40000
AIPS_PACRL_SP3_SHIFT     EQU  18
AIPS_PACRL_TP2_MASK      EQU  0x100000
AIPS_PACRL_TP2_SHIFT     EQU  20
AIPS_PACRL_WP2_MASK      EQU  0x200000
AIPS_PACRL_WP2_SHIFT     EQU  21
AIPS_PACRL_SP2_MASK      EQU  0x400000
AIPS_PACRL_SP2_SHIFT     EQU  22
AIPS_PACRL_TP1_MASK      EQU  0x1000000
AIPS_PACRL_TP1_SHIFT     EQU  24
AIPS_PACRL_WP1_MASK      EQU  0x2000000
AIPS_PACRL_WP1_SHIFT     EQU  25
AIPS_PACRL_SP1_MASK      EQU  0x4000000
AIPS_PACRL_SP1_SHIFT     EQU  26
AIPS_PACRL_TP0_MASK      EQU  0x10000000
AIPS_PACRL_TP0_SHIFT     EQU  28
AIPS_PACRL_WP0_MASK      EQU  0x20000000
AIPS_PACRL_WP0_SHIFT     EQU  29
AIPS_PACRL_SP0_MASK      EQU  0x40000000
AIPS_PACRL_SP0_SHIFT     EQU  30

; PACRM Bit Fields
AIPS_PACRM_TP7_MASK      EQU  0x1
AIPS_PACRM_TP7_SHIFT     EQU  0
AIPS_PACRM_WP7_MASK      EQU  0x2
AIPS_PACRM_WP7_SHIFT     EQU  1
AIPS_PACRM_SP7_MASK      EQU  0x4
AIPS_PACRM_SP7_SHIFT     EQU  2
AIPS_PACRM_TP6_MASK      EQU  0x10
AIPS_PACRM_TP6_SHIFT     EQU  4
AIPS_PACRM_WP6_MASK      EQU  0x20
AIPS_PACRM_WP6_SHIFT     EQU  5
AIPS_PACRM_SP6_MASK      EQU  0x40
AIPS_PACRM_SP6_SHIFT     EQU  6
AIPS_PACRM_TP5_MASK      EQU  0x100
AIPS_PACRM_TP5_SHIFT     EQU  8
AIPS_PACRM_WP5_MASK      EQU  0x200
AIPS_PACRM_WP5_SHIFT     EQU  9
AIPS_PACRM_SP5_MASK      EQU  0x400
AIPS_PACRM_SP5_SHIFT     EQU  10
AIPS_PACRM_TP4_MASK      EQU  0x1000
AIPS_PACRM_TP4_SHIFT     EQU  12
AIPS_PACRM_WP4_MASK      EQU  0x2000
AIPS_PACRM_WP4_SHIFT     EQU  13
AIPS_PACRM_SP4_MASK      EQU  0x4000
AIPS_PACRM_SP4_SHIFT     EQU  14
AIPS_PACRM_TP3_MASK      EQU  0x10000
AIPS_PACRM_TP3_SHIFT     EQU  16
AIPS_PACRM_WP3_MASK      EQU  0x20000
AIPS_PACRM_WP3_SHIFT     EQU  17
AIPS_PACRM_SP3_MASK      EQU  0x40000
AIPS_PACRM_SP3_SHIFT     EQU  18
AIPS_PACRM_TP2_MASK      EQU  0x100000
AIPS_PACRM_TP2_SHIFT     EQU  20
AIPS_PACRM_WP2_MASK      EQU  0x200000
AIPS_PACRM_WP2_SHIFT     EQU  21
AIPS_PACRM_SP2_MASK      EQU  0x400000
AIPS_PACRM_SP2_SHIFT     EQU  22
AIPS_PACRM_TP1_MASK      EQU  0x1000000
AIPS_PACRM_TP1_SHIFT     EQU  24
AIPS_PACRM_WP1_MASK      EQU  0x2000000
AIPS_PACRM_WP1_SHIFT     EQU  25
AIPS_PACRM_SP1_MASK      EQU  0x4000000
AIPS_PACRM_SP1_SHIFT     EQU  26
AIPS_PACRM_TP0_MASK      EQU  0x10000000
AIPS_PACRM_TP0_SHIFT     EQU  28
AIPS_PACRM_WP0_MASK      EQU  0x20000000
AIPS_PACRM_WP0_SHIFT     EQU  29
AIPS_PACRM_SP0_MASK      EQU  0x40000000
AIPS_PACRM_SP0_SHIFT     EQU  30

; PACRN Bit Fields
AIPS_PACRN_TP7_MASK      EQU  0x1
AIPS_PACRN_TP7_SHIFT     EQU  0
AIPS_PACRN_WP7_MASK      EQU  0x2
AIPS_PACRN_WP7_SHIFT     EQU  1
AIPS_PACRN_SP7_MASK      EQU  0x4
AIPS_PACRN_SP7_SHIFT     EQU  2
AIPS_PACRN_TP6_MASK      EQU  0x10
AIPS_PACRN_TP6_SHIFT     EQU  4
AIPS_PACRN_WP6_MASK      EQU  0x20
AIPS_PACRN_WP6_SHIFT     EQU  5
AIPS_PACRN_SP6_MASK      EQU  0x40
AIPS_PACRN_SP6_SHIFT     EQU  6
AIPS_PACRN_TP5_MASK      EQU  0x100
AIPS_PACRN_TP5_SHIFT     EQU  8
AIPS_PACRN_WP5_MASK      EQU  0x200
AIPS_PACRN_WP5_SHIFT     EQU  9
AIPS_PACRN_SP5_MASK      EQU  0x400
AIPS_PACRN_SP5_SHIFT     EQU  10
AIPS_PACRN_TP4_MASK      EQU  0x1000
AIPS_PACRN_TP4_SHIFT     EQU  12
AIPS_PACRN_WP4_MASK      EQU  0x2000
AIPS_PACRN_WP4_SHIFT     EQU  13
AIPS_PACRN_SP4_MASK      EQU  0x4000
AIPS_PACRN_SP4_SHIFT     EQU  14
AIPS_PACRN_TP3_MASK      EQU  0x10000
AIPS_PACRN_TP3_SHIFT     EQU  16
AIPS_PACRN_WP3_MASK      EQU  0x20000
AIPS_PACRN_WP3_SHIFT     EQU  17
AIPS_PACRN_SP3_MASK      EQU  0x40000
AIPS_PACRN_SP3_SHIFT     EQU  18
AIPS_PACRN_TP2_MASK      EQU  0x100000
AIPS_PACRN_TP2_SHIFT     EQU  20
AIPS_PACRN_WP2_MASK      EQU  0x200000
AIPS_PACRN_WP2_SHIFT     EQU  21
AIPS_PACRN_SP2_MASK      EQU  0x400000
AIPS_PACRN_SP2_SHIFT     EQU  22
AIPS_PACRN_TP1_MASK      EQU  0x1000000
AIPS_PACRN_TP1_SHIFT     EQU  24
AIPS_PACRN_WP1_MASK      EQU  0x2000000
AIPS_PACRN_WP1_SHIFT     EQU  25
AIPS_PACRN_SP1_MASK      EQU  0x4000000
AIPS_PACRN_SP1_SHIFT     EQU  26
AIPS_PACRN_TP0_MASK      EQU  0x10000000
AIPS_PACRN_TP0_SHIFT     EQU  28
AIPS_PACRN_WP0_MASK      EQU  0x20000000
AIPS_PACRN_WP0_SHIFT     EQU  29
AIPS_PACRN_SP0_MASK      EQU  0x40000000
AIPS_PACRN_SP0_SHIFT     EQU  30

; PACRO Bit Fields
AIPS_PACRO_TP7_MASK      EQU  0x1
AIPS_PACRO_TP7_SHIFT     EQU  0
AIPS_PACRO_WP7_MASK      EQU  0x2
AIPS_PACRO_WP7_SHIFT     EQU  1
AIPS_PACRO_SP7_MASK      EQU  0x4
AIPS_PACRO_SP7_SHIFT     EQU  2
AIPS_PACRO_TP6_MASK      EQU  0x10
AIPS_PACRO_TP6_SHIFT     EQU  4
AIPS_PACRO_WP6_MASK      EQU  0x20
AIPS_PACRO_WP6_SHIFT     EQU  5
AIPS_PACRO_SP6_MASK      EQU  0x40
AIPS_PACRO_SP6_SHIFT     EQU  6
AIPS_PACRO_TP5_MASK      EQU  0x100
AIPS_PACRO_TP5_SHIFT     EQU  8
AIPS_PACRO_WP5_MASK      EQU  0x200
AIPS_PACRO_WP5_SHIFT     EQU  9
AIPS_PACRO_SP5_MASK      EQU  0x400
AIPS_PACRO_SP5_SHIFT     EQU  10
AIPS_PACRO_TP4_MASK      EQU  0x1000
AIPS_PACRO_TP4_SHIFT     EQU  12
AIPS_PACRO_WP4_MASK      EQU  0x2000
AIPS_PACRO_WP4_SHIFT     EQU  13
AIPS_PACRO_SP4_MASK      EQU  0x4000
AIPS_PACRO_SP4_SHIFT     EQU  14
AIPS_PACRO_TP3_MASK      EQU  0x10000
AIPS_PACRO_TP3_SHIFT     EQU  16
AIPS_PACRO_WP3_MASK      EQU  0x20000
AIPS_PACRO_WP3_SHIFT     EQU  17
AIPS_PACRO_SP3_MASK      EQU  0x40000
AIPS_PACRO_SP3_SHIFT     EQU  18
AIPS_PACRO_TP2_MASK      EQU  0x100000
AIPS_PACRO_TP2_SHIFT     EQU  20
AIPS_PACRO_WP2_MASK      EQU  0x200000
AIPS_PACRO_WP2_SHIFT     EQU  21
AIPS_PACRO_SP2_MASK      EQU  0x400000
AIPS_PACRO_SP2_SHIFT     EQU  22
AIPS_PACRO_TP1_MASK      EQU  0x1000000
AIPS_PACRO_TP1_SHIFT     EQU  24
AIPS_PACRO_WP1_MASK      EQU  0x2000000
AIPS_PACRO_WP1_SHIFT     EQU  25
AIPS_PACRO_SP1_MASK      EQU  0x4000000
AIPS_PACRO_SP1_SHIFT     EQU  26
AIPS_PACRO_TP0_MASK      EQU  0x10000000
AIPS_PACRO_TP0_SHIFT     EQU  28
AIPS_PACRO_WP0_MASK      EQU  0x20000000
AIPS_PACRO_WP0_SHIFT     EQU  29
AIPS_PACRO_SP0_MASK      EQU  0x40000000
AIPS_PACRO_SP0_SHIFT     EQU  30

; PACRP Bit Fields
AIPS_PACRP_TP7_MASK      EQU  0x1
AIPS_PACRP_TP7_SHIFT     EQU  0
AIPS_PACRP_WP7_MASK      EQU  0x2
AIPS_PACRP_WP7_SHIFT     EQU  1
AIPS_PACRP_SP7_MASK      EQU  0x4
AIPS_PACRP_SP7_SHIFT     EQU  2
AIPS_PACRP_TP6_MASK      EQU  0x10
AIPS_PACRP_TP6_SHIFT     EQU  4
AIPS_PACRP_WP6_MASK      EQU  0x20
AIPS_PACRP_WP6_SHIFT     EQU  5
AIPS_PACRP_SP6_MASK      EQU  0x40
AIPS_PACRP_SP6_SHIFT     EQU  6
AIPS_PACRP_TP5_MASK      EQU  0x100
AIPS_PACRP_TP5_SHIFT     EQU  8
AIPS_PACRP_WP5_MASK      EQU  0x200
AIPS_PACRP_WP5_SHIFT     EQU  9
AIPS_PACRP_SP5_MASK      EQU  0x400
AIPS_PACRP_SP5_SHIFT     EQU  10
AIPS_PACRP_TP4_MASK      EQU  0x1000
AIPS_PACRP_TP4_SHIFT     EQU  12
AIPS_PACRP_WP4_MASK      EQU  0x2000
AIPS_PACRP_WP4_SHIFT     EQU  13
AIPS_PACRP_SP4_MASK      EQU  0x4000
AIPS_PACRP_SP4_SHIFT     EQU  14
AIPS_PACRP_TP3_MASK      EQU  0x10000
AIPS_PACRP_TP3_SHIFT     EQU  16
AIPS_PACRP_WP3_MASK      EQU  0x20000
AIPS_PACRP_WP3_SHIFT     EQU  17
AIPS_PACRP_SP3_MASK      EQU  0x40000
AIPS_PACRP_SP3_SHIFT     EQU  18
AIPS_PACRP_TP2_MASK      EQU  0x100000
AIPS_PACRP_TP2_SHIFT     EQU  20
AIPS_PACRP_WP2_MASK      EQU  0x200000
AIPS_PACRP_WP2_SHIFT     EQU  21
AIPS_PACRP_SP2_MASK      EQU  0x400000
AIPS_PACRP_SP2_SHIFT     EQU  22
AIPS_PACRP_TP1_MASK      EQU  0x1000000
AIPS_PACRP_TP1_SHIFT     EQU  24
AIPS_PACRP_WP1_MASK      EQU  0x2000000
AIPS_PACRP_WP1_SHIFT     EQU  25
AIPS_PACRP_SP1_MASK      EQU  0x4000000
AIPS_PACRP_SP1_SHIFT     EQU  26
AIPS_PACRP_TP0_MASK      EQU  0x10000000
AIPS_PACRP_TP0_SHIFT     EQU  28
AIPS_PACRP_WP0_MASK      EQU  0x20000000
AIPS_PACRP_WP0_SHIFT     EQU  29
AIPS_PACRP_SP0_MASK      EQU  0x40000000
AIPS_PACRP_SP0_SHIFT     EQU  30

; PACRU Bit Fields
AIPS_PACRU_TP1_MASK      EQU  0x1000000
AIPS_PACRU_TP1_SHIFT     EQU  24
AIPS_PACRU_WP1_MASK      EQU  0x2000000
AIPS_PACRU_WP1_SHIFT     EQU  25
AIPS_PACRU_SP1_MASK      EQU  0x4000000
AIPS_PACRU_SP1_SHIFT     EQU  26
AIPS_PACRU_TP0_MASK      EQU  0x10000000
AIPS_PACRU_TP0_SHIFT     EQU  28
AIPS_PACRU_WP0_MASK      EQU  0x20000000
AIPS_PACRU_WP0_SHIFT     EQU  29
AIPS_PACRU_SP0_MASK      EQU  0x40000000
AIPS_PACRU_SP0_SHIFT     EQU  30

; ----------------------------------------------------------------------------
; -- AXBS Peripheral Access Layer
; ----------------------------------------------------------------------------

AXBS_SLAVE0_PRS   EQU  0x000   ; Priority Registers Slave, array offset: 0x0, array step: 0x100
AXBS_SLAVE0_CRS   EQU  0x010   ; Control Register, array offset: 0x10, array step: 0x100

AXBS_SLAVE1_PRS   EQU  0x100   ; Priority Registers Slave 1
AXBS_SLAVE1_CRS   EQU  0x110   ;

AXBS_SLAVE2_PRS   EQU  0x200   ; Priority Registers Slave 2
AXBS_SLAVE2_CRS   EQU  0x210   ;

AXBS_SLAVE3_PRS   EQU  0x300   ; Priority Registers Slave 3
AXBS_SLAVE3_CRS   EQU  0x310   ;

AXBS_SLAVE4_PRS   EQU  0x400   ; Priority Registers Slave 4
AXBS_SLAVE4_CRS   EQU  0x410   ;

AXBS_MGPCR0       EQU  0x800   ; Master General Purpose Control Register, offset: 0x800
AXBS_MGPCR1       EQU  0x800   ; Master General Purpose Control Register, offset: 0x900
AXBS_MGPCR2       EQU  0xA00   ; Master General Purpose Control Register, offset: 0xA00
AXBS_MGPCR3       EQU  0xB00   ; Master General Purpose Control Register, offset: 0xB00
AXBS_MGPCR4       EQU  0xC00   ; Master General Purpose Control Register, offset: 0xC00
AXBS_MGPCR5       EQU  0xD00   ; Master General Purpose Control Register, offset: 0xD00

; ----------------------------------------------------------------------------
; -- AXBS Register Masks
; ----------------------------------------------------------------------------

; PRS Bit Fields
AXBS_PRS_M0_MASK         EQU  0x7
AXBS_PRS_M0_SHIFT        EQU  0
AXBS_PRS_M1_MASK         EQU  0x70
AXBS_PRS_M1_SHIFT        EQU  4
AXBS_PRS_M2_MASK         EQU  0x700
AXBS_PRS_M2_SHIFT        EQU  8
AXBS_PRS_M3_MASK         EQU  0x7000
AXBS_PRS_M3_SHIFT        EQU  12
AXBS_PRS_M4_MASK         EQU  0x70000
AXBS_PRS_M4_SHIFT        EQU  16
AXBS_PRS_M5_MASK         EQU  0x700000
AXBS_PRS_M5_SHIFT        EQU  20

; CRS Bit Fields
AXBS_CRS_PARK_MASK       EQU  0x7
AXBS_CRS_PARK_SHIFT      EQU  0
AXBS_CRS_PCTL_MASK       EQU  0x30
AXBS_CRS_PCTL_SHIFT      EQU  4
AXBS_CRS_ARB_MASK        EQU  0x300
AXBS_CRS_ARB_SHIFT       EQU  8
AXBS_CRS_HLP_MASK        EQU  0x40000000
AXBS_CRS_HLP_SHIFT       EQU  30
AXBS_CRS_RO_MASK         EQU  0x80000000
AXBS_CRS_RO_SHIFT        EQU  31

; MGPCR0 Bit Fields
AXBS_MGPCR0_AULB_MASK    EQU  0x7
AXBS_MGPCR0_AULB_SHIFT   EQU  0

; MGPCR1 Bit Fields
AXBS_MGPCR1_AULB_MASK    EQU  0x7
AXBS_MGPCR1_AULB_SHIFT   EQU  0

; MGPCR2 Bit Fields
AXBS_MGPCR2_AULB_MASK    EQU  0x7
AXBS_MGPCR2_AULB_SHIFT   EQU  0

; MGPCR3 Bit Fields
AXBS_MGPCR3_AULB_MASK    EQU  0x7
AXBS_MGPCR3_AULB_SHIFT   EQU  0

; MGPCR4 Bit Fields
AXBS_MGPCR4_AULB_MASK    EQU  0x7
AXBS_MGPCR4_AULB_SHIFT   EQU  0

; MGPCR5 Bit Fields
AXBS_MGPCR5_AULB_MASK    EQU  0x7
AXBS_MGPCR5_AULB_SHIFT   EQU  0

; ----------------------------------------------------------------------------
; -- CAN Peripheral Access Layer
; ----------------------------------------------------------------------------

CAN_MCR         EQU  0x000   ; Module Configuration Register, offset: 0x0
CAN_CTRL1       EQU  0x004   ; Control 1 register, offset: 0x4
CAN_TIMER       EQU  0x008   ; Free Running Timer, offset: 0x8
CAN_RXMGMASK    EQU  0x010   ; Rx Mailboxes Global Mask Register, offset: 0x10
CAN_RX14MASK    EQU  0x014   ; Rx 14 Mask register, offset: 0x14
CAN_RX15MASK    EQU  0x018   ; Rx 15 Mask register, offset: 0x18
CAN_ECR         EQU  0x01C   ; Error Counter, offset: 0x1C
CAN_ESR1        EQU  0x020   ; Error and Status 1 register, offset: 0x20
CAN_IMASK1      EQU  0x028   ; Interrupt Masks 1 register, offset: 0x28
CAN_IFLAG1      EQU  0x030   ; Interrupt Flags 1 register, offset: 0x30
CAN_CTRL2       EQU  0x034   ; Control 2 register, offset: 0x34
CAN_ESR2        EQU  0x038   ; Error and Status 2 register, offset: 0x38
CAN_CRCR        EQU  0x044   ; CRC Register, offset: 0x44
CAN_RXFGMASK    EQU  0x048   ; Rx FIFO Global Mask register, offset: 0x48
CAN_RXFIR       EQU  0x04C   ; Rx FIFO Information Register, offset: 0x4C

CAN_MB0_CS      EQU  0x080   ; Message Buffer 0 CS Register..Message Buffer 15 CS Register, array offset: 0x80, array step: 0x10
CAN_MB0_ID      EQU  0x084   ; Message Buffer 0 ID Register..Message Buffer 15 ID Register, array offset: 0x84, array step: 0x10
CAN_MB0_WORD0   EQU  0x088   ; Message Buffer 0 WORD0 Register..Message Buffer 15 WORD0 Register, array offset: 0x88, array step: 0x10
CAN_MB0_WORD1   EQU  0x08C   ; Message Buffer 0 WORD1 Register..Message Buffer 15 WORD1 Register, array offset: 0x8C, array step: 0x10

CAN_MB1_CS      EQU  0x090   ; Message Buffer 1
CAN_MB1_ID      EQU  0x094   ; Message Buffer 1
CAN_MB1_WORD0   EQU  0x098   ; Message Buffer 1
CAN_MB1_WORD1   EQU  0x09C   ; Message Buffer 1

CAN_MB2_CS      EQU  0x0A0   ; Message Buffer 2
CAN_MB2_ID      EQU  0x0A4   ; Message Buffer 2
CAN_MB2_WORD0   EQU  0x0A8   ; Message Buffer 2
CAN_MB2_WORD1   EQU  0x0AC   ; Message Buffer 2

CAN_MB3_CS      EQU  0x0B0   ; Message Buffer 3
CAN_MB3_ID      EQU  0x0B4   ; Message Buffer 3
CAN_MB3_WORD0   EQU  0x0B8   ; Message Buffer 3
CAN_MB3_WORD1   EQU  0x0BC   ; Message Buffer 3

CAN_MB4_CS      EQU  0x0C0   ; Message Buffer 4
CAN_MB4_ID      EQU  0x0C4   ; Message Buffer 4
CAN_MB4_WORD0   EQU  0x0C8   ; Message Buffer 4
CAN_MB4_WORD1   EQU  0x0CC   ; Message Buffer 4

CAN_MB5_CS      EQU  0x0D0   ; Message Buffer 5
CAN_MB5_ID      EQU  0x0D4   ; Message Buffer 5
CAN_MB5_WORD0   EQU  0x0D8   ; Message Buffer 5
CAN_MB5_WORD1   EQU  0x0DC   ; Message Buffer 5

CAN_MB6_CS      EQU  0x0E0   ; Message Buffer 6
CAN_MB6_ID      EQU  0x0E4   ; Message Buffer 6
CAN_MB6_WORD0   EQU  0x0E8   ; Message Buffer 6
CAN_MB6_WORD1   EQU  0x0EC   ; Message Buffer 6

CAN_MB7_CS      EQU  0x0F0   ; Message Buffer 7
CAN_MB7_ID      EQU  0x0F4   ; Message Buffer 7
CAN_MB7_WORD0   EQU  0x0F8   ; Message Buffer 7
CAN_MB7_WORD1   EQU  0x0FC   ; Message Buffer 7

CAN_MB8_CS      EQU  0x100   ; Message Buffer 8
CAN_MB8_ID      EQU  0x104   ; Message Buffer 8
CAN_MB8_WORD0   EQU  0x108   ; Message Buffer 8
CAN_MB8_WORD1   EQU  0x10C   ; Message Buffer 8

CAN_MB9_CS      EQU  0x110   ; Message Buffer 9
CAN_MB9_ID      EQU  0x114   ; Message Buffer 9
CAN_MB9_WORD0   EQU  0x118   ; Message Buffer 9
CAN_MB9_WORD1   EQU  0x11C   ; Message Buffer 9

CAN_MB10_CS     EQU  0x120   ; Message Buffer 10
CAN_MB10_ID     EQU  0x124   ; Message Buffer 10
CAN_MB10_WORD0  EQU  0x128   ; Message Buffer 10
CAN_MB10_WORD1  EQU  0x12C   ; Message Buffer 10

CAN_MB11_CS     EQU  0x130   ; Message Buffer 11
CAN_MB11_ID     EQU  0x134   ; Message Buffer 11
CAN_MB11_WORD0  EQU  0x138   ; Message Buffer 11
CAN_MB11_WORD1  EQU  0x13C   ; Message Buffer 11

CAN_MB12_CS     EQU  0x140   ; Message Buffer 12
CAN_MB12_ID     EQU  0x144   ; Message Buffer 12
CAN_MB12_WORD0  EQU  0x148   ; Message Buffer 12
CAN_MB12_WORD1  EQU  0x14C   ; Message Buffer 12

CAN_MB13_CS     EQU  0x150   ; Message Buffer 13
CAN_MB13_ID     EQU  0x154   ; Message Buffer 13
CAN_MB13_WORD0  EQU  0x158   ; Message Buffer 13
CAN_MB13_WORD1  EQU  0x15C   ; Message Buffer 13

CAN_MB14_CS     EQU  0x160   ; Message Buffer 14
CAN_MB14_ID     EQU  0x164   ; Message Buffer 14
CAN_MB14_WORD0  EQU  0x168   ; Message Buffer 14
CAN_MB14_WORD1  EQU  0x16C   ; Message Buffer 14

CAN_MB15_CS     EQU  0x170   ; Message Buffer 15
CAN_MB15_ID     EQU  0x174   ; Message Buffer 15
CAN_MB15_WORD0  EQU  0x178   ; Message Buffer 15
CAN_MB15_WORD1  EQU  0x17C   ; Message Buffer 15

CAN_RXIMR0      EQU  0x880   ; Rx Individual Mask Registers, array offset: 0x880, array step: 0x4
CAN_RXIMR1      EQU  0x884   ;
CAN_RXIMR2      EQU  0x888   ;
CAN_RXIMR3      EQU  0x88C   ;
CAN_RXIMR4      EQU  0x890   ;
CAN_RXIMR5      EQU  0x894   ;
CAN_RXIMR6      EQU  0x898   ;
CAN_RXIMR7      EQU  0x89C   ;
CAN_RXIMR8      EQU  0x8A0   ;
CAN_RXIMR9      EQU  0x8A4   ;
CAN_RXIMR10     EQU  0x8A8   ;
CAN_RXIMR11     EQU  0x8AC   ;
CAN_RXIMR12     EQU  0x8B0   ;
CAN_RXIMR13     EQU  0x8B4   ;
CAN_RXIMR14     EQU  0x8B8   ;
CAN_RXIMR15     EQU  0x8BC   ;

; ----------------------------------------------------------------------------
; -- CAN Register Masks
; ----------------------------------------------------------------------------

; MCR Bit Fields
CAN_MCR_MAXMB_MASK       EQU  0x7F
CAN_MCR_MAXMB_SHIFT      EQU  0
CAN_MCR_IDAM_MASK        EQU  0x300
CAN_MCR_IDAM_SHIFT       EQU  8
CAN_MCR_AEN_MASK         EQU  0x1000
CAN_MCR_AEN_SHIFT        EQU  12
CAN_MCR_LPRIOEN_MASK     EQU  0x2000
CAN_MCR_LPRIOEN_SHIFT    EQU  13
CAN_MCR_IRMQ_MASK        EQU  0x10000
CAN_MCR_IRMQ_SHIFT       EQU  16
CAN_MCR_SRXDIS_MASK      EQU  0x20000
CAN_MCR_SRXDIS_SHIFT     EQU  17
CAN_MCR_WAKSRC_MASK      EQU  0x80000
CAN_MCR_WAKSRC_SHIFT     EQU  19
CAN_MCR_LPMACK_MASK      EQU  0x100000
CAN_MCR_LPMACK_SHIFT     EQU  20
CAN_MCR_WRNEN_MASK       EQU  0x200000
CAN_MCR_WRNEN_SHIFT      EQU  21
CAN_MCR_SLFWAK_MASK      EQU  0x400000
CAN_MCR_SLFWAK_SHIFT     EQU  22
CAN_MCR_SUPV_MASK        EQU  0x800000
CAN_MCR_SUPV_SHIFT       EQU  23
CAN_MCR_FRZACK_MASK      EQU  0x1000000
CAN_MCR_FRZACK_SHIFT     EQU  24
CAN_MCR_SOFTRST_MASK     EQU  0x2000000
CAN_MCR_SOFTRST_SHIFT    EQU  25
CAN_MCR_WAKMSK_MASK      EQU  0x4000000
CAN_MCR_WAKMSK_SHIFT     EQU  26
CAN_MCR_NOTRDY_MASK      EQU  0x8000000
CAN_MCR_NOTRDY_SHIFT     EQU  27
CAN_MCR_HALT_MASK        EQU  0x10000000
CAN_MCR_HALT_SHIFT       EQU  28
CAN_MCR_RFEN_MASK        EQU  0x20000000
CAN_MCR_RFEN_SHIFT       EQU  29
CAN_MCR_FRZ_MASK         EQU  0x40000000
CAN_MCR_FRZ_SHIFT        EQU  30
CAN_MCR_MDIS_MASK        EQU  0x80000000
CAN_MCR_MDIS_SHIFT       EQU  31

; CTRL1 Bit Fields
CAN_CTRL1_PROPSEG_MASK   EQU  0x7
CAN_CTRL1_PROPSEG_SHIFT  EQU  0
CAN_CTRL1_LOM_MASK       EQU  0x8
CAN_CTRL1_LOM_SHIFT      EQU  3
CAN_CTRL1_LBUF_MASK      EQU  0x10
CAN_CTRL1_LBUF_SHIFT     EQU  4
CAN_CTRL1_TSYN_MASK      EQU  0x20
CAN_CTRL1_TSYN_SHIFT     EQU  5
CAN_CTRL1_BOFFREC_MASK   EQU  0x40
CAN_CTRL1_BOFFREC_SHIFT  EQU  6
CAN_CTRL1_SMP_MASK       EQU  0x80
CAN_CTRL1_SMP_SHIFT      EQU  7
CAN_CTRL1_RWRNMSK_MASK   EQU  0x400
CAN_CTRL1_RWRNMSK_SHIFT  EQU  10
CAN_CTRL1_TWRNMSK_MASK   EQU  0x800
CAN_CTRL1_TWRNMSK_SHIFT  EQU  11
CAN_CTRL1_LPB_MASK       EQU  0x1000
CAN_CTRL1_LPB_SHIFT      EQU  12
CAN_CTRL1_CLKSRC_MASK    EQU  0x2000
CAN_CTRL1_CLKSRC_SHIFT   EQU  13
CAN_CTRL1_ERRMSK_MASK    EQU  0x4000
CAN_CTRL1_ERRMSK_SHIFT   EQU  14
CAN_CTRL1_BOFFMSK_MASK   EQU  0x8000
CAN_CTRL1_BOFFMSK_SHIFT  EQU  15
CAN_CTRL1_PSEG2_MASK     EQU  0x70000
CAN_CTRL1_PSEG2_SHIFT    EQU  16
CAN_CTRL1_PSEG1_MASK     EQU  0x380000
CAN_CTRL1_PSEG1_SHIFT    EQU  19
CAN_CTRL1_RJW_MASK       EQU  0xC00000
CAN_CTRL1_RJW_SHIFT      EQU  22
CAN_CTRL1_PRESDIV_MASK   EQU  0xFF000000
CAN_CTRL1_PRESDIV_SHIFT  EQU  24

; TIMER Bit Fields
CAN_TIMER_TIMER_MASK     EQU  0xFFFF
CAN_TIMER_TIMER_SHIFT    EQU  0

; RXMGMASK Bit Fields
CAN_RXMGMASK_MG_MASK     EQU  0xFFFFFFFF
CAN_RXMGMASK_MG_SHIFT    EQU  0

; RX14MASK Bit Fields
CAN_RX14MASK_RX14M_MASK  EQU  0xFFFFFFFF
CAN_RX14MASK_RX14M_SHIFT EQU  0

; RX15MASK Bit Fields
CAN_RX15MASK_RX15M_MASK  EQU  0xFFFFFFFF
CAN_RX15MASK_RX15M_SHIFT EQU  0

; ECR Bit Fields
CAN_ECR_TXERRCNT_MASK    EQU  0xFF
CAN_ECR_TXERRCNT_SHIFT   EQU  0
CAN_ECR_RXERRCNT_MASK    EQU  0xFF00
CAN_ECR_RXERRCNT_SHIFT   EQU  8

; ESR1 Bit Fields
CAN_ESR1_WAKINT_MASK     EQU  0x1
CAN_ESR1_WAKINT_SHIFT    EQU  0
CAN_ESR1_ERRINT_MASK     EQU  0x2
CAN_ESR1_ERRINT_SHIFT    EQU  1
CAN_ESR1_BOFFINT_MASK    EQU  0x4
CAN_ESR1_BOFFINT_SHIFT   EQU  2
CAN_ESR1_RX_MASK         EQU  0x8
CAN_ESR1_RX_SHIFT        EQU  3
CAN_ESR1_FLTCONF_MASK    EQU  0x30
CAN_ESR1_FLTCONF_SHIFT   EQU  4
CAN_ESR1_TX_MASK         EQU  0x40
CAN_ESR1_TX_SHIFT        EQU  6
CAN_ESR1_IDLE_MASK       EQU  0x80
CAN_ESR1_IDLE_SHIFT      EQU  7
CAN_ESR1_RXWRN_MASK      EQU  0x100
CAN_ESR1_RXWRN_SHIFT     EQU  8
CAN_ESR1_TXWRN_MASK      EQU  0x200
CAN_ESR1_TXWRN_SHIFT     EQU  9
CAN_ESR1_STFERR_MASK     EQU  0x400
CAN_ESR1_STFERR_SHIFT    EQU  10
CAN_ESR1_FRMERR_MASK     EQU  0x800
CAN_ESR1_FRMERR_SHIFT    EQU  11
CAN_ESR1_CRCERR_MASK     EQU  0x1000
CAN_ESR1_CRCERR_SHIFT    EQU  12
CAN_ESR1_ACKERR_MASK     EQU  0x2000
CAN_ESR1_ACKERR_SHIFT    EQU  13
CAN_ESR1_BIT0ERR_MASK    EQU  0x4000
CAN_ESR1_BIT0ERR_SHIFT   EQU  14
CAN_ESR1_BIT1ERR_MASK    EQU  0x8000
CAN_ESR1_BIT1ERR_SHIFT   EQU  15
CAN_ESR1_RWRNINT_MASK    EQU  0x10000
CAN_ESR1_RWRNINT_SHIFT   EQU  16
CAN_ESR1_TWRNINT_MASK    EQU  0x20000
CAN_ESR1_TWRNINT_SHIFT   EQU  17
CAN_ESR1_SYNCH_MASK      EQU  0x40000
CAN_ESR1_SYNCH_SHIFT     EQU  18

; IMASK1 Bit Fields
CAN_IMASK1_BUFLM_MASK    EQU  0xFFFFFFFF
CAN_IMASK1_BUFLM_SHIFT   EQU  0

; IFLAG1 Bit Fields
CAN_IFLAG1_BUF0I_MASK      EQU  0x1
CAN_IFLAG1_BUF0I_SHIFT     EQU  0
CAN_IFLAG1_BUF4TO1I_MASK   EQU  0x1E
CAN_IFLAG1_BUF4TO1I_SHIFT  EQU  1
CAN_IFLAG1_BUF5I_MASK      EQU  0x20
CAN_IFLAG1_BUF5I_SHIFT     EQU  5
CAN_IFLAG1_BUF6I_MASK      EQU  0x40
CAN_IFLAG1_BUF6I_SHIFT     EQU  6
CAN_IFLAG1_BUF7I_MASK      EQU  0x80
CAN_IFLAG1_BUF7I_SHIFT     EQU  7
CAN_IFLAG1_BUF31TO8I_MASK  EQU  0xFFFFFF00
CAN_IFLAG1_BUF31TO8I_SHIFT EQU  8

; CTRL2 Bit Fields
CAN_CTRL2_EACEN_MASK     EQU  0x10000
CAN_CTRL2_EACEN_SHIFT    EQU  16
CAN_CTRL2_RRS_MASK       EQU  0x20000
CAN_CTRL2_RRS_SHIFT      EQU  17
CAN_CTRL2_MRP_MASK       EQU  0x40000
CAN_CTRL2_MRP_SHIFT      EQU  18
CAN_CTRL2_TASD_MASK      EQU  0xF80000
CAN_CTRL2_TASD_SHIFT     EQU  19
CAN_CTRL2_RFFN_MASK      EQU  0xF000000
CAN_CTRL2_RFFN_SHIFT     EQU  24
CAN_CTRL2_WRMFRZ_MASK    EQU  0x10000000
CAN_CTRL2_WRMFRZ_SHIFT   EQU  28

; ESR2 Bit Fields
CAN_ESR2_IMB_MASK        EQU  0x2000
CAN_ESR2_IMB_SHIFT       EQU  13
CAN_ESR2_VPS_MASK        EQU  0x4000
CAN_ESR2_VPS_SHIFT       EQU  14
CAN_ESR2_LPTM_MASK       EQU  0x7F0000
CAN_ESR2_LPTM_SHIFT      EQU  16

; CRCR Bit Fields
CAN_CRCR_TXCRC_MASK      EQU  0x7FFF
CAN_CRCR_TXCRC_SHIFT     EQU  0
CAN_CRCR_MBCRC_MASK      EQU  0x7F0000
CAN_CRCR_MBCRC_SHIFT     EQU  16

; RXFGMASK Bit Fields
CAN_RXFGMASK_FGM_MASK    EQU  0xFFFFFFFF
CAN_RXFGMASK_FGM_SHIFT   EQU  0

; RXFIR Bit Fields
CAN_RXFIR_IDHIT_MASK     EQU  0x1FF
CAN_RXFIR_IDHIT_SHIFT    EQU  0

; CS Bit Fields
CAN_CS_TIME_STAMP_MASK   EQU  0xFFFF
CAN_CS_TIME_STAMP_SHIFT  EQU  0
CAN_CS_DLC_MASK          EQU  0xF0000
CAN_CS_DLC_SHIFT         EQU  16
CAN_CS_RTR_MASK          EQU  0x100000
CAN_CS_RTR_SHIFT         EQU  20
CAN_CS_IDE_MASK          EQU  0x200000
CAN_CS_IDE_SHIFT         EQU  21
CAN_CS_SRR_MASK          EQU  0x400000
CAN_CS_SRR_SHIFT         EQU  22
CAN_CS_CODE_MASK         EQU  0xF000000
CAN_CS_CODE_SHIFT        EQU  24

; ID Bit Fields
CAN_ID_EXT_MASK          EQU  0x3FFFF
CAN_ID_EXT_SHIFT         EQU  0
CAN_ID_STD_MASK          EQU  0x1FFC0000
CAN_ID_STD_SHIFT         EQU  18
CAN_ID_PRIO_MASK         EQU  0xE0000000
CAN_ID_PRIO_SHIFT        EQU  29

; WORD0 Bit Fields
CAN_WORD0_DATA_BYTE_3_MASK   EQU  0xFF
CAN_WORD0_DATA_BYTE_3_SHIFT  EQU  0
CAN_WORD0_DATA_BYTE_2_MASK   EQU  0xFF00
CAN_WORD0_DATA_BYTE_2_SHIFT  EQU  8
CAN_WORD0_DATA_BYTE_1_MASK   EQU  0xFF0000
CAN_WORD0_DATA_BYTE_1_SHIFT  EQU  16
CAN_WORD0_DATA_BYTE_0_MASK   EQU  0xFF000000
CAN_WORD0_DATA_BYTE_0_SHIFT  EQU  24

; WORD1 Bit Fields
CAN_WORD1_DATA_BYTE_7_MASK   EQU  0xFF
CAN_WORD1_DATA_BYTE_7_SHIFT  EQU  0
CAN_WORD1_DATA_BYTE_6_MASK   EQU  0xFF00
CAN_WORD1_DATA_BYTE_6_SHIFT  EQU  8
CAN_WORD1_DATA_BYTE_5_MASK   EQU  0xFF0000
CAN_WORD1_DATA_BYTE_5_SHIFT  EQU  16
CAN_WORD1_DATA_BYTE_4_MASK   EQU  0xFF000000
CAN_WORD1_DATA_BYTE_4_SHIFT  EQU  24

; RXIMR Bit Fields
CAN_RXIMR_MI_MASK        EQU  0xFFFFFFFF
CAN_RXIMR_MI_SHIFT       EQU  0

; ----------------------------------------------------------------------------
; -- CAU Peripheral Access Layer
; ----------------------------------------------------------------------------

CAU_DIRECT0      EQU  0x000   ; Direct access register 0..Direct access register 15, array offset: 0x0, array step: 0x4
CAU_DIRECT1      EQU  0x004   ; Direct access register 1
CAU_DIRECT2      EQU  0x008   ; Direct access register 2
CAU_DIRECT3      EQU  0x00C   ; Direct access register 3
CAU_DIRECT4      EQU  0x010   ; Direct access register 4
CAU_DIRECT5      EQU  0x014   ; Direct access register 5
CAU_DIRECT6      EQU  0x018   ; Direct access register 6
CAU_DIRECT7      EQU  0x01C   ; Direct access register 7
CAU_DIRECT8      EQU  0x020   ; Direct access register 8
CAU_DIRECT9      EQU  0x024   ; Direct access register 9
CAU_DIRECT10     EQU  0x028   ; Direct access register 10
CAU_DIRECT11     EQU  0x02C   ; Direct access register 11
CAU_DIRECT12     EQU  0x030   ; Direct access register 12
CAU_DIRECT13     EQU  0x034   ; Direct access register 13
CAU_DIRECT14     EQU  0x038   ; Direct access register 14
CAU_DIRECT15     EQU  0x03C   ; Direct access register 15

CAU_LDR_CASR     EQU  0x840   ; Status register  - Load Register command, offset: 0x840
CAU_LDR_CAA      EQU  0x844   ; Accumulator register - Load Register command, offset: 0x844
CAU_LDR_CA0      EQU  0x848   ; General Purpose Register 0 - Load Register command..General Purpose Register 8 - Load Register command, array offset: 0x848, array step: 0x4
CAU_LDR_CA1      EQU  0x84C   ; General Purpose Register 1
CAU_LDR_CA2      EQU  0x850   ; General Purpose Register 2
CAU_LDR_CA3      EQU  0x854   ; General Purpose Register 3
CAU_LDR_CA4      EQU  0x858   ; General Purpose Register 4
CAU_LDR_CA5      EQU  0x85C   ; General Purpose Register 5
CAU_LDR_CA6      EQU  0x860   ; General Purpose Register 6
CAU_LDR_CA7      EQU  0x864   ; General Purpose Register 7
CAU_LDR_CA8      EQU  0x868   ; General Purpose Register 8

CAU_STR_CASR     EQU  0x880   ; Status register  - Store Register command, offset: 0x880
CAU_STR_CAA      EQU  0x884   ; Accumulator register - Store Register command, offset: 0x884
CAU_STR_CA0      EQU  0x888   ; General Purpose Register 0 - Store Register command..General Purpose Register 8 - Store Register command, array offset: 0x888, array step: 0x4
CAU_STR_CA1      EQU  0x88C   ; General Purpose Register 1
CAU_STR_CA2      EQU  0x890   ; General Purpose Register 2
CAU_STR_CA3      EQU  0x894   ; General Purpose Register 3
CAU_STR_CA4      EQU  0x898   ; General Purpose Register 4
CAU_STR_CA5      EQU  0x89C   ; General Purpose Register 5
CAU_STR_CA6      EQU  0x8A0   ; General Purpose Register 6
CAU_STR_CA7      EQU  0x8A4   ; General Purpose Register 7
CAU_STR_CA8      EQU  0x8A8   ; General Purpose Register 8

CAU_ADR_CASR     EQU  0x8C0   ; Status register  - Add Register command, offset: 0x8C0
CAU_ADR_CAA      EQU  0x8C4   ; Accumulator register - Add to register command, offset: 0x8C4
CAU_ADR_CA0      EQU  0x8C8   ; General Purpose Register 0 - Add to register command..General Purpose Register 8 - Add to register command, array offset: 0x8C8, array step: 0x4
CAU_ADR_CA1      EQU  0x8CC   ; General Purpose Register 1
CAU_ADR_CA2      EQU  0x8D0   ; General Purpose Register 2
CAU_ADR_CA3      EQU  0x8D4   ; General Purpose Register 3
CAU_ADR_CA4      EQU  0x8D8   ; General Purpose Register 4
CAU_ADR_CA5      EQU  0x8DC   ; General Purpose Register 5
CAU_ADR_CA6      EQU  0x8E0   ; General Purpose Register 6
CAU_ADR_CA7      EQU  0x8E4   ; General Purpose Register 7
CAU_ADR_CA8      EQU  0x8E8   ; General Purpose Register 8

CAU_RADR_CASR    EQU  0x900   ; Status register  - Reverse and Add to Register command, offset: 0x900
CAU_RADR_CAA     EQU  0x904   ; Accumulator register - Reverse and Add to Register command, offset: 0x904
CAU_RADR_CA0     EQU  0x908   ; General Purpose Register 0 - Reverse and Add to Register command..array offset: 0x908, array step: 0x4
CAU_RADR_CA1     EQU  0x90C   ; General Purpose Register 1
CAU_RADR_CA2     EQU  0x910   ; General Purpose Register 2
CAU_RADR_CA3     EQU  0x914   ; General Purpose Register 3
CAU_RADR_CA4     EQU  0x918   ; General Purpose Register 4
CAU_RADR_CA5     EQU  0x91C   ; General Purpose Register 5
CAU_RADR_CA6     EQU  0x920   ; General Purpose Register 6
CAU_RADR_CA7     EQU  0x924   ; General Purpose Register 7
CAU_RADR_CA8     EQU  0x928   ; General Purpose Register 8

CAU_XOR_CASR     EQU  0x980   ; Status register  - Exclusive Or command, offset: 0x980
CAU_XOR_CAA      EQU  0x984   ; Accumulator register - Exclusive Or command, offset: 0x984
CAU_XOR_CA0      EQU  0x988   ; General Purpose Register 0 - Exclusive Or command..General Purpose Register 8 - Exclusive Or command, array offset: 0x988, array step: 0x4
CAU_XOR_CA1      EQU  0x98C   ; General Purpose Register 1
CAU_XOR_CA2      EQU  0x990   ; General Purpose Register 2
CAU_XOR_CA3      EQU  0x994   ; General Purpose Register 3
CAU_XOR_CA4      EQU  0x998   ; General Purpose Register 4
CAU_XOR_CA5      EQU  0x99C   ; General Purpose Register 5
CAU_XOR_CA6      EQU  0x9A0   ; General Purpose Register 6
CAU_XOR_CA7      EQU  0x9A4   ; General Purpose Register 7
CAU_XOR_CA8      EQU  0x9A8   ; General Purpose Register 8

CAU_ROTL_CASR    EQU  0x9C0   ; Status register  - Rotate Left command, offset: 0x9C0
CAU_ROTL_CAA     EQU  0x9C4   ; Accumulator register - Rotate Left command, offset: 0x9C4
CAU_ROTL_CA0     EQU  0x9C8   ; General Purpose Register 0 - Rotate Left command..General Purpose Register 8 - Rotate Left command, array offset: 0x9C8, array step: 0x4
CAU_ROTL_CA1     EQU  0x9CC   ; General Purpose Register 1
CAU_ROTL_CA2     EQU  0x9D0   ; General Purpose Register 2
CAU_ROTL_CA3     EQU  0x9D4   ; General Purpose Register 3
CAU_ROTL_CA4     EQU  0x9D8   ; General Purpose Register 4
CAU_ROTL_CA5     EQU  0x9DC   ; General Purpose Register 5
CAU_ROTL_CA6     EQU  0x9E0   ; General Purpose Register 6
CAU_ROTL_CA7     EQU  0x9E4   ; General Purpose Register 7
CAU_ROTL_CA8     EQU  0x9E8   ; General Purpose Register 8

CAU_AESC_CASR    EQU  0xB00   ; Status register  - AES Column Operation command, offset: 0xB00
CAU_AESC_CAA     EQU  0xB04   ; Accumulator register - AES Column Operation command, offset: 0xB04
CAU_AESC_CA0     EQU  0xB08   ; General Purpose Register 0 - AES Column Operation command..General Purpose Register 8 - AES Column Operation command, array offset: 0xB08, array step: 0x4
CAU_AESC_CA1     EQU  0xB0C   ; General Purpose Register 1
CAU_AESC_CA2     EQU  0xB10   ; General Purpose Register 2
CAU_AESC_CA3     EQU  0xB14   ; General Purpose Register 3
CAU_AESC_CA4     EQU  0xB18   ; General Purpose Register 4
CAU_AESC_CA5     EQU  0xB1C   ; General Purpose Register 5
CAU_AESC_CA6     EQU  0xB20   ; General Purpose Register 6
CAU_AESC_CA7     EQU  0xB24   ; General Purpose Register 7
CAU_AESC_CA8     EQU  0xB28   ; General Purpose Register 8

CAU_AESIC_CASR   EQU  0xB40   ; Status register  - AES Inverse Column Operation command, offset: 0xB40
CAU_AESIC_CAA    EQU  0xB44   ; Accumulator register - AES Inverse Column Operation command, offset: 0xB44
CAU_AESIC_CA0    EQU  0xB48   ; General Purpose Register 0 - AES Inverse Column Operation command, array offset: 0xB48, array step: 0x4
CAU_AESIC_CA1    EQU  0xB4C   ; General Purpose Register 1
CAU_AESIC_CA2    EQU  0xB50   ; General Purpose Register 2
CAU_AESIC_CA3    EQU  0xB54   ; General Purpose Register 3
CAU_AESIC_CA4    EQU  0xB58   ; General Purpose Register 4
CAU_AESIC_CA5    EQU  0xB5C   ; General Purpose Register 5
CAU_AESIC_CA6    EQU  0xB60   ; General Purpose Register 6
CAU_AESIC_CA7    EQU  0xB64   ; General Purpose Register 7
CAU_AESIC_CA8    EQU  0xB68   ; General Purpose Register 8

; ----------------------------------------------------------------------------
; -- CAU Register Masks
; ----------------------------------------------------------------------------

CAU_LDR_CASR_IC_MASK     EQU  0x1
CAU_LDR_CASR_IC_SHIFT    EQU  0
CAU_LDR_CASR_DPE_MASK    EQU  0x2
CAU_LDR_CASR_DPE_SHIFT   EQU  1
CAU_LDR_CASR_VER_MASK    EQU  0xF0000000
CAU_LDR_CASR_VER_SHIFT   EQU  28

; STR_CASR Bit Fields
CAU_STR_CASR_IC_MASK     EQU  0x1
CAU_STR_CASR_IC_SHIFT    EQU  0
CAU_STR_CASR_DPE_MASK    EQU  0x2
CAU_STR_CASR_DPE_SHIFT   EQU  1
CAU_STR_CASR_VER_MASK    EQU  0xF0000000
CAU_STR_CASR_VER_SHIFT   EQU  28

; ADR_CASR Bit Fields
CAU_ADR_CASR_IC_MASK     EQU  0x1
CAU_ADR_CASR_IC_SHIFT    EQU  0
CAU_ADR_CASR_DPE_MASK    EQU  0x2
CAU_ADR_CASR_DPE_SHIFT   EQU  1
CAU_ADR_CASR_VER_MASK    EQU  0xF0000000
CAU_ADR_CASR_VER_SHIFT   EQU  28

; RADR_CASR Bit Fields
CAU_RADR_CASR_IC_MASK    EQU  0x1
CAU_RADR_CASR_IC_SHIFT   EQU  0
CAU_RADR_CASR_DPE_MASK   EQU  0x2
CAU_RADR_CASR_DPE_SHIFT  EQU  1
CAU_RADR_CASR_VER_MASK   EQU  0xF0000000
CAU_RADR_CASR_VER_SHIFT  EQU  28

; XOR_CASR Bit Fields
CAU_XOR_CASR_IC_MASK     EQU  0x1
CAU_XOR_CASR_IC_SHIFT    EQU  0
CAU_XOR_CASR_DPE_MASK    EQU  0x2
CAU_XOR_CASR_DPE_SHIFT   EQU  1
CAU_XOR_CASR_VER_MASK    EQU  0xF0000000
CAU_XOR_CASR_VER_SHIFT   EQU  28

; ROTL_CASR Bit Fields
CAU_ROTL_CASR_IC_MASK    EQU  0x1
CAU_ROTL_CASR_IC_SHIFT   EQU  0
CAU_ROTL_CASR_DPE_MASK   EQU  0x2
CAU_ROTL_CASR_DPE_SHIFT  EQU  1
CAU_ROTL_CASR_VER_MASK   EQU  0xF0000000
CAU_ROTL_CASR_VER_SHIFT  EQU  28

; AESC_CASR Bit Fields
CAU_AESC_CASR_IC_MASK    EQU  0x1
CAU_AESC_CASR_IC_SHIFT   EQU  0
CAU_AESC_CASR_DPE_MASK   EQU  0x2
CAU_AESC_CASR_DPE_SHIFT  EQU  1
CAU_AESC_CASR_VER_MASK   EQU  0xF0000000
CAU_AESC_CASR_VER_SHIFT  EQU  28

; AESIC_CASR Bit Fields
CAU_AESIC_CASR_IC_MASK   EQU  0x1
CAU_AESIC_CASR_IC_SHIFT  EQU  0
CAU_AESIC_CASR_DPE_MASK  EQU  0x2
CAU_AESIC_CASR_DPE_SHIFT EQU  1
CAU_AESIC_CASR_VER_MASK  EQU  0xF0000000
CAU_AESIC_CASR_VER_SHIFT EQU  28

; ----------------------------------------------------------------------------
; -- CMP Peripheral Access Layer
; ----------------------------------------------------------------------------

CMP_CR0     EQU  0x0  ; CMP Control Register 0, offset: 0x0
CMP_CR1     EQU  0x1  ; CMP Control Register 1, offset: 0x1
CMP_FPR     EQU  0x2  ; CMP Filter Period Register, offset: 0x2
CMP_SCR     EQU  0x3  ; CMP Status and Control Register, offset: 0x3
CMP_DACCR   EQU  0x4  ; DAC Control Register, offset: 0x4
CMP_MUXCR   EQU  0x5  ; MUX Control Register, offset: 0x5

; ----------------------------------------------------------------------------
; -- CMP Register Masks
; ----------------------------------------------------------------------------

; CR0 Bit Fields
CMP_CR0_HYSTCTR_MASK     EQU  0x3
CMP_CR0_HYSTCTR_SHIFT    EQU  0
CMP_CR0_FILTER_CNT_MASK  EQU  0x70
CMP_CR0_FILTER_CNT_SHIFT EQU  4

; CR1 Bit Fields
CMP_CR1_EN_MASK          EQU  0x1
CMP_CR1_EN_SHIFT         EQU  0
CMP_CR1_OPE_MASK         EQU  0x2
CMP_CR1_OPE_SHIFT        EQU  1
CMP_CR1_COS_MASK         EQU  0x4
CMP_CR1_COS_SHIFT        EQU  2
CMP_CR1_INV_MASK         EQU  0x8
CMP_CR1_INV_SHIFT        EQU  3
CMP_CR1_PMODE_MASK       EQU  0x10
CMP_CR1_PMODE_SHIFT      EQU  4
CMP_CR1_WE_MASK          EQU  0x40
CMP_CR1_WE_SHIFT         EQU  6
CMP_CR1_SE_MASK          EQU  0x80
CMP_CR1_SE_SHIFT         EQU  7

; FPR Bit Fields
CMP_FPR_FILT_PER_MASK    EQU  0xFF
CMP_FPR_FILT_PER_SHIFT   EQU  0

; SCR Bit Fields
CMP_SCR_COUT_MASK        EQU  0x1
CMP_SCR_COUT_SHIFT       EQU  0
CMP_SCR_CFF_MASK         EQU  0x2
CMP_SCR_CFF_SHIFT        EQU  1
CMP_SCR_CFR_MASK         EQU  0x4
CMP_SCR_CFR_SHIFT        EQU  2
CMP_SCR_IEF_MASK         EQU  0x8
CMP_SCR_IEF_SHIFT        EQU  3
CMP_SCR_IER_MASK         EQU  0x10
CMP_SCR_IER_SHIFT        EQU  4
CMP_SCR_DMAEN_MASK       EQU  0x40
CMP_SCR_DMAEN_SHIFT      EQU  6

; DACCR Bit Fields
CMP_DACCR_VOSEL_MASK     EQU  0x3F
CMP_DACCR_VOSEL_SHIFT    EQU  0
CMP_DACCR_VRSEL_MASK     EQU  0x40
CMP_DACCR_VRSEL_SHIFT    EQU  6
CMP_DACCR_DACEN_MASK     EQU  0x80
CMP_DACCR_DACEN_SHIFT    EQU  7

; MUXCR Bit Fields
CMP_MUXCR_MSEL_MASK      EQU  0x7
CMP_MUXCR_MSEL_SHIFT     EQU  0
CMP_MUXCR_PSEL_MASK      EQU  0x38
CMP_MUXCR_PSEL_SHIFT     EQU  3
CMP_MUXCR_PSTM_MASK      EQU  0x80
CMP_MUXCR_PSTM_SHIFT     EQU  7

; ----------------------------------------------------------------------------
; -- CMT Peripheral Access Layer
; ----------------------------------------------------------------------------

CMT_CGH1  EQU  0x0   ; CMT Carrier Generator High Data Register 1, offset: 0x0
CMT_CGL1  EQU  0x1   ; CMT Carrier Generator Low Data Register 1, offset: 0x1
CMT_CGH2  EQU  0x2   ; CMT Carrier Generator High Data Register 2, offset: 0x2
CMT_CGL2  EQU  0x3   ; CMT Carrier Generator Low Data Register 2, offset: 0x3
CMT_OC    EQU  0x4   ; CMT Output Control Register, offset: 0x4
CMT_MSC   EQU  0x5   ; CMT Modulator Status and Control Register, offset: 0x5
CMT_CMD1  EQU  0x6   ; CMT Modulator Data Register Mark High, offset: 0x6
CMT_CMD2  EQU  0x7   ; CMT Modulator Data Register Mark Low, offset: 0x7
CMT_CMD3  EQU  0x8   ; CMT Modulator Data Register Space High, offset: 0x8
CMT_CMD4  EQU  0x9   ; CMT Modulator Data Register Space Low, offset: 0x9
CMT_PPS   EQU  0xA   ; CMT Primary Prescaler Register, offset: 0xA
CMT_DMA   EQU  0xB   ; CMT Direct Memory Access Register, offset: 0xB

; ----------------------------------------------------------------------------
; -- CMT Register Masks
; ----------------------------------------------------------------------------

; CGH1 Bit Fields
CMT_CGH1_PH_MASK         EQU  0xFF
CMT_CGH1_PH_SHIFT        EQU  0

; CGL1 Bit Fields
CMT_CGL1_PL_MASK         EQU  0xFF
CMT_CGL1_PL_SHIFT        EQU  0

; CGH2 Bit Fields
CMT_CGH2_SH_MASK         EQU  0xFF
CMT_CGH2_SH_SHIFT        EQU  0

; CGL2 Bit Fields
CMT_CGL2_SL_MASK         EQU  0xFF
CMT_CGL2_SL_SHIFT        EQU  0
; OC Bit Fields

CMT_OC_IROPEN_MASK       EQU  0x20
CMT_OC_IROPEN_SHIFT      EQU  5
CMT_OC_CMTPOL_MASK       EQU  0x40
CMT_OC_CMTPOL_SHIFT      EQU  6
CMT_OC_IROL_MASK         EQU  0x80
CMT_OC_IROL_SHIFT        EQU  7

; MSC Bit Fields
CMT_MSC_MCGEN_MASK       EQU  0x1
CMT_MSC_MCGEN_SHIFT      EQU  0
CMT_MSC_EOCIE_MASK       EQU  0x2
CMT_MSC_EOCIE_SHIFT      EQU  1
CMT_MSC_FSK_MASK         EQU  0x4
CMT_MSC_FSK_SHIFT        EQU  2
CMT_MSC_BASE_MASK        EQU  0x8
CMT_MSC_BASE_SHIFT       EQU  3
CMT_MSC_EXSPC_MASK       EQU  0x10
CMT_MSC_EXSPC_SHIFT      EQU  4
CMT_MSC_CMTDIV_MASK      EQU  0x60
CMT_MSC_CMTDIV_SHIFT     EQU  5
CMT_MSC_EOCF_MASK        EQU  0x80
CMT_MSC_EOCF_SHIFT       EQU  7

; CMD1 Bit Fields
CMT_CMD1_MB_MASK         EQU  0xFF
CMT_CMD1_MB_SHIFT        EQU  0

; CMD2 Bit Fields
CMT_CMD2_MB_MASK         EQU  0xFF
CMT_CMD2_MB_SHIFT        EQU  0

; CMD3 Bit Fields
CMT_CMD3_SB_MASK         EQU  0xFF
CMT_CMD3_SB_SHIFT        EQU  0

; CMD4 Bit Fields
CMT_CMD4_SB_MASK         EQU  0xFF
CMT_CMD4_SB_SHIFT        EQU  0

; PPS Bit Fields
CMT_PPS_PPSDIV_MASK      EQU  0xF
CMT_PPS_PPSDIV_SHIFT     EQU  0

; DMA Bit Fields
CMT_DMA_DMA_MASK         EQU  0x1
CMT_DMA_DMA_SHIFT        EQU  0

; ----------------------------------------------------------------------------
; -- CRC Peripheral Access Layer
; ----------------------------------------------------------------------------

CRC_ACCESS16BIT         EQU   0x0
CRC_ACCESS16BIT_DATAL   EQU   0x0    ; CRC_DATAL register., offset: 0x0
CRC_ACCESS16BIT_DATAH   EQU   0x2    ; CRC_DATAH register., offset: 0x2

CRC_DATA                EQU   0x0    ; CRC Data register, offset: 0x0

CRC_ACCESS8BIT          EQU   0x0
CRC_ACCESS8BIT_DATALL   EQU   0x0    ; CRC_DATALL register., offset: 0x0
CRC_ACCESS8BIT_DATALU   EQU   0x1    ; CRC_DATALU register., offset: 0x1
CRC_ACCESS8BIT_DATAHL   EQU   0x2    ; CRC_DATAHL register., offset: 0x2
CRC_ACCESS8BIT_DATAHU   EQU   0x3    ; CRC_DATAHU register., offset: 0x3

CRC_GPOLY_ACCESS16BIT   EQU   0x4
CRC_GPOLYL              EQU   0x4    ; CRC_GPOLYL register., offset: 0x4
CRC_GPOLYH              EQU   0x6    ; CRC_GPOLYH register., offset: 0x6

CRC_GPOLY               EQU   0x4    ; CRC Polynomial register, offset: 0x4
CRC_ACCESS8BIT_GPOLYLL  EQU   0x4    ; CRC_GPOLYLL register., offset: 0x4
CRC_ACCESS8BIT_GPOLYLU  EQU   0x5    ; CRC_GPOLYLU register., offset: 0x5
CRC_ACCESS8BIT_GPOLYHL  EQU   0x6    ; CRC_GPOLYHL register., offset: 0x6
CRC_ACCESS8BIT_GPOLYHU  EQU   0x7    ; CRC_GPOLYHU register., offset: 0x7

CRC_CTRL                EQU   0x8    ; CRC Control register, offset: 0x8

CRC_CTRL__ACCESS8BIT_CTRLHU   EQU   0xB    ; CRC_CTRLHU register., offset: 0xB

; ----------------------------------------------------------------------------
; -- CRC Register Masks
; ----------------------------------------------------------------------------

; DATAL Bit Fields
CRC_DATAL_DATAL_MASK     EQU  0xFFFF
CRC_DATAL_DATAL_SHIFT    EQU  0

; DATAH Bit Fields
CRC_DATAH_DATAH_MASK     EQU  0xFFFF
CRC_DATAH_DATAH_SHIFT    EQU  0

; DATA Bit Fields
CRC_DATA_LL_MASK         EQU  0xFF
CRC_DATA_LL_SHIFT        EQU  0
CRC_DATA_LU_MASK         EQU  0xFF00
CRC_DATA_LU_SHIFT        EQU  8
CRC_DATA_HL_MASK         EQU  0xFF0000
CRC_DATA_HL_SHIFT        EQU  16
CRC_DATA_HU_MASK         EQU  0xFF000000
CRC_DATA_HU_SHIFT        EQU  24

; DATALL Bit Fields
CRC_DATALL_DATALL_MASK   EQU  0xFF
CRC_DATALL_DATALL_SHIFT  EQU  0

; DATALU Bit Fields
CRC_DATALU_DATALU_MASK   EQU  0xFF
CRC_DATALU_DATALU_SHIFT  EQU  0

; DATAHL Bit Fields
CRC_DATAHL_DATAHL_MASK   EQU  0xFF
CRC_DATAHL_DATAHL_SHIFT  EQU  0

; DATAHU Bit Fields
CRC_DATAHU_DATAHU_MASK   EQU  0xFF
CRC_DATAHU_DATAHU_SHIFT  EQU  0

; GPOLYL Bit Fields
CRC_GPOLYL_GPOLYL_MASK   EQU  0xFFFF
CRC_GPOLYL_GPOLYL_SHIFT  EQU  0

; GPOLYH Bit Fields
CRC_GPOLYH_GPOLYH_MASK   EQU  0xFFFF
CRC_GPOLYH_GPOLYH_SHIFT  EQU  0

; GPOLY Bit Fields
CRC_GPOLY_LOW_MASK       EQU  0xFFFF
CRC_GPOLY_LOW_SHIFT      EQU  0
CRC_GPOLY_HIGH_MASK      EQU  0xFFFF0000
CRC_GPOLY_HIGH_SHIFT     EQU  16

; GPOLYLL Bit Fields
CRC_GPOLYLL_GPOLYLL_MASK  EQU  0xFF
CRC_GPOLYLL_GPOLYLL_SHIFT EQU  0

; GPOLYLU Bit Fields
CRC_GPOLYLU_GPOLYLU_MASK  EQU  0xFF
CRC_GPOLYLU_GPOLYLU_SHIFT EQU  0

; GPOLYHL Bit Fields
CRC_GPOLYHL_GPOLYHL_MASK  EQU  0xFF
CRC_GPOLYHL_GPOLYHL_SHIFT EQU  0

; GPOLYHU Bit Fields
CRC_GPOLYHU_GPOLYHU_MASK  EQU  0xFF
CRC_GPOLYHU_GPOLYHU_SHIFT EQU  0

; CTRL Bit Fields
CRC_CTRL_TCRC_MASK       EQU  0x1000000
CRC_CTRL_TCRC_SHIFT      EQU  24
CRC_CTRL_WAS_MASK        EQU  0x2000000
CRC_CTRL_WAS_SHIFT       EQU  25
CRC_CTRL_FXOR_MASK       EQU  0x4000000
CRC_CTRL_FXOR_SHIFT      EQU  26
CRC_CTRL_TOTR_MASK       EQU  0x30000000
CRC_CTRL_TOTR_SHIFT      EQU  28
CRC_CTRL_TOT_MASK        EQU  0xC0000000
CRC_CTRL_TOT_SHIFT       EQU  30

; CTRLHU Bit Fields
CRC_CTRLHU_TCRC_MASK     EQU  0x1
CRC_CTRLHU_TCRC_SHIFT    EQU  0
CRC_CTRLHU_WAS_MASK      EQU  0x2
CRC_CTRLHU_WAS_SHIFT     EQU  1
CRC_CTRLHU_FXOR_MASK     EQU  0x4
CRC_CTRLHU_FXOR_SHIFT    EQU  2
CRC_CTRLHU_TOTR_MASK     EQU  0x30
CRC_CTRLHU_TOTR_SHIFT    EQU  4
CRC_CTRLHU_TOT_MASK      EQU  0xC0
CRC_CTRLHU_TOT_SHIFT     EQU  6

; ----------------------------------------------------------------------------
; -- DAC Peripheral Access Layer
; ----------------------------------------------------------------------------

DAC_DAT0   EQU  0x00   ; DAC Data Register 0, array step: 0x2
DAC_DAT1   EQU  0x02   ; DAC Data Register 1, array step: 0x2
DAC_DAT2   EQU  0x04   ; DAC Data Register 2, array step: 0x2
DAC_DAT3   EQU  0x06   ; DAC Data Register 3, array step: 0x2
DAC_DAT4   EQU  0x08   ; DAC Data Register 4, array step: 0x2
DAC_DAT5   EQU  0x0A   ; DAC Data Register 5, array step: 0x2
DAC_DAT6   EQU  0x0C   ; DAC Data Register 6, array step: 0x2
DAC_DAT7   EQU  0x0E   ; DAC Data Register 7, array step: 0x2
DAC_DAT8   EQU  0x10   ; DAC Data Register 8, array step: 0x2
DAC_DAT9   EQU  0x12   ; DAC Data Register 9, array step: 0x2
DAC_DAT10  EQU  0x14   ; DAC Data Register 10, array step: 0x2
DAC_DAT11  EQU  0x16   ; DAC Data Register 11, array step: 0x2
DAC_DAT12  EQU  0x18   ; DAC Data Register 12, array step: 0x2
DAC_DAT13  EQU  0x1A   ; DAC Data Register 13, array step: 0x2
DAC_DAT14  EQU  0x1C   ; DAC Data Register 14, array step: 0x2
DAC_DAT15  EQU  0x1E   ; DAC Data Register 15, array step: 0x2
DAC_SR     EQU  0x20   ; DAC Status Register, offset: 0x20
DAC_C0     EQU  0x21   ; DAC Control Register, offset: 0x21
DAC_C1     EQU  0x22   ; DAC Control Register 1, offset: 0x22
DAC_C2     EQU  0x23   ; DAC Control Register 2, offset: 0x23

; ----------------------------------------------------------------------------
; -- DAC Register Masks
; ----------------------------------------------------------------------------

; DATL Bit Fields
DAC_DATL_DATA0_MASK      EQU  0xFF
DAC_DATL_DATA0_SHIFT     EQU  0

; DATH Bit Fields
DAC_DATH_DATA1_MASK      EQU  0xF
DAC_DATH_DATA1_SHIFT     EQU  0

; SR Bit Fields
DAC_SR_DACBFRPBF_MASK    EQU  0x1
DAC_SR_DACBFRPBF_SHIFT   EQU  0
DAC_SR_DACBFRPTF_MASK    EQU  0x2
DAC_SR_DACBFRPTF_SHIFT   EQU  1
DAC_SR_DACBFWMF_MASK     EQU  0x4
DAC_SR_DACBFWMF_SHIFT    EQU  2

; C0 Bit Fields
DAC_C0_DACBBIEN_MASK     EQU  0x1
DAC_C0_DACBBIEN_SHIFT    EQU  0
DAC_C0_DACBTIEN_MASK     EQU  0x2
DAC_C0_DACBTIEN_SHIFT    EQU  1
DAC_C0_DACBWIEN_MASK     EQU  0x4
DAC_C0_DACBWIEN_SHIFT    EQU  2
DAC_C0_LPEN_MASK         EQU  0x8
DAC_C0_LPEN_SHIFT        EQU  3
DAC_C0_DACSWTRG_MASK     EQU  0x10
DAC_C0_DACSWTRG_SHIFT    EQU  4
DAC_C0_DACTRGSEL_MASK    EQU  0x20
DAC_C0_DACTRGSEL_SHIFT   EQU  5
DAC_C0_DACRFS_MASK       EQU  0x40
DAC_C0_DACRFS_SHIFT      EQU  6
DAC_C0_DACEN_MASK        EQU  0x80
DAC_C0_DACEN_SHIFT       EQU  7

; C1 Bit Fields
DAC_C1_DACBFEN_MASK      EQU  0x1
DAC_C1_DACBFEN_SHIFT     EQU  0
DAC_C1_DACBFMD_MASK      EQU  0x6
DAC_C1_DACBFMD_SHIFT     EQU  1
DAC_C1_DACBFWM_MASK      EQU  0x18
DAC_C1_DACBFWM_SHIFT     EQU  3
DAC_C1_DMAEN_MASK        EQU  0x80
DAC_C1_DMAEN_SHIFT       EQU  7

; C2 Bit Fields
DAC_C2_DACBFUP_MASK      EQU  0xF
DAC_C2_DACBFUP_SHIFT     EQU  0
DAC_C2_DACBFRP_MASK      EQU  0xF0
DAC_C2_DACBFRP_SHIFT     EQU  4

; ----------------------------------------------------------------------------
; -- DMA Peripheral Access Layer
; ----------------------------------------------------------------------------

DMA_CR               EQU  0x000   ; Control Register, offset: 0x0
DMA_ES               EQU  0x004   ; Error Status Register, offset: 0x4
DMA_ERQ              EQU  0x004   ; Enable Request Register, offset: 0xC
DMA_EEI              EQU  0x014   ; Enable Error Interrupt Register, offset: 0x14
DMA_CEEI             EQU  0x018   ; Clear Enable Error Interrupt Register, offset: 0x18
DMA_SEEI             EQU  0x019   ; Set Enable Error Interrupt Register, offset: 0x19
DMA_CERQ             EQU  0x01A   ; Clear Enable Request Register, offset: 0x1A
DMA_SERQ             EQU  0x01B   ; Set Enable Request Register, offset: 0x1B
DMA_CDNE             EQU  0x01C   ; Clear DONE Status Bit Register, offset: 0x1C
DMA_SSRT             EQU  0x01D   ; Set START Bit Register, offset: 0x1D
DMA_CERR             EQU  0x01E   ; Clear Error Register, offset: 0x1E
DMA_CINT             EQU  0x01F   ; Clear Interrupt Request Register, offset: 0x1F
DMA_INT              EQU  0x024   ; Interrupt Request Register, offset: 0x24
DMA_ERR              EQU  0x000   ; Error Register, offset: 0x2C
DMA_HRS              EQU  0x000   ; Hardware Request Status Register, offset: 0x34
DMA_DCHPRI3          EQU  0x100   ; Channel n Priority Register, offset: 0x100
DMA_DCHPRI2          EQU  0x101   ; Channel n Priority Register, offset: 0x101
DMA_DCHPRI1          EQU  0x102   ; Channel n Priority Register, offset: 0x102
DMA_DCHPRI0          EQU  0x103   ; Channel n Priority Register, offset: 0x103
DMA_DCHPRI7          EQU  0x104   ; Channel n Priority Register, offset: 0x104
DMA_DCHPRI6          EQU  0x105   ; Channel n Priority Register, offset: 0x105
DMA_DCHPRI5          EQU  0x106   ; Channel n Priority Register, offset: 0x106
DMA_DCHPRI4          EQU  0x107   ; Channel n Priority Register, offset: 0x107
DMA_DCHPRI11         EQU  0x108   ; Channel n Priority Register, offset: 0x108
DMA_DCHPRI10         EQU  0x109   ; Channel n Priority Register, offset: 0x109
DMA_DCHPRI9          EQU  0x10A   ; Channel n Priority Register, offset: 0x10A
DMA_DCHPRI8          EQU  0x10B   ; Channel n Priority Register, offset: 0x10B
DMA_DCHPRI15         EQU  0x10C   ; Channel n Priority Register, offset: 0x10C
DMA_DCHPRI14         EQU  0x10D   ; Channel n Priority Register, offset: 0x10D
DMA_DCHPRI13         EQU  0x10E   ; Channel n Priority Register, offset: 0x10E
DMA_DCHPRI12         EQU  0x10F   ; Channel n Priority Register, offset: 0x10F

DMA_TCD0_SADDR             EQU  0x1000   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD0_SOFF              EQU  0x1004   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD0_ATTR              EQU  0x1006   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD0_NBYTES_MLNO       EQU  0x1008   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD0_NBYTES_MLOFFNO    EQU  0x1008   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD0_NBYTES_MLOFFYES   EQU  0x1008   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD0_SLAST             EQU  0x100C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD0_DADDR             EQU  0x1010   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD0_DOFF              EQU  0x1014   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD0_CITER_ELINKNO     EQU  0x1016   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD0_CITER_ELINKYES    EQU  0x1016   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD0_DLAST_SGA         EQU  0x1018   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD0_CSR               EQU  0x1018   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD0_BITER_ELINKNO     EQU  0x101E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD0_BITER_ELINKYES    EQU  0x101E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD1_SADDR             EQU  0x1020   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD1_SOFF              EQU  0x1024   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD1_ATTR              EQU  0x1026   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD1_NBYTES_MLNO       EQU  0x1028   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD1_NBYTES_MLOFFNO    EQU  0x1028   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD1_NBYTES_MLOFFYES   EQU  0x1028   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD1_SLAST             EQU  0x102C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD1_DADDR             EQU  0x1030   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD1_DOFF              EQU  0x1034   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD1_CITER_ELINKNO     EQU  0x1036   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD1_CITER_ELINKYES    EQU  0x1036   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD1_DLAST_SGA         EQU  0x1038   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD1_CSR               EQU  0x1038   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD1_BITER_ELINKNO     EQU  0x103E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD1_BITER_ELINKYES    EQU  0x103E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD2_SADDR             EQU  0x1040   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD2_SOFF              EQU  0x1044   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD2_ATTR              EQU  0x1046   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD2_NBYTES_MLNO       EQU  0x1048   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD2_NBYTES_MLOFFNO    EQU  0x1048   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD2_NBYTES_MLOFFYES   EQU  0x1048   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD2_SLAST             EQU  0x104C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD2_DADDR             EQU  0x1050   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD2_DOFF              EQU  0x1054   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD2_CITER_ELINKNO     EQU  0x1056   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD2_CITER_ELINKYES    EQU  0x1056   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD2_DLAST_SGA         EQU  0x1058   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD2_CSR               EQU  0x1058   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD2_BITER_ELINKNO     EQU  0x105E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD2_BITER_ELINKYES    EQU  0x105E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD3_SADDR             EQU  0x1060   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD3_SOFF              EQU  0x1064   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD3_ATTR              EQU  0x1066   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD3_NBYTES_MLNO       EQU  0x1068   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD3_NBYTES_MLOFFNO    EQU  0x1068   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD3_NBYTES_MLOFFYES   EQU  0x1068   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD3_SLAST             EQU  0x106C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD3_DADDR             EQU  0x1070   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD3_DOFF              EQU  0x1074   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD3_CITER_ELINKNO     EQU  0x1076   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD3_CITER_ELINKYES    EQU  0x1076   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD3_DLAST_SGA         EQU  0x1078   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD3_CSR               EQU  0x1078   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD3_BITER_ELINKNO     EQU  0x107E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD3_BITER_ELINKYES    EQU  0x107E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD4_SADDR             EQU  0x1080   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD4_SOFF              EQU  0x1084   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD4_ATTR              EQU  0x1086   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD4_NBYTES_MLNO       EQU  0x1088   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD4_NBYTES_MLOFFNO    EQU  0x1088   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD4_NBYTES_MLOFFYES   EQU  0x1088   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD4_SLAST             EQU  0x108C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD4_DADDR             EQU  0x1090   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD4_DOFF              EQU  0x1094   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD4_CITER_ELINKNO     EQU  0x1096   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD4_CITER_ELINKYES    EQU  0x1096   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD4_DLAST_SGA         EQU  0x1098   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD4_CSR               EQU  0x1098   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD4_BITER_ELINKNO     EQU  0x109E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD4_BITER_ELINKYES    EQU  0x109E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD5_SADDR             EQU  0x10A0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD5_SOFF              EQU  0x10A4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD5_ATTR              EQU  0x10A6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD5_NBYTES_MLNO       EQU  0x10A8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD5_NBYTES_MLOFFNO    EQU  0x10A8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD5_NBYTES_MLOFFYES   EQU  0x10A8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD5_SLAST             EQU  0x10AC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD5_DADDR             EQU  0x10B0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD5_DOFF              EQU  0x10B4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD5_CITER_ELINKNO     EQU  0x10B6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD5_CITER_ELINKYES    EQU  0x10B6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD5_DLAST_SGA         EQU  0x10B8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD5_CSR               EQU  0x10B8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD5_BITER_ELINKNO     EQU  0x10BE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD5_BITER_ELINKYES    EQU  0x10BE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD6_SADDR             EQU  0x10C0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD6_SOFF              EQU  0x10C4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD6_ATTR              EQU  0x10C6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD6_NBYTES_MLNO       EQU  0x10C8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD6_NBYTES_MLOFFNO    EQU  0x10C8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD6_NBYTES_MLOFFYES   EQU  0x10C8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD6_SLAST             EQU  0x10CC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD6_DADDR             EQU  0x10D0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD6_DOFF              EQU  0x10D4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD6_CITER_ELINKNO     EQU  0x10D6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD6_CITER_ELINKYES    EQU  0x10D6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD6_DLAST_SGA         EQU  0x10D8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD6_CSR               EQU  0x10D8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD6_BITER_ELINKNO     EQU  0x10DE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD6_BITER_ELINKYES    EQU  0x10DE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD7_SADDR             EQU  0x10E0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD7_SOFF              EQU  0x10E4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD7_ATTR              EQU  0x10E6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD7_NBYTES_MLNO       EQU  0x10E8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD7_NBYTES_MLOFFNO    EQU  0x10E8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD7_NBYTES_MLOFFYES   EQU  0x10E8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD7_SLAST             EQU  0x10EC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD7_DADDR             EQU  0x10F0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD7_DOFF              EQU  0x10F4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD7_CITER_ELINKNO     EQU  0x10F6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD7_CITER_ELINKYES    EQU  0x10F6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD7_DLAST_SGA         EQU  0x10F8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD7_CSR               EQU  0x10F8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD7_BITER_ELINKNO     EQU  0x10FE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD7_BITER_ELINKYES    EQU  0x10FE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD8_SADDR             EQU  0x1100   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD8_SOFF              EQU  0x1104   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD8_ATTR              EQU  0x1106   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD8_NBYTES_MLNO       EQU  0x1108   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD8_NBYTES_MLOFFNO    EQU  0x1108   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD8_NBYTES_MLOFFYES   EQU  0x1108   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD8_SLAST             EQU  0x110C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD8_DADDR             EQU  0x1110   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD8_DOFF              EQU  0x1114   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD8_CITER_ELINKNO     EQU  0x1116   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD8_CITER_ELINKYES    EQU  0x1116   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD8_DLAST_SGA         EQU  0x1118   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD8_CSR               EQU  0x1118   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD8_BITER_ELINKNO     EQU  0x111E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD8_BITER_ELINKYES    EQU  0x111E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD9_SADDR             EQU  0x1120   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD9_SOFF              EQU  0x1124   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD9_ATTR              EQU  0x1126   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD9_NBYTES_MLNO       EQU  0x1128   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD9_NBYTES_MLOFFNO    EQU  0x1128   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD9_NBYTES_MLOFFYES   EQU  0x1128   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD9_SLAST             EQU  0x112C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD9_DADDR             EQU  0x1130   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD9_DOFF              EQU  0x1134   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD9_CITER_ELINKNO     EQU  0x1136   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD9_CITER_ELINKYES    EQU  0x1136   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD9_DLAST_SGA         EQU  0x1138   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD9_CSR               EQU  0x1138   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD9_BITER_ELINKNO     EQU  0x113E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD9_BITER_ELINKYES    EQU  0x113E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD10_SADDR             EQU  0x1140   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD10_SOFF              EQU  0x1144   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD10_ATTR              EQU  0x1146   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD10_NBYTES_MLNO       EQU  0x1148   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD10_NBYTES_MLOFFNO    EQU  0x1148   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD10_NBYTES_MLOFFYES   EQU  0x1148   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD10_SLAST             EQU  0x114C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD10_DADDR             EQU  0x1150   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD10_DOFF              EQU  0x1154   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD10_CITER_ELINKNO     EQU  0x1156   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD10_CITER_ELINKYES    EQU  0x1156   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD10_DLAST_SGA         EQU  0x1158   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD10_CSR               EQU  0x1158   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD10_BITER_ELINKNO     EQU  0x115E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD10_BITER_ELINKYES    EQU  0x115E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD11_SADDR             EQU  0x1160   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD11_SOFF              EQU  0x1164   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD11_ATTR              EQU  0x1166   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD11_NBYTES_MLNO       EQU  0x1168   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD11_NBYTES_MLOFFNO    EQU  0x1168   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD11_NBYTES_MLOFFYES   EQU  0x1168   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD11_SLAST             EQU  0x116C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD11_DADDR             EQU  0x1170   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD11_DOFF              EQU  0x1174   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD11_CITER_ELINKNO     EQU  0x1176   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD11_CITER_ELINKYES    EQU  0x1176   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD11_DLAST_SGA         EQU  0x1178   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD11_CSR               EQU  0x1178   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD11_BITER_ELINKNO     EQU  0x117E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD11_BITER_ELINKYES    EQU  0x117E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD12_SADDR             EQU  0x1180   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD12_SOFF              EQU  0x1184   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD12_ATTR              EQU  0x1186   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD12_NBYTES_MLNO       EQU  0x1188   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD12_NBYTES_MLOFFNO    EQU  0x1188   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD12_NBYTES_MLOFFYES   EQU  0x1188   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD12_SLAST             EQU  0x118C   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD12_DADDR             EQU  0x1190   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD12_DOFF              EQU  0x1194   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD12_CITER_ELINKNO     EQU  0x1196   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD12_CITER_ELINKYES    EQU  0x1196   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD12_DLAST_SGA         EQU  0x1198   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD12_CSR               EQU  0x1198   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD12_BITER_ELINKNO     EQU  0x119E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD12_BITER_ELINKYES    EQU  0x119E   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD13_SADDR             EQU  0x11A0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD13_SOFF              EQU  0x11A4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD13_ATTR              EQU  0x11A6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD13_NBYTES_MLNO       EQU  0x11A8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD13_NBYTES_MLOFFNO    EQU  0x11A8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD13_NBYTES_MLOFFYES   EQU  0x11A8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD13_SLAST             EQU  0x11AC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD13_DADDR             EQU  0x11B0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD13_DOFF              EQU  0x11B4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD13_CITER_ELINKNO     EQU  0x11B6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD13_CITER_ELINKYES    EQU  0x11B6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD13_DLAST_SGA         EQU  0x11B8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD13_CSR               EQU  0x11B8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD13_BITER_ELINKNO     EQU  0x11BE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD13_BITER_ELINKYES    EQU  0x11BE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD14_SADDR             EQU  0x11C0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD14_SOFF              EQU  0x11C4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD14_ATTR              EQU  0x11C6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD14_NBYTES_MLNO       EQU  0x11C8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD14_NBYTES_MLOFFNO    EQU  0x11C8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD14_NBYTES_MLOFFYES   EQU  0x11C8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD14_SLAST             EQU  0x11CC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD14_DADDR             EQU  0x11D0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD14_DOFF              EQU  0x11D4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD14_CITER_ELINKNO     EQU  0x11D6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD14_CITER_ELINKYES    EQU  0x11D6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD14_DLAST_SGA         EQU  0x11D8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD14_CSR               EQU  0x11D8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD14_BITER_ELINKNO     EQU  0x11DE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD14_BITER_ELINKYES    EQU  0x11DE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20

DMA_TCD15_SADDR             EQU  0x11E0   ; TCD Source Address, array offset: 0x1000, array step: 0x20
DMA_TCD15_SOFF              EQU  0x11E4   ; TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20
DMA_TCD15_ATTR              EQU  0x11E6   ; TCD Transfer Attributes, array offset: 0x1006, array step: 0x20
DMA_TCD15_NBYTES_MLNO       EQU  0x11E8   ; TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD15_NBYTES_MLOFFNO    EQU  0x11E8   ; TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20
DMA_TCD15_NBYTES_MLOFFYES   EQU  0x11E8   ; TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20
DMA_TCD15_SLAST             EQU  0x11EC   ; TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20
DMA_TCD15_DADDR             EQU  0x11F0   ; TCD Destination Address, array offset: 0x1010, array step: 0x20
DMA_TCD15_DOFF              EQU  0x11F4   ; TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20
DMA_TCD15_CITER_ELINKNO     EQU  0x11F6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20
DMA_TCD15_CITER_ELINKYES    EQU  0x11F6   ; TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20
DMA_TCD15_DLAST_SGA         EQU  0x11F8   ; TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20
DMA_TCD15_CSR               EQU  0x11F8   ; TCD Control and Status, array offset: 0x101C, array step: 0x20
DMA_TCD15_BITER_ELINKNO     EQU  0x11FE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20
DMA_TCD15_BITER_ELINKYES    EQU  0x11FE   ; TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20



; ----------------------------------------------------------------------------
; -- DMA Register Masks
; ----------------------------------------------------------------------------

; CR Bit Fields
DMA_CR_EDBG_MASK         EQU  0x2
DMA_CR_EDBG_SHIFT        EQU  1
DMA_CR_ERCA_MASK         EQU  0x4
DMA_CR_ERCA_SHIFT        EQU  2
DMA_CR_HOE_MASK          EQU  0x10
DMA_CR_HOE_SHIFT         EQU  4
DMA_CR_HALT_MASK         EQU  0x20
DMA_CR_HALT_SHIFT        EQU  5
DMA_CR_CLM_MASK          EQU  0x40
DMA_CR_CLM_SHIFT         EQU  6
DMA_CR_EMLM_MASK         EQU  0x80
DMA_CR_EMLM_SHIFT        EQU  7
DMA_CR_ECX_MASK          EQU  0x10000
DMA_CR_ECX_SHIFT         EQU  16
DMA_CR_CX_MASK           EQU  0x20000
DMA_CR_CX_SHIFT          EQU  17

; ES Bit Fields
DMA_ES_DBE_MASK          EQU  0x1
DMA_ES_DBE_SHIFT         EQU  0
DMA_ES_SBE_MASK          EQU  0x2
DMA_ES_SBE_SHIFT         EQU  1
DMA_ES_SGE_MASK          EQU  0x4
DMA_ES_SGE_SHIFT         EQU  2
DMA_ES_NCE_MASK          EQU  0x8
DMA_ES_NCE_SHIFT         EQU  3
DMA_ES_DOE_MASK          EQU  0x10
DMA_ES_DOE_SHIFT         EQU  4
DMA_ES_DAE_MASK          EQU  0x20
DMA_ES_DAE_SHIFT         EQU  5
DMA_ES_SOE_MASK          EQU  0x40
DMA_ES_SOE_SHIFT         EQU  6
DMA_ES_SAE_MASK          EQU  0x80
DMA_ES_SAE_SHIFT         EQU  7
DMA_ES_ERRCHN_MASK       EQU  0xF00
DMA_ES_ERRCHN_SHIFT      EQU  8
DMA_ES_CPE_MASK          EQU  0x4000
DMA_ES_CPE_SHIFT         EQU  14
DMA_ES_ECX_MASK          EQU  0x10000
DMA_ES_ECX_SHIFT         EQU  16
DMA_ES_VLD_MASK          EQU  0x80000000
DMA_ES_VLD_SHIFT         EQU  31

; ERQ Bit Fields
DMA_ERQ_ERQ0_MASK        EQU  0x1
DMA_ERQ_ERQ0_SHIFT       EQU  0
DMA_ERQ_ERQ1_MASK        EQU  0x2
DMA_ERQ_ERQ1_SHIFT       EQU  1
DMA_ERQ_ERQ2_MASK        EQU  0x4
DMA_ERQ_ERQ2_SHIFT       EQU  2
DMA_ERQ_ERQ3_MASK        EQU  0x8
DMA_ERQ_ERQ3_SHIFT       EQU  3
DMA_ERQ_ERQ4_MASK        EQU  0x10
DMA_ERQ_ERQ4_SHIFT       EQU  4
DMA_ERQ_ERQ5_MASK        EQU  0x20
DMA_ERQ_ERQ5_SHIFT       EQU  5
DMA_ERQ_ERQ6_MASK        EQU  0x40
DMA_ERQ_ERQ6_SHIFT       EQU  6
DMA_ERQ_ERQ7_MASK        EQU  0x80
DMA_ERQ_ERQ7_SHIFT       EQU  7
DMA_ERQ_ERQ8_MASK        EQU  0x100
DMA_ERQ_ERQ8_SHIFT       EQU  8
DMA_ERQ_ERQ9_MASK        EQU  0x200
DMA_ERQ_ERQ9_SHIFT       EQU  9
DMA_ERQ_ERQ10_MASK       EQU  0x400
DMA_ERQ_ERQ10_SHIFT      EQU  10
DMA_ERQ_ERQ11_MASK       EQU  0x800
DMA_ERQ_ERQ11_SHIFT      EQU  11
DMA_ERQ_ERQ12_MASK       EQU  0x1000
DMA_ERQ_ERQ12_SHIFT      EQU  12
DMA_ERQ_ERQ13_MASK       EQU  0x2000
DMA_ERQ_ERQ13_SHIFT      EQU  13
DMA_ERQ_ERQ14_MASK       EQU  0x4000
DMA_ERQ_ERQ14_SHIFT      EQU  14
DMA_ERQ_ERQ15_MASK       EQU  0x8000
DMA_ERQ_ERQ15_SHIFT      EQU  15

; EEI Bit Fields
DMA_EEI_EEI0_MASK        EQU  0x1
DMA_EEI_EEI0_SHIFT       EQU  0
DMA_EEI_EEI1_MASK        EQU  0x2
DMA_EEI_EEI1_SHIFT       EQU  1
DMA_EEI_EEI2_MASK        EQU  0x4
DMA_EEI_EEI2_SHIFT       EQU  2
DMA_EEI_EEI3_MASK        EQU  0x8
DMA_EEI_EEI3_SHIFT       EQU  3
DMA_EEI_EEI4_MASK        EQU  0x10
DMA_EEI_EEI4_SHIFT       EQU  4
DMA_EEI_EEI5_MASK        EQU  0x20
DMA_EEI_EEI5_SHIFT       EQU  5
DMA_EEI_EEI6_MASK        EQU  0x40
DMA_EEI_EEI6_SHIFT       EQU  6
DMA_EEI_EEI7_MASK        EQU  0x80
DMA_EEI_EEI7_SHIFT       EQU  7
DMA_EEI_EEI8_MASK        EQU  0x100
DMA_EEI_EEI8_SHIFT       EQU  8
DMA_EEI_EEI9_MASK        EQU  0x200
DMA_EEI_EEI9_SHIFT       EQU  9
DMA_EEI_EEI10_MASK       EQU  0x400
DMA_EEI_EEI10_SHIFT      EQU  10
DMA_EEI_EEI11_MASK       EQU  0x800
DMA_EEI_EEI11_SHIFT      EQU  11
DMA_EEI_EEI12_MASK       EQU  0x1000
DMA_EEI_EEI12_SHIFT      EQU  12
DMA_EEI_EEI13_MASK       EQU  0x2000
DMA_EEI_EEI13_SHIFT      EQU  13
DMA_EEI_EEI14_MASK       EQU  0x4000
DMA_EEI_EEI14_SHIFT      EQU  14
DMA_EEI_EEI15_MASK       EQU  0x8000
DMA_EEI_EEI15_SHIFT      EQU  15

; CEEI Bit Fields
DMA_CEEI_CEEI_MASK       EQU  0xF
DMA_CEEI_CEEI_SHIFT      EQU  0
DMA_CEEI_CAEE_MASK       EQU  0x40
DMA_CEEI_CAEE_SHIFT      EQU  6
DMA_CEEI_NOP_MASK        EQU  0x80
DMA_CEEI_NOP_SHIFT       EQU  7

; SEEI Bit Fields
DMA_SEEI_SEEI_MASK       EQU  0xF
DMA_SEEI_SEEI_SHIFT      EQU  0
DMA_SEEI_SAEE_MASK       EQU  0x40
DMA_SEEI_SAEE_SHIFT      EQU  6
DMA_SEEI_NOP_MASK        EQU  0x80
DMA_SEEI_NOP_SHIFT       EQU  7

; CERQ Bit Fields
DMA_CERQ_CERQ_MASK       EQU  0xF
DMA_CERQ_CERQ_SHIFT      EQU  0
DMA_CERQ_CAER_MASK       EQU  0x40
DMA_CERQ_CAER_SHIFT      EQU  6
DMA_CERQ_NOP_MASK        EQU  0x80
DMA_CERQ_NOP_SHIFT       EQU  7

; SERQ Bit Fields
DMA_SERQ_SERQ_MASK       EQU  0xF
DMA_SERQ_SERQ_SHIFT      EQU  0
DMA_SERQ_SAER_MASK       EQU  0x40
DMA_SERQ_SAER_SHIFT      EQU  6
DMA_SERQ_NOP_MASK        EQU  0x80
DMA_SERQ_NOP_SHIFT       EQU  7

; CDNE Bit Fields
DMA_CDNE_CDNE_MASK       EQU  0xF
DMA_CDNE_CDNE_SHIFT      EQU  0
DMA_CDNE_CADN_MASK       EQU  0x40
DMA_CDNE_CADN_SHIFT      EQU  6
DMA_CDNE_NOP_MASK        EQU  0x80
DMA_CDNE_NOP_SHIFT       EQU  7

; SSRT Bit Fields
DMA_SSRT_SSRT_MASK       EQU  0xF
DMA_SSRT_SSRT_SHIFT      EQU  0
DMA_SSRT_SAST_MASK       EQU  0x40
DMA_SSRT_SAST_SHIFT      EQU  6
DMA_SSRT_NOP_MASK        EQU  0x80
DMA_SSRT_NOP_SHIFT       EQU  7

; CERR Bit Fields
DMA_CERR_CERR_MASK       EQU  0xF
DMA_CERR_CERR_SHIFT      EQU  0
DMA_CERR_CAEI_MASK       EQU  0x40
DMA_CERR_CAEI_SHIFT      EQU  6
DMA_CERR_NOP_MASK        EQU  0x80
DMA_CERR_NOP_SHIFT       EQU  7

; CINT Bit Fields
DMA_CINT_CINT_MASK       EQU  0xF
DMA_CINT_CINT_SHIFT      EQU  0
DMA_CINT_CAIR_MASK       EQU  0x40
DMA_CINT_CAIR_SHIFT      EQU  6
DMA_CINT_NOP_MASK        EQU  0x80
DMA_CINT_NOP_SHIFT       EQU  7

; INT Bit Fields
DMA_INT_INT0_MASK        EQU  0x1
DMA_INT_INT0_SHIFT       EQU  0
DMA_INT_INT1_MASK        EQU  0x2
DMA_INT_INT1_SHIFT       EQU  1
DMA_INT_INT2_MASK        EQU  0x4
DMA_INT_INT2_SHIFT       EQU  2
DMA_INT_INT3_MASK        EQU  0x8
DMA_INT_INT3_SHIFT       EQU  3
DMA_INT_INT4_MASK        EQU  0x10
DMA_INT_INT4_SHIFT       EQU  4
DMA_INT_INT5_MASK        EQU  0x20
DMA_INT_INT5_SHIFT       EQU  5
DMA_INT_INT6_MASK        EQU  0x40
DMA_INT_INT6_SHIFT       EQU  6
DMA_INT_INT7_MASK        EQU  0x80
DMA_INT_INT7_SHIFT       EQU  7
DMA_INT_INT8_MASK        EQU  0x100
DMA_INT_INT8_SHIFT       EQU  8
DMA_INT_INT9_MASK        EQU  0x200
DMA_INT_INT9_SHIFT       EQU  9
DMA_INT_INT10_MASK       EQU  0x400
DMA_INT_INT10_SHIFT      EQU  10
DMA_INT_INT11_MASK       EQU  0x800
DMA_INT_INT11_SHIFT      EQU  11
DMA_INT_INT12_MASK       EQU  0x1000
DMA_INT_INT12_SHIFT      EQU  12
DMA_INT_INT13_MASK       EQU  0x2000
DMA_INT_INT13_SHIFT      EQU  13
DMA_INT_INT14_MASK       EQU  0x4000
DMA_INT_INT14_SHIFT      EQU  14
DMA_INT_INT15_MASK       EQU  0x8000
DMA_INT_INT15_SHIFT      EQU  15

; ERR Bit Fields
DMA_ERR_ERR0_MASK        EQU  0x1
DMA_ERR_ERR0_SHIFT       EQU  0
DMA_ERR_ERR1_MASK        EQU  0x2
DMA_ERR_ERR1_SHIFT       EQU  1
DMA_ERR_ERR2_MASK        EQU  0x4
DMA_ERR_ERR2_SHIFT       EQU  2
DMA_ERR_ERR3_MASK        EQU  0x8
DMA_ERR_ERR3_SHIFT       EQU  3
DMA_ERR_ERR4_MASK        EQU  0x10
DMA_ERR_ERR4_SHIFT       EQU  4
DMA_ERR_ERR5_MASK        EQU  0x20
DMA_ERR_ERR5_SHIFT       EQU  5
DMA_ERR_ERR6_MASK        EQU  0x40
DMA_ERR_ERR6_SHIFT       EQU  6
DMA_ERR_ERR7_MASK        EQU  0x80
DMA_ERR_ERR7_SHIFT       EQU  7
DMA_ERR_ERR8_MASK        EQU  0x100
DMA_ERR_ERR8_SHIFT       EQU  8
DMA_ERR_ERR9_MASK        EQU  0x200
DMA_ERR_ERR9_SHIFT       EQU  9
DMA_ERR_ERR10_MASK       EQU  0x400
DMA_ERR_ERR10_SHIFT      EQU  10
DMA_ERR_ERR11_MASK       EQU  0x800
DMA_ERR_ERR11_SHIFT      EQU  11
DMA_ERR_ERR12_MASK       EQU  0x1000
DMA_ERR_ERR12_SHIFT      EQU  12
DMA_ERR_ERR13_MASK       EQU  0x2000
DMA_ERR_ERR13_SHIFT      EQU  13
DMA_ERR_ERR14_MASK       EQU  0x4000
DMA_ERR_ERR14_SHIFT      EQU  14
DMA_ERR_ERR15_MASK       EQU  0x8000
DMA_ERR_ERR15_SHIFT      EQU  15

; HRS Bit Fields
DMA_HRS_HRS0_MASK        EQU  0x1
DMA_HRS_HRS0_SHIFT       EQU  0
DMA_HRS_HRS1_MASK        EQU  0x2
DMA_HRS_HRS1_SHIFT       EQU  1
DMA_HRS_HRS2_MASK        EQU  0x4
DMA_HRS_HRS2_SHIFT       EQU  2
DMA_HRS_HRS3_MASK        EQU  0x8
DMA_HRS_HRS3_SHIFT       EQU  3
DMA_HRS_HRS4_MASK        EQU  0x10
DMA_HRS_HRS4_SHIFT       EQU  4
DMA_HRS_HRS5_MASK        EQU  0x20
DMA_HRS_HRS5_SHIFT       EQU  5
DMA_HRS_HRS6_MASK        EQU  0x40
DMA_HRS_HRS6_SHIFT       EQU  6
DMA_HRS_HRS7_MASK        EQU  0x80
DMA_HRS_HRS7_SHIFT       EQU  7
DMA_HRS_HRS8_MASK        EQU  0x100
DMA_HRS_HRS8_SHIFT       EQU  8
DMA_HRS_HRS9_MASK        EQU  0x200
DMA_HRS_HRS9_SHIFT       EQU  9
DMA_HRS_HRS10_MASK       EQU  0x400
DMA_HRS_HRS10_SHIFT      EQU  10
DMA_HRS_HRS11_MASK       EQU  0x800
DMA_HRS_HRS11_SHIFT      EQU  11
DMA_HRS_HRS12_MASK       EQU  0x1000
DMA_HRS_HRS12_SHIFT      EQU  12
DMA_HRS_HRS13_MASK       EQU  0x2000
DMA_HRS_HRS13_SHIFT      EQU  13
DMA_HRS_HRS14_MASK       EQU  0x4000
DMA_HRS_HRS14_SHIFT      EQU  14
DMA_HRS_HRS15_MASK       EQU  0x8000
DMA_HRS_HRS15_SHIFT      EQU  15

; DCHPRI3 Bit Fields
DMA_DCHPRI3_CHPRI_MASK   EQU  0xF
DMA_DCHPRI3_CHPRI_SHIFT  EQU  0
DMA_DCHPRI3_DPA_MASK     EQU  0x40
DMA_DCHPRI3_DPA_SHIFT    EQU  6
DMA_DCHPRI3_ECP_MASK     EQU  0x80
DMA_DCHPRI3_ECP_SHIFT    EQU  7

; DCHPRI2 Bit Fields
DMA_DCHPRI2_CHPRI_MASK   EQU  0xF
DMA_DCHPRI2_CHPRI_SHIFT  EQU  0
DMA_DCHPRI2_DPA_MASK     EQU  0x40
DMA_DCHPRI2_DPA_SHIFT    EQU  6
DMA_DCHPRI2_ECP_MASK     EQU  0x80
DMA_DCHPRI2_ECP_SHIFT    EQU  7

; DCHPRI1 Bit Fields
DMA_DCHPRI1_CHPRI_MASK   EQU  0xF
DMA_DCHPRI1_CHPRI_SHIFT  EQU  0
DMA_DCHPRI1_DPA_MASK     EQU  0x40
DMA_DCHPRI1_DPA_SHIFT    EQU  6
DMA_DCHPRI1_ECP_MASK     EQU  0x80
DMA_DCHPRI1_ECP_SHIFT    EQU  7

; DCHPRI0 Bit Fields
DMA_DCHPRI0_CHPRI_MASK   EQU  0xF
DMA_DCHPRI0_CHPRI_SHIFT  EQU  0
DMA_DCHPRI0_DPA_MASK     EQU  0x40
DMA_DCHPRI0_DPA_SHIFT    EQU  6
DMA_DCHPRI0_ECP_MASK     EQU  0x80
DMA_DCHPRI0_ECP_SHIFT    EQU  7

; DCHPRI7 Bit Fields
DMA_DCHPRI7_CHPRI_MASK   EQU  0xF
DMA_DCHPRI7_CHPRI_SHIFT  EQU  0
DMA_DCHPRI7_DPA_MASK     EQU  0x40
DMA_DCHPRI7_DPA_SHIFT    EQU  6
DMA_DCHPRI7_ECP_MASK     EQU  0x80
DMA_DCHPRI7_ECP_SHIFT    EQU  7
; DCHPRI6 Bit Fields

DMA_DCHPRI6_CHPRI_MASK   EQU  0xF
DMA_DCHPRI6_CHPRI_SHIFT  EQU  0
DMA_DCHPRI6_DPA_MASK     EQU  0x40
DMA_DCHPRI6_DPA_SHIFT    EQU  6
DMA_DCHPRI6_ECP_MASK     EQU  0x80
DMA_DCHPRI6_ECP_SHIFT    EQU  7

; DCHPRI5 Bit Fields
DMA_DCHPRI5_CHPRI_MASK   EQU  0xF
DMA_DCHPRI5_CHPRI_SHIFT  EQU  0
DMA_DCHPRI5_DPA_MASK     EQU  0x40
DMA_DCHPRI5_DPA_SHIFT    EQU  6
DMA_DCHPRI5_ECP_MASK     EQU  0x80
DMA_DCHPRI5_ECP_SHIFT    EQU  7

; DCHPRI4 Bit Fields
DMA_DCHPRI4_CHPRI_MASK   EQU  0xF
DMA_DCHPRI4_CHPRI_SHIFT  EQU  0
DMA_DCHPRI4_DPA_MASK     EQU  0x40
DMA_DCHPRI4_DPA_SHIFT    EQU  6
DMA_DCHPRI4_ECP_MASK     EQU  0x80
DMA_DCHPRI4_ECP_SHIFT    EQU  7

; DCHPRI11 Bit Fields
DMA_DCHPRI11_CHPRI_MASK  EQU  0xF
DMA_DCHPRI11_CHPRI_SHIFT EQU  0
DMA_DCHPRI11_DPA_MASK    EQU  0x40
DMA_DCHPRI11_DPA_SHIFT   EQU  6
DMA_DCHPRI11_ECP_MASK    EQU  0x80
DMA_DCHPRI11_ECP_SHIFT   EQU  7

; DCHPRI10 Bit Fields
DMA_DCHPRI10_CHPRI_MASK  EQU  0xF
DMA_DCHPRI10_CHPRI_SHIFT EQU  0
DMA_DCHPRI10_DPA_MASK    EQU  0x40
DMA_DCHPRI10_DPA_SHIFT   EQU  6
DMA_DCHPRI10_ECP_MASK    EQU  0x80
DMA_DCHPRI10_ECP_SHIFT   EQU  7

; DCHPRI9 Bit Fields
DMA_DCHPRI9_CHPRI_MASK   EQU  0xF
DMA_DCHPRI9_CHPRI_SHIFT  EQU  0
DMA_DCHPRI9_DPA_MASK     EQU  0x40
DMA_DCHPRI9_DPA_SHIFT    EQU  6
DMA_DCHPRI9_ECP_MASK     EQU  0x80
DMA_DCHPRI9_ECP_SHIFT    EQU  7

; DCHPRI8 Bit Fields
DMA_DCHPRI8_CHPRI_MASK   EQU  0xF
DMA_DCHPRI8_CHPRI_SHIFT  EQU  0
DMA_DCHPRI8_DPA_MASK     EQU  0x40
DMA_DCHPRI8_DPA_SHIFT    EQU  6
DMA_DCHPRI8_ECP_MASK     EQU  0x80
DMA_DCHPRI8_ECP_SHIFT    EQU  7

; DCHPRI15 Bit Fields
DMA_DCHPRI15_CHPRI_MASK  EQU  0xF
DMA_DCHPRI15_CHPRI_SHIFT EQU  0
DMA_DCHPRI15_DPA_MASK    EQU  0x40
DMA_DCHPRI15_DPA_SHIFT   EQU  6
DMA_DCHPRI15_ECP_MASK    EQU  0x80
DMA_DCHPRI15_ECP_SHIFT   EQU  7

; DCHPRI14 Bit Fields
DMA_DCHPRI14_CHPRI_MASK  EQU  0xF
DMA_DCHPRI14_CHPRI_SHIFT EQU  0
DMA_DCHPRI14_DPA_MASK    EQU  0x40
DMA_DCHPRI14_DPA_SHIFT   EQU  6
DMA_DCHPRI14_ECP_MASK    EQU  0x80
DMA_DCHPRI14_ECP_SHIFT   EQU  7

; DCHPRI13 Bit Fields
DMA_DCHPRI13_CHPRI_MASK  EQU  0xF
DMA_DCHPRI13_CHPRI_SHIFT EQU  0
DMA_DCHPRI13_DPA_MASK    EQU  0x40
DMA_DCHPRI13_DPA_SHIFT   EQU  6
DMA_DCHPRI13_ECP_MASK    EQU  0x80
DMA_DCHPRI13_ECP_SHIFT   EQU  7

; DCHPRI12 Bit Fields
DMA_DCHPRI12_CHPRI_MASK  EQU  0xF
DMA_DCHPRI12_CHPRI_SHIFT EQU  0
DMA_DCHPRI12_DPA_MASK    EQU  0x40
DMA_DCHPRI12_DPA_SHIFT   EQU  6
DMA_DCHPRI12_ECP_MASK    EQU  0x80
DMA_DCHPRI12_ECP_SHIFT   EQU  7

; SADDR Bit Fields
DMA_SADDR_SADDR_MASK     EQU  0xFFFFFFFF
DMA_SADDR_SADDR_SHIFT    EQU  0

; SOFF Bit Fields
DMA_SOFF_SOFF_MASK       EQU  0xFFFF
DMA_SOFF_SOFF_SHIFT      EQU  0

; ATTR Bit Fields
DMA_ATTR_DSIZE_MASK      EQU  0x7
DMA_ATTR_DSIZE_SHIFT     EQU  0
DMA_ATTR_DMOD_MASK       EQU  0xF8
DMA_ATTR_DMOD_SHIFT      EQU  3
DMA_ATTR_SSIZE_MASK      EQU  0x700
DMA_ATTR_SSIZE_SHIFT     EQU  8
DMA_ATTR_SMOD_MASK       EQU  0xF800
DMA_ATTR_SMOD_SHIFT      EQU  11

; NBYTES_MLNO Bit Fields
DMA_NBYTES_MLNO_NBYTES_MASK        EQU  0xFFFFFFFF
DMA_NBYTES_MLNO_NBYTES_SHIFT       EQU  0

; NBYTES_MLOFFNO Bit Fields
DMA_NBYTES_MLOFFNO_NBYTES_MASK     EQU  0x3FFFFFFF
DMA_NBYTES_MLOFFNO_NBYTES_SHIFT    EQU  0
DMA_NBYTES_MLOFFNO_DMLOE_MASK      EQU  0x40000000
DMA_NBYTES_MLOFFNO_DMLOE_SHIFT     EQU  30
DMA_NBYTES_MLOFFNO_SMLOE_MASK      EQU  0x80000000
DMA_NBYTES_MLOFFNO_SMLOE_SHIFT     EQU  31

; NBYTES_MLOFFYES Bit Fields
DMA_NBYTES_MLOFFYES_NBYTES_MASK    EQU  0x3FF
DMA_NBYTES_MLOFFYES_NBYTES_SHIFT   EQU  0
DMA_NBYTES_MLOFFYES_MLOFF_MASK     EQU  0x3FFFFC00
DMA_NBYTES_MLOFFYES_MLOFF_SHIFT    EQU  10
DMA_NBYTES_MLOFFYES_DMLOE_MASK     EQU  0x40000000
DMA_NBYTES_MLOFFYES_DMLOE_SHIFT    EQU  30
DMA_NBYTES_MLOFFYES_SMLOE_MASK     EQU  0x80000000
DMA_NBYTES_MLOFFYES_SMLOE_SHIFT    EQU  31

; SLAST Bit Fields
DMA_SLAST_SLAST_MASK     EQU  0xFFFFFFFF
DMA_SLAST_SLAST_SHIFT    EQU  0

; DADDR Bit Fields
DMA_DADDR_DADDR_MASK     EQU  0xFFFFFFFF
DMA_DADDR_DADDR_SHIFT    EQU  0

; DOFF Bit Fields
DMA_DOFF_DOFF_MASK       EQU  0xFFFF
DMA_DOFF_DOFF_SHIFT      EQU  0

; CITER_ELINKNO Bit Fields
DMA_CITER_ELINKNO_CITER_MASK       EQU  0x7FFF
DMA_CITER_ELINKNO_CITER_SHIFT      EQU  0
DMA_CITER_ELINKNO_ELINK_MASK       EQU  0x8000
DMA_CITER_ELINKNO_ELINK_SHIFT      EQU  15

; CITER_ELINKYES Bit Fields
DMA_CITER_ELINKYES_CITER_MASK      EQU  0x1FF
DMA_CITER_ELINKYES_CITER_SHIFT     EQU  0
DMA_CITER_ELINKYES_LINKCH_MASK     EQU  0x1E00
DMA_CITER_ELINKYES_LINKCH_SHIFT    EQU  9
DMA_CITER_ELINKYES_ELINK_MASK      EQU  0x8000
DMA_CITER_ELINKYES_ELINK_SHIFT     EQU  15

; DLAST_SGA Bit Fields
DMA_DLAST_SGA_DLASTSGA_MASK        EQU  0xFFFFFFFF
DMA_DLAST_SGA_DLASTSGA_SHIFT       EQU  0

; CSR Bit Fields
DMA_CSR_START_MASK        EQU  0x1
DMA_CSR_START_SHIFT       EQU  0
DMA_CSR_INTMAJOR_MASK     EQU  0x2
DMA_CSR_INTMAJOR_SHIFT    EQU  1
DMA_CSR_INTHALF_MASK      EQU  0x4
DMA_CSR_INTHALF_SHIFT     EQU  2
DMA_CSR_DREQ_MASK         EQU  0x8
DMA_CSR_DREQ_SHIFT        EQU  3
DMA_CSR_ESG_MASK          EQU  0x10
DMA_CSR_ESG_SHIFT         EQU  4
DMA_CSR_MAJORELINK_MASK   EQU  0x20
DMA_CSR_MAJORELINK_SHIFT  EQU  5
DMA_CSR_ACTIVE_MASK       EQU  0x40
DMA_CSR_ACTIVE_SHIFT      EQU  6
DMA_CSR_DONE_MASK         EQU  0x80
DMA_CSR_DONE_SHIFT        EQU  7
DMA_CSR_MAJORLINKCH_MASK  EQU  0xF00
DMA_CSR_MAJORLINKCH_SHIFT EQU  8
DMA_CSR_BWC_MASK          EQU  0xC000
DMA_CSR_BWC_SHIFT         EQU  14

; BITER_ELINKNO Bit Fields
DMA_BITER_ELINKNO_BITER_MASK       EQU  0x7FFF
DMA_BITER_ELINKNO_BITER_SHIFT      EQU  0
DMA_BITER_ELINKNO_ELINK_MASK       EQU  0x8000
DMA_BITER_ELINKNO_ELINK_SHIFT      EQU  15

; BITER_ELINKYES Bit Fields
DMA_BITER_ELINKYES_BITER_MASK      EQU  0x1FF
DMA_BITER_ELINKYES_BITER_SHIFT     EQU  0
DMA_BITER_ELINKYES_LINKCH_MASK     EQU  0x1E00
DMA_BITER_ELINKYES_LINKCH_SHIFT    EQU  9
DMA_BITER_ELINKYES_ELINK_MASK      EQU  0x8000
DMA_BITER_ELINKYES_ELINK_SHIFT     EQU  15

; ----------------------------------------------------------------------------
; -- DMAMUX Peripheral Access Layer
; ----------------------------------------------------------------------------

DMAMUX_CHCFG0    EQU   0x0  ; Channel Configuration register, array offset: 0x0, array step: 0x1
DMAMUX_CHCFG1    EQU   0x1  ; Channel Configuration register
DMAMUX_CHCFG2    EQU   0x2  ; Channel Configuration register
DMAMUX_CHCFG3    EQU   0x3  ; Channel Configuration register
DMAMUX_CHCFG4    EQU   0x4  ; Channel Configuration register
DMAMUX_CHCFG5    EQU   0x5  ; Channel Configuration register
DMAMUX_CHCFG6    EQU   0x6  ; Channel Configuration register
DMAMUX_CHCFG7    EQU   0x7  ; Channel Configuration register
DMAMUX_CHCFG8    EQU   0x8  ; Channel Configuration register
DMAMUX_CHCFG9    EQU   0x9  ; Channel Configuration register
DMAMUX_CHCFG10   EQU   0xA  ; Channel Configuration register
DMAMUX_CHCFG11   EQU   0xB  ; Channel Configuration register
DMAMUX_CHCFG12   EQU   0xC  ; Channel Configuration register
DMAMUX_CHCFG13   EQU   0xD  ; Channel Configuration register
DMAMUX_CHCFG14   EQU   0xE  ; Channel Configuration register
DMAMUX_CHCFG15   EQU   0xF  ; Channel Configuration register

; ----------------------------------------------------------------------------
; -- DMAMUX Register Masks
; ----------------------------------------------------------------------------

DMAMUX_CHCFG_SOURCE_MASK  EQU  0x3F
DMAMUX_CHCFG_SOURCE_SHIFT EQU  0
DMAMUX_CHCFG_TRIG_MASK    EQU  0x40
DMAMUX_CHCFG_TRIG_SHIFT   EQU  6
DMAMUX_CHCFG_ENBL_MASK    EQU  0x80
DMAMUX_CHCFG_ENBL_SHIFT   EQU  7

; ----------------------------------------------------------------------------
; -- ENET Peripheral Access Layer
; ----------------------------------------------------------------------------

ENET_EIR                 EQU  0x004  ; Interrupt Event Register, offset: 0x4
ENET_EIMR                EQU  0x008  ; Interrupt Mask Register, offset: 0x8
ENET_RDAR                EQU  0x010  ; Receive Descriptor Active Register, offset: 0x10
ENET_TDAR                EQU  0x014  ; Transmit Descriptor Active Register, offset: 0x14
ENET_ECR                 EQU  0x024  ; Ethernet Control Register, offset: 0x24
ENET_MMFR                EQU  0x040  ; MII Management Frame Register, offset: 0x40
ENET_MSCR                EQU  0x044  ; MII Speed Control Register, offset: 0x44
ENET_MIBC                EQU  0x064  ; MIB Control Register, offset: 0x64
ENET_RCR                 EQU  0x084  ; Receive Control Register, offset: 0x84
ENET_TCR                 EQU  0x0C4  ; Transmit Control Register, offset: 0xC4
ENET_PALR                EQU  0x0E4  ; Physical Address Lower Register, offset: 0xE4
ENET_PAUR                EQU  0x0E8  ; Physical Address Upper Register, offset: 0xE8
ENET_OPD                 EQU  0x0EC  ; Opcode/Pause Duration Register, offset: 0xEC
ENET_IAUR                EQU  0x118  ; Descriptor Individual Upper Address Register, offset: 0x118
ENET_IALR                EQU  0x11C  ; Descriptor Individual Lower Address Register, offset: 0x11C
ENET_GAUR                EQU  0x120  ; Descriptor Group Upper Address Register, offset: 0x120
ENET_GALR                EQU  0x124  ; Descriptor Group Lower Address Register, offset: 0x124
ENET_TFWR                EQU  0x144  ; Transmit FIFO Watermark Register, offset: 0x144
ENET_RDSR                EQU  0x180  ; Receive Descriptor Ring Start Register, offset: 0x180
ENET_TDSR                EQU  0x184  ; Transmit Buffer Descriptor Ring Start Register, offset: 0x184
ENET_MRBR                EQU  0x188  ; Maximum Receive Buffer Size Register, offset: 0x188
ENET_RSFL                EQU  0x190  ; Receive FIFO Section Full Threshold, offset: 0x190
ENET_RSEM                EQU  0x194  ; Receive FIFO Section Empty Threshold, offset: 0x194
ENET_RAEM                EQU  0x198  ; Receive FIFO Almost Empty Threshold, offset: 0x198
ENET_RAFL                EQU  0x19C  ; Receive FIFO Almost Full Threshold, offset: 0x19C
ENET_TSEM                EQU  0x1A0  ; Transmit FIFO Section Empty Threshold, offset: 0x1A0
ENET_TAEM                EQU  0x1A4  ; Transmit FIFO Almost Empty Threshold, offset: 0x1A4
ENET_TAFL                EQU  0x1A8  ; Transmit FIFO Almost Full Threshold, offset: 0x1A8
ENET_TIPG                EQU  0x1AC  ; Transmit Inter-Packet Gap, offset: 0x1AC
ENET_FTRL                EQU  0x1B0  ; Frame Truncation Length, offset: 0x1B0
ENET_TACC                EQU  0x1C0  ; Transmit Accelerator Function Configuration, offset: 0x1C0
ENET_RACC                EQU  0x1C4  ; Receive Accelerator Function Configuration, offset: 0x1C4
ENET_RMON_T_PACKETS      EQU  0x204  ; Tx Packet Count Statistic Register, offset: 0x204
ENET_RMON_T_BC_PKT       EQU  0x208  ; Tx Broadcast Packets Statistic Register, offset: 0x208
ENET_RMON_T_MC_PKT       EQU  0x20C  ; Tx Multicast Packets Statistic Register, offset: 0x20C
ENET_RMON_T_CRC_ALIGN    EQU  0x210  ; Tx Packets with CRC/Align Error Statistic Register, offset: 0x210
ENET_RMON_T_UNDERSIZE    EQU  0x214  ; Tx Packets Less Than Bytes and Good CRC Statistic Register, offset: 0x214
ENET_RMON_T_OVERSIZE     EQU  0x218  ; Tx Packets GT MAX_FL bytes and Good CRC Statistic Register, offset: 0x218
ENET_RMON_T_FRAG         EQU  0x21C  ; Tx Packets Less Than 64 Bytes and Bad CRC Statistic Register, offset: 0x21C
ENET_RMON_T_JAB          EQU  0x220  ; Tx Packets Greater Than MAX_FL bytes and Bad CRC Statistic Register, offset: 0x220
ENET_RMON_T_COL          EQU  0x224  ; Tx Collision Count Statistic Register, offset: 0x224
ENET_RMON_T_P64          EQU  0x228  ; Tx 64-Byte Packets Statistic Register, offset: 0x228
ENET_RMON_T_P65TO127     EQU  0x22C  ; Tx 65- to 127-byte Packets Statistic Register, offset: 0x22C
ENET_RMON_T_P128TO255    EQU  0x230  ; Tx 128- to 255-byte Packets Statistic Register, offset: 0x230
ENET_RMON_T_P256TO511    EQU  0x234  ; Tx 256- to 511-byte Packets Statistic Register, offset: 0x234
ENET_RMON_T_P512TO1023   EQU  0x238  ; Tx 512- to 1023-byte Packets Statistic Register, offset: 0x238
ENET_RMON_T_P1024TO2047  EQU  0x23C  ; Tx 1024- to 2047-byte Packets Statistic Register, offset: 0x23C
ENET_RMON_T_P_GTE2048    EQU  0x240  ; Tx Packets Greater Than 2048 Bytes Statistic Register, offset: 0x240
ENET_RMON_T_OCTETS       EQU  0x244  ; Tx Octets Statistic Register, offset: 0x244
ENET_IEEE_T_FRAME_OK     EQU  0x24C  ; Frames Transmitted OK Statistic Register, offset: 0x24C
ENET_IEEE_T_1COL         EQU  0x250  ; Frames Transmitted with Single Collision Statistic Register, offset: 0x250
ENET_IEEE_T_MCOL         EQU  0x254  ; Frames Transmitted with Multiple Collisions Statistic Register, offset: 0x254
ENET_IEEE_T_DEF          EQU  0x258  ; Frames Transmitted after Deferral Delay Statistic Register, offset: 0x258
ENET_IEEE_T_LCOL         EQU  0x25C  ; Frames Transmitted with Late Collision Statistic Register, offset: 0x25C
ENET_IEEE_T_EXCOL        EQU  0x260  ; Frames Transmitted with Excessive Collisions Statistic Register, offset: 0x260
ENET_IEEE_T_MACERR       EQU  0x264  ; Frames Transmitted with Tx FIFO Underrun Statistic Register, offset: 0x264
ENET_IEEE_T_CSERR        EQU  0x268  ; Frames Transmitted with Carrier Sense Error Statistic Register, offset: 0x268
ENET_IEEE_T_FDXFC        EQU  0x270  ; Flow Control Pause Frames Transmitted Statistic Register, offset: 0x270
ENET_IEEE_T_OCTETS_OK    EQU  0x274  ; Octet Count for Frames Transmitted w/o Error Statistic Register, offset: 0x274
ENET_ENET_RMON_R_PACKETS EQU  0x284  ; Rx Packet Count Statistic Register, offset: 0x284
ENET_RMON_R_BC_PKT       EQU  0x284  ; Rx Broadcast Packets Statistic Register, offset: 0x288
ENET_RMON_R_MC_PKT       EQU  0x288  ; Rx Multicast Packets Statistic Register, offset: 0x28C
ENET_RMON_R_CRC_ALIGN    EQU  0x28C  ; Rx Packets with CRC/Align Error Statistic Register, offset: 0x290
ENET_RMON_R_UNDERSIZE    EQU  0x290  ; Rx Packets with Less Than 64 Bytes and Good CRC Statistic Register, offset: 0x294
ENET_RMON_R_OVERSIZE     EQU  0x294  ; Rx Packets Greater Than MAX_FL and Good CRC Statistic Register, offset: 0x298
ENET_RMON_R_FRAG         EQU  0x298  ; Rx Packets Less Than 64 Bytes and Bad CRC Statistic Register, offset: 0x29C
ENET_RMON_R_JAB          EQU  0x2A0  ; Rx Packets Greater Than MAX_FL Bytes and Bad CRC Statistic Register, offset: 0x2A0
ENET_RMON_R_P64          EQU  0x2A8  ; Rx 64-Byte Packets Statistic Register, offset: 0x2A8
ENET_RMON_R_P65TO127     EQU  0x2AC  ; Rx 65- to 127-Byte Packets Statistic Register, offset: 0x2AC
ENET_RMON_R_P128TO255    EQU  0x2B0  ; Rx 128- to 255-Byte Packets Statistic Register, offset: 0x2B0
ENET_RMON_R_P256TO511    EQU  0x2B4  ; Rx 256- to 511-Byte Packets Statistic Register, offset: 0x2B4
ENET_RMON_R_P512TO1023   EQU  0x2B8  ; Rx 512- to 1023-Byte Packets Statistic Register, offset: 0x2B8
ENET_RMON_R_P1024TO2047  EQU  0x2C0  ; Rx 1024- to 2047-Byte Packets Statistic Register, offset: 0x2BC
ENET_RMON_R_P_GTE2048    EQU  0x2C0  ; Rx Packets Greater than 2048 Bytes Statistic Register, offset: 0x2C0
ENET_RMON_R_OCTETS       EQU  0x2C4  ; Rx Octets Statistic Register, offset: 0x2C4
ENET_IEEE_R_DROP         EQU  0x2C8  ; Frames not Counted Correctly Statistic Register, offset: 0x2C8
ENET_IEEE_R_FRAME_OK     EQU  0x2CC  ; Frames Received OK Statistic Register, offset: 0x2CC
ENET_IEEE_R_CRC          EQU  0x2D0  ; Frames Received with CRC Error Statistic Register, offset: 0x2D0
ENET_IEEE_R_ALIGN        EQU  0x2D4  ; Frames Received with Alignment Error Statistic Register, offset: 0x2D4
ENET_IEEE_R_MACERR       EQU  0x2D8  ; Receive FIFO Overflow Count Statistic Register, offset: 0x2D8
ENET_IEEE_R_FDXFC        EQU  0x2DC  ; Flow Control Pause Frames Received Statistic Register, offset: 0x2DC
ENET_IEEE_R_OCTETS_OK    EQU  0x2E0  ; Octet Count for Frames Received without Error Statistic Register, offset: 0x2E0
ENET_ATCR                EQU  0x400  ; Adjustable Timer Control Register, offset: 0x400
ENET_ATVR                EQU  0x404  ; Timer Value Register, offset: 0x404
ENET_ATOFF               EQU  0x408  ; Timer Offset Register, offset: 0x408
ENET_ATPER               EQU  0x40C  ; Timer Period Register, offset: 0x40C
ENET_ATCOR               EQU  0x410  ; Timer Correction Register, offset: 0x410
ENET_ATINC               EQU  0x414  ; Time-Stamping Clock Period Register, offset: 0x414
ENET_ATSTMP              EQU  0x418  ; Timestamp of Last Transmitted Frame, offset: 0x418
ENET_TGSR                EQU  0x604  ; Timer Global Status Register, offset: 0x604

ENET_CHANNEL0_TCSR       EQU  0x608  ; Timer Control Status Register, array offset: 0x608, array step: 0x8
ENET_CHANNEL0_TCCR       EQU  0x60C  ; Timer Compare Capture Register, array offset: 0x60C, array step: 0x8

ENET_CHANNEL1_TCSR       EQU  0x610  ; Timer Control Status Register, array offset: 0x608, array step: 0x8
ENET_CHANNEL1_TCCR       EQU  0x614  ; Timer Compare Capture Register, array offset: 0x60C, array step: 0x8

ENET_CHANNEL2_TCSR       EQU  0x618  ; Timer Control Status Register, array offset: 0x608, array step: 0x8
ENET_CHANNEL2_TCCR       EQU  0x61C  ; Timer Compare Capture Register, array offset: 0x60C, array step: 0x8

ENET_CHANNEL3_TCSR       EQU  0x620  ; Timer Control Status Register, array offset: 0x608, array step: 0x8
ENET_CHANNEL3_TCCR       EQU  0x624  ; Timer Compare Capture Register, array offset: 0x60C, array step: 0x8

; ----------------------------------------------------------------------------
; -- ENET Register Masks
; ----------------------------------------------------------------------------

; EIR Bit Fields
ENET_EIR_TS_TIMER_MASK   EQU  0x8000
ENET_EIR_TS_TIMER_SHIFT  EQU  15
ENET_EIR_TS_AVAIL_MASK   EQU  0x10000
ENET_EIR_TS_AVAIL_SHIFT  EQU  16
ENET_EIR_WAKEUP_MASK     EQU  0x20000
ENET_EIR_WAKEUP_SHIFT    EQU  17
ENET_EIR_PLR_MASK        EQU  0x40000
ENET_EIR_PLR_SHIFT       EQU  18
ENET_EIR_UN_MASK         EQU  0x80000
ENET_EIR_UN_SHIFT        EQU  19
ENET_EIR_RL_MASK         EQU  0x100000
ENET_EIR_RL_SHIFT        EQU  20
ENET_EIR_LC_MASK         EQU  0x200000
ENET_EIR_LC_SHIFT        EQU  21
ENET_EIR_EBERR_MASK      EQU  0x400000
ENET_EIR_EBERR_SHIFT     EQU  22
ENET_EIR_MII_MASK        EQU  0x800000
ENET_EIR_MII_SHIFT       EQU  23
ENET_EIR_RXB_MASK        EQU  0x1000000
ENET_EIR_RXB_SHIFT       EQU  24
ENET_EIR_RXF_MASK        EQU  0x2000000
ENET_EIR_RXF_SHIFT       EQU  25
ENET_EIR_TXB_MASK        EQU  0x4000000
ENET_EIR_TXB_SHIFT       EQU  26
ENET_EIR_TXF_MASK        EQU  0x8000000
ENET_EIR_TXF_SHIFT       EQU  27
ENET_EIR_GRA_MASK        EQU  0x10000000
ENET_EIR_GRA_SHIFT       EQU  28
ENET_EIR_BABT_MASK       EQU  0x20000000
ENET_EIR_BABT_SHIFT      EQU  29
ENET_EIR_BABR_MASK       EQU  0x40000000
ENET_EIR_BABR_SHIFT      EQU  30

; EIMR Bit Fields
ENET_EIMR_TS_TIMER_MASK  EQU  0x8000
ENET_EIMR_TS_TIMER_SHIFT EQU  15
ENET_EIMR_TS_AVAIL_MASK  EQU  0x10000
ENET_EIMR_TS_AVAIL_SHIFT EQU  16
ENET_EIMR_WAKEUP_MASK    EQU  0x20000
ENET_EIMR_WAKEUP_SHIFT   EQU  17
ENET_EIMR_PLR_MASK       EQU  0x40000
ENET_EIMR_PLR_SHIFT      EQU  18
ENET_EIMR_UN_MASK        EQU  0x80000
ENET_EIMR_UN_SHIFT       EQU  19
ENET_EIMR_RL_MASK        EQU  0x100000
ENET_EIMR_RL_SHIFT       EQU  20
ENET_EIMR_LC_MASK        EQU  0x200000
ENET_EIMR_LC_SHIFT       EQU  21
ENET_EIMR_EBERR_MASK     EQU  0x400000
ENET_EIMR_EBERR_SHIFT    EQU  22
ENET_EIMR_MII_MASK       EQU  0x800000
ENET_EIMR_MII_SHIFT      EQU  23
ENET_EIMR_RXB_MASK       EQU  0x1000000
ENET_EIMR_RXB_SHIFT      EQU  24
ENET_EIMR_RXF_MASK       EQU  0x2000000
ENET_EIMR_RXF_SHIFT      EQU  25
ENET_EIMR_TXB_MASK       EQU  0x4000000
ENET_EIMR_TXB_SHIFT      EQU  26
ENET_EIMR_TXF_MASK       EQU  0x8000000
ENET_EIMR_TXF_SHIFT      EQU  27
ENET_EIMR_GRA_MASK       EQU  0x10000000
ENET_EIMR_GRA_SHIFT      EQU  28
ENET_EIMR_BABT_MASK      EQU  0x20000000
ENET_EIMR_BABT_SHIFT     EQU  29
ENET_EIMR_BABR_MASK      EQU  0x40000000
ENET_EIMR_BABR_SHIFT     EQU  30

; RDAR Bit Fields
ENET_RDAR_RDAR_MASK      EQU  0x1000000
ENET_RDAR_RDAR_SHIFT     EQU  24

; TDAR Bit Fields
ENET_TDAR_TDAR_MASK      EQU  0x1000000
ENET_TDAR_TDAR_SHIFT     EQU  24

; ECR Bit Fields
ENET_ECR_RESET_MASK      EQU  0x1
ENET_ECR_RESET_SHIFT     EQU  0
ENET_ECR_ETHEREN_MASK    EQU  0x2
ENET_ECR_ETHEREN_SHIFT   EQU  1
ENET_ECR_MAGICEN_MASK    EQU  0x4
ENET_ECR_MAGICEN_SHIFT   EQU  2
ENET_ECR_SLEEP_MASK      EQU  0x8
ENET_ECR_SLEEP_SHIFT     EQU  3
ENET_ECR_EN1588_MASK     EQU  0x10
ENET_ECR_EN1588_SHIFT    EQU  4
ENET_ECR_DBGEN_MASK      EQU  0x40
ENET_ECR_DBGEN_SHIFT     EQU  6
ENET_ECR_STOPEN_MASK     EQU  0x80
ENET_ECR_STOPEN_SHIFT    EQU  7
ENET_ECR_DBSWP_MASK      EQU  0x100
ENET_ECR_DBSWP_SHIFT     EQU  8

; MMFR Bit Fields
ENET_MMFR_DATA_MASK      EQU  0xFFFF
ENET_MMFR_DATA_SHIFT     EQU  0
ENET_MMFR_TA_MASK        EQU  0x30000
ENET_MMFR_TA_SHIFT       EQU  16
ENET_MMFR_RA_MASK        EQU  0x7C0000
ENET_MMFR_RA_SHIFT       EQU  18
ENET_MMFR_PA_MASK        EQU  0xF800000
ENET_MMFR_PA_SHIFT       EQU  23
ENET_MMFR_OP_MASK        EQU  0x30000000
ENET_MMFR_OP_SHIFT       EQU  28
ENET_MMFR_ST_MASK        EQU  0xC0000000
ENET_MMFR_ST_SHIFT       EQU  30

; MSCR Bit Fields
ENET_MSCR_MII_SPEED_MASK  EQU  0x7E
ENET_MSCR_MII_SPEED_SHIFT EQU  1
ENET_MSCR_DIS_PRE_MASK    EQU  0x80
ENET_MSCR_DIS_PRE_SHIFT   EQU  7
ENET_MSCR_HOLDTIME_MASK   EQU  0x700
ENET_MSCR_HOLDTIME_SHIFT  EQU  8
; MIBC Bit Fields

ENET_MIBC_MIB_CLEAR_MASK  EQU  0x20000000
ENET_MIBC_MIB_CLEAR_SHIFT EQU  29
ENET_MIBC_MIB_IDLE_MASK   EQU  0x40000000
ENET_MIBC_MIB_IDLE_SHIFT  EQU  30
ENET_MIBC_MIB_DIS_MASK    EQU  0x80000000
ENET_MIBC_MIB_DIS_SHIFT   EQU  31
; RCR Bit Fields

ENET_RCR_LOOP_MASK       EQU  0x1
ENET_RCR_LOOP_SHIFT      EQU  0
ENET_RCR_DRT_MASK        EQU  0x2
ENET_RCR_DRT_SHIFT       EQU  1
ENET_RCR_MII_MODE_MASK   EQU  0x4
ENET_RCR_MII_MODE_SHIFT  EQU  2
ENET_RCR_PROM_MASK       EQU  0x8
ENET_RCR_PROM_SHIFT      EQU  3
ENET_RCR_BC_REJ_MASK     EQU  0x10
ENET_RCR_BC_REJ_SHIFT    EQU  4
ENET_RCR_FCE_MASK        EQU  0x20
ENET_RCR_FCE_SHIFT       EQU  5
ENET_RCR_RMII_MODE_MASK  EQU  0x100
ENET_RCR_RMII_MODE_SHIFT EQU  8
ENET_RCR_RMII_10T_MASK   EQU  0x200
ENET_RCR_RMII_10T_SHIFT  EQU  9
ENET_RCR_PADEN_MASK      EQU  0x1000
ENET_RCR_PADEN_SHIFT     EQU  12
ENET_RCR_PAUFWD_MASK     EQU  0x2000
ENET_RCR_PAUFWD_SHIFT    EQU  13
ENET_RCR_CRCFWD_MASK     EQU  0x4000
ENET_RCR_CRCFWD_SHIFT    EQU  14
ENET_RCR_CFEN_MASK       EQU  0x8000
ENET_RCR_CFEN_SHIFT      EQU  15
ENET_RCR_MAX_FL_MASK     EQU  0x3FFF0000
ENET_RCR_MAX_FL_SHIFT    EQU  16
ENET_RCR_NLC_MASK        EQU  0x40000000
ENET_RCR_NLC_SHIFT       EQU  30
ENET_RCR_GRS_MASK        EQU  0x80000000
ENET_RCR_GRS_SHIFT       EQU  31

; TCR Bit Fields
ENET_TCR_GTS_MASK        EQU  0x1
ENET_TCR_GTS_SHIFT       EQU  0
ENET_TCR_FDEN_MASK       EQU  0x4
ENET_TCR_FDEN_SHIFT      EQU  2
ENET_TCR_TFC_PAUSE_MASK  EQU  0x8
ENET_TCR_TFC_PAUSE_SHIFT EQU  3
ENET_TCR_RFC_PAUSE_MASK  EQU  0x10
ENET_TCR_RFC_PAUSE_SHIFT EQU  4
ENET_TCR_ADDSEL_MASK     EQU  0xE0
ENET_TCR_ADDSEL_SHIFT    EQU  5
ENET_TCR_ADDINS_MASK     EQU  0x100
ENET_TCR_ADDINS_SHIFT    EQU  8
ENET_TCR_CRCFWD_MASK     EQU  0x200
ENET_TCR_CRCFWD_SHIFT    EQU  9

; PALR Bit Fields
ENET_PALR_PADDR1_MASK    EQU  0xFFFFFFFF
ENET_PALR_PADDR1_SHIFT   EQU  0

; PAUR Bit Fields
ENET_PAUR_TYPE_MASK      EQU  0xFFFF
ENET_PAUR_TYPE_SHIFT     EQU  0
ENET_PAUR_PADDR2_MASK    EQU  0xFFFF0000
ENET_PAUR_PADDR2_SHIFT   EQU  16

; OPD Bit Fields
ENET_OPD_PAUSE_DUR_MASK  EQU  0xFFFF
ENET_OPD_PAUSE_DUR_SHIFT EQU  0
ENET_OPD_OPCODE_MASK     EQU  0xFFFF0000
ENET_OPD_OPCODE_SHIFT    EQU  16

; IAUR Bit Fields
ENET_IAUR_IADDR1_MASK    EQU  0xFFFFFFFF
ENET_IAUR_IADDR1_SHIFT   EQU  0

; IALR Bit Fields
ENET_IALR_IADDR2_MASK    EQU  0xFFFFFFFF
ENET_IALR_IADDR2_SHIFT   EQU  0

; GAUR Bit Fields
ENET_GAUR_GADDR1_MASK    EQU  0xFFFFFFFF
ENET_GAUR_GADDR1_SHIFT   EQU  0

; GALR Bit Fields
ENET_GALR_GADDR2_MASK    EQU  0xFFFFFFFF
ENET_GALR_GADDR2_SHIFT   EQU  0

; TFWR Bit Fields
ENET_TFWR_TFWR_MASK      EQU  0x3F
ENET_TFWR_TFWR_SHIFT     EQU  0
ENET_TFWR_STRFWD_MASK    EQU  0x100
ENET_TFWR_STRFWD_SHIFT   EQU  8

; RDSR Bit Fields
ENET_RDSR_R_DES_START_MASK           EQU  0xFFFFFFF8
ENET_RDSR_R_DES_START_SHIFT          EQU  3

; TDSR Bit Fields
ENET_TDSR_X_DES_START_MASK           EQU  0xFFFFFFF8
ENET_TDSR_X_DES_START_SHIFT          EQU  3

; MRBR Bit Fields
ENET_MRBR_R_BUF_SIZE_MASK            EQU  0x3FF0
ENET_MRBR_R_BUF_SIZE_SHIFT           EQU  4
; RSFL Bit Fields
ENET_RSFL_RX_SECTION_FULL_MASK       EQU  0xFF
ENET_RSFL_RX_SECTION_FULL_SHIFT      EQU  0

; RSEM Bit Fields
ENET_RSEM_RX_SECTION_EMPTY_MASK      EQU  0xFF
ENET_RSEM_RX_SECTION_EMPTY_SHIFT     EQU  0
ENET_RSEM_STAT_SECTION_EMPTY_MASK    EQU  0x1F0000
ENET_RSEM_STAT_SECTION_EMPTY_SHIFT   EQU  16

; RAEM Bit Fields
ENET_RAEM_RX_ALMOST_EMPTY_MASK       EQU  0xFF
ENET_RAEM_RX_ALMOST_EMPTY_SHIFT      EQU  0

; RAFL Bit Fields
ENET_RAFL_RX_ALMOST_FULL_MASK        EQU  0xFF
ENET_RAFL_RX_ALMOST_FULL_SHIFT       EQU  0

; TSEM Bit Fields
ENET_TSEM_TX_SECTION_EMPTY_MASK      EQU  0xFF
ENET_TSEM_TX_SECTION_EMPTY_SHIFT     EQU  0

; TAEM Bit Fields
ENET_TAEM_TX_ALMOST_EMPTY_MASK       EQU  0xFF
ENET_TAEM_TX_ALMOST_EMPTY_SHIFT      EQU  0

; TAFL Bit Fields
ENET_TAFL_TX_ALMOST_FULL_MASK        EQU  0xFF
ENET_TAFL_TX_ALMOST_FULL_SHIFT       EQU  0

; TIPG Bit Fields
ENET_TIPG_IPG_MASK                   EQU  0x1F
ENET_TIPG_IPG_SHIFT                  EQU  0

; FTRL Bit Fields
ENET_FTRL_TRUNC_FL_MASK              EQU  0x3FFF
ENET_FTRL_TRUNC_FL_SHIFT             EQU  0

; TACC Bit Fields
ENET_TACC_SHIFT16_MASK               EQU  0x1
ENET_TACC_SHIFT16_SHIFT              EQU  0
ENET_TACC_IPCHK_MASK                 EQU  0x8
ENET_TACC_IPCHK_SHIFT                EQU  3
ENET_TACC_PROCHK_MASK                EQU  0x10
ENET_TACC_PROCHK_SHIFT               EQU  4

; RACC Bit Fields
ENET_RACC_PADREM_MASK                EQU  0x1
ENET_RACC_PADREM_SHIFT               EQU  0
ENET_RACC_IPDIS_MASK                 EQU  0x2
ENET_RACC_IPDIS_SHIFT                EQU  1
ENET_RACC_PRODIS_MASK                EQU  0x4
ENET_RACC_PRODIS_SHIFT               EQU  2
ENET_RACC_LINEDIS_MASK               EQU  0x40
ENET_RACC_LINEDIS_SHIFT              EQU  6
ENET_RACC_SHIFT16_MASK               EQU  0x80
ENET_RACC_SHIFT16_SHIFT              EQU  7

; RMON_T_PACKETS Bit Fields
ENET_RMON_T_PACKETS_TXPKTS_MASK      EQU  0xFFFF
ENET_RMON_T_PACKETS_TXPKTS_SHIFT     EQU  0

; RMON_T_BC_PKT Bit Fields
ENET_RMON_T_BC_PKT_TXPKTS_MASK       EQU  0xFFFF
ENET_RMON_T_BC_PKT_TXPKTS_SHIFT      EQU  0

; RMON_T_MC_PKT Bit Fields
ENET_RMON_T_MC_PKT_TXPKTS_MASK       EQU  0xFFFF
ENET_RMON_T_MC_PKT_TXPKTS_SHIFT      EQU  0

; RMON_T_CRC_ALIGN Bit Fields
ENET_RMON_T_CRC_ALIGN_TXPKTS_MASK    EQU  0xFFFF
ENET_RMON_T_CRC_ALIGN_TXPKTS_SHIFT   EQU  0

; RMON_T_UNDERSIZE Bit Fields
ENET_RMON_T_UNDERSIZE_TXPKTS_MASK    EQU  0xFFFF
ENET_RMON_T_UNDERSIZE_TXPKTS_SHIFT   EQU  0

; RMON_T_OVERSIZE Bit Fields
ENET_RMON_T_OVERSIZE_TXPKTS_MASK     EQU  0xFFFF
ENET_RMON_T_OVERSIZE_TXPKTS_SHIFT    EQU  0

; RMON_T_FRAG Bit Fields
ENET_RMON_T_FRAG_TXPKTS_MASK         EQU  0xFFFF
ENET_RMON_T_FRAG_TXPKTS_SHIFT        EQU  0

; RMON_T_JAB Bit Fields
ENET_RMON_T_JAB_TXPKTS_MASK          EQU  0xFFFF
ENET_RMON_T_JAB_TXPKTS_SHIFT         EQU  0

; RMON_T_COL Bit Fields
ENET_RMON_T_COL_TXPKTS_MASK          EQU  0xFFFF
ENET_RMON_T_COL_TXPKTS_SHIFT         EQU  0

; RMON_T_P64 Bit Fields
ENET_RMON_T_P64_TXPKTS_MASK          EQU  0xFFFF
ENET_RMON_T_P64_TXPKTS_SHIFT         EQU  0

; RMON_T_P65TO127 Bit Fields
ENET_RMON_T_P65TO127_TXPKTS_MASK     EQU  0xFFFF
ENET_RMON_T_P65TO127_TXPKTS_SHIFT    EQU  0

; RMON_T_P128TO255 Bit Fields
ENET_RMON_T_P128TO255_TXPKTS_MASK    EQU  0xFFFF
ENET_RMON_T_P128TO255_TXPKTS_SHIFT   EQU  0

; RMON_T_P256TO511 Bit Fields
ENET_RMON_T_P256TO511_TXPKTS_MASK    EQU  0xFFFF
ENET_RMON_T_P256TO511_TXPKTS_SHIFT   EQU  0

; RMON_T_P512TO1023 Bit Fields
ENET_RMON_T_P512TO1023_TXPKTS_MASK   EQU  0xFFFF
ENET_RMON_T_P512TO1023_TXPKTS_SHIFT  EQU  0

; RMON_T_P1024TO2047 Bit Fields
ENET_RMON_T_P1024TO2047_TXPKTS_MASK  EQU  0xFFFF
ENET_RMON_T_P1024TO2047_TXPKTS_SHIFT EQU  0

; RMON_T_P_GTE2048 Bit Fields
ENET_RMON_T_P_GTE2048_TXPKTS_MASK    EQU  0xFFFF
ENET_RMON_T_P_GTE2048_TXPKTS_SHIFT   EQU  0

; RMON_T_OCTETS Bit Fields
ENET_RMON_T_OCTETS_TXOCTS_MASK       EQU  0xFFFFFFFF
ENET_RMON_T_OCTETS_TXOCTS_SHIFT      EQU  0

; IEEE_T_FRAME_OK Bit Fields
ENET_IEEE_T_FRAME_OK_COUNT_MASK      EQU  0xFFFF
ENET_IEEE_T_FRAME_OK_COUNT_SHIFT     EQU  0

; IEEE_T_1COL Bit Fields
ENET_IEEE_T_1COL_COUNT_MASK          EQU  0xFFFF
ENET_IEEE_T_1COL_COUNT_SHIFT         EQU  0

; IEEE_T_MCOL Bit Fields
ENET_IEEE_T_MCOL_COUNT_MASK          EQU  0xFFFF
ENET_IEEE_T_MCOL_COUNT_SHIFT         EQU  0

; IEEE_T_DEF Bit Fields
ENET_IEEE_T_DEF_COUNT_MASK           EQU  0xFFFF
ENET_IEEE_T_DEF_COUNT_SHIFT          EQU  0

; IEEE_T_LCOL Bit Fields
ENET_IEEE_T_LCOL_COUNT_MASK          EQU  0xFFFF
ENET_IEEE_T_LCOL_COUNT_SHIFT         EQU  0

; IEEE_T_EXCOL Bit Fields
ENET_IEEE_T_EXCOL_COUNT_MASK         EQU  0xFFFF
ENET_IEEE_T_EXCOL_COUNT_SHIFT        EQU  0

; IEEE_T_MACERR Bit Fields
ENET_IEEE_T_MACERR_COUNT_MASK        EQU  0xFFFF
ENET_IEEE_T_MACERR_COUNT_SHIFT       EQU  0

; IEEE_T_CSERR Bit Fields
ENET_IEEE_T_CSERR_COUNT_MASK         EQU  0xFFFF
ENET_IEEE_T_CSERR_COUNT_SHIFT        EQU  0

; IEEE_T_FDXFC Bit Fields
ENET_IEEE_T_FDXFC_COUNT_MASK         EQU  0xFFFF
ENET_IEEE_T_FDXFC_COUNT_SHIFT        EQU  0

; IEEE_T_OCTETS_OK Bit Fields
ENET_IEEE_T_OCTETS_OK_COUNT_MASK     EQU  0xFFFFFFFF
ENET_IEEE_T_OCTETS_OK_COUNT_SHIFT    EQU  0

; RMON_R_PACKETS Bit Fields
ENET_RMON_R_PACKETS_COUNT_MASK       EQU  0xFFFF
ENET_RMON_R_PACKETS_COUNT_SHIFT      EQU  0

; RMON_R_BC_PKT Bit Fields
ENET_RMON_R_BC_PKT_COUNT_MASK        EQU  0xFFFF
ENET_RMON_R_BC_PKT_COUNT_SHIFT       EQU  0

; RMON_R_MC_PKT Bit Fields
ENET_RMON_R_MC_PKT_COUNT_MASK        EQU  0xFFFF
ENET_RMON_R_MC_PKT_COUNT_SHIFT       EQU  0

; RMON_R_CRC_ALIGN Bit Fields
ENET_RMON_R_CRC_ALIGN_COUNT_MASK     EQU  0xFFFF
ENET_RMON_R_CRC_ALIGN_COUNT_SHIFT    EQU  0

; RMON_R_UNDERSIZE Bit Fields
ENET_RMON_R_UNDERSIZE_COUNT_MASK     EQU  0xFFFF
ENET_RMON_R_UNDERSIZE_COUNT_SHIFT    EQU  0

; RMON_R_OVERSIZE Bit Fields
ENET_RMON_R_OVERSIZE_COUNT_MASK      EQU  0xFFFF
ENET_RMON_R_OVERSIZE_COUNT_SHIFT     EQU  0

; RMON_R_FRAG Bit Fields
ENET_RMON_R_FRAG_COUNT_MASK          EQU  0xFFFF
ENET_RMON_R_FRAG_COUNT_SHIFT         EQU  0

; RMON_R_JAB Bit Fields
ENET_RMON_R_JAB_COUNT_MASK           EQU  0xFFFF
ENET_RMON_R_JAB_COUNT_SHIFT          EQU  0

; RMON_R_P64 Bit Fields
ENET_RMON_R_P64_COUNT_MASK           EQU  0xFFFF
ENET_RMON_R_P64_COUNT_SHIFT          EQU  0

; RMON_R_P65TO127 Bit Fields
ENET_RMON_R_P65TO127_COUNT_MASK      EQU  0xFFFF
ENET_RMON_R_P65TO127_COUNT_SHIFT     EQU  0

; RMON_R_P128TO255 Bit Fields
ENET_RMON_R_P128TO255_COUNT_MASK     EQU  0xFFFF
ENET_RMON_R_P128TO255_COUNT_SHIFT    EQU  0

; RMON_R_P256TO511 Bit Fields
ENET_RMON_R_P256TO511_COUNT_MASK     EQU  0xFFFF
ENET_RMON_R_P256TO511_COUNT_SHIFT    EQU  0

; RMON_R_P512TO1023 Bit Fields
ENET_RMON_R_P512TO1023_COUNT_MASK    EQU  0xFFFF
ENET_RMON_R_P512TO1023_COUNT_SHIFT   EQU  0

; RMON_R_P1024TO2047 Bit Fields
ENET_RMON_R_P1024TO2047_COUNT_MASK   EQU  0xFFFF
ENET_RMON_R_P1024TO2047_COUNT_SHIFT  EQU  0

; RMON_R_P_GTE2048 Bit Fields
ENET_RMON_R_P_GTE2048_COUNT_MASK     EQU  0xFFFF
ENET_RMON_R_P_GTE2048_COUNT_SHIFT    EQU  0

; RMON_R_OCTETS Bit Fields
ENET_RMON_R_OCTETS_COUNT_MASK        EQU  0xFFFFFFFF
ENET_RMON_R_OCTETS_COUNT_SHIFT       EQU  0

; IEEE_R_DROP Bit Fields
ENET_IEEE_R_DROP_COUNT_MASK          EQU  0xFFFF
ENET_IEEE_R_DROP_COUNT_SHIFT         EQU  0

; IEEE_R_FRAME_OK Bit Fields
ENET_IEEE_R_FRAME_OK_COUNT_MASK      EQU  0xFFFF
ENET_IEEE_R_FRAME_OK_COUNT_SHIFT     EQU  0

; IEEE_R_CRC Bit Fields
ENET_IEEE_R_CRC_COUNT_MASK           EQU  0xFFFF
ENET_IEEE_R_CRC_COUNT_SHIFT          EQU  0

; IEEE_R_ALIGN Bit Fields
ENET_IEEE_R_ALIGN_COUNT_MASK         EQU  0xFFFF
ENET_IEEE_R_ALIGN_COUNT_SHIFT        EQU  0

; IEEE_R_MACERR Bit Fields
ENET_IEEE_R_MACERR_COUNT_MASK        EQU  0xFFFF
ENET_IEEE_R_MACERR_COUNT_SHIFT       EQU  0

; IEEE_R_FDXFC Bit Fields
ENET_IEEE_R_FDXFC_COUNT_MASK         EQU  0xFFFF
ENET_IEEE_R_FDXFC_COUNT_SHIFT        EQU  0

; IEEE_R_OCTETS_OK Bit Fields
ENET_IEEE_R_OCTETS_OK_COUNT_MASK     EQU  0xFFFFFFFF
ENET_IEEE_R_OCTETS_OK_COUNT_SHIFT    EQU  0

; ATCR Bit Fields
ENET_ATCR_EN_MASK        EQU  0x1
ENET_ATCR_EN_SHIFT       EQU  0
ENET_ATCR_OFFEN_MASK     EQU  0x4
ENET_ATCR_OFFEN_SHIFT    EQU  2
ENET_ATCR_OFFRST_MASK    EQU  0x8
ENET_ATCR_OFFRST_SHIFT   EQU  3
ENET_ATCR_PEREN_MASK     EQU  0x10
ENET_ATCR_PEREN_SHIFT    EQU  4
ENET_ATCR_PINPER_MASK    EQU  0x80
ENET_ATCR_PINPER_SHIFT   EQU  7
ENET_ATCR_RESTART_MASK   EQU  0x200
ENET_ATCR_RESTART_SHIFT  EQU  9
ENET_ATCR_CAPTURE_MASK   EQU  0x800
ENET_ATCR_CAPTURE_SHIFT  EQU  11
ENET_ATCR_SLAVE_MASK     EQU  0x2000
ENET_ATCR_SLAVE_SHIFT    EQU  13

; ATVR Bit Fields
ENET_ATVR_ATIME_MASK     EQU  0xFFFFFFFF
ENET_ATVR_ATIME_SHIFT    EQU  0

; ATOFF Bit Fields
ENET_ATOFF_OFFSET_MASK   EQU  0xFFFFFFFF
ENET_ATOFF_OFFSET_SHIFT  EQU  0

; ATPER Bit Fields
ENET_ATPER_PERIOD_MASK   EQU  0xFFFFFFFF
ENET_ATPER_PERIOD_SHIFT  EQU  0

; ATCOR Bit Fields
ENET_ATCOR_COR_MASK      EQU  0x7FFFFFFF
ENET_ATCOR_COR_SHIFT     EQU  0

; ATINC Bit Fields
ENET_ATINC_INC_MASK       EQU  0x7F
ENET_ATINC_INC_SHIFT      EQU  0
ENET_ATINC_INC_CORR_MASK  EQU  0x7F00
ENET_ATINC_INC_CORR_SHIFT EQU  8

; ATSTMP Bit Fields
ENET_ATSTMP_TIMESTAMP_MASK   EQU  0xFFFFFFFF
ENET_ATSTMP_TIMESTAMP_SHIFT  EQU  0

; TGSR Bit Fields
ENET_TGSR_TF0_MASK       EQU  0x1
ENET_TGSR_TF0_SHIFT      EQU  0
ENET_TGSR_TF1_MASK       EQU  0x2
ENET_TGSR_TF1_SHIFT      EQU  1
ENET_TGSR_TF2_MASK       EQU  0x4
ENET_TGSR_TF2_SHIFT      EQU  2
ENET_TGSR_TF3_MASK       EQU  0x8
ENET_TGSR_TF3_SHIFT      EQU  3

; TCSR Bit Fields
ENET_TCSR_TDRE_MASK      EQU  0x1
ENET_TCSR_TDRE_SHIFT     EQU  0
ENET_TCSR_TMODE_MASK     EQU  0x3C
ENET_TCSR_TMODE_SHIFT    EQU  2
ENET_TCSR_TIE_MASK       EQU  0x40
ENET_TCSR_TIE_SHIFT      EQU  6
ENET_TCSR_TF_MASK        EQU  0x80
ENET_TCSR_TF_SHIFT       EQU  7

; TCCR Bit Fields
ENET_TCCR_TCC_MASK       EQU  0xFFFFFFFF
ENET_TCCR_TCC_SHIFT      EQU  0

; ----------------------------------------------------------------------------
; -- EWM Peripheral Access Layer
; ----------------------------------------------------------------------------

EWM_CTRL  EQU  0x0    ; Control Register, offset: 0x0
EWM_SERV  EQU  0x1    ; Service Register, offset: 0x1
EWM_CMPL  EQU  0x2    ; Compare Low Register, offset: 0x2
EWM_CMPH  EQU  0x3    ; Compare High Register, offset: 0x3

; ----------------------------------------------------------------------------
; -- EWM Register Masks
; ----------------------------------------------------------------------------

; CTRL Bit Fields
EWM_CTRL_EWMEN_MASK      EQU  0x1
EWM_CTRL_EWMEN_SHIFT     EQU  0
EWM_CTRL_ASSIN_MASK      EQU  0x2
EWM_CTRL_ASSIN_SHIFT     EQU  1
EWM_CTRL_INEN_MASK       EQU  0x4
EWM_CTRL_INEN_SHIFT      EQU  2
EWM_CTRL_INTEN_MASK      EQU  0x8
EWM_CTRL_INTEN_SHIFT     EQU  3

; SERV Bit Fields
EWM_SERV_SERVICE_MASK    EQU  0xFF
EWM_SERV_SERVICE_SHIFT   EQU  0

; CMPL Bit Fields
EWM_CMPL_COMPAREL_MASK   EQU  0xFF
EWM_CMPL_COMPAREL_SHIFT  EQU  0

; CMPH Bit Fields
EWM_CMPH_COMPAREH_MASK   EQU  0xFF
EWM_CMPH_COMPAREH_SHIFT  EQU  0

; ----------------------------------------------------------------------------
; -- FB Peripheral Access Layer
; ----------------------------------------------------------------------------

FB_CS0_CSAR   EQU  0x00   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS0_CSMR   EQU  0x04   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS0_CSCR   EQU  0x08   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CS1_CSAR   EQU  0x0C   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS1_CSMR   EQU  0x10   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS1_CSCR   EQU  0x14   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CS2_CSAR   EQU  0x18   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS2_CSMR   EQU  0x1C   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS2_CSCR   EQU  0x20   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CS3_CSAR   EQU  0x24   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS3_CSMR   EQU  0x28   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS3_CSCR   EQU  0x2C   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CS4_CSAR   EQU  0x30   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS4_CSMR   EQU  0x34   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS4_CSCR   EQU  0x38   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CS5_CSAR   EQU  0x3C   ; Chip Select Address Register, array offset: 0x0, array step: 0xC
FB_CS5_CSMR   EQU  0x40   ; Chip Select Mask Register, array offset: 0x4, array step: 0xC
FB_CS5_CSCR   EQU  0x44   ; Chip Select Control Register, array offset: 0x8, array step: 0xC

FB_CSPMCR     EQU  0x60   ; Chip Select port Multiplexing Control Register, offset: 0x60

; ----------------------------------------------------------------------------
; -- FB Register Masks
; ----------------------------------------------------------------------------

; CSAR Bit Fields
FB_CSAR_BA_MASK          EQU  0xFFFF0000
FB_CSAR_BA_SHIFT         EQU  16

; CSMR Bit Fields
FB_CSMR_V_MASK           EQU  0x1
FB_CSMR_V_SHIFT          EQU  0
FB_CSMR_WP_MASK          EQU  0x100
FB_CSMR_WP_SHIFT         EQU  8
FB_CSMR_BAM_MASK         EQU  0xFFFF0000
FB_CSMR_BAM_SHIFT        EQU  16

; CSCR Bit Fields
FB_CSCR_BSTW_MASK        EQU  0x8
FB_CSCR_BSTW_SHIFT       EQU  3
FB_CSCR_BSTR_MASK        EQU  0x10
FB_CSCR_BSTR_SHIFT       EQU  4
FB_CSCR_BEM_MASK         EQU  0x20
FB_CSCR_BEM_SHIFT        EQU  5
FB_CSCR_PS_MASK          EQU  0xC0
FB_CSCR_PS_SHIFT         EQU  6
FB_CSCR_AA_MASK          EQU  0x100
FB_CSCR_AA_SHIFT         EQU  8
FB_CSCR_BLS_MASK         EQU  0x200
FB_CSCR_BLS_SHIFT        EQU  9
FB_CSCR_WS_MASK          EQU  0xFC00
FB_CSCR_WS_SHIFT         EQU  10
FB_CSCR_WRAH_MASK        EQU  0x30000
FB_CSCR_WRAH_SHIFT       EQU  16
FB_CSCR_RDAH_MASK        EQU  0xC0000
FB_CSCR_RDAH_SHIFT       EQU  18
FB_CSCR_ASET_MASK        EQU  0x300000
FB_CSCR_ASET_SHIFT       EQU  20
FB_CSCR_EXTS_MASK        EQU  0x400000
FB_CSCR_EXTS_SHIFT       EQU  22
FB_CSCR_SWSEN_MASK       EQU  0x800000
FB_CSCR_SWSEN_SHIFT      EQU  23
FB_CSCR_SWS_MASK         EQU  0xFC000000
FB_CSCR_SWS_SHIFT        EQU  26

; CSPMCR Bit Fields
FB_CSPMCR_GROUP5_MASK    EQU  0xF000
FB_CSPMCR_GROUP5_SHIFT   EQU  12
FB_CSPMCR_GROUP4_MASK    EQU  0xF0000
FB_CSPMCR_GROUP4_SHIFT   EQU  16
FB_CSPMCR_GROUP3_MASK    EQU  0xF00000
FB_CSPMCR_GROUP3_SHIFT   EQU  20
FB_CSPMCR_GROUP2_MASK    EQU  0xF000000
FB_CSPMCR_GROUP2_SHIFT   EQU  24
FB_CSPMCR_GROUP1_MASK    EQU  0xF0000000
FB_CSPMCR_GROUP1_SHIFT   EQU  28

; ----------------------------------------------------------------------------
; -- FMC Peripheral Access Layer
; ----------------------------------------------------------------------------

FMC_PFAPR       EQU  0x00   ; Flash Access Protection Register, offset: 0x0
FMC_PFB0CR      EQU  0X04   ; Flash Bank 0 Control Register, offset: 0x4
FMC_PFB1CR      EQU  0X08   ; Flash Bank 1 Control Register, offset: 0x8

FMC_TAGVDW0S_0  EQU  0x100  ; Cache Tag Storage, array offset: 0x100, array step: 0x4
FMC_TAGVDW0S_1  EQU  0x104  ; Cache Tag Storage, array offset: 0x100, array step: 0x4
FMC_TAGVDW0S_2  EQU  0x108  ; Cache Tag Storage, array offset: 0x100, array step: 0x4
FMC_TAGVDW0S_3  EQU  0x10C  ; Cache Tag Storage, array offset: 0x100, array step: 0x4

FMC_TAGVDW1S_0  EQU  0x110  ; Cache Tag Storage, array offset: 0x110, array step: 0x4
FMC_TAGVDW1S_1  EQU  0x114  ; Cache Tag Storage, array offset: 0x110, array step: 0x4
FMC_TAGVDW1S_2  EQU  0x118  ; Cache Tag Storage, array offset: 0x110, array step: 0x4
FMC_TAGVDW1S_3  EQU  0x11C  ; Cache Tag Storage, array offset: 0x110, array step: 0x4

FMC_TAGVDW2S_0  EQU  0x120  ; Cache Tag Storage, array offset: 0x120, array step: 0x4
FMC_TAGVDW2S_1  EQU  0x124  ; Cache Tag Storage, array offset: 0x120, array step: 0x4
FMC_TAGVDW2S_2  EQU  0x128  ; Cache Tag Storage, array offset: 0x120, array step: 0x4
FMC_TAGVDW2S_3  EQU  0x12C  ; Cache Tag Storage, array offset: 0x120, array step: 0x4

FMC_TAGVDW3S_0  EQU  0x130  ; Cache Tag Storage, array offset: 0x130, array step: 0x4
FMC_TAGVDW3S_1  EQU  0x134  ; Cache Tag Storage, array offset: 0x130, array step: 0x4
FMC_TAGVDW3S_2  EQU  0x138  ; Cache Tag Storage, array offset: 0x130, array step: 0x4
FMC_TAGVDW3S_3  EQU  0x13C  ; Cache Tag Storage, array offset: 0x130, array step: 0x4

FMC_SET         EQU  0x200
;  struct {       ; offset: 0x200, array step: index*0x20, index2*0x8
;  DATA_U  ; Cache Data Storage (upper word), array offset: 0x200, array step: index*0x20, index2*0x8
;  DATA_L  ; Cache Data Storage (lower word), array offset: 0x204, array step: index*0x20, index2*0x8
;  } SET[4][4];

; ----------------------------------------------------------------------------
; -- FMC Register Masks
; ----------------------------------------------------------------------------

; PFAPR Bit Fields
FMC_PFAPR_M0AP_MASK      EQU  0x3
FMC_PFAPR_M0AP_SHIFT     EQU  0
FMC_PFAPR_M1AP_MASK      EQU  0xC
FMC_PFAPR_M1AP_SHIFT     EQU  2
FMC_PFAPR_M2AP_MASK      EQU  0x30
FMC_PFAPR_M2AP_SHIFT     EQU  4
FMC_PFAPR_M3AP_MASK      EQU  0xC0
FMC_PFAPR_M3AP_SHIFT     EQU  6
FMC_PFAPR_M4AP_MASK      EQU  0x300
FMC_PFAPR_M4AP_SHIFT     EQU  8
FMC_PFAPR_M5AP_MASK      EQU  0xC00
FMC_PFAPR_M5AP_SHIFT     EQU  10
FMC_PFAPR_M6AP_MASK      EQU  0x3000
FMC_PFAPR_M6AP_SHIFT     EQU  12
FMC_PFAPR_M7AP_MASK      EQU  0xC000
FMC_PFAPR_M7AP_SHIFT     EQU  14
FMC_PFAPR_M0PFD_MASK     EQU  0x10000
FMC_PFAPR_M0PFD_SHIFT    EQU  16
FMC_PFAPR_M1PFD_MASK     EQU  0x20000
FMC_PFAPR_M1PFD_SHIFT    EQU  17
FMC_PFAPR_M2PFD_MASK     EQU  0x40000
FMC_PFAPR_M2PFD_SHIFT    EQU  18
FMC_PFAPR_M3PFD_MASK     EQU  0x80000
FMC_PFAPR_M3PFD_SHIFT    EQU  19
FMC_PFAPR_M4PFD_MASK     EQU  0x100000
FMC_PFAPR_M4PFD_SHIFT    EQU  20
FMC_PFAPR_M5PFD_MASK     EQU  0x200000
FMC_PFAPR_M5PFD_SHIFT    EQU  21
FMC_PFAPR_M6PFD_MASK     EQU  0x400000
FMC_PFAPR_M6PFD_SHIFT    EQU  22
FMC_PFAPR_M7PFD_MASK     EQU  0x800000
FMC_PFAPR_M7PFD_SHIFT    EQU  23

; PFB0CR Bit Fields
FMC_PFB0CR_B0SEBE_MASK    EQU  0x1
FMC_PFB0CR_B0SEBE_SHIFT   EQU  0
FMC_PFB0CR_B0IPE_MASK     EQU  0x2
FMC_PFB0CR_B0IPE_SHIFT    EQU  1
FMC_PFB0CR_B0DPE_MASK     EQU  0x4
FMC_PFB0CR_B0DPE_SHIFT    EQU  2
FMC_PFB0CR_B0ICE_MASK     EQU  0x8
FMC_PFB0CR_B0ICE_SHIFT    EQU  3
FMC_PFB0CR_B0DCE_MASK     EQU  0x10
FMC_PFB0CR_B0DCE_SHIFT    EQU  4
FMC_PFB0CR_CRC_MASK       EQU  0xE0
FMC_PFB0CR_CRC_SHIFT      EQU  5
FMC_PFB0CR_B0MW_MASK      EQU  0x60000
FMC_PFB0CR_B0MW_SHIFT     EQU  17
FMC_PFB0CR_S_B_INV_MASK   EQU  0x80000
FMC_PFB0CR_S_B_INV_SHIFT  EQU  19
FMC_PFB0CR_CINV_WAY_MASK  EQU  0xF00000
FMC_PFB0CR_CINV_WAY_SHIFT EQU  20
FMC_PFB0CR_CLCK_WAY_MASK  EQU  0xF000000
FMC_PFB0CR_CLCK_WAY_SHIFT EQU  24
FMC_PFB0CR_B0RWSC_MASK    EQU  0xF0000000
FMC_PFB0CR_B0RWSC_SHIFT   EQU  28

; PFB1CR Bit Fields
FMC_PFB1CR_B1SEBE_MASK   EQU  0x1
FMC_PFB1CR_B1SEBE_SHIFT  EQU  0
FMC_PFB1CR_B1IPE_MASK    EQU  0x2
FMC_PFB1CR_B1IPE_SHIFT   EQU  1
FMC_PFB1CR_B1DPE_MASK    EQU  0x4
FMC_PFB1CR_B1DPE_SHIFT   EQU  2
FMC_PFB1CR_B1ICE_MASK    EQU  0x8
FMC_PFB1CR_B1ICE_SHIFT   EQU  3
FMC_PFB1CR_B1DCE_MASK    EQU  0x10
FMC_PFB1CR_B1DCE_SHIFT   EQU  4
FMC_PFB1CR_B1MW_MASK     EQU  0x60000
FMC_PFB1CR_B1MW_SHIFT    EQU  17
FMC_PFB1CR_B1RWSC_MASK   EQU  0xF0000000
FMC_PFB1CR_B1RWSC_SHIFT  EQU  28

; TAGVDW0S Bit Fields
FMC_TAGVDW0S_valid_MASK  EQU  0x1
FMC_TAGVDW0S_valid_SHIFT EQU  0
FMC_TAGVDW0S_tag_MASK    EQU  0x7FFE0
FMC_TAGVDW0S_tag_SHIFT   EQU  5

; TAGVDW1S Bit Fields
FMC_TAGVDW1S_valid_MASK  EQU  0x1
FMC_TAGVDW1S_valid_SHIFT EQU  0
FMC_TAGVDW1S_tag_MASK    EQU  0x7FFE0
FMC_TAGVDW1S_tag_SHIFT   EQU  5

; TAGVDW2S Bit Fields
FMC_TAGVDW2S_valid_MASK  EQU  0x1
FMC_TAGVDW2S_valid_SHIFT EQU  0
FMC_TAGVDW2S_tag_MASK    EQU  0x7FFE0
FMC_TAGVDW2S_tag_SHIFT   EQU  5

; TAGVDW3S Bit Fields
FMC_TAGVDW3S_valid_MASK  EQU  0x1
FMC_TAGVDW3S_valid_SHIFT EQU  0
FMC_TAGVDW3S_tag_MASK    EQU  0x7FFE0
FMC_TAGVDW3S_tag_SHIFT   EQU  5

; DATA_U Bit Fields
FMC_DATA_U_data_MASK     EQU  0xFFFFFFFF
FMC_DATA_U_data_SHIFT    EQU  0

; DATA_L Bit Fields
FMC_DATA_L_data_MASK     EQU  0xFFFFFFFF
FMC_DATA_L_data_SHIFT    EQU  0

; ----------------------------------------------------------------------------
; -- FTFE Peripheral Access Layer
; ----------------------------------------------------------------------------

FTFE_FSTAT   EQU  0x00   ;  Flash Status Register, offset: 0x0
FTFE_FCNFG   EQU  0x01   ;  Flash Configuration Register, offset: 0x1
FTFE_FSEC    EQU  0x02   ;  Flash Security Register, offset: 0x2
FTFE_FOPT    EQU  0x03   ;  Flash Option Register, offset: 0x3
FTFE_FCCOB3  EQU  0x04   ;  Flash Common Command Object Registers, offset: 0x4
FTFE_FCCOB2  EQU  0x05   ;  Flash Common Command Object Registers, offset: 0x5
FTFE_FCCOB1  EQU  0x06   ;  Flash Common Command Object Registers, offset: 0x6
FTFE_FCCOB0  EQU  0x07   ;  Flash Common Command Object Registers, offset: 0x7
FTFE_FCCOB7  EQU  0x08   ;  Flash Common Command Object Registers, offset: 0x8
FTFE_FCCOB6  EQU  0x09   ;  Flash Common Command Object Registers, offset: 0x9
FTFE_FCCOB5  EQU  0x0A   ;  Flash Common Command Object Registers, offset: 0xA
FTFE_FCCOB4  EQU  0x0B   ;  Flash Common Command Object Registers, offset: 0xB
FTFE_FCCOBB  EQU  0x0C   ;  Flash Common Command Object Registers, offset: 0xC
FTFE_FCCOBA  EQU  0x0D   ;  Flash Common Command Object Registers, offset: 0xD
FTFE_FCCOB9  EQU  0x0E   ;  Flash Common Command Object Registers, offset: 0xE
FTFE_FCCOB8  EQU  0x0F   ;  Flash Common Command Object Registers, offset: 0xF
FTFE_FPROT3  EQU  0x10   ;  Program Flash Protection Registers, offset: 0x10
FTFE_FPROT2  EQU  0x11   ;  Program Flash Protection Registers, offset: 0x11
FTFE_FPROT1  EQU  0x12   ;  Program Flash Protection Registers, offset: 0x12
FTFE_FPROT0  EQU  0x13   ;  Program Flash Protection Registers, offset: 0x13
FTFE_FEPROT  EQU  0x16   ;  EEPROM Protection Register, offset: 0x16
FTFE_FDPROT  EQU  0x17   ;  Data Flash Protection Register, offset: 0x17

; ----------------------------------------------------------------------------
; -- FTFE Register Masks
; ----------------------------------------------------------------------------

; FSTAT Bit Fields
FTFE_FSTAT_MGSTAT0_MASK   EQU  0x1
FTFE_FSTAT_MGSTAT0_SHIFT  EQU  0
FTFE_FSTAT_FPVIOL_MASK    EQU  0x10
FTFE_FSTAT_FPVIOL_SHIFT   EQU  4
FTFE_FSTAT_ACCERR_MASK    EQU  0x20
FTFE_FSTAT_ACCERR_SHIFT   EQU  5
FTFE_FSTAT_RDCOLERR_MASK  EQU  0x40
FTFE_FSTAT_RDCOLERR_SHIFT EQU  6
FTFE_FSTAT_CCIF_MASK      EQU  0x80
FTFE_FSTAT_CCIF_SHIFT     EQU  7

; FCNFG Bit Fields
FTFE_FCNFG_EEERDY_MASK    EQU  0x1
FTFE_FCNFG_EEERDY_SHIFT   EQU  0
FTFE_FCNFG_RAMRDY_MASK    EQU  0x2
FTFE_FCNFG_RAMRDY_SHIFT   EQU  1
FTFE_FCNFG_PFLSH_MASK     EQU  0x4
FTFE_FCNFG_PFLSH_SHIFT    EQU  2
FTFE_FCNFG_SWAP_MASK      EQU  0x8
FTFE_FCNFG_SWAP_SHIFT     EQU  3
FTFE_FCNFG_ERSSUSP_MASK   EQU  0x10
FTFE_FCNFG_ERSSUSP_SHIFT  EQU  4
FTFE_FCNFG_ERSAREQ_MASK   EQU  0x20
FTFE_FCNFG_ERSAREQ_SHIFT  EQU  5
FTFE_FCNFG_RDCOLLIE_MASK  EQU  0x40
FTFE_FCNFG_RDCOLLIE_SHIFT EQU  6
FTFE_FCNFG_CCIE_MASK      EQU  0x80
FTFE_FCNFG_CCIE_SHIFT     EQU  7

; FSEC Bit Fields
FTFE_FSEC_SEC_MASK        EQU  0x3
FTFE_FSEC_SEC_SHIFT       EQU  0
FTFE_FSEC_FSLACC_MASK     EQU  0xC
FTFE_FSEC_FSLACC_SHIFT    EQU  2
FTFE_FSEC_MEEN_MASK       EQU  0x30
FTFE_FSEC_MEEN_SHIFT      EQU  4
FTFE_FSEC_KEYEN_MASK      EQU  0xC0
FTFE_FSEC_KEYEN_SHIFT     EQU  6

; FOPT Bit Fields
FTFE_FOPT_OPT_MASK        EQU  0xFF
FTFE_FOPT_OPT_SHIFT       EQU  0

; FCCOB3 Bit Fields
FTFE_FCCOB3_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB3_CCOBn_SHIFT   EQU  0

; FCCOB2 Bit Fields
FTFE_FCCOB2_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB2_CCOBn_SHIFT   EQU  0

; FCCOB1 Bit Fields
FTFE_FCCOB1_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB1_CCOBn_SHIFT   EQU  0

; FCCOB0 Bit Fields
FTFE_FCCOB0_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB0_CCOBn_SHIFT   EQU  0

; FCCOB7 Bit Fields
FTFE_FCCOB7_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB7_CCOBn_SHIFT   EQU  0

; FCCOB6 Bit Fields
FTFE_FCCOB6_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB6_CCOBn_SHIFT   EQU  0

; FCCOB5 Bit Fields
FTFE_FCCOB5_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB5_CCOBn_SHIFT   EQU  0

; FCCOB4 Bit Fields
FTFE_FCCOB4_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB4_CCOBn_SHIFT   EQU  0

; FCCOBB Bit Fields
FTFE_FCCOBB_CCOBn_MASK    EQU  0xFF
FTFE_FCCOBB_CCOBn_SHIFT   EQU  0

; FCCOBA Bit Fields
FTFE_FCCOBA_CCOBn_MASK    EQU  0xFF
FTFE_FCCOBA_CCOBn_SHIFT   EQU  0

; FCCOB9 Bit Fields
FTFE_FCCOB9_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB9_CCOBn_SHIFT   EQU  0

; FCCOB8 Bit Fields
FTFE_FCCOB8_CCOBn_MASK    EQU  0xFF
FTFE_FCCOB8_CCOBn_SHIFT   EQU  0

; FPROT3 Bit Fields
FTFE_FPROT3_PROT_MASK     EQU  0xFF
FTFE_FPROT3_PROT_SHIFT    EQU  0

; FPROT2 Bit Fields
FTFE_FPROT2_PROT_MASK     EQU  0xFF
FTFE_FPROT2_PROT_SHIFT    EQU  0

; FPROT1 Bit Fields
FTFE_FPROT1_PROT_MASK     EQU  0xFF
FTFE_FPROT1_PROT_SHIFT    EQU  0

; FPROT0 Bit Fields
FTFE_FPROT0_PROT_MASK     EQU  0xFF
FTFE_FPROT0_PROT_SHIFT    EQU  0

; FEPROT Bit Fields
FTFE_FEPROT_EPROT_MASK    EQU  0xFF
FTFE_FEPROT_EPROT_SHIFT   EQU  0

; FDPROT Bit Fields
FTFE_FDPROT_DPROT_MASK    EQU  0xFF
FTFE_FDPROT_DPROT_SHIFT   EQU  0

; ----------------------------------------------------------------------------
; -- FTM Peripheral Access Layer
; ----------------------------------------------------------------------------

FTM_SC               EQU  0x00   ; Status And Control, offset: 0x0
FTM_CNT              EQU  0x04   ; Counter, offset: 0x4
FTM_MOD              EQU  0x08   ; Modulo, offset: 0x8
FTM_CONTROLS0_CnSC   EQU  0x0C   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS0_CnV    EQU  0x10   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS1_CnSC   EQU  0x14   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS1_CnV    EQU  0x18   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS2_CnSC   EQU  0x1C   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS2_CnV    EQU  0x20   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS3_CnSC   EQU  0x24   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS3_CnV    EQU  0x28   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS4_CnSC   EQU  0x2C   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS4_CnV    EQU  0x30   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS5_CnSC   EQU  0x34   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS5_CnV    EQU  0x38   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS6_CnSC   EQU  0x3C   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS6_CnV    EQU  0x40   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CONTROLS7_CnSC   EQU  0x44   ; Channel (n) Status And Control, array offset: 0xC, array step: 0x8
FTM_CONTROLS7_CnV    EQU  0x48   ; Channel (n) Value, array offset: 0x10, array step: 0x8
FTM_CNTIN            EQU  0x4C   ; Counter Initial Value, offset: 0x4C
FTM_STATUS           EQU  0x50   ; Capture And Compare Status, offset: 0x50
FTM_MODE             EQU  0x54   ; Features Mode Selection, offset: 0x54
FTM_SYNC             EQU  0x58   ; Synchronization, offset: 0x58
FTM_OUTINIT          EQU  0x5C   ; Initial State For Channels Output, offset: 0x5C
FTM_OUTMASK          EQU  0x60   ; Output Mask, offset: 0x60
FTM_COMBINE          EQU  0x64   ; Function For Linked Channels, offset: 0x64
FTM_DEADTIME         EQU  0x68   ; Deadtime Insertion Control, offset: 0x68
FTM_EXTTRIG          EQU  0x6C   ; FTM External Trigger, offset: 0x6C
FTM_POL              EQU  0x70   ; Channels Polarity, offset: 0x70
FTM_FMS              EQU  0x74   ; Fault Mode Status, offset: 0x74
FTM_FILTER           EQU  0x78   ; Input Capture Filter Control, offset: 0x78
FTM_FLTCTRL          EQU  0x7C   ; Fault Control, offset: 0x7C
FTM_QDCTRL           EQU  0x80   ; Quadrature Decoder Control And Status, offset: 0x80
FTM_CONF             EQU  0x84   ; Configuration, offset: 0x84
FTM_FLTPOL           EQU  0x88   ; FTM Fault Input Polarity, offset: 0x88
FTM_SYNCONF          EQU  0x8C   ; Synchronization Configuration, offset: 0x8C
FTM_INVCTRL          EQU  0x90   ; FTM Inverting Control, offset: 0x90
FTM_SWOCTRL          EQU  0x94   ; FTM Software Output Control, offset: 0x94
FTM_PWMLOAD          EQU  0x98   ; FTM PWM Load, offset: 0x98

; ----------------------------------------------------------------------------
; -- FTM Register Masks
; ----------------------------------------------------------------------------

; SC Bit Fields
FTM_SC_PS_MASK           EQU  0x7
FTM_SC_PS_SHIFT          EQU  0
FTM_SC_CLKS_MASK         EQU  0x18
FTM_SC_CLKS_SHIFT        EQU  3
FTM_SC_CPWMS_MASK        EQU  0x20
FTM_SC_CPWMS_SHIFT       EQU  5
FTM_SC_TOIE_MASK         EQU  0x40
FTM_SC_TOIE_SHIFT        EQU  6
FTM_SC_TOF_MASK          EQU  0x80
FTM_SC_TOF_SHIFT         EQU  7

; CNT Bit Fields
FTM_CNT_COUNT_MASK       EQU  0xFFFF
FTM_CNT_COUNT_SHIFT      EQU  0

; MOD Bit Fields
FTM_MOD_MOD_MASK         EQU  0xFFFF
FTM_MOD_MOD_SHIFT        EQU  0

; CnSC Bit Fields
FTM_CnSC_DMA_MASK        EQU  0x1
FTM_CnSC_DMA_SHIFT       EQU  0
FTM_CnSC_ELSA_MASK       EQU  0x4
FTM_CnSC_ELSA_SHIFT      EQU  2
FTM_CnSC_ELSB_MASK       EQU  0x8
FTM_CnSC_ELSB_SHIFT      EQU  3
FTM_CnSC_MSA_MASK        EQU  0x10
FTM_CnSC_MSA_SHIFT       EQU  4
FTM_CnSC_MSB_MASK        EQU  0x20
FTM_CnSC_MSB_SHIFT       EQU  5
FTM_CnSC_CHIE_MASK       EQU  0x40
FTM_CnSC_CHIE_SHIFT      EQU  6
FTM_CnSC_CHF_MASK        EQU  0x80
FTM_CnSC_CHF_SHIFT       EQU  7

; CnV Bit Fields
FTM_CnV_VAL_MASK         EQU  0xFFFF
FTM_CnV_VAL_SHIFT        EQU  0

; CNTIN Bit Fields
FTM_CNTIN_INIT_MASK      EQU  0xFFFF
FTM_CNTIN_INIT_SHIFT     EQU  0

; STATUS Bit Fields
FTM_STATUS_CH0F_MASK     EQU  0x1
FTM_STATUS_CH0F_SHIFT    EQU  0
FTM_STATUS_CH1F_MASK     EQU  0x2
FTM_STATUS_CH1F_SHIFT    EQU  1
FTM_STATUS_CH2F_MASK     EQU  0x4
FTM_STATUS_CH2F_SHIFT    EQU  2
FTM_STATUS_CH3F_MASK     EQU  0x8
FTM_STATUS_CH3F_SHIFT    EQU  3
FTM_STATUS_CH4F_MASK     EQU  0x10
FTM_STATUS_CH4F_SHIFT    EQU  4
FTM_STATUS_CH5F_MASK     EQU  0x20
FTM_STATUS_CH5F_SHIFT    EQU  5
FTM_STATUS_CH6F_MASK     EQU  0x40
FTM_STATUS_CH6F_SHIFT    EQU  6
FTM_STATUS_CH7F_MASK     EQU  0x80
FTM_STATUS_CH7F_SHIFT    EQU  7

; MODE Bit Fields
FTM_MODE_FTMEN_MASK      EQU  0x1
FTM_MODE_FTMEN_SHIFT     EQU  0
FTM_MODE_INIT_MASK       EQU  0x2
FTM_MODE_INIT_SHIFT      EQU  1
FTM_MODE_WPDIS_MASK      EQU  0x4
FTM_MODE_WPDIS_SHIFT     EQU  2
FTM_MODE_PWMSYNC_MASK    EQU  0x8
FTM_MODE_PWMSYNC_SHIFT   EQU  3
FTM_MODE_CAPTEST_MASK    EQU  0x10
FTM_MODE_CAPTEST_SHIFT   EQU  4
FTM_MODE_FAULTM_MASK     EQU  0x60
FTM_MODE_FAULTM_SHIFT    EQU  5
FTM_MODE_FAULTIE_MASK    EQU  0x80
FTM_MODE_FAULTIE_SHIFT   EQU  7

; SYNC Bit Fields
FTM_SYNC_CNTMIN_MASK     EQU  0x1
FTM_SYNC_CNTMIN_SHIFT    EQU  0
FTM_SYNC_CNTMAX_MASK     EQU  0x2
FTM_SYNC_CNTMAX_SHIFT    EQU  1
FTM_SYNC_REINIT_MASK     EQU  0x4
FTM_SYNC_REINIT_SHIFT    EQU  2
FTM_SYNC_SYNCHOM_MASK    EQU  0x8
FTM_SYNC_SYNCHOM_SHIFT   EQU  3
FTM_SYNC_TRIG0_MASK      EQU  0x10
FTM_SYNC_TRIG0_SHIFT     EQU  4
FTM_SYNC_TRIG1_MASK      EQU  0x20
FTM_SYNC_TRIG1_SHIFT     EQU  5
FTM_SYNC_TRIG2_MASK      EQU  0x40
FTM_SYNC_TRIG2_SHIFT     EQU  6
FTM_SYNC_SWSYNC_MASK     EQU  0x80
FTM_SYNC_SWSYNC_SHIFT    EQU  7

; OUTINIT Bit Fields
FTM_OUTINIT_CH0OI_MASK   EQU  0x1
FTM_OUTINIT_CH0OI_SHIFT  EQU  0
FTM_OUTINIT_CH1OI_MASK   EQU  0x2
FTM_OUTINIT_CH1OI_SHIFT  EQU  1
FTM_OUTINIT_CH2OI_MASK   EQU  0x4
FTM_OUTINIT_CH2OI_SHIFT  EQU  2
FTM_OUTINIT_CH3OI_MASK   EQU  0x8
FTM_OUTINIT_CH3OI_SHIFT  EQU  3
FTM_OUTINIT_CH4OI_MASK   EQU  0x10
FTM_OUTINIT_CH4OI_SHIFT  EQU  4
FTM_OUTINIT_CH5OI_MASK   EQU  0x20
FTM_OUTINIT_CH5OI_SHIFT  EQU  5
FTM_OUTINIT_CH6OI_MASK   EQU  0x40
FTM_OUTINIT_CH6OI_SHIFT  EQU  6
FTM_OUTINIT_CH7OI_MASK   EQU  0x80
FTM_OUTINIT_CH7OI_SHIFT  EQU  7

; OUTMASK Bit Fields
FTM_OUTMASK_CH0OM_MASK   EQU  0x1
FTM_OUTMASK_CH0OM_SHIFT  EQU  0
FTM_OUTMASK_CH1OM_MASK   EQU  0x2
FTM_OUTMASK_CH1OM_SHIFT  EQU  1
FTM_OUTMASK_CH2OM_MASK   EQU  0x4
FTM_OUTMASK_CH2OM_SHIFT  EQU  2
FTM_OUTMASK_CH3OM_MASK   EQU  0x8
FTM_OUTMASK_CH3OM_SHIFT  EQU  3
FTM_OUTMASK_CH4OM_MASK   EQU  0x10
FTM_OUTMASK_CH4OM_SHIFT  EQU  4
FTM_OUTMASK_CH5OM_MASK   EQU  0x20
FTM_OUTMASK_CH5OM_SHIFT  EQU  5
FTM_OUTMASK_CH6OM_MASK   EQU  0x40
FTM_OUTMASK_CH6OM_SHIFT  EQU  6
FTM_OUTMASK_CH7OM_MASK   EQU  0x80
FTM_OUTMASK_CH7OM_SHIFT  EQU  7

; COMBINE Bit Fields
FTM_COMBINE_COMBINE0_MASK   EQU  0x1
FTM_COMBINE_COMBINE0_SHIFT  EQU  0
FTM_COMBINE_COMP0_MASK      EQU  0x2
FTM_COMBINE_COMP0_SHIFT     EQU  1
FTM_COMBINE_DECAPEN0_MASK   EQU  0x4
FTM_COMBINE_DECAPEN0_SHIFT  EQU  2
FTM_COMBINE_DECAP0_MASK     EQU  0x8
FTM_COMBINE_DECAP0_SHIFT    EQU  3
FTM_COMBINE_DTEN0_MASK      EQU  0x10
FTM_COMBINE_DTEN0_SHIFT     EQU  4
FTM_COMBINE_SYNCEN0_MASK    EQU  0x20
FTM_COMBINE_SYNCEN0_SHIFT   EQU  5
FTM_COMBINE_FAULTEN0_MASK   EQU  0x40
FTM_COMBINE_FAULTEN0_SHIFT  EQU  6
FTM_COMBINE_COMBINE1_MASK   EQU  0x100
FTM_COMBINE_COMBINE1_SHIFT  EQU  8
FTM_COMBINE_COMP1_MASK      EQU  0x200
FTM_COMBINE_COMP1_SHIFT     EQU  9
FTM_COMBINE_DECAPEN1_MASK   EQU  0x400
FTM_COMBINE_DECAPEN1_SHIFT  EQU  10
FTM_COMBINE_DECAP1_MASK     EQU  0x800
FTM_COMBINE_DECAP1_SHIFT    EQU  11
FTM_COMBINE_DTEN1_MASK      EQU  0x1000
FTM_COMBINE_DTEN1_SHIFT     EQU  12
FTM_COMBINE_SYNCEN1_MASK    EQU  0x2000
FTM_COMBINE_SYNCEN1_SHIFT   EQU  13
FTM_COMBINE_FAULTEN1_MASK   EQU  0x4000
FTM_COMBINE_FAULTEN1_SHIFT  EQU  14
FTM_COMBINE_COMBINE2_MASK   EQU  0x10000
FTM_COMBINE_COMBINE2_SHIFT  EQU  16
FTM_COMBINE_COMP2_MASK      EQU  0x20000
FTM_COMBINE_COMP2_SHIFT     EQU  17
FTM_COMBINE_DECAPEN2_MASK   EQU  0x40000
FTM_COMBINE_DECAPEN2_SHIFT  EQU  18
FTM_COMBINE_DECAP2_MASK     EQU  0x80000
FTM_COMBINE_DECAP2_SHIFT    EQU  19
FTM_COMBINE_DTEN2_MASK      EQU  0x100000
FTM_COMBINE_DTEN2_SHIFT     EQU  20
FTM_COMBINE_SYNCEN2_MASK    EQU  0x200000
FTM_COMBINE_SYNCEN2_SHIFT   EQU  21
FTM_COMBINE_FAULTEN2_MASK   EQU  0x400000
FTM_COMBINE_FAULTEN2_SHIFT  EQU  22
FTM_COMBINE_COMBINE3_MASK   EQU  0x1000000
FTM_COMBINE_COMBINE3_SHIFT  EQU  24
FTM_COMBINE_COMP3_MASK      EQU  0x2000000
FTM_COMBINE_COMP3_SHIFT     EQU  25
FTM_COMBINE_DECAPEN3_MASK   EQU  0x4000000
FTM_COMBINE_DECAPEN3_SHIFT  EQU  26
FTM_COMBINE_DECAP3_MASK     EQU  0x8000000
FTM_COMBINE_DECAP3_SHIFT    EQU  27
FTM_COMBINE_DTEN3_MASK      EQU  0x10000000
FTM_COMBINE_DTEN3_SHIFT     EQU  28
FTM_COMBINE_SYNCEN3_MASK    EQU  0x20000000
FTM_COMBINE_SYNCEN3_SHIFT   EQU  29
FTM_COMBINE_FAULTEN3_MASK   EQU  0x40000000
FTM_COMBINE_FAULTEN3_SHIFT  EQU  30

; DEADTIME Bit Fields
FTM_DEADTIME_DTVAL_MASK     EQU  0x3F
FTM_DEADTIME_DTVAL_SHIFT    EQU  0
FTM_DEADTIME_DTPS_MASK      EQU  0xC0
FTM_DEADTIME_DTPS_SHIFT     EQU  6

; EXTTRIG Bit Fields
FTM_EXTTRIG_CH2TRIG_MASK     EQU  0x1
FTM_EXTTRIG_CH2TRIG_SHIFT    EQU  0
FTM_EXTTRIG_CH3TRIG_MASK     EQU  0x2
FTM_EXTTRIG_CH3TRIG_SHIFT    EQU  1
FTM_EXTTRIG_CH4TRIG_MASK     EQU  0x4
FTM_EXTTRIG_CH4TRIG_SHIFT    EQU  2
FTM_EXTTRIG_CH5TRIG_MASK     EQU  0x8
FTM_EXTTRIG_CH5TRIG_SHIFT    EQU  3
FTM_EXTTRIG_CH0TRIG_MASK     EQU  0x10
FTM_EXTTRIG_CH0TRIG_SHIFT    EQU  4
FTM_EXTTRIG_CH1TRIG_MASK     EQU  0x20
FTM_EXTTRIG_CH1TRIG_SHIFT    EQU  5
FTM_EXTTRIG_INITTRIGEN_MASK  EQU  0x40
FTM_EXTTRIG_INITTRIGEN_SHIFT EQU  6
FTM_EXTTRIG_TRIGF_MASK       EQU  0x80
FTM_EXTTRIG_TRIGF_SHIFT      EQU  7

; POL Bit Fields
FTM_POL_POL0_MASK        EQU  0x1
FTM_POL_POL0_SHIFT       EQU  0
FTM_POL_POL1_MASK        EQU  0x2
FTM_POL_POL1_SHIFT       EQU  1
FTM_POL_POL2_MASK        EQU  0x4
FTM_POL_POL2_SHIFT       EQU  2
FTM_POL_POL3_MASK        EQU  0x8
FTM_POL_POL3_SHIFT       EQU  3
FTM_POL_POL4_MASK        EQU  0x10
FTM_POL_POL4_SHIFT       EQU  4
FTM_POL_POL5_MASK        EQU  0x20
FTM_POL_POL5_SHIFT       EQU  5
FTM_POL_POL6_MASK        EQU  0x40
FTM_POL_POL6_SHIFT       EQU  6
FTM_POL_POL7_MASK        EQU  0x80
FTM_POL_POL7_SHIFT       EQU  7

; FMS Bit Fields
FTM_FMS_FAULTF0_MASK     EQU  0x1
FTM_FMS_FAULTF0_SHIFT    EQU  0
FTM_FMS_FAULTF1_MASK     EQU  0x2
FTM_FMS_FAULTF1_SHIFT    EQU  1
FTM_FMS_FAULTF2_MASK     EQU  0x4
FTM_FMS_FAULTF2_SHIFT    EQU  2
FTM_FMS_FAULTF3_MASK     EQU  0x8
FTM_FMS_FAULTF3_SHIFT    EQU  3
FTM_FMS_FAULTIN_MASK     EQU  0x20
FTM_FMS_FAULTIN_SHIFT    EQU  5
FTM_FMS_WPEN_MASK        EQU  0x40
FTM_FMS_WPEN_SHIFT       EQU  6
FTM_FMS_FAULTF_MASK      EQU  0x80
FTM_FMS_FAULTF_SHIFT     EQU  7

; FILTER Bit Fields
FTM_FILTER_CH0FVAL_MASK  EQU  0xF
FTM_FILTER_CH0FVAL_SHIFT EQU  0
FTM_FILTER_CH1FVAL_MASK  EQU  0xF0
FTM_FILTER_CH1FVAL_SHIFT EQU  4
FTM_FILTER_CH2FVAL_MASK  EQU  0xF00
FTM_FILTER_CH2FVAL_SHIFT EQU  8
FTM_FILTER_CH3FVAL_MASK  EQU  0xF000
FTM_FILTER_CH3FVAL_SHIFT EQU  12

; FLTCTRL Bit Fields
FTM_FLTCTRL_FAULT0EN_MASK    EQU  0x1
FTM_FLTCTRL_FAULT0EN_SHIFT   EQU  0
FTM_FLTCTRL_FAULT1EN_MASK    EQU  0x2
FTM_FLTCTRL_FAULT1EN_SHIFT   EQU  1
FTM_FLTCTRL_FAULT2EN_MASK    EQU  0x4
FTM_FLTCTRL_FAULT2EN_SHIFT   EQU  2
FTM_FLTCTRL_FAULT3EN_MASK    EQU  0x8
FTM_FLTCTRL_FAULT3EN_SHIFT   EQU  3
FTM_FLTCTRL_FFLTR0EN_MASK    EQU  0x10
FTM_FLTCTRL_FFLTR0EN_SHIFT   EQU  4
FTM_FLTCTRL_FFLTR1EN_MASK    EQU  0x20
FTM_FLTCTRL_FFLTR1EN_SHIFT   EQU  5
FTM_FLTCTRL_FFLTR2EN_MASK    EQU  0x40
FTM_FLTCTRL_FFLTR2EN_SHIFT   EQU  6
FTM_FLTCTRL_FFLTR3EN_MASK    EQU  0x80
FTM_FLTCTRL_FFLTR3EN_SHIFT   EQU  7
FTM_FLTCTRL_FFVAL_MASK       EQU  0xF00
FTM_FLTCTRL_FFVAL_SHIFT      EQU  8

; QDCTRL Bit Fields
FTM_QDCTRL_QUADEN_MASK       EQU  0x1
FTM_QDCTRL_QUADEN_SHIFT      EQU  0
FTM_QDCTRL_TOFDIR_MASK       EQU  0x2
FTM_QDCTRL_TOFDIR_SHIFT      EQU  1
FTM_QDCTRL_QUADIR_MASK       EQU  0x4
FTM_QDCTRL_QUADIR_SHIFT      EQU  2
FTM_QDCTRL_QUADMODE_MASK     EQU  0x8
FTM_QDCTRL_QUADMODE_SHIFT    EQU  3
FTM_QDCTRL_PHBPOL_MASK       EQU  0x10
FTM_QDCTRL_PHBPOL_SHIFT      EQU  4
FTM_QDCTRL_PHAPOL_MASK       EQU  0x20
FTM_QDCTRL_PHAPOL_SHIFT      EQU  5
FTM_QDCTRL_PHBFLTREN_MASK    EQU  0x40
FTM_QDCTRL_PHBFLTREN_SHIFT   EQU  6
FTM_QDCTRL_PHAFLTREN_MASK    EQU  0x80
FTM_QDCTRL_PHAFLTREN_SHIFT   EQU  7

; CONF Bit Fields
FTM_CONF_NUMTOF_MASK         EQU  0x1F
FTM_CONF_NUMTOF_SHIFT        EQU  0
FTM_CONF_BDMMODE_MASK        EQU  0xC0
FTM_CONF_BDMMODE_SHIFT       EQU  6
FTM_CONF_GTBEEN_MASK         EQU  0x200
FTM_CONF_GTBEEN_SHIFT        EQU  9
FTM_CONF_GTBEOUT_MASK        EQU  0x400
FTM_CONF_GTBEOUT_SHIFT       EQU  10

; FLTPOL Bit Fields
FTM_FLTPOL_FLT0POL_MASK      EQU  0x1
FTM_FLTPOL_FLT0POL_SHIFT     EQU  0
FTM_FLTPOL_FLT1POL_MASK      EQU  0x2
FTM_FLTPOL_FLT1POL_SHIFT     EQU  1
FTM_FLTPOL_FLT2POL_MASK      EQU  0x4
FTM_FLTPOL_FLT2POL_SHIFT     EQU  2
FTM_FLTPOL_FLT3POL_MASK      EQU  0x8
FTM_FLTPOL_FLT3POL_SHIFT     EQU  3

; SYNCONF Bit Fields
FTM_SYNCONF_HWTRIGMODE_MASK  EQU  0x1
FTM_SYNCONF_HWTRIGMODE_SHIFT EQU  0
FTM_SYNCONF_CNTINC_MASK      EQU  0x4
FTM_SYNCONF_CNTINC_SHIFT     EQU  2
FTM_SYNCONF_INVC_MASK        EQU  0x10
FTM_SYNCONF_INVC_SHIFT       EQU  4
FTM_SYNCONF_SWOC_MASK        EQU  0x20
FTM_SYNCONF_SWOC_SHIFT       EQU  5
FTM_SYNCONF_SYNCMODE_MASK    EQU  0x80
FTM_SYNCONF_SYNCMODE_SHIFT   EQU  7
FTM_SYNCONF_SWRSTCNT_MASK    EQU  0x100
FTM_SYNCONF_SWRSTCNT_SHIFT   EQU  8
FTM_SYNCONF_SWWRBUF_MASK     EQU  0x200
FTM_SYNCONF_SWWRBUF_SHIFT    EQU  9
FTM_SYNCONF_SWOM_MASK        EQU  0x400
FTM_SYNCONF_SWOM_SHIFT       EQU  10
FTM_SYNCONF_SWINVC_MASK      EQU  0x800
FTM_SYNCONF_SWINVC_SHIFT     EQU  11
FTM_SYNCONF_SWSOC_MASK       EQU  0x1000
FTM_SYNCONF_SWSOC_SHIFT      EQU  12
FTM_SYNCONF_HWRSTCNT_MASK    EQU  0x10000
FTM_SYNCONF_HWRSTCNT_SHIFT   EQU  16
FTM_SYNCONF_HWWRBUF_MASK     EQU  0x20000
FTM_SYNCONF_HWWRBUF_SHIFT    EQU  17
FTM_SYNCONF_HWOM_MASK        EQU  0x40000
FTM_SYNCONF_HWOM_SHIFT       EQU  18
FTM_SYNCONF_HWINVC_MASK      EQU  0x80000
FTM_SYNCONF_HWINVC_SHIFT     EQU  19
FTM_SYNCONF_HWSOC_MASK       EQU  0x100000
FTM_SYNCONF_HWSOC_SHIFT      EQU  20

; INVCTRL Bit Fields
FTM_INVCTRL_INV0EN_MASK      EQU  0x1
FTM_INVCTRL_INV0EN_SHIFT     EQU  0
FTM_INVCTRL_INV1EN_MASK      EQU  0x2
FTM_INVCTRL_INV1EN_SHIFT     EQU  1
FTM_INVCTRL_INV2EN_MASK      EQU  0x4
FTM_INVCTRL_INV2EN_SHIFT     EQU  2
FTM_INVCTRL_INV3EN_MASK      EQU  0x8
FTM_INVCTRL_INV3EN_SHIFT     EQU  3

; SWOCTRL Bit Fields
FTM_SWOCTRL_CH0OC_MASK       EQU  0x1
FTM_SWOCTRL_CH0OC_SHIFT      EQU  0
FTM_SWOCTRL_CH1OC_MASK       EQU  0x2
FTM_SWOCTRL_CH1OC_SHIFT      EQU  1
FTM_SWOCTRL_CH2OC_MASK       EQU  0x4
FTM_SWOCTRL_CH2OC_SHIFT      EQU  2
FTM_SWOCTRL_CH3OC_MASK       EQU  0x8
FTM_SWOCTRL_CH3OC_SHIFT      EQU  3
FTM_SWOCTRL_CH4OC_MASK       EQU  0x10
FTM_SWOCTRL_CH4OC_SHIFT      EQU  4
FTM_SWOCTRL_CH5OC_MASK       EQU  0x20
FTM_SWOCTRL_CH5OC_SHIFT      EQU  5
FTM_SWOCTRL_CH6OC_MASK       EQU  0x40
FTM_SWOCTRL_CH6OC_SHIFT      EQU  6
FTM_SWOCTRL_CH7OC_MASK       EQU  0x80
FTM_SWOCTRL_CH7OC_SHIFT      EQU  7
FTM_SWOCTRL_CH0OCV_MASK      EQU  0x100
FTM_SWOCTRL_CH0OCV_SHIFT     EQU  8
FTM_SWOCTRL_CH1OCV_MASK      EQU  0x200
FTM_SWOCTRL_CH1OCV_SHIFT     EQU  9
FTM_SWOCTRL_CH2OCV_MASK      EQU  0x400
FTM_SWOCTRL_CH2OCV_SHIFT     EQU  10
FTM_SWOCTRL_CH3OCV_MASK      EQU  0x800
FTM_SWOCTRL_CH3OCV_SHIFT     EQU  11
FTM_SWOCTRL_CH4OCV_MASK      EQU  0x1000
FTM_SWOCTRL_CH4OCV_SHIFT     EQU  12
FTM_SWOCTRL_CH5OCV_MASK      EQU  0x2000
FTM_SWOCTRL_CH5OCV_SHIFT     EQU  13
FTM_SWOCTRL_CH6OCV_MASK      EQU  0x4000
FTM_SWOCTRL_CH6OCV_SHIFT     EQU  14
FTM_SWOCTRL_CH7OCV_MASK      EQU  0x8000
FTM_SWOCTRL_CH7OCV_SHIFT     EQU  15

; PWMLOAD Bit Fields
FTM_PWMLOAD_CH0SEL_MASK      EQU  0x1
FTM_PWMLOAD_CH0SEL_SHIFT     EQU  0
FTM_PWMLOAD_CH1SEL_MASK      EQU  0x2
FTM_PWMLOAD_CH1SEL_SHIFT     EQU  1
FTM_PWMLOAD_CH2SEL_MASK      EQU  0x4
FTM_PWMLOAD_CH2SEL_SHIFT     EQU  2
FTM_PWMLOAD_CH3SEL_MASK      EQU  0x8
FTM_PWMLOAD_CH3SEL_SHIFT     EQU  3
FTM_PWMLOAD_CH4SEL_MASK      EQU  0x10
FTM_PWMLOAD_CH4SEL_SHIFT     EQU  4
FTM_PWMLOAD_CH5SEL_MASK      EQU  0x20
FTM_PWMLOAD_CH5SEL_SHIFT     EQU  5
FTM_PWMLOAD_CH6SEL_MASK      EQU  0x40
FTM_PWMLOAD_CH6SEL_SHIFT     EQU  6
FTM_PWMLOAD_CH7SEL_MASK      EQU  0x80
FTM_PWMLOAD_CH7SEL_SHIFT     EQU  7
FTM_PWMLOAD_LDOK_MASK        EQU  0x200
FTM_PWMLOAD_LDOK_SHIFT       EQU  9

; ----------------------------------------------------------------------------
; -- GPIO Peripheral Access Layer
; ----------------------------------------------------------------------------

GPIO_PDOR  EQU  0x00   ; Port Data Output Register, offset: 0x0
GPIO_PSOR  EQU  0x04   ; Port Set Output Register, offset: 0x4
GPIO_PCOR  EQU  0x08   ; Port Clear Output Register, offset: 0x8
GPIO_PTOR  EQU  0x0C   ; Port Toggle Output Register, offset: 0xC
GPIO_PDIR  EQU  0x10   ; Port Data Input Register, offset: 0x10
GPIO_PDDR  EQU  0x14   ; Port Data Direction Register, offset: 0x14

; ----------------------------------------------------------------------------
; -- GPIO Register Masks
; ----------------------------------------------------------------------------

; PDOR Bit Fields
GPIO_PDOR_PDO_MASK       EQU  0xFFFFFFFF
GPIO_PDOR_PDO_SHIFT      EQU  0

; PSOR Bit Fields
GPIO_PSOR_PTSO_MASK      EQU  0xFFFFFFFF
GPIO_PSOR_PTSO_SHIFT     EQU  0

; PCOR Bit Fields
GPIO_PCOR_PTCO_MASK      EQU  0xFFFFFFFF
GPIO_PCOR_PTCO_SHIFT     EQU  0

; PTOR Bit Fields
GPIO_PTOR_PTTO_MASK      EQU  0xFFFFFFFF
GPIO_PTOR_PTTO_SHIFT     EQU  0

; PDIR Bit Fields
GPIO_PDIR_PDI_MASK       EQU  0xFFFFFFFF
GPIO_PDIR_PDI_SHIFT      EQU  0

; PDDR Bit Fields
GPIO_PDDR_PDD_MASK       EQU  0xFFFFFFFF
GPIO_PDDR_PDD_SHIFT      EQU  0

; ----------------------------------------------------------------------------
; -- I2C Peripheral Access Layer
; ----------------------------------------------------------------------------

I2C_A1     EQU  0x0   ; I2C Address Register 1, offset: 0x0
I2C_F      EQU  0x1   ; I2C Frequency Divider register, offset: 0x1
I2C_C1     EQU  0x2   ; I2C Control Register 1, offset: 0x2
I2C_S      EQU  0x3   ; I2C Status register, offset: 0x3
I2C_D      EQU  0x4   ; I2C Data I/O register, offset: 0x4
I2C_C2     EQU  0x5   ; I2C Control Register 2, offset: 0x5
I2C_FLT    EQU  0x6   ; I2C Programmable Input Glitch Filter register, offset: 0x6
I2C_RA     EQU  0x7   ; I2C Range Address register, offset: 0x7
I2C_SMB    EQU  0x8   ; I2C SMBus Control and Status register, offset: 0x8
I2C_A2     EQU  0x9   ; I2C Address Register 2, offset: 0x9
I2C_SLTH   EQU  0xA   ; I2C SCL Low Timeout Register High, offset: 0xA
I2C_SLTL   EQU  0xB   ; I2C SCL Low Timeout Register Low, offset: 0xB

; ----------------------------------------------------------------------------
; -- I2C Register Masks
; ----------------------------------------------------------------------------

; A1 Bit Fields
I2C_A1_AD_MASK           EQU  0xFE
I2C_A1_AD_SHIFT          EQU  1

; F Bit Fields
I2C_F_ICR_MASK           EQU  0x3F
I2C_F_ICR_SHIFT          EQU  0
I2C_F_MULT_MASK          EQU  0xC0
I2C_F_MULT_SHIFT         EQU  6

; C1 Bit Fields
I2C_C1_DMAEN_MASK        EQU  0x1
I2C_C1_DMAEN_SHIFT       EQU  0
I2C_C1_WUEN_MASK         EQU  0x2
I2C_C1_WUEN_SHIFT        EQU  1
I2C_C1_RSTA_MASK         EQU  0x4
I2C_C1_RSTA_SHIFT        EQU  2
I2C_C1_TXAK_MASK         EQU  0x8
I2C_C1_TXAK_SHIFT        EQU  3
I2C_C1_TX_MASK           EQU  0x10
I2C_C1_TX_SHIFT          EQU  4
I2C_C1_MST_MASK          EQU  0x20
I2C_C1_MST_SHIFT         EQU  5
I2C_C1_IICIE_MASK        EQU  0x40
I2C_C1_IICIE_SHIFT       EQU  6
I2C_C1_IICEN_MASK        EQU  0x80
I2C_C1_IICEN_SHIFT       EQU  7

; S Bit Fields
I2C_S_RXAK_MASK          EQU  0x1
I2C_S_RXAK_SHIFT         EQU  0
I2C_S_IICIF_MASK         EQU  0x2
I2C_S_IICIF_SHIFT        EQU  1
I2C_S_SRW_MASK           EQU  0x4
I2C_S_SRW_SHIFT          EQU  2
I2C_S_RAM_MASK           EQU  0x8
I2C_S_RAM_SHIFT          EQU  3
I2C_S_ARBL_MASK          EQU  0x10
I2C_S_ARBL_SHIFT         EQU  4
I2C_S_BUSY_MASK          EQU  0x20
I2C_S_BUSY_SHIFT         EQU  5
I2C_S_IAAS_MASK          EQU  0x40
I2C_S_IAAS_SHIFT         EQU  6
I2C_S_TCF_MASK           EQU  0x80
I2C_S_TCF_SHIFT          EQU  7

; D Bit Fields
I2C_D_DATA_MASK          EQU  0xFF
I2C_D_DATA_SHIFT         EQU  0

; C2 Bit Fields
I2C_C2_AD_MASK           EQU  0x7
I2C_C2_AD_SHIFT          EQU  0
I2C_C2_RMEN_MASK         EQU  0x8
I2C_C2_RMEN_SHIFT        EQU  3
I2C_C2_SBRC_MASK         EQU  0x10
I2C_C2_SBRC_SHIFT        EQU  4
I2C_C2_HDRS_MASK         EQU  0x20
I2C_C2_HDRS_SHIFT        EQU  5
I2C_C2_ADEXT_MASK        EQU  0x40
I2C_C2_ADEXT_SHIFT       EQU  6
I2C_C2_GCAEN_MASK        EQU  0x80
I2C_C2_GCAEN_SHIFT       EQU  7

; FLT Bit Fields
I2C_FLT_FLT_MASK         EQU  0xF
I2C_FLT_FLT_SHIFT        EQU  0
I2C_FLT_STARTF_MASK      EQU  0x10
I2C_FLT_STARTF_SHIFT     EQU  4
I2C_FLT_SSIE_MASK        EQU  0x20
I2C_FLT_SSIE_SHIFT       EQU  5
I2C_FLT_STOPF_MASK       EQU  0x40
I2C_FLT_STOPF_SHIFT      EQU  6
I2C_FLT_SHEN_MASK        EQU  0x80
I2C_FLT_SHEN_SHIFT       EQU  7

; RA Bit Fields
I2C_RA_RAD_MASK          EQU  0xFE
I2C_RA_RAD_SHIFT         EQU  1

; SMB Bit Fields
I2C_SMB_SHTF2IE_MASK     EQU  0x1
I2C_SMB_SHTF2IE_SHIFT    EQU  0
I2C_SMB_SHTF2_MASK       EQU  0x2
I2C_SMB_SHTF2_SHIFT      EQU  1
I2C_SMB_SHTF1_MASK       EQU  0x4
I2C_SMB_SHTF1_SHIFT      EQU  2
I2C_SMB_SLTF_MASK        EQU  0x8
I2C_SMB_SLTF_SHIFT       EQU  3
I2C_SMB_TCKSEL_MASK      EQU  0x10
I2C_SMB_TCKSEL_SHIFT     EQU  4
I2C_SMB_SIICAEN_MASK     EQU  0x20
I2C_SMB_SIICAEN_SHIFT    EQU  5
I2C_SMB_ALERTEN_MASK     EQU  0x40
I2C_SMB_ALERTEN_SHIFT    EQU  6
I2C_SMB_FACK_MASK        EQU  0x80
I2C_SMB_FACK_SHIFT       EQU  7

; A2 Bit Fields
I2C_A2_SAD_MASK          EQU  0xFE
I2C_A2_SAD_SHIFT         EQU  1

; SLTH Bit Fields
I2C_SLTH_SSLT_MASK       EQU  0xFF
I2C_SLTH_SSLT_SHIFT      EQU  0

; SLTL Bit Fields
I2C_SLTL_SSLT_MASK       EQU  0xFF
I2C_SLTL_SSLT_SHIFT      EQU  0

; ----------------------------------------------------------------------------
; -- I2S Peripheral Access Layer
; ----------------------------------------------------------------------------

I2S_TCSR    EQU  0x000   ; SAI Transmit Control Register, offset: 0x0
I2S_TCR1    EQU  0x004   ; SAI Transmit Configuration 1 Register, offset: 0x4
I2S_TCR2    EQU  0x005   ; SAI Transmit Configuration 2 Register, offset: 0x8
I2S_TCR3    EQU  0x00C   ; SAI Transmit Configuration 3 Register, offset: 0xC
I2S_TCR4    EQU  0x010   ; SAI Transmit Configuration 4 Register, offset: 0x10
I2S_TCR5    EQU  0x014   ; SAI Transmit Configuration 5 Register, offset: 0x14
I2S_TDR0    EQU  0x020   ; SAI Transmit Data Register, array offset: 0x20, array step: 0x4
I2S_TDR1    EQU  0x024   ; SAI Transmit Data Register, array offset: 0x20, array step: 0x4
I2S_TFR0    EQU  0x040   ; SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4
I2S_TFR1    EQU  0x044   ; SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4
I2S_TMR     EQU  0x060   ; SAI Transmit Mask Register, offset: 0x60
I2S_RCSR    EQU  0x080   ; SAI Receive Control Register, offset: 0x80
I2S_RCR1    EQU  0x084   ; SAI Receive Configuration 1 Register, offset: 0x84
I2S_RCR2    EQU  0x088   ; SAI Receive Configuration 2 Register, offset: 0x88
I2S_RCR3    EQU  0x08C   ; SAI Receive Configuration 3 Register, offset: 0x8C
I2S_RCR4    EQU  0x090   ; SAI Receive Configuration 4 Register, offset: 0x90
I2S_RCR5    EQU  0x094   ; SAI Receive Configuration 5 Register, offset: 0x94
I2S_RDR0    EQU  0x0A0   ; SAI Receive Data Register, array offset: 0xA0, array step: 0x4
I2S_RDR1    EQU  0x0A4   ; SAI Receive Data Register, array offset: 0xA0, array step: 0x4
I2S_RFR0    EQU  0x0C0   ; SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4
I2S_RFR1    EQU  0x0C4   ; SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4
I2S_RMR     EQU  0x0E0   ; SAI Receive Mask Register, offset: 0xE0
I2S_MCR     EQU  0x100   ; SAI MCLK Control Register, offset: 0x100
I2S_MDR     EQU  0x104   ; SAI MCLK Divide Register, offset: 0x104

; ----------------------------------------------------------------------------
; -- I2S Register Masks
; ----------------------------------------------------------------------------

; TCSR Bit Fields
I2S_TCSR_FRDE_MASK       EQU  0x1
I2S_TCSR_FRDE_SHIFT      EQU  0
I2S_TCSR_FWDE_MASK       EQU  0x2
I2S_TCSR_FWDE_SHIFT      EQU  1
I2S_TCSR_FRIE_MASK       EQU  0x100
I2S_TCSR_FRIE_SHIFT      EQU  8
I2S_TCSR_FWIE_MASK       EQU  0x200
I2S_TCSR_FWIE_SHIFT      EQU  9
I2S_TCSR_FEIE_MASK       EQU  0x400
I2S_TCSR_FEIE_SHIFT      EQU  10
I2S_TCSR_SEIE_MASK       EQU  0x800
I2S_TCSR_SEIE_SHIFT      EQU  11
I2S_TCSR_WSIE_MASK       EQU  0x1000
I2S_TCSR_WSIE_SHIFT      EQU  12
I2S_TCSR_FRF_MASK        EQU  0x10000
I2S_TCSR_FRF_SHIFT       EQU  16
I2S_TCSR_FWF_MASK        EQU  0x20000
I2S_TCSR_FWF_SHIFT       EQU  17
I2S_TCSR_FEF_MASK        EQU  0x40000
I2S_TCSR_FEF_SHIFT       EQU  18
I2S_TCSR_SEF_MASK        EQU  0x80000
I2S_TCSR_SEF_SHIFT       EQU  19
I2S_TCSR_WSF_MASK        EQU  0x100000
I2S_TCSR_WSF_SHIFT       EQU  20
I2S_TCSR_SR_MASK         EQU  0x1000000
I2S_TCSR_SR_SHIFT        EQU  24
I2S_TCSR_FR_MASK         EQU  0x2000000
I2S_TCSR_FR_SHIFT        EQU  25
I2S_TCSR_BCE_MASK        EQU  0x10000000
I2S_TCSR_BCE_SHIFT       EQU  28
I2S_TCSR_DBGE_MASK       EQU  0x20000000
I2S_TCSR_DBGE_SHIFT      EQU  29
I2S_TCSR_STOPE_MASK      EQU  0x40000000
I2S_TCSR_STOPE_SHIFT     EQU  30
I2S_TCSR_TE_MASK         EQU  0x80000000
I2S_TCSR_TE_SHIFT        EQU  31

; TCR1 Bit Fields
I2S_TCR1_TFW_MASK        EQU  0x7
I2S_TCR1_TFW_SHIFT       EQU  0

; TCR2 Bit Fields
I2S_TCR2_DIV_MASK        EQU  0xFF
I2S_TCR2_DIV_SHIFT       EQU  0
I2S_TCR2_BCD_MASK        EQU  0x1000000
I2S_TCR2_BCD_SHIFT       EQU  24
I2S_TCR2_BCP_MASK        EQU  0x2000000
I2S_TCR2_BCP_SHIFT       EQU  25
I2S_TCR2_MSEL_MASK       EQU  0xC000000
I2S_TCR2_MSEL_SHIFT      EQU  26
I2S_TCR2_BCI_MASK        EQU  0x10000000
I2S_TCR2_BCI_SHIFT       EQU  28
I2S_TCR2_BCS_MASK        EQU  0x20000000
I2S_TCR2_BCS_SHIFT       EQU  29
I2S_TCR2_SYNC_MASK       EQU  0xC0000000
I2S_TCR2_SYNC_SHIFT      EQU  30

; TCR3 Bit Fields
I2S_TCR3_WDFL_MASK       EQU  0x1F
I2S_TCR3_WDFL_SHIFT      EQU  0
I2S_TCR3_TCE_MASK        EQU  0x30000
I2S_TCR3_TCE_SHIFT       EQU  16

; TCR4 Bit Fields
I2S_TCR4_FSD_MASK        EQU  0x1
I2S_TCR4_FSD_SHIFT       EQU  0
I2S_TCR4_FSP_MASK        EQU  0x2
I2S_TCR4_FSP_SHIFT       EQU  1
I2S_TCR4_FSE_MASK        EQU  0x8
I2S_TCR4_FSE_SHIFT       EQU  3
I2S_TCR4_MF_MASK         EQU  0x10
I2S_TCR4_MF_SHIFT        EQU  4
I2S_TCR4_SYWD_MASK       EQU  0x1F00
I2S_TCR4_SYWD_SHIFT      EQU  8
I2S_TCR4_FRSZ_MASK       EQU  0x1F0000
I2S_TCR4_FRSZ_SHIFT      EQU  16

; TCR5 Bit Fields
I2S_TCR5_FBT_MASK        EQU  0x1F00
I2S_TCR5_FBT_SHIFT       EQU  8
I2S_TCR5_W0W_MASK        EQU  0x1F0000
I2S_TCR5_W0W_SHIFT       EQU  16
I2S_TCR5_WNW_MASK        EQU  0x1F000000
I2S_TCR5_WNW_SHIFT       EQU  24

; TDR Bit Fields
I2S_TDR_TDR_MASK         EQU  0xFFFFFFFF
I2S_TDR_TDR_SHIFT        EQU  0
; TFR Bit Fields
I2S_TFR_RFP_MASK         EQU  0xF
I2S_TFR_RFP_SHIFT        EQU  0
I2S_TFR_WFP_MASK         EQU  0xF0000
I2S_TFR_WFP_SHIFT        EQU  16
; TMR Bit Fields
I2S_TMR_TWM_MASK         EQU  0xFFFFFFFF
I2S_TMR_TWM_SHIFT        EQU  0

; RCSR Bit Fields
I2S_RCSR_FRDE_MASK       EQU  0x1
I2S_RCSR_FRDE_SHIFT      EQU  0
I2S_RCSR_FWDE_MASK       EQU  0x2
I2S_RCSR_FWDE_SHIFT      EQU  1
I2S_RCSR_FRIE_MASK       EQU  0x100
I2S_RCSR_FRIE_SHIFT      EQU  8
I2S_RCSR_FWIE_MASK       EQU  0x200
I2S_RCSR_FWIE_SHIFT      EQU  9
I2S_RCSR_FEIE_MASK       EQU  0x400
I2S_RCSR_FEIE_SHIFT      EQU  10
I2S_RCSR_SEIE_MASK       EQU  0x800
I2S_RCSR_SEIE_SHIFT      EQU  11
I2S_RCSR_WSIE_MASK       EQU  0x1000
I2S_RCSR_WSIE_SHIFT      EQU  12
I2S_RCSR_FRF_MASK        EQU  0x10000
I2S_RCSR_FRF_SHIFT       EQU  16
I2S_RCSR_FWF_MASK        EQU  0x20000
I2S_RCSR_FWF_SHIFT       EQU  17
I2S_RCSR_FEF_MASK        EQU  0x40000
I2S_RCSR_FEF_SHIFT       EQU  18
I2S_RCSR_SEF_MASK        EQU  0x80000
I2S_RCSR_SEF_SHIFT       EQU  19
I2S_RCSR_WSF_MASK        EQU  0x100000
I2S_RCSR_WSF_SHIFT       EQU  20
I2S_RCSR_SR_MASK         EQU  0x1000000
I2S_RCSR_SR_SHIFT        EQU  24
I2S_RCSR_FR_MASK         EQU  0x2000000
I2S_RCSR_FR_SHIFT        EQU  25
I2S_RCSR_BCE_MASK        EQU  0x10000000
I2S_RCSR_BCE_SHIFT       EQU  28
I2S_RCSR_DBGE_MASK       EQU  0x20000000
I2S_RCSR_DBGE_SHIFT      EQU  29
I2S_RCSR_STOPE_MASK      EQU  0x40000000
I2S_RCSR_STOPE_SHIFT     EQU  30
I2S_RCSR_RE_MASK         EQU  0x80000000
I2S_RCSR_RE_SHIFT        EQU  31

; RCR1 Bit Fields
I2S_RCR1_RFW_MASK        EQU  0x7
I2S_RCR1_RFW_SHIFT       EQU  0

; RCR2 Bit Fields
I2S_RCR2_DIV_MASK        EQU  0xFF
I2S_RCR2_DIV_SHIFT       EQU  0
I2S_RCR2_BCD_MASK        EQU  0x1000000
I2S_RCR2_BCD_SHIFT       EQU  24
I2S_RCR2_BCP_MASK        EQU  0x2000000
I2S_RCR2_BCP_SHIFT       EQU  25
I2S_RCR2_MSEL_MASK       EQU  0xC000000
I2S_RCR2_MSEL_SHIFT      EQU  26
I2S_RCR2_BCI_MASK        EQU  0x10000000
I2S_RCR2_BCI_SHIFT       EQU  28
I2S_RCR2_BCS_MASK        EQU  0x20000000
I2S_RCR2_BCS_SHIFT       EQU  29
I2S_RCR2_SYNC_MASK       EQU  0xC0000000
I2S_RCR2_SYNC_SHIFT      EQU  30

; RCR3 Bit Fields
I2S_RCR3_WDFL_MASK       EQU  0x1F
I2S_RCR3_WDFL_SHIFT      EQU  0
I2S_RCR3_RCE_MASK        EQU  0x30000
I2S_RCR3_RCE_SHIFT       EQU  16

; RCR4 Bit Fields
I2S_RCR4_FSD_MASK        EQU  0x1
I2S_RCR4_FSD_SHIFT       EQU  0
I2S_RCR4_FSP_MASK        EQU  0x2
I2S_RCR4_FSP_SHIFT       EQU  1
I2S_RCR4_FSE_MASK        EQU  0x8
I2S_RCR4_FSE_SHIFT       EQU  3
I2S_RCR4_MF_MASK         EQU  0x10
I2S_RCR4_MF_SHIFT        EQU  4
I2S_RCR4_SYWD_MASK       EQU  0x1F00
I2S_RCR4_SYWD_SHIFT      EQU  8
I2S_RCR4_FRSZ_MASK       EQU  0x1F0000
I2S_RCR4_FRSZ_SHIFT      EQU  16

; RCR5 Bit Fields
I2S_RCR5_FBT_MASK        EQU  0x1F00
I2S_RCR5_FBT_SHIFT       EQU  8
I2S_RCR5_W0W_MASK        EQU  0x1F0000
I2S_RCR5_W0W_SHIFT       EQU  16
I2S_RCR5_WNW_MASK        EQU  0x1F000000
I2S_RCR5_WNW_SHIFT       EQU  24

; RDR Bit Fields
I2S_RDR_RDR_MASK         EQU  0xFFFFFFFF
I2S_RDR_RDR_SHIFT        EQU  0

; RFR Bit Fields
I2S_RFR_RFP_MASK         EQU  0xF
I2S_RFR_RFP_SHIFT        EQU  0
I2S_RFR_WFP_MASK         EQU  0xF0000
I2S_RFR_WFP_SHIFT        EQU  16

; RMR Bit Fields
I2S_RMR_RWM_MASK         EQU  0xFFFFFFFF
I2S_RMR_RWM_SHIFT        EQU  0

; MCR Bit Fields
I2S_MCR_MICS_MASK        EQU  0x3000000
I2S_MCR_MICS_SHIFT       EQU  24
I2S_MCR_MOE_MASK         EQU  0x40000000
I2S_MCR_MOE_SHIFT        EQU  30
I2S_MCR_DUF_MASK         EQU  0x80000000
I2S_MCR_DUF_SHIFT        EQU  31

; MDR Bit Fields
I2S_MDR_DIVIDE_MASK      EQU  0xFFF
I2S_MDR_DIVIDE_SHIFT     EQU  0
I2S_MDR_FRACT_MASK       EQU  0xFF000
I2S_MDR_FRACT_SHIFT      EQU  12

; ----------------------------------------------------------------------------
; -- LLWU Peripheral Access Layer
; ----------------------------------------------------------------------------

LLWU_PE1      EQU  0x0   ; LLWU Pin Enable 1 register, offset: 0x0
LLWU_PE2      EQU  0x1   ; LLWU Pin Enable 2 register, offset: 0x1
LLWU_PE3      EQU  0x2   ; LLWU Pin Enable 3 register, offset: 0x2
LLWU_PE4      EQU  0x3   ; LLWU Pin Enable 4 register, offset: 0x3
LLWU_ME       EQU  0x4   ; LLWU Module Enable register, offset: 0x4
LLWU_F1       EQU  0x5   ; LLWU Flag 1 register, offset: 0x5
LLWU_F2       EQU  0x6   ; LLWU Flag 2 register, offset: 0x6
LLWU_F3       EQU  0x7   ; LLWU Flag 3 register, offset: 0x7
LLWU_FILT1    EQU  0x8   ; LLWU Pin Filter 1 register, offset: 0x8
LLWU_FILT2    EQU  0x9   ; LLWU Pin Filter 2 register, offset: 0x9
LLWU_RST      EQU  0xA   ; LLWU Reset Enable register, offset: 0xA

; ----------------------------------------------------------------------------
; -- LLWU Register Masks
; ----------------------------------------------------------------------------

; PE1 Bit Fields
LLWU_PE1_WUPE0_MASK      EQU  0x3
LLWU_PE1_WUPE0_SHIFT     EQU  0
LLWU_PE1_WUPE1_MASK      EQU  0xC
LLWU_PE1_WUPE1_SHIFT     EQU  2
LLWU_PE1_WUPE2_MASK      EQU  0x30
LLWU_PE1_WUPE2_SHIFT     EQU  4
LLWU_PE1_WUPE3_MASK      EQU  0xC0
LLWU_PE1_WUPE3_SHIFT     EQU  6

; PE2 Bit Fields
LLWU_PE2_WUPE4_MASK      EQU  0x3
LLWU_PE2_WUPE4_SHIFT     EQU  0
LLWU_PE2_WUPE5_MASK      EQU  0xC
LLWU_PE2_WUPE5_SHIFT     EQU  2
LLWU_PE2_WUPE6_MASK      EQU  0x30
LLWU_PE2_WUPE6_SHIFT     EQU  4
LLWU_PE2_WUPE7_MASK      EQU  0xC0
LLWU_PE2_WUPE7_SHIFT     EQU  6

; PE3 Bit Fields
LLWU_PE3_WUPE8_MASK      EQU  0x3
LLWU_PE3_WUPE8_SHIFT     EQU  0
LLWU_PE3_WUPE9_MASK      EQU  0xC
LLWU_PE3_WUPE9_SHIFT     EQU  2
LLWU_PE3_WUPE10_MASK     EQU  0x30
LLWU_PE3_WUPE10_SHIFT    EQU  4
LLWU_PE3_WUPE11_MASK     EQU  0xC0
LLWU_PE3_WUPE11_SHIFT    EQU  6

; PE4 Bit Fields
LLWU_PE4_WUPE12_MASK     EQU  0x3
LLWU_PE4_WUPE12_SHIFT    EQU  0
LLWU_PE4_WUPE13_MASK     EQU  0xC
LLWU_PE4_WUPE13_SHIFT    EQU  2
LLWU_PE4_WUPE14_MASK     EQU  0x30
LLWU_PE4_WUPE14_SHIFT    EQU  4
LLWU_PE4_WUPE15_MASK     EQU  0xC0
LLWU_PE4_WUPE15_SHIFT    EQU  6

; ME Bit Fields
LLWU_ME_WUME0_MASK       EQU  0x1
LLWU_ME_WUME0_SHIFT      EQU  0
LLWU_ME_WUME1_MASK       EQU  0x2
LLWU_ME_WUME1_SHIFT      EQU  1
LLWU_ME_WUME2_MASK       EQU  0x4
LLWU_ME_WUME2_SHIFT      EQU  2
LLWU_ME_WUME3_MASK       EQU  0x8
LLWU_ME_WUME3_SHIFT      EQU  3
LLWU_ME_WUME4_MASK       EQU  0x10
LLWU_ME_WUME4_SHIFT      EQU  4
LLWU_ME_WUME5_MASK       EQU  0x20
LLWU_ME_WUME5_SHIFT      EQU  5
LLWU_ME_WUME6_MASK       EQU  0x40
LLWU_ME_WUME6_SHIFT      EQU  6
LLWU_ME_WUME7_MASK       EQU  0x80
LLWU_ME_WUME7_SHIFT      EQU  7

; F1 Bit Fields
LLWU_F1_WUF0_MASK        EQU  0x1
LLWU_F1_WUF0_SHIFT       EQU  0
LLWU_F1_WUF1_MASK        EQU  0x2
LLWU_F1_WUF1_SHIFT       EQU  1
LLWU_F1_WUF2_MASK        EQU  0x4
LLWU_F1_WUF2_SHIFT       EQU  2
LLWU_F1_WUF3_MASK        EQU  0x8
LLWU_F1_WUF3_SHIFT       EQU  3
LLWU_F1_WUF4_MASK        EQU  0x10
LLWU_F1_WUF4_SHIFT       EQU  4
LLWU_F1_WUF5_MASK        EQU  0x20
LLWU_F1_WUF5_SHIFT       EQU  5
LLWU_F1_WUF6_MASK        EQU  0x40
LLWU_F1_WUF6_SHIFT       EQU  6
LLWU_F1_WUF7_MASK        EQU  0x80
LLWU_F1_WUF7_SHIFT       EQU  7

; F2 Bit Fields
LLWU_F2_WUF8_MASK        EQU  0x1
LLWU_F2_WUF8_SHIFT       EQU  0
LLWU_F2_WUF9_MASK        EQU  0x2
LLWU_F2_WUF9_SHIFT       EQU  1
LLWU_F2_WUF10_MASK       EQU  0x4
LLWU_F2_WUF10_SHIFT      EQU  2
LLWU_F2_WUF11_MASK       EQU  0x8
LLWU_F2_WUF11_SHIFT      EQU  3
LLWU_F2_WUF12_MASK       EQU  0x10
LLWU_F2_WUF12_SHIFT      EQU  4
LLWU_F2_WUF13_MASK       EQU  0x20
LLWU_F2_WUF13_SHIFT      EQU  5
LLWU_F2_WUF14_MASK       EQU  0x40
LLWU_F2_WUF14_SHIFT      EQU  6
LLWU_F2_WUF15_MASK       EQU  0x80
LLWU_F2_WUF15_SHIFT      EQU  7

; F3 Bit Fields
LLWU_F3_MWUF0_MASK       EQU  0x1
LLWU_F3_MWUF0_SHIFT      EQU  0
LLWU_F3_MWUF1_MASK       EQU  0x2
LLWU_F3_MWUF1_SHIFT      EQU  1
LLWU_F3_MWUF2_MASK       EQU  0x4
LLWU_F3_MWUF2_SHIFT      EQU  2
LLWU_F3_MWUF3_MASK       EQU  0x8
LLWU_F3_MWUF3_SHIFT      EQU  3
LLWU_F3_MWUF4_MASK       EQU  0x10
LLWU_F3_MWUF4_SHIFT      EQU  4
LLWU_F3_MWUF5_MASK       EQU  0x20
LLWU_F3_MWUF5_SHIFT      EQU  5
LLWU_F3_MWUF6_MASK       EQU  0x40
LLWU_F3_MWUF6_SHIFT      EQU  6
LLWU_F3_MWUF7_MASK       EQU  0x80
LLWU_F3_MWUF7_SHIFT      EQU  7

; FILT1 Bit Fields
LLWU_FILT1_FILTSEL_MASK  EQU  0xF
LLWU_FILT1_FILTSEL_SHIFT EQU  0
LLWU_FILT1_FILTE_MASK    EQU  0x60
LLWU_FILT1_FILTE_SHIFT   EQU  5
LLWU_FILT1_FILTF_MASK    EQU  0x80
LLWU_FILT1_FILTF_SHIFT   EQU  7

; FILT2 Bit Fields
LLWU_FILT2_FILTSEL_MASK  EQU  0xF
LLWU_FILT2_FILTSEL_SHIFT EQU  0
LLWU_FILT2_FILTE_MASK    EQU  0x60
LLWU_FILT2_FILTE_SHIFT   EQU  5
LLWU_FILT2_FILTF_MASK    EQU  0x80
LLWU_FILT2_FILTF_SHIFT   EQU  7

; RST Bit Fields
LLWU_RST_RSTFILT_MASK    EQU  0x1
LLWU_RST_RSTFILT_SHIFT   EQU  0
LLWU_RST_LLRSTE_MASK     EQU  0x2
LLWU_RST_LLRSTE_SHIFT    EQU  1

; ----------------------------------------------------------------------------
; -- LPTMR Peripheral Access Layer
; ----------------------------------------------------------------------------

LPTMR_CSR     EQU  0x0   ; Low Power Timer Control Status Register, offset: 0x0
LPTMR_PSR     EQU  0x4   ; Low Power Timer Prescale Register, offset: 0x4
LPTMR_CMR     EQU  0x8   ; Low Power Timer Compare Register, offset: 0x8
LPTMR_CNR     EQU  0xC   ; Low Power Timer Counter Register, offset: 0xC

; ----------------------------------------------------------------------------
; -- LPTMR Register Masks
; ----------------------------------------------------------------------------

; CSR Bit Fields
LPTMR_CSR_TEN_MASK       EQU  0x1
LPTMR_CSR_TEN_SHIFT      EQU  0
LPTMR_CSR_TMS_MASK       EQU  0x2
LPTMR_CSR_TMS_SHIFT      EQU  1
LPTMR_CSR_TFC_MASK       EQU  0x4
LPTMR_CSR_TFC_SHIFT      EQU  2
LPTMR_CSR_TPP_MASK       EQU  0x8
LPTMR_CSR_TPP_SHIFT      EQU  3
LPTMR_CSR_TPS_MASK       EQU  0x30
LPTMR_CSR_TPS_SHIFT      EQU  4
LPTMR_CSR_TIE_MASK       EQU  0x40
LPTMR_CSR_TIE_SHIFT      EQU  6
LPTMR_CSR_TCF_MASK       EQU  0x80
LPTMR_CSR_TCF_SHIFT      EQU  7

; PSR Bit Fields
LPTMR_PSR_PCS_MASK       EQU  0x3
LPTMR_PSR_PCS_SHIFT      EQU  0
LPTMR_PSR_PBYP_MASK      EQU  0x4
LPTMR_PSR_PBYP_SHIFT     EQU  2
LPTMR_PSR_PRESCALE_MASK  EQU  0x78
LPTMR_PSR_PRESCALE_SHIFT EQU  3

; CMR Bit Fields
LPTMR_CMR_COMPARE_MASK   EQU  0xFFFF
LPTMR_CMR_COMPARE_SHIFT  EQU  0

; CNR Bit Fields
LPTMR_CNR_COUNTER_MASK   EQU  0xFFFF
LPTMR_CNR_COUNTER_SHIFT  EQU  0

; ----------------------------------------------------------------------------
; -- MCG Peripheral Access Layer
; ----------------------------------------------------------------------------

MCG_C1       EQU  0x0   ; MCG Control 1 Register, offset: 0x0
MCG_C2       EQU  0x1   ; MCG Control 2 Register, offset: 0x1
MCG_C3       EQU  0x2   ; MCG Control 3 Register, offset: 0x2
MCG_C4       EQU  0x3   ; MCG Control 4 Register, offset: 0x3
MCG_C5       EQU  0x4   ; MCG Control 5 Register, offset: 0x4
MCG_C6       EQU  0x5   ; MCG Control 6 Register, offset: 0x5
MCG_S        EQU  0x6   ; MCG Status Register, offset: 0x6
MCG_SC       EQU  0x8   ; MCG Status and Control Register, offset: 0x8
MCG_ATCVH    EQU  0xA   ; MCG Auto Trim Compare Value High Register, offset: 0xA
MCG_ATCVL    EQU  0xB   ; MCG Auto Trim Compare Value Low Register, offset: 0xB
MCG_C7       EQU  0xC   ; MCG Control 7 Register, offset: 0xC
MCG_C8       EQU  0xD   ; MCG Control 8 Register, offset: 0xD

; ----------------------------------------------------------------------------
; -- MCG Register Masks
; ----------------------------------------------------------------------------

; C1 Bit Fields
MCG_C1_IREFSTEN_MASK     EQU  0x1
MCG_C1_IREFSTEN_SHIFT    EQU  0
MCG_C1_IRCLKEN_MASK      EQU  0x2
MCG_C1_IRCLKEN_SHIFT     EQU  1
MCG_C1_IREFS_MASK        EQU  0x4
MCG_C1_IREFS_SHIFT       EQU  2
MCG_C1_FRDIV_MASK        EQU  0x38
MCG_C1_FRDIV_SHIFT       EQU  3
MCG_C1_CLKS_MASK         EQU  0xC0
MCG_C1_CLKS_SHIFT        EQU  6

; C2 Bit Fields
MCG_C2_IRCS_MASK         EQU  0x1
MCG_C2_IRCS_SHIFT        EQU  0
MCG_C2_LP_MASK           EQU  0x2
MCG_C2_LP_SHIFT          EQU  1
MCG_C2_EREFS_MASK        EQU  0x4
MCG_C2_EREFS_SHIFT       EQU  2
MCG_C2_HGO_MASK          EQU  0x8
MCG_C2_HGO_SHIFT         EQU  3
MCG_C2_RANGE_MASK        EQU  0x30
MCG_C2_RANGE_SHIFT       EQU  4
MCG_C2_FCFTRIM_MASK      EQU  0x40
MCG_C2_FCFTRIM_SHIFT     EQU  6
MCG_C2_LOCRE0_MASK       EQU  0x80
MCG_C2_LOCRE0_SHIFT      EQU  7

; C3 Bit Fields
MCG_C3_SCTRIM_MASK       EQU  0xFF
MCG_C3_SCTRIM_SHIFT      EQU  0

; C4 Bit Fields
MCG_C4_SCFTRIM_MASK      EQU  0x1
MCG_C4_SCFTRIM_SHIFT     EQU  0
MCG_C4_FCTRIM_MASK       EQU  0x1E
MCG_C4_FCTRIM_SHIFT      EQU  1
MCG_C4_DRST_DRS_MASK     EQU  0x60
MCG_C4_DRST_DRS_SHIFT    EQU  5
MCG_C4_DMX32_MASK        EQU  0x80
MCG_C4_DMX32_SHIFT       EQU  7

; C5 Bit Fields
MCG_C5_PRDIV0_MASK       EQU  0x1F
MCG_C5_PRDIV0_SHIFT      EQU  0
MCG_C5_PLLSTEN0_MASK     EQU  0x20
MCG_C5_PLLSTEN0_SHIFT    EQU  5
MCG_C5_PLLCLKEN0_MASK    EQU  0x40
MCG_C5_PLLCLKEN0_SHIFT   EQU  6

; C6 Bit Fields
MCG_C6_VDIV0_MASK        EQU  0x1F
MCG_C6_VDIV0_SHIFT       EQU  0
MCG_C6_CME0_MASK         EQU  0x20
MCG_C6_CME0_SHIFT        EQU  5
MCG_C6_PLLS_MASK         EQU  0x40
MCG_C6_PLLS_SHIFT        EQU  6
MCG_C6_LOLIE0_MASK       EQU  0x80
MCG_C6_LOLIE0_SHIFT      EQU  7

; S Bit Fields
MCG_S_IRCST_MASK         EQU  0x1
MCG_S_IRCST_SHIFT        EQU  0
MCG_S_OSCINIT0_MASK      EQU  0x2
MCG_S_OSCINIT0_SHIFT     EQU  1
MCG_S_CLKST_MASK         EQU  0xC
MCG_S_CLKST_SHIFT        EQU  2
MCG_S_IREFST_MASK        EQU  0x10
MCG_S_IREFST_SHIFT       EQU  4
MCG_S_PLLST_MASK         EQU  0x20
MCG_S_PLLST_SHIFT        EQU  5
MCG_S_LOCK0_MASK         EQU  0x40
MCG_S_LOCK0_SHIFT        EQU  6
MCG_S_LOLS0_MASK         EQU  0x80
MCG_S_LOLS0_SHIFT        EQU  7

; SC Bit Fields
MCG_SC_LOCS0_MASK        EQU  0x1
MCG_SC_LOCS0_SHIFT       EQU  0
MCG_SC_FCRDIV_MASK       EQU  0xE
MCG_SC_FCRDIV_SHIFT      EQU  1
MCG_SC_FLTPRSRV_MASK     EQU  0x10
MCG_SC_FLTPRSRV_SHIFT    EQU  4
MCG_SC_ATMF_MASK         EQU  0x20
MCG_SC_ATMF_SHIFT        EQU  5
MCG_SC_ATMS_MASK         EQU  0x40
MCG_SC_ATMS_SHIFT        EQU  6
MCG_SC_ATME_MASK         EQU  0x80
MCG_SC_ATME_SHIFT        EQU  7

; ATCVH Bit Fields
MCG_ATCVH_ATCVH_MASK     EQU  0xFF
MCG_ATCVH_ATCVH_SHIFT    EQU  0

; ATCVL Bit Fields
MCG_ATCVL_ATCVL_MASK     EQU  0xFF
MCG_ATCVL_ATCVL_SHIFT    EQU  0

; C7 Bit Fields
MCG_C7_OSCSEL_MASK       EQU  0x3
MCG_C7_OSCSEL_SHIFT      EQU  0

; C8 Bit Fields
MCG_C8_LOCS1_MASK        EQU  0x1
MCG_C8_LOCS1_SHIFT       EQU  0
MCG_C8_CME1_MASK         EQU  0x20
MCG_C8_CME1_SHIFT        EQU  5
MCG_C8_LOLRE_MASK        EQU  0x40
MCG_C8_LOLRE_SHIFT       EQU  6
MCG_C8_LOCRE1_MASK       EQU  0x80
MCG_C8_LOCRE1_SHIFT      EQU  7

; ----------------------------------------------------------------------------
; -- MCM Peripheral Access Layer
; ----------------------------------------------------------------------------

MCM_PLASC   EQU  0x08   ; Crossbar Switch (AXBS) Slave Configuration, offset: 0x8
MCM_PLAMC   EQU  0x0A   ; Crossbar Switch (AXBS) Master Configuration, offset: 0xA
MCM_CR      EQU  0x0C   ; Control Register, offset: 0xC
MCM_ISR     EQU  0x10   ; Interrupt Status Register, offset: 0x10
MCM_ETBCC   EQU  0x14   ; ETB Counter Control register, offset: 0x14
MCM_ETBRL   EQU  0x18   ; ETB Reload register, offset: 0x18
MCM_ETBCNT  EQU  0x1C   ; ETB Counter Value register, offset: 0x1C
MCM_PID     EQU  0x30   ; Process ID register, offset: 0x30

; ----------------------------------------------------------------------------
; -- MCM Register Masks
; ----------------------------------------------------------------------------

; PLASC Bit Fields
MCM_PLASC_ASC_MASK       EQU  0xFF
MCM_PLASC_ASC_SHIFT      EQU  0

; PLAMC Bit Fields
MCM_PLAMC_AMC_MASK       EQU  0xFF
MCM_PLAMC_AMC_SHIFT      EQU  0

; CR Bit Fields
MCM_CR_SRAMUAP_MASK      EQU  0x3000000
MCM_CR_SRAMUAP_SHIFT     EQU  24
MCM_CR_SRAMUWP_MASK      EQU  0x4000000
MCM_CR_SRAMUWP_SHIFT     EQU  26
MCM_CR_SRAMLAP_MASK      EQU  0x30000000
MCM_CR_SRAMLAP_SHIFT     EQU  28
MCM_CR_SRAMLWP_MASK      EQU  0x40000000
MCM_CR_SRAMLWP_SHIFT     EQU  30

; ISR Bit Fields
MCM_ISR_IRQ_MASK         EQU  0x2
MCM_ISR_IRQ_SHIFT        EQU  1
MCM_ISR_NMI_MASK         EQU  0x4
MCM_ISR_NMI_SHIFT        EQU  2
MCM_ISR_DHREQ_MASK       EQU  0x8
MCM_ISR_DHREQ_SHIFT      EQU  3
MCM_ISR_FIOC_MASK        EQU  0x100
MCM_ISR_FIOC_SHIFT       EQU  8
MCM_ISR_FDZC_MASK        EQU  0x200
MCM_ISR_FDZC_SHIFT       EQU  9
MCM_ISR_FOFC_MASK        EQU  0x400
MCM_ISR_FOFC_SHIFT       EQU  10
MCM_ISR_FUFC_MASK        EQU  0x800
MCM_ISR_FUFC_SHIFT       EQU  11
MCM_ISR_FIXC_MASK        EQU  0x1000
MCM_ISR_FIXC_SHIFT       EQU  12
MCM_ISR_FIDC_MASK        EQU  0x8000
MCM_ISR_FIDC_SHIFT       EQU  15
MCM_ISR_FIOCE_MASK       EQU  0x1000000
MCM_ISR_FIOCE_SHIFT      EQU  24
MCM_ISR_FDZCE_MASK       EQU  0x2000000
MCM_ISR_FDZCE_SHIFT      EQU  25
MCM_ISR_FOFCE_MASK       EQU  0x4000000
MCM_ISR_FOFCE_SHIFT      EQU  26
MCM_ISR_FUFCE_MASK       EQU  0x8000000
MCM_ISR_FUFCE_SHIFT      EQU  27
MCM_ISR_FIXCE_MASK       EQU  0x10000000
MCM_ISR_FIXCE_SHIFT      EQU  28
MCM_ISR_FIDCE_MASK       EQU  0x80000000
MCM_ISR_FIDCE_SHIFT      EQU  31

; ETBCC Bit Fields
MCM_ETBCC_CNTEN_MASK     EQU  0x1
MCM_ETBCC_CNTEN_SHIFT    EQU  0
MCM_ETBCC_RSPT_MASK      EQU  0x6
MCM_ETBCC_RSPT_SHIFT     EQU  1
MCM_ETBCC_RLRQ_MASK      EQU  0x8
MCM_ETBCC_RLRQ_SHIFT     EQU  3
MCM_ETBCC_ETDIS_MASK     EQU  0x10
MCM_ETBCC_ETDIS_SHIFT    EQU  4
MCM_ETBCC_ITDIS_MASK     EQU  0x20
MCM_ETBCC_ITDIS_SHIFT    EQU  5

; ETBRL Bit Fields
MCM_ETBRL_RELOAD_MASK    EQU  0x7FF
MCM_ETBRL_RELOAD_SHIFT   EQU  0

; ETBCNT Bit Fields
MCM_ETBCNT_COUNTER_MASK  EQU  0x7FF
MCM_ETBCNT_COUNTER_SHIFT EQU  0

; PID Bit Fields
MCM_PID_PID_MASK         EQU  0xFF
MCM_PID_PID_SHIFT        EQU  0

; ----------------------------------------------------------------------------
; -- MPU Peripheral Access Layer
; ----------------------------------------------------------------------------

MPU_CESR        EQU  0x000   ; Control/Error Status Register, offset: 0x0
MPU_SP0_EAR     EQU  0x010   ; Error Address Register, slave port n, array offset: 0x10, array step: 0x8
MPU_SP0_EDR     EQU  0x014   ; Error Detail Register, slave port n, array offset: 0x14, array step: 0x8
MPU_SP1_EAR     EQU  0x018   ; Error Address Register, slave port n, array offset: 0x10, array step: 0x8
MPU_SP1_EDR     EQU  0x02C   ; Error Detail Register, slave port n, array offset: 0x14, array step: 0x8
MPU_SP2_EAR     EQU  0x020   ; Error Address Register, slave port n, array offset: 0x10, array step: 0x8
MPU_SP2_EDR     EQU  0x024   ; Error Detail Register, slave port n, array offset: 0x14, array step: 0x8
MPU_SP3_EAR     EQU  0x028   ; Error Address Register, slave port n, array offset: 0x10, array step: 0x8
MPU_SP3_EDR     EQU  0x03C   ; Error Detail Register, slave port n, array offset: 0x14, array step: 0x8
MPU_SP4_EAR     EQU  0x030   ; Error Address Register, slave port n, array offset: 0x10, array step: 0x8
MPU_SP4_EDR     EQU  0x034   ; Error Detail Register, slave port n, array offset: 0x14, array step: 0x8

MPU_WORD        EQU  0x400   ; Region Descriptor n, Word 0..Region Descriptor n, Word 3, array offset: 0x400, array step: index*0x10, index2*0x4

MPU_RGDAAC0     EQU  0x800   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC1     EQU  0x804   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC2     EQU  0x808   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC3     EQU  0x80C   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC4     EQU  0x810   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC5     EQU  0x814   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC6     EQU  0x818   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC7     EQU  0x81C   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC8     EQU  0x820   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC9     EQU  0x824   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC10    EQU  0x828   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
MPU_RGDAAC11    EQU  0x80C   ; Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4

; ----------------------------------------------------------------------------
; -- MPU Register Masks
; ----------------------------------------------------------------------------

; CESR Bit Fields
MPU_CESR_VLD_MASK        EQU  0x1
MPU_CESR_VLD_SHIFT       EQU  0
MPU_CESR_NRGD_MASK       EQU  0xF00
MPU_CESR_NRGD_SHIFT      EQU  8
MPU_CESR_NSP_MASK        EQU  0xF000
MPU_CESR_NSP_SHIFT       EQU  12
MPU_CESR_HRL_MASK        EQU  0xF0000
MPU_CESR_HRL_SHIFT       EQU  16
MPU_CESR_SPERR_MASK      EQU  0xF8000000
MPU_CESR_SPERR_SHIFT     EQU  27

; EAR Bit Fields
MPU_EAR_EADDR_MASK       EQU  0xFFFFFFFF
MPU_EAR_EADDR_SHIFT      EQU  0

; EDR Bit Fields
MPU_EDR_ERW_MASK         EQU  0x1
MPU_EDR_ERW_SHIFT        EQU  0
MPU_EDR_EATTR_MASK       EQU  0xE
MPU_EDR_EATTR_SHIFT      EQU  1
MPU_EDR_EMN_MASK         EQU  0xF0
MPU_EDR_EMN_SHIFT        EQU  4
MPU_EDR_EPID_MASK        EQU  0xFF00
MPU_EDR_EPID_SHIFT       EQU  8
MPU_EDR_EACD_MASK        EQU  0xFFFF0000
MPU_EDR_EACD_SHIFT       EQU  16

; WORD Bit Fields
MPU_WORD_VLD_MASK        EQU  0x1
MPU_WORD_VLD_SHIFT       EQU  0
MPU_WORD_M0UM_MASK       EQU  0x7
MPU_WORD_M0UM_SHIFT      EQU  0
MPU_WORD_M0SM_MASK       EQU  0x18
MPU_WORD_M0SM_SHIFT      EQU  3
MPU_WORD_M0PE_MASK       EQU  0x20
MPU_WORD_M0PE_SHIFT      EQU  5
MPU_WORD_ENDADDR_MASK    EQU  0xFFFFFFE0
MPU_WORD_ENDADDR_SHIFT   EQU  5
MPU_WORD_SRTADDR_MASK    EQU  0xFFFFFFE0
MPU_WORD_SRTADDR_SHIFT   EQU  5
MPU_WORD_M1UM_MASK       EQU  0x1C0
MPU_WORD_M1UM_SHIFT      EQU  6
MPU_WORD_M1SM_MASK       EQU  0x600
MPU_WORD_M1SM_SHIFT      EQU  9
MPU_WORD_M1PE_MASK       EQU  0x800
MPU_WORD_M1PE_SHIFT      EQU  11
MPU_WORD_M2UM_MASK       EQU  0x7000
MPU_WORD_M2UM_SHIFT      EQU  12
MPU_WORD_M2SM_MASK       EQU  0x18000
MPU_WORD_M2SM_SHIFT      EQU  15
MPU_WORD_PIDMASK_MASK    EQU  0xFF0000
MPU_WORD_PIDMASK_SHIFT   EQU  16
MPU_WORD_M2PE_MASK       EQU  0x20000
MPU_WORD_M2PE_SHIFT      EQU  17
MPU_WORD_M3UM_MASK       EQU  0x1C0000
MPU_WORD_M3UM_SHIFT      EQU  18
MPU_WORD_M3SM_MASK       EQU  0x600000
MPU_WORD_M3SM_SHIFT      EQU  21
MPU_WORD_M3PE_MASK       EQU  0x800000
MPU_WORD_M3PE_SHIFT      EQU  23
MPU_WORD_PID_MASK        EQU  0xFF000000
MPU_WORD_PID_SHIFT       EQU  24
MPU_WORD_M4WE_MASK       EQU  0x1000000
MPU_WORD_M4WE_SHIFT      EQU  24
MPU_WORD_M4RE_MASK       EQU  0x2000000
MPU_WORD_M4RE_SHIFT      EQU  25
MPU_WORD_M5WE_MASK       EQU  0x4000000
MPU_WORD_M5WE_SHIFT      EQU  26
MPU_WORD_M5RE_MASK       EQU  0x8000000
MPU_WORD_M5RE_SHIFT      EQU  27
MPU_WORD_M6WE_MASK       EQU  0x10000000
MPU_WORD_M6WE_SHIFT      EQU  28
MPU_WORD_M6RE_MASK       EQU  0x20000000
MPU_WORD_M6RE_SHIFT      EQU  29
MPU_WORD_M7WE_MASK       EQU  0x40000000
MPU_WORD_M7WE_SHIFT      EQU  30
MPU_WORD_M7RE_MASK       EQU  0x80000000
MPU_WORD_M7RE_SHIFT      EQU  31

; RGDAAC Bit Fields
MPU_RGDAAC_M0UM_MASK     EQU  0x7
MPU_RGDAAC_M0UM_SHIFT    EQU  0
MPU_RGDAAC_M0SM_MASK     EQU  0x18
MPU_RGDAAC_M0SM_SHIFT    EQU  3
MPU_RGDAAC_M0PE_MASK     EQU  0x20
MPU_RGDAAC_M0PE_SHIFT    EQU  5
MPU_RGDAAC_M1UM_MASK     EQU  0x1C0
MPU_RGDAAC_M1UM_SHIFT    EQU  6
MPU_RGDAAC_M1SM_MASK     EQU  0x600
MPU_RGDAAC_M1SM_SHIFT    EQU  9
MPU_RGDAAC_M1PE_MASK     EQU  0x800
MPU_RGDAAC_M1PE_SHIFT    EQU  11
MPU_RGDAAC_M2UM_MASK     EQU  0x7000
MPU_RGDAAC_M2UM_SHIFT    EQU  12
MPU_RGDAAC_M2SM_MASK     EQU  0x18000
MPU_RGDAAC_M2SM_SHIFT    EQU  15
MPU_RGDAAC_M2PE_MASK     EQU  0x20000
MPU_RGDAAC_M2PE_SHIFT    EQU  17
MPU_RGDAAC_M3UM_MASK     EQU  0x1C0000
MPU_RGDAAC_M3UM_SHIFT    EQU  18
MPU_RGDAAC_M3SM_MASK     EQU  0x600000
MPU_RGDAAC_M3SM_SHIFT    EQU  21
MPU_RGDAAC_M3PE_MASK     EQU  0x800000
MPU_RGDAAC_M3PE_SHIFT    EQU  23
MPU_RGDAAC_M4WE_MASK     EQU  0x1000000
MPU_RGDAAC_M4WE_SHIFT    EQU  24
MPU_RGDAAC_M4RE_MASK     EQU  0x2000000
MPU_RGDAAC_M4RE_SHIFT    EQU  25
MPU_RGDAAC_M5WE_MASK     EQU  0x4000000
MPU_RGDAAC_M5WE_SHIFT    EQU  26
MPU_RGDAAC_M5RE_MASK     EQU  0x8000000
MPU_RGDAAC_M5RE_SHIFT    EQU  27
MPU_RGDAAC_M6WE_MASK     EQU  0x10000000
MPU_RGDAAC_M6WE_SHIFT    EQU  28
MPU_RGDAAC_M6RE_MASK     EQU  0x20000000
MPU_RGDAAC_M6RE_SHIFT    EQU  29
MPU_RGDAAC_M7WE_MASK     EQU  0x40000000
MPU_RGDAAC_M7WE_SHIFT    EQU  30
MPU_RGDAAC_M7RE_MASK     EQU  0x80000000
MPU_RGDAAC_M7RE_SHIFT    EQU  31

; ----------------------------------------------------------------------------
; -- NV Peripheral Access Layer
; ----------------------------------------------------------------------------

NV_BACKKEY3 EQU  0x0   ; Backdoor Comparison Key 3., offset: 0x0
NV_BACKKEY2 EQU  0x1   ; Backdoor Comparison Key 2., offset: 0x1
NV_BACKKEY1 EQU  0x2   ; Backdoor Comparison Key 1., offset: 0x2
NV_BACKKEY0 EQU  0x3   ; Backdoor Comparison Key 0., offset: 0x3
NV_BACKKEY7 EQU  0x4   ; Backdoor Comparison Key 7., offset: 0x4
NV_BACKKEY6 EQU  0x5   ; Backdoor Comparison Key 6., offset: 0x5
NV_BACKKEY5 EQU  0x6   ; Backdoor Comparison Key 5., offset: 0x6
NV_BACKKEY4 EQU  0x7   ; Backdoor Comparison Key 4., offset: 0x7
NV_FPROT3   EQU  0x8   ; Non-volatile P-Flash Protection 1 - Low Register, offset: 0x8
NV_FPROT2   EQU  0x9   ; Non-volatile P-Flash Protection 1 - High Register, offset: 0x9
NV_FPROT1   EQU  0xA   ; Non-volatile P-Flash Protection 0 - Low Register, offset: 0xA
NV_FPROT0   EQU  0xB   ; Non-volatile P-Flash Protection 0 - High Register, offset: 0xB
NV_FSEC     EQU  0xC   ; Non-volatile Flash Security Register, offset: 0xC
NV_FOPT     EQU  0xD   ; Non-volatile Flash Option Register, offset: 0xD
NV_FEPROT   EQU  0xE   ; Non-volatile EERAM Protection Register, offset: 0xE
NV_FDPROT   EQU  0xF   ; Non-volatile D-Flash Protection Register, offset: 0xF

; NV - Register accessors

; ----------------------------------------------------------------------------
; -- NV Register Masks
; ----------------------------------------------------------------------------

; BACKKEY3 Bit Fields
NV_BACKKEY3_KEY_MASK     EQU  0xFF
NV_BACKKEY3_KEY_SHIFT    EQU  0

; BACKKEY2 Bit Fields
NV_BACKKEY2_KEY_MASK     EQU  0xFF
NV_BACKKEY2_KEY_SHIFT    EQU  0

; BACKKEY1 Bit Fields
NV_BACKKEY1_KEY_MASK     EQU  0xFF
NV_BACKKEY1_KEY_SHIFT    EQU  0

; BACKKEY0 Bit Fields
NV_BACKKEY0_KEY_MASK     EQU  0xFF
NV_BACKKEY0_KEY_SHIFT    EQU  0

; BACKKEY7 Bit Fields
NV_BACKKEY7_KEY_MASK     EQU  0xFF
NV_BACKKEY7_KEY_SHIFT    EQU  0

; BACKKEY6 Bit Fields
NV_BACKKEY6_KEY_MASK     EQU  0xFF
NV_BACKKEY6_KEY_SHIFT    EQU  0

; BACKKEY5 Bit Fields
NV_BACKKEY5_KEY_MASK     EQU  0xFF
NV_BACKKEY5_KEY_SHIFT    EQU  0

; BACKKEY4 Bit Fields
NV_BACKKEY4_KEY_MASK     EQU  0xFF
NV_BACKKEY4_KEY_SHIFT    EQU  0

; FPROT3 Bit Fields
NV_FPROT3_PROT_MASK      EQU  0xFF
NV_FPROT3_PROT_SHIFT     EQU  0

; FPROT2 Bit Fields
NV_FPROT2_PROT_MASK      EQU  0xFF
NV_FPROT2_PROT_SHIFT     EQU  0

; FPROT1 Bit Fields
NV_FPROT1_PROT_MASK      EQU  0xFF
NV_FPROT1_PROT_SHIFT     EQU  0

; FPROT0 Bit Fields
NV_FPROT0_PROT_MASK      EQU  0xFF
NV_FPROT0_PROT_SHIFT     EQU  0

; FSEC Bit Fields
NV_FSEC_SEC_MASK         EQU  0x3
NV_FSEC_SEC_SHIFT        EQU  0
NV_FSEC_FSLACC_MASK      EQU  0xC
NV_FSEC_FSLACC_SHIFT     EQU  2
NV_FSEC_MEEN_MASK        EQU  0x30
NV_FSEC_MEEN_SHIFT       EQU  4
NV_FSEC_KEYEN_MASK       EQU  0xC0
NV_FSEC_KEYEN_SHIFT      EQU  6

; FOPT Bit Fields
NV_FOPT_LPBOOT_MASK      EQU  0x1
NV_FOPT_LPBOOT_SHIFT     EQU  0
NV_FOPT_EZPORT_DIS_MASK  EQU  0x2
NV_FOPT_EZPORT_DIS_SHIFT EQU  1

; FEPROT Bit Fields
NV_FEPROT_EPROT_MASK     EQU  0xFF
NV_FEPROT_EPROT_SHIFT    EQU  0

; FDPROT Bit Fields
NV_FDPROT_DPROT_MASK     EQU  0xFF
NV_FDPROT_DPROT_SHIFT    EQU  0

; ----------------------------------------------------------------------------
; -- OSC Peripheral Access Layer
; ----------------------------------------------------------------------------

OSC_CR       EQU  0x000   ; OSC Control Register, offset: 0x0

; ----------------------------------------------------------------------------
; -- OSC Register Masks
; ----------------------------------------------------------------------------

; CR Bit Fields
OSC_CR_SC16P_MASK        EQU  0x1
OSC_CR_SC16P_SHIFT       EQU  0
OSC_CR_SC8P_MASK         EQU  0x2
OSC_CR_SC8P_SHIFT        EQU  1
OSC_CR_SC4P_MASK         EQU  0x4
OSC_CR_SC4P_SHIFT        EQU  2
OSC_CR_SC2P_MASK         EQU  0x8
OSC_CR_SC2P_SHIFT        EQU  3
OSC_CR_EREFSTEN_MASK     EQU  0x20
OSC_CR_EREFSTEN_SHIFT    EQU  5
OSC_CR_ERCLKEN_MASK      EQU  0x80
OSC_CR_ERCLKEN_SHIFT     EQU  7

; ----------------------------------------------------------------------------
; -- PDB Peripheral Access Layer
; ----------------------------------------------------------------------------

PDB_SC          EQU  0x000   ; Status and Control register, offset: 0x0
PDB_MOD         EQU  0x004   ; Modulus register, offset: 0x4
PDB_CNT         EQU  0x008   ; Counter register, offset: 0x8
PDB_IDLY        EQU  0x00C   ; Interrupt Delay register, offset: 0xC

PDB_CH0_C1      EQU  0x010   ; Channel n Control register 1, array offset: 0x10, array step: 0x28
PDB_CH0_S       EQU  0x014   ; Channel n Status register, array offset: 0x14, array step: 0x28
PDB_CH0_DLY0    EQU  0x018   ; Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4
PDB_CH0_DLY1    EQU  0x01C

PDB_CH1_C1      EQU  0x038   ; Channel n Control register 1, array offset: 0x10, array step: 0x28
PDB_CH1_S       EQU  0x03C   ; Channel n Status register, array offset: 0x14, array step: 0x28
PDB_CH1_DLY0    EQU  0x040   ; Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4
PDB_CH1_DLY1    EQU  0x044
  
PDB_DAC0_INTC   EQU  0x150   ; DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8
PDB_DAC0_INT    EQU  0x154   ; DAC Interval n register, array offset: 0x154, array step: 0x8
PDB_DAC1_INTC   EQU  0x158   ; DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8
PDB_DAC1_INT    EQU  0x15C   ; DAC Interval n register, array offset: 0x154, array step: 0x8

PDB_POEN        EQU  0x190   ; Pulse-Out n Enable register, offset: 0x190
PDB_PODLY0      EQU  0x194   ; Pulse-Out n Delay register, array offset: 0x194, array step: 0x4
PDB_PODLY1      EQU  0x198   ; Pulse-Out n Delay register, array offset: 0x194, array step: 0x4
PDB_PODLY2      EQU  0x19C   ; Pulse-Out n Delay register, array offset: 0x194, array step: 0x4

; ----------------------------------------------------------------------------
; -- PDB Register Masks
; ----------------------------------------------------------------------------

; SC Bit Fields
PDB_SC_LDOK_MASK         EQU  0x1
PDB_SC_LDOK_SHIFT        EQU  0
PDB_SC_CONT_MASK         EQU  0x2
PDB_SC_CONT_SHIFT        EQU  1
PDB_SC_MULT_MASK         EQU  0xC
PDB_SC_MULT_SHIFT        EQU  2
PDB_SC_PDBIE_MASK        EQU  0x20
PDB_SC_PDBIE_SHIFT       EQU  5
PDB_SC_PDBIF_MASK        EQU  0x40
PDB_SC_PDBIF_SHIFT       EQU  6
PDB_SC_PDBEN_MASK        EQU  0x80
PDB_SC_PDBEN_SHIFT       EQU  7
PDB_SC_TRGSEL_MASK       EQU  0xF00
PDB_SC_TRGSEL_SHIFT      EQU  8
PDB_SC_PRESCALER_MASK    EQU  0x7000
PDB_SC_PRESCALER_SHIFT   EQU  12
PDB_SC_DMAEN_MASK        EQU  0x8000
PDB_SC_DMAEN_SHIFT       EQU  15
PDB_SC_SWTRIG_MASK       EQU  0x10000
PDB_SC_SWTRIG_SHIFT      EQU  16
PDB_SC_PDBEIE_MASK       EQU  0x20000
PDB_SC_PDBEIE_SHIFT      EQU  17
PDB_SC_LDMOD_MASK        EQU  0xC0000
PDB_SC_LDMOD_SHIFT       EQU  18

; MOD Bit Fields
PDB_MOD_MOD_MASK         EQU  0xFFFF
PDB_MOD_MOD_SHIFT        EQU  0

; CNT Bit Fields
PDB_CNT_CNT_MASK         EQU  0xFFFF
PDB_CNT_CNT_SHIFT        EQU  0

; IDLY Bit Fields
PDB_IDLY_IDLY_MASK       EQU  0xFFFF
PDB_IDLY_IDLY_SHIFT      EQU  0

; C1 Bit Fields
PDB_C1_EN_MASK           EQU  0xFF
PDB_C1_EN_SHIFT          EQU  0
PDB_C1_TOS_MASK          EQU  0xFF00
PDB_C1_TOS_SHIFT         EQU  8
PDB_C1_BB_MASK           EQU  0xFF0000
PDB_C1_BB_SHIFT          EQU  16

; S Bit Fields
PDB_S_ERR_MASK           EQU  0xFF
PDB_S_ERR_SHIFT          EQU  0
PDB_S_CF_MASK            EQU  0xFF0000
PDB_S_CF_SHIFT           EQU  16

; DLY Bit Fields
PDB_DLY_DLY_MASK         EQU  0xFFFF
PDB_DLY_DLY_SHIFT        EQU  0

; INTC Bit Fields
PDB_INTC_TOE_MASK        EQU  0x1
PDB_INTC_TOE_SHIFT       EQU  0
PDB_INTC_EXT_MASK        EQU  0x2
PDB_INTC_EXT_SHIFT       EQU  1

; INT Bit Fields
PDB_INT_INT_MASK         EQU  0xFFFF
PDB_INT_INT_SHIFT        EQU  0

; POEN Bit Fields
PDB_POEN_POEN_MASK       EQU  0xFF
PDB_POEN_POEN_SHIFT      EQU  0

; PODLY Bit Fields
PDB_PODLY_DLY2_MASK      EQU  0xFFFF
PDB_PODLY_DLY2_SHIFT     EQU  0
PDB_PODLY_DLY1_MASK      EQU  0xFFFF0000
PDB_PODLY_DLY1_SHIFT     EQU  16

; ----------------------------------------------------------------------------
; -- PIT Peripheral Access Layer
; ----------------------------------------------------------------------------

PIT_MCR              EQU  0x000   ; PIT Module Control Register, offset: 0x0

PIT_CHANNEL0_LDVAL   EQU  0x100   ; Timer Load Value Register, array offset: 0x100, array step: 0x10
PIT_CHANNEL0_CVAL    EQU  0x104   ; Current Timer Value Register, array offset: 0x104, array step: 0x10
PIT_CHANNEL0_TCTRL   EQU  0x108   ; Timer Control Register, array offset: 0x108, array step: 0x10
PIT_CHANNEL0_TFLG    EQU  0x10C   ; Timer Flag Register, array offset: 0x10C, array step: 0x10

PIT_CHANNEL1_LDVAL   EQU  0x110   ; Timer Load Value Register, array offset: 0x100, array step: 0x10
PIT_CHANNEL1_CVAL    EQU  0x114   ; Current Timer Value Register, array offset: 0x104, array step: 0x10
PIT_CHANNEL1_TCTRL   EQU  0x118   ; Timer Control Register, array offset: 0x108, array step: 0x10
PIT_CHANNEL1_TFLG    EQU  0x11C   ; Timer Flag Register, array offset: 0x10C, array step: 0x10

PIT_CHANNEL2_LDVAL   EQU  0x120   ; Timer Load Value Register, array offset: 0x100, array step: 0x10
PIT_CHANNEL2_CVAL    EQU  0x124   ; Current Timer Value Register, array offset: 0x104, array step: 0x10
PIT_CHANNEL2_TCTRL   EQU  0x128   ; Timer Control Register, array offset: 0x108, array step: 0x10
PIT_CHANNEL2_TFLG    EQU  0x12C   ; Timer Flag Register, array offset: 0x10C, array step: 0x10

PIT_CHANNEL3_LDVAL   EQU  0x130   ; Timer Load Value Register, array offset: 0x100, array step: 0x10
PIT_CHANNEL3_CVAL    EQU  0x134   ; Current Timer Value Register, array offset: 0x104, array step: 0x10
PIT_CHANNEL3_TCTRL   EQU  0x138   ; Timer Control Register, array offset: 0x108, array step: 0x10
PIT_CHANNEL3_TFLG    EQU  0x13C   ; Timer Flag Register, array offset: 0x10C, array step: 0x10


; ----------------------------------------------------------------------------
; -- PIT Register Masks
; ----------------------------------------------------------------------------

; MCR Bit Fields
PIT_MCR_FRZ_MASK         EQU  0x1
PIT_MCR_FRZ_SHIFT        EQU  0
PIT_MCR_MDIS_MASK        EQU  0x2
PIT_MCR_MDIS_SHIFT       EQU  1

; LDVAL Bit Fields
PIT_LDVAL_TSV_MASK       EQU  0xFFFFFFFF
PIT_LDVAL_TSV_SHIFT      EQU  0

; CVAL Bit Fields
PIT_CVAL_TVL_MASK        EQU  0xFFFFFFFF
PIT_CVAL_TVL_SHIFT       EQU  0

; TCTRL Bit Fields
PIT_TCTRL_TEN_MASK       EQU  0x1
PIT_TCTRL_TEN_SHIFT      EQU  0
PIT_TCTRL_TIE_MASK       EQU  0x2
PIT_TCTRL_TIE_SHIFT      EQU  1
PIT_TCTRL_CHN_MASK       EQU  0x4
PIT_TCTRL_CHN_SHIFT      EQU  2

; TFLG Bit Fields
PIT_TFLG_TIF_MASK        EQU  0x1
PIT_TFLG_TIF_SHIFT       EQU  0

; ----------------------------------------------------------------------------
; -- PMC Peripheral Access Layer
; ----------------------------------------------------------------------------

PMC_LVDSC1   EQU  0x0   ; Low Voltage Detect Status And Control 1 register, offset: 0x0
PMC_LVDSC2   EQU  0x1   ; Low Voltage Detect Status And Control 2 register, offset: 0x1
PMC_REGSC    EQU  0x2   ; Regulator Status And Control register, offset: 0x2

; ----------------------------------------------------------------------------
; -- PMC Register Masks
; ----------------------------------------------------------------------------

; LVDSC1 Bit Fields
PMC_LVDSC1_LVDV_MASK     EQU  0x3
PMC_LVDSC1_LVDV_SHIFT    EQU  0
PMC_LVDSC1_LVDRE_MASK    EQU  0x10
PMC_LVDSC1_LVDRE_SHIFT   EQU  4
PMC_LVDSC1_LVDIE_MASK    EQU  0x20
PMC_LVDSC1_LVDIE_SHIFT   EQU  5
PMC_LVDSC1_LVDACK_MASK   EQU  0x40
PMC_LVDSC1_LVDACK_SHIFT  EQU  6
PMC_LVDSC1_LVDF_MASK     EQU  0x80
PMC_LVDSC1_LVDF_SHIFT    EQU  7

; LVDSC2 Bit Fields
PMC_LVDSC2_LVWV_MASK     EQU  0x3
PMC_LVDSC2_LVWV_SHIFT    EQU  0
PMC_LVDSC2_LVWIE_MASK    EQU  0x20
PMC_LVDSC2_LVWIE_SHIFT   EQU  5
PMC_LVDSC2_LVWACK_MASK   EQU  0x40
PMC_LVDSC2_LVWACK_SHIFT  EQU  6
PMC_LVDSC2_LVWF_MASK     EQU  0x80
PMC_LVDSC2_LVWF_SHIFT    EQU  7

; REGSC Bit Fields
PMC_REGSC_BGBE_MASK      EQU  0x1
PMC_REGSC_BGBE_SHIFT     EQU  0
PMC_REGSC_REGONS_MASK    EQU  0x4
PMC_REGSC_REGONS_SHIFT   EQU  2
PMC_REGSC_ACKISO_MASK    EQU  0x8
PMC_REGSC_ACKISO_SHIFT   EQU  3
PMC_REGSC_BGEN_MASK      EQU  0x10
PMC_REGSC_BGEN_SHIFT     EQU  4

; ----------------------------------------------------------------------------
; -- PORT Peripheral Access Layer
; ----------------------------------------------------------------------------

PORT_PCR0    EQU  0x00   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR1    EQU  0x04   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR2    EQU  0x08   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR3    EQU  0x0C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR4    EQU  0x10   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR5    EQU  0x14   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR6    EQU  0x18   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR7    EQU  0x1C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR8    EQU  0x20   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR9    EQU  0x24   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR10   EQU  0x28   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR11   EQU  0x2C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR12   EQU  0x30   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR13   EQU  0x34   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR14   EQU  0x38   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR15   EQU  0x3C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR16   EQU  0x40   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR17   EQU  0x44   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR18   EQU  0x48   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR19   EQU  0x4C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR20   EQU  0x50   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR21   EQU  0x54   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR22   EQU  0x58   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR23   EQU  0x5C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR24   EQU  0x60   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR25   EQU  0x64   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR26   EQU  0x68   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR27   EQU  0x6C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR28   EQU  0x70   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR29   EQU  0x74   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR30   EQU  0x78   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_PCR31   EQU  0x7C   ; Pin Control Register n, array offset: 0x0, array step: 0x4
PORT_GPCLR   EQU  0x80   ; Global Pin Control Low Register, offset: 0x80
PORT_GPCHR   EQU  0x84   ; Global Pin Control High Register, offset: 0x84
PORT_ISFR    EQU  0xA0   ; Interrupt Status Flag Register, offset: 0xA0
PORT_DFER    EQU  0xC0   ; Digital Filter Enable Register, offset: 0xC0
PORT_DFCR    EQU  0xC4   ; Digital Filter Clock Register, offset: 0xC4
PORT_DFWR    EQU  0xC8   ; Digital Filter Width Register, offset: 0xC8

; ----------------------------------------------------------------------------
; -- PORT Register Masks
; ----------------------------------------------------------------------------

; PCR Bit Fields
PORT_PCR_PS_MASK         EQU  0x1
PORT_PCR_PS_SHIFT        EQU  0
PORT_PCR_PE_MASK         EQU  0x2
PORT_PCR_PE_SHIFT        EQU  1
PORT_PCR_SRE_MASK        EQU  0x4
PORT_PCR_SRE_SHIFT       EQU  2
PORT_PCR_PFE_MASK        EQU  0x10
PORT_PCR_PFE_SHIFT       EQU  4
PORT_PCR_ODE_MASK        EQU  0x20
PORT_PCR_ODE_SHIFT       EQU  5
PORT_PCR_DSE_MASK        EQU  0x40
PORT_PCR_DSE_SHIFT       EQU  6
PORT_PCR_MUX_MASK        EQU  0x700
PORT_PCR_MUX_SHIFT       EQU  8
PORT_PCR_LK_MASK         EQU  0x8000
PORT_PCR_LK_SHIFT        EQU  15
PORT_PCR_IRQC_MASK       EQU  0xF0000
PORT_PCR_IRQC_SHIFT      EQU  16
PORT_PCR_ISF_MASK        EQU  0x1000000
PORT_PCR_ISF_SHIFT       EQU  24

; GPCLR Bit Fields
PORT_GPCLR_GPWD_MASK     EQU  0xFFFF
PORT_GPCLR_GPWD_SHIFT    EQU  0
PORT_GPCLR_GPWE_MASK     EQU  0xFFFF0000
PORT_GPCLR_GPWE_SHIFT    EQU  16

; GPCHR Bit Fields
PORT_GPCHR_GPWD_MASK     EQU  0xFFFF
PORT_GPCHR_GPWD_SHIFT    EQU  0
PORT_GPCHR_GPWE_MASK     EQU  0xFFFF0000
PORT_GPCHR_GPWE_SHIFT    EQU  16

; ISFR Bit Fields
PORT_ISFR_ISF_MASK       EQU  0xFFFFFFFF
PORT_ISFR_ISF_SHIFT      EQU  0

; DFER Bit Fields
PORT_DFER_DFE_MASK       EQU  0xFFFFFFFF
PORT_DFER_DFE_SHIFT      EQU  0

; DFCR Bit Fields
PORT_DFCR_CS_MASK        EQU  0x1
PORT_DFCR_CS_SHIFT       EQU  0

; DFWR Bit Fields
PORT_DFWR_FILT_MASK      EQU  0x1F
PORT_DFWR_FILT_SHIFT     EQU  0

; ----------------------------------------------------------------------------
; -- RCM Peripheral Access Layer
; ----------------------------------------------------------------------------

RCM_SRS0     EQU  0x0   ; System Reset Status Register 0, offset: 0x0
RCM_SRS1     EQU  0x1   ; System Reset Status Register 1, offset: 0x1
RCM_RPFC     EQU  0x4   ; Reset Pin Filter Control register, offset: 0x4
RCM_RPFW     EQU  0x5   ; Reset Pin Filter Width register, offset: 0x5
RCM_MR       EQU  0x7   ; Mode Register, offset: 0x7

; ----------------------------------------------------------------------------
; -- RCM Register Masks
; ----------------------------------------------------------------------------

; SRS0 Bit Fields
RCM_SRS0_WAKEUP_MASK     EQU  0x1
RCM_SRS0_WAKEUP_SHIFT    EQU  0
RCM_SRS0_LVD_MASK        EQU  0x2
RCM_SRS0_LVD_SHIFT       EQU  1
RCM_SRS0_LOC_MASK        EQU  0x4
RCM_SRS0_LOC_SHIFT       EQU  2
RCM_SRS0_LOL_MASK        EQU  0x8
RCM_SRS0_LOL_SHIFT       EQU  3
RCM_SRS0_WDOG_MASK       EQU  0x20
RCM_SRS0_WDOG_SHIFT      EQU  5
RCM_SRS0_PIN_MASK        EQU  0x40
RCM_SRS0_PIN_SHIFT       EQU  6
RCM_SRS0_POR_MASK        EQU  0x80
RCM_SRS0_POR_SHIFT       EQU  7

; SRS1 Bit Fields
RCM_SRS1_JTAG_MASK       EQU  0x1
RCM_SRS1_JTAG_SHIFT      EQU  0
RCM_SRS1_LOCKUP_MASK     EQU  0x2
RCM_SRS1_LOCKUP_SHIFT    EQU  1
RCM_SRS1_SW_MASK         EQU  0x4
RCM_SRS1_SW_SHIFT        EQU  2
RCM_SRS1_MDM_AP_MASK     EQU  0x8
RCM_SRS1_MDM_AP_SHIFT    EQU  3
RCM_SRS1_EZPT_MASK       EQU  0x10
RCM_SRS1_EZPT_SHIFT      EQU  4
RCM_SRS1_SACKERR_MASK    EQU  0x20
RCM_SRS1_SACKERR_SHIFT   EQU  5

; RPFC Bit Fields
RCM_RPFC_RSTFLTSRW_MASK  EQU  0x3
RCM_RPFC_RSTFLTSRW_SHIFT EQU  0
RCM_RPFC_RSTFLTSS_MASK   EQU  0x4
RCM_RPFC_RSTFLTSS_SHIFT  EQU  2

; RPFW Bit Fields
RCM_RPFW_RSTFLTSEL_MASK  EQU  0x1F
RCM_RPFW_RSTFLTSEL_SHIFT EQU  0

; MR Bit Fields
RCM_MR_EZP_MS_MASK       EQU  0x2
RCM_MR_EZP_MS_SHIFT      EQU  1

; ----------------------------------------------------------------------------
; -- RFSYS Peripheral Access Layer
; ----------------------------------------------------------------------------

RFSYS_REG0  EQU  0x00   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG1  EQU  0x04   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG2  EQU  0x08   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG3  EQU  0x0C   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG4  EQU  0x10   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG5  EQU  0x14   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG6  EQU  0x18   ; Register file register, array offset: 0x0, array step: 0x4
RFSYS_REG7  EQU  0x1C   ; Register file register, array offset: 0x0, array step: 0x4

; ----------------------------------------------------------------------------
; -- RFSYS Register Masks
; ----------------------------------------------------------------------------

; REG Bit Fields
RFSYS_REG_LL_MASK        EQU  0xFF
RFSYS_REG_LL_SHIFT       EQU  0
RFSYS_REG_LH_MASK        EQU  0xFF00
RFSYS_REG_LH_SHIFT       EQU  8
RFSYS_REG_HL_MASK        EQU  0xFF0000
RFSYS_REG_HL_SHIFT       EQU  16
RFSYS_REG_HH_MASK        EQU  0xFF000000
RFSYS_REG_HH_SHIFT       EQU  24

; ----------------------------------------------------------------------------
; -- RFVBAT Peripheral Access Layer
; ----------------------------------------------------------------------------

RFVBAT_REG0  EQU  0x00   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG1  EQU  0x04   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG2  EQU  0x08   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG3  EQU  0x0C   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG4  EQU  0x10   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG5  EQU  0x14   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG6  EQU  0x18   ; VBAT register file register, array offset: 0x0, array step: 0x4
RFVBAT_REG7  EQU  0x1C   ; VBAT register file register, array offset: 0x0, array step: 0x4

; ----------------------------------------------------------------------------
; -- RFVBAT Register Masks
; ----------------------------------------------------------------------------

; REG Bit Fields
RFVBAT_REG_LL_MASK       EQU  0xFF
RFVBAT_REG_LL_SHIFT      EQU  0
RFVBAT_REG_LH_MASK       EQU  0xFF00
RFVBAT_REG_LH_SHIFT      EQU  8
RFVBAT_REG_HL_MASK       EQU  0xFF0000
RFVBAT_REG_HL_SHIFT      EQU  16
RFVBAT_REG_HH_MASK       EQU  0xFF000000
RFVBAT_REG_HH_SHIFT      EQU  24

; ----------------------------------------------------------------------------
; -- RNG Peripheral Access Layer
; ----------------------------------------------------------------------------

RNG_CR  EQU  0x0   ; RNGA Control Register, offset: 0x0
RNG_SR  EQU  0x4   ; RNGA Status Register, offset: 0x4
RNG_ER  EQU  0x8   ; RNGA Entropy Register, offset: 0x8
RNG_OR  EQU  0xC   ; RNGA Output Register, offset: 0xC

; ----------------------------------------------------------------------------
; -- RNG Register Masks
; ----------------------------------------------------------------------------

; CR Bit Fields
RNG_CR_GO_MASK           EQU  0x1
RNG_CR_GO_SHIFT          EQU  0
RNG_CR_HA_MASK           EQU  0x2
RNG_CR_HA_SHIFT          EQU  1
RNG_CR_INTM_MASK         EQU  0x4
RNG_CR_INTM_SHIFT        EQU  2
RNG_CR_CLRI_MASK         EQU  0x8
RNG_CR_CLRI_SHIFT        EQU  3
RNG_CR_SLP_MASK          EQU  0x10
RNG_CR_SLP_SHIFT         EQU  4

; SR Bit Fields
RNG_SR_SECV_MASK         EQU  0x1
RNG_SR_SECV_SHIFT        EQU  0
RNG_SR_LRS_MASK          EQU  0x2
RNG_SR_LRS_SHIFT         EQU  1
RNG_SR_ORU_MASK          EQU  0x4
RNG_SR_ORU_SHIFT         EQU  2
RNG_SR_ERRI_MASK         EQU  0x8
RNG_SR_ERRI_SHIFT        EQU  3
RNG_SR_SLP_MASK          EQU  0x10
RNG_SR_SLP_SHIFT         EQU  4
RNG_SR_OREG_LVL_MASK     EQU  0xFF00
RNG_SR_OREG_LVL_SHIFT    EQU  8
RNG_SR_OREG_SIZE_MASK    EQU  0xFF0000
RNG_SR_OREG_SIZE_SHIFT   EQU  16

; ER Bit Fields
RNG_ER_EXT_ENT_MASK      EQU  0xFFFFFFFF
RNG_ER_EXT_ENT_SHIFT     EQU  0

; OR Bit Fields
RNG_OR_RANDOUT_MASK      EQU  0xFFFFFFFF
RNG_OR_RANDOUT_SHIFT     EQU  0

; ----------------------------------------------------------------------------
; -- RTC Peripheral Access Layer
; ----------------------------------------------------------------------------

RTC_TSR     EQU  0x000   ; RTC Time Seconds Register, offset: 0x0
RTC_TPR     EQU  0x004   ; RTC Time Prescaler Register, offset: 0x4
RTC_TAR     EQU  0x008   ; RTC Time Alarm Register, offset: 0x8
RTC_TCR     EQU  0x00C   ; RTC Time Compensation Register, offset: 0xC
RTC_CR      EQU  0x010   ; RTC Control Register, offset: 0x10
RTC_SR      EQU  0x014   ; RTC Status Register, offset: 0x14
RTC_LR      EQU  0x018   ; RTC Lock Register, offset: 0x18
RTC_IER     EQU  0x01C   ; RTC Interrupt Enable Register, offset: 0x1C
RTC_WAR     EQU  0x800   ; RTC Write Access Register, offset: 0x800
RTC_RAR     EQU  0x804   ; RTC Read Access Register, offset: 0x804

; ----------------------------------------------------------------------------
; -- RTC Register Masks
; ----------------------------------------------------------------------------

; TSR Bit Fields
RTC_TSR_TSR_MASK         EQU  0xFFFFFFFF
RTC_TSR_TSR_SHIFT        EQU  0

; TPR Bit Fields
RTC_TPR_TPR_MASK         EQU  0xFFFF
RTC_TPR_TPR_SHIFT        EQU  0

; TAR Bit Fields
RTC_TAR_TAR_MASK         EQU  0xFFFFFFFF
RTC_TAR_TAR_SHIFT        EQU  0

; TCR Bit Fields
RTC_TCR_TCR_MASK         EQU  0xFF
RTC_TCR_TCR_SHIFT        EQU  0
RTC_TCR_CIR_MASK         EQU  0xFF00
RTC_TCR_CIR_SHIFT        EQU  8
RTC_TCR_TCV_MASK         EQU  0xFF0000
RTC_TCR_TCV_SHIFT        EQU  16
RTC_TCR_CIC_MASK         EQU  0xFF000000
RTC_TCR_CIC_SHIFT        EQU  24

; CR Bit Fields
RTC_CR_SWR_MASK          EQU  0x1
RTC_CR_SWR_SHIFT         EQU  0
RTC_CR_WPE_MASK          EQU  0x2
RTC_CR_WPE_SHIFT         EQU  1
RTC_CR_SUP_MASK          EQU  0x4
RTC_CR_SUP_SHIFT         EQU  2
RTC_CR_UM_MASK           EQU  0x8
RTC_CR_UM_SHIFT          EQU  3
RTC_CR_WPS_MASK          EQU  0x10
RTC_CR_WPS_SHIFT         EQU  4
RTC_CR_OSCE_MASK         EQU  0x100
RTC_CR_OSCE_SHIFT        EQU  8
RTC_CR_CLKO_MASK         EQU  0x200
RTC_CR_CLKO_SHIFT        EQU  9
RTC_CR_SC16P_MASK        EQU  0x400
RTC_CR_SC16P_SHIFT       EQU  10
RTC_CR_SC8P_MASK         EQU  0x800
RTC_CR_SC8P_SHIFT        EQU  11
RTC_CR_SC4P_MASK         EQU  0x1000
RTC_CR_SC4P_SHIFT        EQU  12
RTC_CR_SC2P_MASK         EQU  0x2000
RTC_CR_SC2P_SHIFT        EQU  13

; SR Bit Fields
RTC_SR_TIF_MASK          EQU  0x1
RTC_SR_TIF_SHIFT         EQU  0
RTC_SR_TOF_MASK          EQU  0x2
RTC_SR_TOF_SHIFT         EQU  1
RTC_SR_TAF_MASK          EQU  0x4
RTC_SR_TAF_SHIFT         EQU  2
RTC_SR_TCE_MASK          EQU  0x10
RTC_SR_TCE_SHIFT         EQU  4

; LR Bit Fields
RTC_LR_TCL_MASK          EQU  0x8
RTC_LR_TCL_SHIFT         EQU  3
RTC_LR_CRL_MASK          EQU  0x10
RTC_LR_CRL_SHIFT         EQU  4
RTC_LR_SRL_MASK          EQU  0x20
RTC_LR_SRL_SHIFT         EQU  5
RTC_LR_LRL_MASK          EQU  0x40
RTC_LR_LRL_SHIFT         EQU  6

; IER Bit Fields
RTC_IER_TIIE_MASK        EQU  0x1
RTC_IER_TIIE_SHIFT       EQU  0
RTC_IER_TOIE_MASK        EQU  0x2
RTC_IER_TOIE_SHIFT       EQU  1
RTC_IER_TAIE_MASK        EQU  0x4
RTC_IER_TAIE_SHIFT       EQU  2
RTC_IER_TSIE_MASK        EQU  0x10
RTC_IER_TSIE_SHIFT       EQU  4
RTC_IER_WPON_MASK        EQU  0x80
RTC_IER_WPON_SHIFT       EQU  7

; WAR Bit Fields
RTC_WAR_TSRW_MASK        EQU  0x1
RTC_WAR_TSRW_SHIFT       EQU  0
RTC_WAR_TPRW_MASK        EQU  0x2
RTC_WAR_TPRW_SHIFT       EQU  1
RTC_WAR_TARW_MASK        EQU  0x4
RTC_WAR_TARW_SHIFT       EQU  2
RTC_WAR_TCRW_MASK        EQU  0x8
RTC_WAR_TCRW_SHIFT       EQU  3
RTC_WAR_CRW_MASK         EQU  0x10
RTC_WAR_CRW_SHIFT        EQU  4
RTC_WAR_SRW_MASK         EQU  0x20
RTC_WAR_SRW_SHIFT        EQU  5
RTC_WAR_LRW_MASK         EQU  0x40
RTC_WAR_LRW_SHIFT        EQU  6
RTC_WAR_IERW_MASK        EQU  0x80
RTC_WAR_IERW_SHIFT       EQU  7

; RAR Bit Fields
RTC_RAR_TSRR_MASK        EQU  0x1
RTC_RAR_TSRR_SHIFT       EQU  0
RTC_RAR_TPRR_MASK        EQU  0x2
RTC_RAR_TPRR_SHIFT       EQU  1
RTC_RAR_TARR_MASK        EQU  0x4
RTC_RAR_TARR_SHIFT       EQU  2
RTC_RAR_TCRR_MASK        EQU  0x8
RTC_RAR_TCRR_SHIFT       EQU  3
RTC_RAR_CRR_MASK         EQU  0x10
RTC_RAR_CRR_SHIFT        EQU  4
RTC_RAR_SRR_MASK         EQU  0x20
RTC_RAR_SRR_SHIFT        EQU  5
RTC_RAR_LRR_MASK         EQU  0x40
RTC_RAR_LRR_SHIFT        EQU  6
RTC_RAR_IERR_MASK        EQU  0x80
RTC_RAR_IERR_SHIFT       EQU  7

; ----------------------------------------------------------------------------
; -- SDHC Peripheral Access Layer
; ----------------------------------------------------------------------------

SDHC_DSADDR     EQU  0x00   ; DMA System Address register, offset: 0x0
SDHC_BLKATTR    EQU  0x04   ; Block Attributes register, offset: 0x4
SDHC_CMDARG     EQU  0x05   ; Command Argument register, offset: 0x8
SDHC_XFERTYP    EQU  0x0C   ; Transfer Type register, offset: 0xC
SDHC_CMDRSP0    EQU  0x10   ; Command Response 0..Command Response 3, array offset: 0x10, array step: 0x4
SDHC_CMDRSP1    EQU  0x14   ; Command Response 0..Command Response 3, array offset: 0x10, array step: 0x4
SDHC_CMDRSP2    EQU  0x18   ; Command Response 0..Command Response 3, array offset: 0x10, array step: 0x4
SDHC_CMDRSP3    EQU  0x1C   ; Command Response 0..Command Response 3, array offset: 0x10, array step: 0x4
SDHC_DATPORT    EQU  0x20   ; Buffer Data Port register, offset: 0x20
SDHC_PRSSTAT    EQU  0x24   ; Present State register, offset: 0x24
SDHC_PROCTL     EQU  0x28   ; Protocol Control register, offset: 0x28
SDHC_SYSCTL     EQU  0x2C   ; System Control register, offset: 0x2C
SDHC_IRQSTAT    EQU  0x30   ; Interrupt Status register, offset: 0x30
SDHC_IRQSTATEN  EQU  0x34   ; Interrupt Status Enable register, offset: 0x34
SDHC_IRQSIGEN   EQU  0x38   ; Interrupt Signal Enable register, offset: 0x38
SDHC_AC12ERR    EQU  0x3C   ; Auto CMD12 Error Status Register, offset: 0x3C
SDHC_HTCAPBLT   EQU  0x40   ; Host Controller Capabilities, offset: 0x40
SDHC_WML        EQU  0x44   ; Watermark Level Register, offset: 0x44
SDHC_FEVT       EQU  0x50   ; Force Event register, offset: 0x50
SDHC_ADMAES     EQU  0x54   ; ADMA Error Status register, offset: 0x54
SDHC_ADSADDR    EQU  0x58   ; ADMA System Address register, offset: 0x58
SDHC_VENDOR     EQU  0xC0   ; Vendor Specific register, offset: 0xC0
SDHC_MMCBOOT    EQU  0xC4   ; MMC Boot register, offset: 0xC4
SDHC_HOSTVER    EQU  0xFC   ; Host Controller Version, offset: 0xFC

; ----------------------------------------------------------------------------
; -- SDHC Register Masks
; ----------------------------------------------------------------------------

; DSADDR Bit Fields
SDHC_DSADDR_DSADDR_MASK    EQU  0xFFFFFFFC
SDHC_DSADDR_DSADDR_SHIFT   EQU  2

; BLKATTR Bit Fields
SDHC_BLKATTR_BLKSIZE_MASK  EQU  0x1FFF
SDHC_BLKATTR_BLKSIZE_SHIFT EQU  0
SDHC_BLKATTR_BLKCNT_MASK   EQU  0xFFFF0000
SDHC_BLKATTR_BLKCNT_SHIFT  EQU  16

; CMDARG Bit Fields
SDHC_CMDARG_CMDARG_MASK   EQU  0xFFFFFFFF
SDHC_CMDARG_CMDARG_SHIFT  EQU  0

; XFERTYP Bit Fields
SDHC_XFERTYP_DMAEN_MASK   EQU  0x1
SDHC_XFERTYP_DMAEN_SHIFT  EQU  0
SDHC_XFERTYP_BCEN_MASK    EQU  0x2
SDHC_XFERTYP_BCEN_SHIFT   EQU  1
SDHC_XFERTYP_AC12EN_MASK  EQU  0x4
SDHC_XFERTYP_AC12EN_SHIFT EQU  2
SDHC_XFERTYP_DTDSEL_MASK  EQU  0x10
SDHC_XFERTYP_DTDSEL_SHIFT EQU  4
SDHC_XFERTYP_MSBSEL_MASK  EQU  0x20
SDHC_XFERTYP_MSBSEL_SHIFT EQU  5
SDHC_XFERTYP_RSPTYP_MASK  EQU  0x30000
SDHC_XFERTYP_RSPTYP_SHIFT EQU  16
SDHC_XFERTYP_CCCEN_MASK   EQU  0x80000
SDHC_XFERTYP_CCCEN_SHIFT  EQU  19
SDHC_XFERTYP_CICEN_MASK   EQU  0x100000
SDHC_XFERTYP_CICEN_SHIFT  EQU  20
SDHC_XFERTYP_DPSEL_MASK   EQU  0x200000
SDHC_XFERTYP_DPSEL_SHIFT  EQU  21
SDHC_XFERTYP_CMDTYP_MASK  EQU  0xC00000
SDHC_XFERTYP_CMDTYP_SHIFT EQU  22
SDHC_XFERTYP_CMDINX_MASK  EQU  0x3F000000
SDHC_XFERTYP_CMDINX_SHIFT EQU  24

; CMDRSP Bit Fields
SDHC_CMDRSP_CMDRSP0_MASK  EQU  0xFFFFFFFF
SDHC_CMDRSP_CMDRSP0_SHIFT EQU  0
SDHC_CMDRSP_CMDRSP1_MASK  EQU  0xFFFFFFFF
SDHC_CMDRSP_CMDRSP1_SHIFT EQU  0
SDHC_CMDRSP_CMDRSP2_MASK  EQU  0xFFFFFFFF
SDHC_CMDRSP_CMDRSP2_SHIFT EQU  0
SDHC_CMDRSP_CMDRSP3_MASK  EQU  0xFFFFFFFF
SDHC_CMDRSP_CMDRSP3_SHIFT EQU  0

; DATPORT Bit Fields
SDHC_DATPORT_DATCONT_MASK   EQU  0xFFFFFFFF
SDHC_DATPORT_DATCONT_SHIFT  EQU  0

; PRSSTAT Bit Fields
SDHC_PRSSTAT_CIHB_MASK    EQU  0x1
SDHC_PRSSTAT_CIHB_SHIFT   EQU  0
SDHC_PRSSTAT_CDIHB_MASK   EQU  0x2
SDHC_PRSSTAT_CDIHB_SHIFT  EQU  1
SDHC_PRSSTAT_DLA_MASK     EQU  0x4
SDHC_PRSSTAT_DLA_SHIFT    EQU  2
SDHC_PRSSTAT_SDSTB_MASK   EQU  0x8
SDHC_PRSSTAT_SDSTB_SHIFT  EQU  3
SDHC_PRSSTAT_IPGOFF_MASK  EQU  0x10
SDHC_PRSSTAT_IPGOFF_SHIFT EQU  4
SDHC_PRSSTAT_HCKOFF_MASK  EQU  0x20
SDHC_PRSSTAT_HCKOFF_SHIFT EQU  5
SDHC_PRSSTAT_PEROFF_MASK  EQU  0x40
SDHC_PRSSTAT_PEROFF_SHIFT EQU  6
SDHC_PRSSTAT_SDOFF_MASK   EQU  0x80
SDHC_PRSSTAT_SDOFF_SHIFT  EQU  7
SDHC_PRSSTAT_WTA_MASK     EQU  0x100
SDHC_PRSSTAT_WTA_SHIFT    EQU  8
SDHC_PRSSTAT_RTA_MASK     EQU  0x200
SDHC_PRSSTAT_RTA_SHIFT    EQU  9
SDHC_PRSSTAT_BWEN_MASK    EQU  0x400
SDHC_PRSSTAT_BWEN_SHIFT   EQU  10
SDHC_PRSSTAT_BREN_MASK    EQU  0x800
SDHC_PRSSTAT_BREN_SHIFT   EQU  11
SDHC_PRSSTAT_CINS_MASK    EQU  0x10000
SDHC_PRSSTAT_CINS_SHIFT   EQU  16
SDHC_PRSSTAT_CLSL_MASK    EQU  0x800000
SDHC_PRSSTAT_CLSL_SHIFT   EQU  23
SDHC_PRSSTAT_DLSL_MASK    EQU  0xFF000000
SDHC_PRSSTAT_DLSL_SHIFT   EQU  24

; PROCTL Bit Fields
SDHC_PROCTL_LCTL_MASK     EQU  0x1
SDHC_PROCTL_LCTL_SHIFT    EQU  0
SDHC_PROCTL_DTW_MASK      EQU  0x6
SDHC_PROCTL_DTW_SHIFT     EQU  1
SDHC_PROCTL_D3CD_MASK     EQU  0x8
SDHC_PROCTL_D3CD_SHIFT    EQU  3
SDHC_PROCTL_EMODE_MASK    EQU  0x30
SDHC_PROCTL_EMODE_SHIFT   EQU  4
SDHC_PROCTL_CDTL_MASK     EQU  0x40
SDHC_PROCTL_CDTL_SHIFT    EQU  6
SDHC_PROCTL_CDSS_MASK     EQU  0x80
SDHC_PROCTL_CDSS_SHIFT    EQU  7
SDHC_PROCTL_DMAS_MASK     EQU  0x300
SDHC_PROCTL_DMAS_SHIFT    EQU  8
SDHC_PROCTL_SABGREQ_MASK  EQU  0x10000
SDHC_PROCTL_SABGREQ_SHIFT EQU  16
SDHC_PROCTL_CREQ_MASK     EQU  0x20000
SDHC_PROCTL_CREQ_SHIFT    EQU  17
SDHC_PROCTL_RWCTL_MASK    EQU  0x40000
SDHC_PROCTL_RWCTL_SHIFT   EQU  18
SDHC_PROCTL_IABG_MASK     EQU  0x80000
SDHC_PROCTL_IABG_SHIFT    EQU  19
SDHC_PROCTL_WECINT_MASK   EQU  0x1000000
SDHC_PROCTL_WECINT_SHIFT  EQU  24
SDHC_PROCTL_WECINS_MASK   EQU  0x2000000
SDHC_PROCTL_WECINS_SHIFT  EQU  25
SDHC_PROCTL_WECRM_MASK    EQU  0x4000000
SDHC_PROCTL_WECRM_SHIFT   EQU  26

; SYSCTL Bit Fields
SDHC_SYSCTL_IPGEN_MASK    EQU  0x1
SDHC_SYSCTL_IPGEN_SHIFT   EQU  0
SDHC_SYSCTL_HCKEN_MASK    EQU  0x2
SDHC_SYSCTL_HCKEN_SHIFT   EQU  1
SDHC_SYSCTL_PEREN_MASK    EQU  0x4
SDHC_SYSCTL_PEREN_SHIFT   EQU  2
SDHC_SYSCTL_SDCLKEN_MASK  EQU  0x8
SDHC_SYSCTL_SDCLKEN_SHIFT EQU  3
SDHC_SYSCTL_DVS_MASK      EQU  0xF0
SDHC_SYSCTL_DVS_SHIFT     EQU  4
SDHC_SYSCTL_SDCLKFS_MASK  EQU  0xFF00
SDHC_SYSCTL_SDCLKFS_SHIFT EQU  8
SDHC_SYSCTL_DTOCV_MASK    EQU  0xF0000
SDHC_SYSCTL_DTOCV_SHIFT   EQU  16
SDHC_SYSCTL_RSTA_MASK     EQU  0x1000000
SDHC_SYSCTL_RSTA_SHIFT    EQU  24
SDHC_SYSCTL_RSTC_MASK     EQU  0x2000000
SDHC_SYSCTL_RSTC_SHIFT    EQU  25
SDHC_SYSCTL_RSTD_MASK     EQU  0x4000000
SDHC_SYSCTL_RSTD_SHIFT    EQU  26
SDHC_SYSCTL_INITA_MASK    EQU  0x8000000
SDHC_SYSCTL_INITA_SHIFT   EQU  27

; IRQSTAT Bit Fields
SDHC_IRQSTAT_CC_MASK     EQU  0x1
SDHC_IRQSTAT_CC_SHIFT    EQU  0
SDHC_IRQSTAT_TC_MASK     EQU  0x2
SDHC_IRQSTAT_TC_SHIFT    EQU  1
SDHC_IRQSTAT_BGE_MASK    EQU  0x4
SDHC_IRQSTAT_BGE_SHIFT   EQU  2
SDHC_IRQSTAT_DINT_MASK   EQU  0x8
SDHC_IRQSTAT_DINT_SHIFT  EQU  3
SDHC_IRQSTAT_BWR_MASK    EQU  0x10
SDHC_IRQSTAT_BWR_SHIFT   EQU  4
SDHC_IRQSTAT_BRR_MASK    EQU  0x20
SDHC_IRQSTAT_BRR_SHIFT   EQU  5
SDHC_IRQSTAT_CINS_MASK   EQU  0x40
SDHC_IRQSTAT_CINS_SHIFT  EQU  6
SDHC_IRQSTAT_CRM_MASK    EQU  0x80
SDHC_IRQSTAT_CRM_SHIFT   EQU  7
SDHC_IRQSTAT_CINT_MASK   EQU  0x100
SDHC_IRQSTAT_CINT_SHIFT  EQU  8
SDHC_IRQSTAT_CTOE_MASK   EQU  0x10000
SDHC_IRQSTAT_CTOE_SHIFT  EQU  16
SDHC_IRQSTAT_CCE_MASK    EQU  0x20000
SDHC_IRQSTAT_CCE_SHIFT   EQU  17
SDHC_IRQSTAT_CEBE_MASK   EQU  0x40000
SDHC_IRQSTAT_CEBE_SHIFT  EQU  18
SDHC_IRQSTAT_CIE_MASK    EQU  0x80000
SDHC_IRQSTAT_CIE_SHIFT   EQU  19
SDHC_IRQSTAT_DTOE_MASK   EQU  0x100000
SDHC_IRQSTAT_DTOE_SHIFT  EQU  20
SDHC_IRQSTAT_DCE_MASK    EQU  0x200000
SDHC_IRQSTAT_DCE_SHIFT   EQU  21
SDHC_IRQSTAT_DEBE_MASK   EQU  0x400000
SDHC_IRQSTAT_DEBE_SHIFT  EQU  22
SDHC_IRQSTAT_AC12E_MASK  EQU  0x1000000
SDHC_IRQSTAT_AC12E_SHIFT EQU  24
SDHC_IRQSTAT_DMAE_MASK   EQU  0x10000000
SDHC_IRQSTAT_DMAE_SHIFT  EQU  28

; IRQSTATEN Bit Fields
SDHC_IRQSTATEN_CCSEN_MASK       EQU  0x1
SDHC_IRQSTATEN_CCSEN_SHIFT      EQU  0
SDHC_IRQSTATEN_TCSEN_MASK       EQU  0x2
SDHC_IRQSTATEN_TCSEN_SHIFT      EQU  1
SDHC_IRQSTATEN_BGESEN_MASK      EQU  0x4
SDHC_IRQSTATEN_BGESEN_SHIFT     EQU  2
SDHC_IRQSTATEN_DINTSEN_MASK     EQU  0x8
SDHC_IRQSTATEN_DINTSEN_SHIFT    EQU  3
SDHC_IRQSTATEN_BWRSEN_MASK      EQU  0x10
SDHC_IRQSTATEN_BWRSEN_SHIFT     EQU  4
SDHC_IRQSTATEN_BRRSEN_MASK      EQU  0x20
SDHC_IRQSTATEN_BRRSEN_SHIFT     EQU  5
SDHC_IRQSTATEN_CINSEN_MASK      EQU  0x40
SDHC_IRQSTATEN_CINSEN_SHIFT     EQU  6
SDHC_IRQSTATEN_CRMSEN_MASK      EQU  0x80
SDHC_IRQSTATEN_CRMSEN_SHIFT     EQU  7
SDHC_IRQSTATEN_CINTSEN_MASK     EQU  0x100
SDHC_IRQSTATEN_CINTSEN_SHIFT    EQU  8
SDHC_IRQSTATEN_CTOESEN_MASK     EQU  0x10000
SDHC_IRQSTATEN_CTOESEN_SHIFT    EQU  16
SDHC_IRQSTATEN_CCESEN_MASK      EQU  0x20000
SDHC_IRQSTATEN_CCESEN_SHIFT     EQU  17
SDHC_IRQSTATEN_CEBESEN_MASK     EQU  0x40000
SDHC_IRQSTATEN_CEBESEN_SHIFT    EQU  18
SDHC_IRQSTATEN_CIESEN_MASK      EQU  0x80000
SDHC_IRQSTATEN_CIESEN_SHIFT     EQU  19
SDHC_IRQSTATEN_DTOESEN_MASK     EQU  0x100000
SDHC_IRQSTATEN_DTOESEN_SHIFT    EQU  20
SDHC_IRQSTATEN_DCESEN_MASK      EQU  0x200000
SDHC_IRQSTATEN_DCESEN_SHIFT     EQU  21
SDHC_IRQSTATEN_DEBESEN_MASK     EQU  0x400000
SDHC_IRQSTATEN_DEBESEN_SHIFT    EQU  22
SDHC_IRQSTATEN_AC12ESEN_MASK    EQU  0x1000000
SDHC_IRQSTATEN_AC12ESEN_SHIFT   EQU  24
SDHC_IRQSTATEN_DMAESEN_MASK     EQU  0x10000000
SDHC_IRQSTATEN_DMAESEN_SHIFT    EQU  28

; IRQSIGEN Bit Fields
SDHC_IRQSIGEN_CCIEN_MASK        EQU  0x1
SDHC_IRQSIGEN_CCIEN_SHIFT       EQU  0
SDHC_IRQSIGEN_TCIEN_MASK        EQU  0x2
SDHC_IRQSIGEN_TCIEN_SHIFT       EQU  1
SDHC_IRQSIGEN_BGEIEN_MASK       EQU  0x4
SDHC_IRQSIGEN_BGEIEN_SHIFT      EQU  2
SDHC_IRQSIGEN_DINTIEN_MASK      EQU  0x8
SDHC_IRQSIGEN_DINTIEN_SHIFT     EQU  3
SDHC_IRQSIGEN_BWRIEN_MASK       EQU  0x10
SDHC_IRQSIGEN_BWRIEN_SHIFT      EQU  4
SDHC_IRQSIGEN_BRRIEN_MASK       EQU  0x20
SDHC_IRQSIGEN_BRRIEN_SHIFT      EQU  5
SDHC_IRQSIGEN_CINSIEN_MASK      EQU  0x40
SDHC_IRQSIGEN_CINSIEN_SHIFT     EQU  6
SDHC_IRQSIGEN_CRMIEN_MASK       EQU  0x80
SDHC_IRQSIGEN_CRMIEN_SHIFT      EQU  7
SDHC_IRQSIGEN_CINTIEN_MASK      EQU  0x100
SDHC_IRQSIGEN_CINTIEN_SHIFT     EQU  8
SDHC_IRQSIGEN_CTOEIEN_MASK      EQU  0x10000
SDHC_IRQSIGEN_CTOEIEN_SHIFT     EQU  16
SDHC_IRQSIGEN_CCEIEN_MASK       EQU  0x20000
SDHC_IRQSIGEN_CCEIEN_SHIFT      EQU  17
SDHC_IRQSIGEN_CEBEIEN_MASK      EQU  0x40000
SDHC_IRQSIGEN_CEBEIEN_SHIFT     EQU  18
SDHC_IRQSIGEN_CIEIEN_MASK       EQU  0x80000
SDHC_IRQSIGEN_CIEIEN_SHIFT      EQU  19
SDHC_IRQSIGEN_DTOEIEN_MASK      EQU  0x100000
SDHC_IRQSIGEN_DTOEIEN_SHIFT     EQU  20
SDHC_IRQSIGEN_DCEIEN_MASK       EQU  0x200000
SDHC_IRQSIGEN_DCEIEN_SHIFT      EQU  21
SDHC_IRQSIGEN_DEBEIEN_MASK      EQU  0x400000
SDHC_IRQSIGEN_DEBEIEN_SHIFT     EQU  22
SDHC_IRQSIGEN_AC12EIEN_MASK     EQU  0x1000000
SDHC_IRQSIGEN_AC12EIEN_SHIFT    EQU  24
SDHC_IRQSIGEN_DMAEIEN_MASK      EQU  0x10000000
SDHC_IRQSIGEN_DMAEIEN_SHIFT     EQU  28

; AC12ERR Bit Fields
SDHC_AC12ERR_AC12NE_MASK        EQU  0x1
SDHC_AC12ERR_AC12NE_SHIFT       EQU  0
SDHC_AC12ERR_AC12TOE_MASK       EQU  0x2
SDHC_AC12ERR_AC12TOE_SHIFT      EQU  1
SDHC_AC12ERR_AC12EBE_MASK       EQU  0x4
SDHC_AC12ERR_AC12EBE_SHIFT      EQU  2
SDHC_AC12ERR_AC12CE_MASK        EQU  0x8
SDHC_AC12ERR_AC12CE_SHIFT       EQU  3
SDHC_AC12ERR_AC12IE_MASK        EQU  0x10
SDHC_AC12ERR_AC12IE_SHIFT       EQU  4
SDHC_AC12ERR_CNIBAC12E_MASK     EQU  0x80
SDHC_AC12ERR_CNIBAC12E_SHIFT    EQU  7

; HTCAPBLT Bit Fields
SDHC_HTCAPBLT_MBL_MASK    EQU  0x70000
SDHC_HTCAPBLT_MBL_SHIFT   EQU  16
SDHC_HTCAPBLT_ADMAS_MASK  EQU  0x100000
SDHC_HTCAPBLT_ADMAS_SHIFT EQU  20
SDHC_HTCAPBLT_HSS_MASK    EQU  0x200000
SDHC_HTCAPBLT_HSS_SHIFT   EQU  21
SDHC_HTCAPBLT_DMAS_MASK   EQU  0x400000
SDHC_HTCAPBLT_DMAS_SHIFT  EQU  22
SDHC_HTCAPBLT_SRS_MASK    EQU  0x800000
SDHC_HTCAPBLT_SRS_SHIFT   EQU  23
SDHC_HTCAPBLT_VS33_MASK   EQU  0x1000000
SDHC_HTCAPBLT_VS33_SHIFT  EQU  24

; WML Bit Fields
SDHC_WML_RDWML_MASK       EQU  0xFF
SDHC_WML_RDWML_SHIFT      EQU  0
SDHC_WML_WRWML_MASK       EQU  0xFF0000
SDHC_WML_WRWML_SHIFT      EQU  16

; FEVT Bit Fields
SDHC_FEVT_AC12NE_MASK     EQU  0x1
SDHC_FEVT_AC12NE_SHIFT    EQU  0
SDHC_FEVT_AC12TOE_MASK    EQU  0x2
SDHC_FEVT_AC12TOE_SHIFT   EQU  1
SDHC_FEVT_AC12CE_MASK     EQU  0x4
SDHC_FEVT_AC12CE_SHIFT    EQU  2
SDHC_FEVT_AC12EBE_MASK    EQU  0x8
SDHC_FEVT_AC12EBE_SHIFT   EQU  3
SDHC_FEVT_AC12IE_MASK     EQU  0x10
SDHC_FEVT_AC12IE_SHIFT    EQU  4
SDHC_FEVT_CNIBAC12E_MASK  EQU  0x80
SDHC_FEVT_CNIBAC12E_SHIFT EQU  7
SDHC_FEVT_CTOE_MASK       EQU  0x10000
SDHC_FEVT_CTOE_SHIFT      EQU  16
SDHC_FEVT_CCE_MASK        EQU  0x20000
SDHC_FEVT_CCE_SHIFT       EQU  17
SDHC_FEVT_CEBE_MASK       EQU  0x40000
SDHC_FEVT_CEBE_SHIFT      EQU  18
SDHC_FEVT_CIE_MASK        EQU  0x80000
SDHC_FEVT_CIE_SHIFT       EQU  19
SDHC_FEVT_DTOE_MASK       EQU  0x100000
SDHC_FEVT_DTOE_SHIFT      EQU  20
SDHC_FEVT_DCE_MASK        EQU  0x200000
SDHC_FEVT_DCE_SHIFT       EQU  21
SDHC_FEVT_DEBE_MASK       EQU  0x400000
SDHC_FEVT_DEBE_SHIFT      EQU  22
SDHC_FEVT_AC12E_MASK      EQU  0x1000000
SDHC_FEVT_AC12E_SHIFT     EQU  24
SDHC_FEVT_DMAE_MASK       EQU  0x10000000
SDHC_FEVT_DMAE_SHIFT      EQU  28
SDHC_FEVT_CINT_MASK       EQU  0x80000000
SDHC_FEVT_CINT_SHIFT      EQU  31

; ADMAES Bit Fields
SDHC_ADMAES_ADMAES_MASK        EQU  0x3
SDHC_ADMAES_ADMAES_SHIFT       EQU  0
SDHC_ADMAES_ADMALME_MASK       EQU  0x4
SDHC_ADMAES_ADMALME_SHIFT      EQU  2
SDHC_ADMAES_ADMADCE_MASK       EQU  0x8
SDHC_ADMAES_ADMADCE_SHIFT      EQU  3

; ADSADDR Bit Fields
SDHC_ADSADDR_ADSADDR_MASK EQU  0xFFFFFFFC
SDHC_ADSADDR_ADSADDR_SHIFT      EQU  2

; VENDOR Bit Fields
SDHC_VENDOR_EXTDMAEN_MASK       EQU  0x1
SDHC_VENDOR_EXTDMAEN_SHIFT      EQU  0
SDHC_VENDOR_EXBLKNU_MASK        EQU  0x2
SDHC_VENDOR_EXBLKNU_SHIFT       EQU  1
SDHC_VENDOR_INTSTVAL_MASK       EQU  0xFF0000
SDHC_VENDOR_INTSTVAL_SHIFT      EQU  16

; MMCBOOT Bit Fields
SDHC_MMCBOOT_DTOCVACK_MASK      EQU  0xF
SDHC_MMCBOOT_DTOCVACK_SHIFT     EQU  0
SDHC_MMCBOOT_BOOTACK_MASK       EQU  0x10
SDHC_MMCBOOT_BOOTACK_SHIFT      EQU  4
SDHC_MMCBOOT_BOOTMODE_MASK      EQU  0x20
SDHC_MMCBOOT_BOOTMODE_SHIFT     EQU  5
SDHC_MMCBOOT_BOOTEN_MASK        EQU  0x40
SDHC_MMCBOOT_BOOTEN_SHIFT       EQU  6
SDHC_MMCBOOT_AUTOSABGEN_MASK    EQU  0x80
SDHC_MMCBOOT_AUTOSABGEN_SHIFT   EQU  7
SDHC_MMCBOOT_BOOTBLKCNT_MASK    EQU  0xFFFF0000
SDHC_MMCBOOT_BOOTBLKCNT_SHIFT   EQU  16

; HOSTVER Bit Fields
SDHC_HOSTVER_SVN_MASK    EQU  0xFF
SDHC_HOSTVER_SVN_SHIFT   EQU  0
SDHC_HOSTVER_VVN_MASK    EQU  0xFF00
SDHC_HOSTVER_VVN_SHIFT   EQU  8

; ----------------------------------------------------------------------------
; -- SIM Peripheral Access Layer
; ----------------------------------------------------------------------------

SIM_SOPT1    EQU  0x0000   ; System Options Register 1, offset: 0x0
SIM_SOPT1CFG EQU  0x0004   ; SOPT1 Configuration Register, offset: 0x4
SIM_SOPT2    EQU  0x1004   ; System Options Register 2, offset: 0x1004
SIM_SOPT4    EQU  0x100C   ; System Options Register 4, offset: 0x100C
SIM_SOPT5    EQU  0x1010   ; System Options Register 5, offset: 0x1010
SIM_SOPT7    EQU  0x1018   ; System Options Register 7, offset: 0x1018
SIM_SDID     EQU  0x1024   ; System Device Identification Register, offset: 0x1024
SIM_SCGC1    EQU  0x1028   ; System Clock Gating Control Register 1, offset: 0x1028
SIM_SCGC2    EQU  0x102C   ; System Clock Gating Control Register 2, offset: 0x102C
SIM_SCGC3    EQU  0x1030   ; System Clock Gating Control Register 3, offset: 0x1030
SIM_SCGC4    EQU  0x1034   ; System Clock Gating Control Register 4, offset: 0x1034
SIM_SCGC5    EQU  0x1038   ; System Clock Gating Control Register 5, offset: 0x1038
SIM_SCGC6    EQU  0x103C   ; System Clock Gating Control Register 6, offset: 0x103C
SIM_SCGC7    EQU  0x1040   ; System Clock Gating Control Register 7, offset: 0x1040
SIM_CLKDIV1  EQU  0x1044   ; System Clock Divider Register 1, offset: 0x1044
SIM_CLKDIV2  EQU  0x1048   ; System Clock Divider Register 2, offset: 0x1048
SIM_FCFG1    EQU  0x104C   ; Flash Configuration Register 1, offset: 0x104C
SIM_FCFG2    EQU  0x1050   ; Flash Configuration Register 2, offset: 0x1050
SIM_UIDH     EQU  0x1054   ; Unique Identification Register High, offset: 0x1054
SIM_UIDMH    EQU  0x1058   ; Unique Identification Register Mid-High, offset: 0x1058
SIM_UIDML    EQU  0x105C   ; Unique Identification Register Mid Low, offset: 0x105C
SIM_UIDL     EQU  0x1060   ; Unique Identification Register Low, offset: 0x1060

; ----------------------------------------------------------------------------
; -- SIM Register Masks
; ----------------------------------------------------------------------------

; SOPT1 Bit Fields
SIM_SOPT1_RAMSIZE_MASK    EQU  0xF000
SIM_SOPT1_RAMSIZE_SHIFT   EQU  12
SIM_SOPT1_OSC32KSEL_MASK  EQU  0xC0000
SIM_SOPT1_OSC32KSEL_SHIFT EQU  18
SIM_SOPT1_USBVSTBY_MASK   EQU  0x20000000
SIM_SOPT1_USBVSTBY_SHIFT  EQU  29
SIM_SOPT1_USBSSTBY_MASK   EQU  0x40000000
SIM_SOPT1_USBSSTBY_SHIFT  EQU  30
SIM_SOPT1_USBREGEN_MASK   EQU  0x80000000
SIM_SOPT1_USBREGEN_SHIFT  EQU  31

; SOPT1CFG Bit Fields
SIM_SOPT1CFG_URWE_MASK    EQU  0x1000000
SIM_SOPT1CFG_URWE_SHIFT   EQU  24
SIM_SOPT1CFG_UVSWE_MASK   EQU  0x2000000
SIM_SOPT1CFG_UVSWE_SHIFT  EQU  25
SIM_SOPT1CFG_USSWE_MASK   EQU  0x4000000
SIM_SOPT1CFG_USSWE_SHIFT  EQU  26

; SOPT2 Bit Fields
SIM_SOPT2_RTCCLKOUTSEL_MASK  EQU  0x10
SIM_SOPT2_RTCCLKOUTSEL_SHIFT EQU  4
SIM_SOPT2_CLKOUTSEL_MASK     EQU  0xE0
SIM_SOPT2_CLKOUTSEL_SHIFT    EQU  5
SIM_SOPT2_FBSL_MASK          EQU  0x300
SIM_SOPT2_FBSL_SHIFT         EQU  8
SIM_SOPT2_PTD7PAD_MASK       EQU  0x800
SIM_SOPT2_PTD7PAD_SHIFT      EQU  11
SIM_SOPT2_TRACECLKSEL_MASK   EQU  0x1000
SIM_SOPT2_TRACECLKSEL_SHIFT  EQU  12
SIM_SOPT2_PLLFLLSEL_MASK     EQU  0x30000
SIM_SOPT2_PLLFLLSEL_SHIFT    EQU  16
SIM_SOPT2_USBSRC_MASK        EQU  0x40000
SIM_SOPT2_USBSRC_SHIFT       EQU  18
SIM_SOPT2_RMIISRC_MASK       EQU  0x80000
SIM_SOPT2_RMIISRC_SHIFT      EQU  19
SIM_SOPT2_TIMESRC_MASK       EQU  0x300000
SIM_SOPT2_TIMESRC_SHIFT      EQU  20
SIM_SOPT2_SDHCSRC_MASK       EQU  0x30000000
SIM_SOPT2_SDHCSRC_SHIFT      EQU  28

; SOPT4 Bit Fields
SIM_SOPT4_FTM0FLT0_MASK      EQU  0x1
SIM_SOPT4_FTM0FLT0_SHIFT     EQU  0
SIM_SOPT4_FTM0FLT1_MASK      EQU  0x2
SIM_SOPT4_FTM0FLT1_SHIFT     EQU  1
SIM_SOPT4_FTM0FLT2_MASK      EQU  0x4
SIM_SOPT4_FTM0FLT2_SHIFT     EQU  2
SIM_SOPT4_FTM1FLT0_MASK      EQU  0x10
SIM_SOPT4_FTM1FLT0_SHIFT     EQU  4
SIM_SOPT4_FTM2FLT0_MASK      EQU  0x100
SIM_SOPT4_FTM2FLT0_SHIFT     EQU  8
SIM_SOPT4_FTM3FLT0_MASK      EQU  0x1000
SIM_SOPT4_FTM3FLT0_SHIFT     EQU  12
SIM_SOPT4_FTM1CH0SRC_MASK    EQU  0xC0000
SIM_SOPT4_FTM1CH0SRC_SHIFT   EQU  18
SIM_SOPT4_FTM2CH0SRC_MASK    EQU  0x300000
SIM_SOPT4_FTM2CH0SRC_SHIFT   EQU  20
SIM_SOPT4_FTM0CLKSEL_MASK    EQU  0x1000000
SIM_SOPT4_FTM0CLKSEL_SHIFT   EQU  24
SIM_SOPT4_FTM1CLKSEL_MASK    EQU  0x2000000
SIM_SOPT4_FTM1CLKSEL_SHIFT   EQU  25
SIM_SOPT4_FTM2CLKSEL_MASK    EQU  0x4000000
SIM_SOPT4_FTM2CLKSEL_SHIFT   EQU  26
SIM_SOPT4_FTM3CLKSEL_MASK    EQU  0x8000000
SIM_SOPT4_FTM3CLKSEL_SHIFT   EQU  27
SIM_SOPT4_FTM0TRG0SRC_MASK   EQU  0x10000000
SIM_SOPT4_FTM0TRG0SRC_SHIFT  EQU  28
SIM_SOPT4_FTM0TRG1SRC_MASK   EQU  0x20000000
SIM_SOPT4_FTM0TRG1SRC_SHIFT  EQU  29
SIM_SOPT4_FTM3TRG0SRC_MASK   EQU  0x40000000
SIM_SOPT4_FTM3TRG0SRC_SHIFT  EQU  30
SIM_SOPT4_FTM3TRG1SRC_MASK   EQU  0x80000000
SIM_SOPT4_FTM3TRG1SRC_SHIFT  EQU  31

; SOPT5 Bit Fields
SIM_SOPT5_UART0TXSRC_MASK    EQU  0x3
SIM_SOPT5_UART0TXSRC_SHIFT   EQU  0
SIM_SOPT5_UART0RXSRC_MASK    EQU  0xC
SIM_SOPT5_UART0RXSRC_SHIFT   EQU  2
SIM_SOPT5_UART1TXSRC_MASK    EQU  0x30
SIM_SOPT5_UART1TXSRC_SHIFT   EQU  4
SIM_SOPT5_UART1RXSRC_MASK    EQU  0xC0
SIM_SOPT5_UART1RXSRC_SHIFT   EQU  6

; SOPT7 Bit Fields
SIM_SOPT7_ADC0TRGSEL_MASK       EQU  0xF
SIM_SOPT7_ADC0TRGSEL_SHIFT      EQU  0
SIM_SOPT7_ADC0PRETRGSEL_MASK    EQU  0x10
SIM_SOPT7_ADC0PRETRGSEL_SHIFT   EQU  4
SIM_SOPT7_ADC0ALTTRGEN_MASK     EQU  0x80
SIM_SOPT7_ADC0ALTTRGEN_SHIFT    EQU  7
SIM_SOPT7_ADC1TRGSEL_MASK       EQU  0xF00
SIM_SOPT7_ADC1TRGSEL_SHIFT      EQU  8
SIM_SOPT7_ADC1PRETRGSEL_MASK    EQU  0x1000
SIM_SOPT7_ADC1PRETRGSEL_SHIFT   EQU  12
SIM_SOPT7_ADC1ALTTRGEN_MASK     EQU  0x8000
SIM_SOPT7_ADC1ALTTRGEN_SHIFT    EQU  15

; SDID Bit Fields
SIM_SDID_PINID_MASK      EQU  0xF
SIM_SDID_PINID_SHIFT     EQU  0
SIM_SDID_FAMID_MASK      EQU  0x70
SIM_SDID_FAMID_SHIFT     EQU  4
SIM_SDID_DIEID_MASK      EQU  0xF80
SIM_SDID_DIEID_SHIFT     EQU  7
SIM_SDID_REVID_MASK      EQU  0xF000
SIM_SDID_REVID_SHIFT     EQU  12
SIM_SDID_SERIESID_MASK   EQU  0xF00000
SIM_SDID_SERIESID_SHIFT  EQU  20
SIM_SDID_SUBFAMID_MASK   EQU  0xF000000
SIM_SDID_SUBFAMID_SHIFT  EQU  24
SIM_SDID_FAMILYID_MASK   EQU  0xF0000000
SIM_SDID_FAMILYID_SHIFT  EQU  28

; SCGC1 Bit Fields
SIM_SCGC1_I2C2_MASK      EQU  0x40
SIM_SCGC1_I2C2_SHIFT     EQU  6
SIM_SCGC1_UART4_MASK     EQU  0x400
SIM_SCGC1_UART4_SHIFT    EQU  10
SIM_SCGC1_UART5_MASK     EQU  0x800
SIM_SCGC1_UART5_SHIFT    EQU  11

; SCGC2 Bit Fields
SIM_SCGC2_ENET_MASK      EQU  0x1
SIM_SCGC2_ENET_SHIFT     EQU  0
SIM_SCGC2_DAC0_MASK      EQU  0x1000
SIM_SCGC2_DAC0_SHIFT     EQU  12
SIM_SCGC2_DAC1_MASK      EQU  0x2000
SIM_SCGC2_DAC1_SHIFT     EQU  13

; SCGC3 Bit Fields
SIM_SCGC3_RNGA_MASK      EQU  0x1
SIM_SCGC3_RNGA_SHIFT     EQU  0
SIM_SCGC3_SPI2_MASK      EQU  0x1000
SIM_SCGC3_SPI2_SHIFT     EQU  12
SIM_SCGC3_SDHC_MASK      EQU  0x20000
SIM_SCGC3_SDHC_SHIFT     EQU  17
SIM_SCGC3_FTM2_MASK      EQU  0x1000000
SIM_SCGC3_FTM2_SHIFT     EQU  24
SIM_SCGC3_FTM3_MASK      EQU  0x2000000
SIM_SCGC3_FTM3_SHIFT     EQU  25
SIM_SCGC3_ADC1_MASK      EQU  0x8000000
SIM_SCGC3_ADC1_SHIFT     EQU  27

; SCGC4 Bit Fields
SIM_SCGC4_EWM_MASK       EQU  0x2
SIM_SCGC4_EWM_SHIFT      EQU  1
SIM_SCGC4_CMT_MASK       EQU  0x4
SIM_SCGC4_CMT_SHIFT      EQU  2
SIM_SCGC4_I2C0_MASK      EQU  0x40
SIM_SCGC4_I2C0_SHIFT     EQU  6
SIM_SCGC4_I2C1_MASK      EQU  0x80
SIM_SCGC4_I2C1_SHIFT     EQU  7
SIM_SCGC4_UART0_MASK     EQU  0x400
SIM_SCGC4_UART0_SHIFT    EQU  10
SIM_SCGC4_UART1_MASK     EQU  0x800
SIM_SCGC4_UART1_SHIFT    EQU  11
SIM_SCGC4_UART2_MASK     EQU  0x1000
SIM_SCGC4_UART2_SHIFT    EQU  12
SIM_SCGC4_UART3_MASK     EQU  0x2000
SIM_SCGC4_UART3_SHIFT    EQU  13
SIM_SCGC4_USBOTG_MASK    EQU  0x40000
SIM_SCGC4_USBOTG_SHIFT   EQU  18
SIM_SCGC4_CMP_MASK       EQU  0x80000
SIM_SCGC4_CMP_SHIFT      EQU  19
SIM_SCGC4_VREF_MASK      EQU  0x100000
SIM_SCGC4_VREF_SHIFT     EQU  20

; SCGC5 Bit Fields
SIM_SCGC5_LPTMR_MASK     EQU  0x1
SIM_SCGC5_LPTMR_SHIFT    EQU  0
SIM_SCGC5_PORTA_MASK     EQU  0x200
SIM_SCGC5_PORTA_SHIFT    EQU  9
SIM_SCGC5_PORTB_MASK     EQU  0x400
SIM_SCGC5_PORTB_SHIFT    EQU  10
SIM_SCGC5_PORTC_MASK     EQU  0x800
SIM_SCGC5_PORTC_SHIFT    EQU  11
SIM_SCGC5_PORTD_MASK     EQU  0x1000
SIM_SCGC5_PORTD_SHIFT    EQU  12
SIM_SCGC5_PORTE_MASK     EQU  0x2000
SIM_SCGC5_PORTE_SHIFT    EQU  13

; SCGC6 Bit Fields
SIM_SCGC6_FTF_MASK       EQU  0x1
SIM_SCGC6_FTF_SHIFT      EQU  0
SIM_SCGC6_DMAMUX_MASK    EQU  0x2
SIM_SCGC6_DMAMUX_SHIFT   EQU  1
SIM_SCGC6_FLEXCAN0_MASK  EQU  0x10
SIM_SCGC6_FLEXCAN0_SHIFT EQU  4
SIM_SCGC6_RNGA_MASK      EQU  0x200
SIM_SCGC6_RNGA_SHIFT     EQU  9
SIM_SCGC6_SPI0_MASK      EQU  0x1000
SIM_SCGC6_SPI0_SHIFT     EQU  12
SIM_SCGC6_SPI1_MASK      EQU  0x2000
SIM_SCGC6_SPI1_SHIFT     EQU  13
SIM_SCGC6_I2S_MASK       EQU  0x8000
SIM_SCGC6_I2S_SHIFT      EQU  15
SIM_SCGC6_CRC_MASK       EQU  0x40000
SIM_SCGC6_CRC_SHIFT      EQU  18
SIM_SCGC6_USBDCD_MASK    EQU  0x200000
SIM_SCGC6_USBDCD_SHIFT   EQU  21
SIM_SCGC6_PDB_MASK       EQU  0x400000
SIM_SCGC6_PDB_SHIFT      EQU  22
SIM_SCGC6_PIT_MASK       EQU  0x800000
SIM_SCGC6_PIT_SHIFT      EQU  23
SIM_SCGC6_FTM0_MASK      EQU  0x1000000
SIM_SCGC6_FTM0_SHIFT     EQU  24
SIM_SCGC6_FTM1_MASK      EQU  0x2000000
SIM_SCGC6_FTM1_SHIFT     EQU  25
SIM_SCGC6_FTM2_MASK      EQU  0x4000000
SIM_SCGC6_FTM2_SHIFT     EQU  26
SIM_SCGC6_ADC0_MASK      EQU  0x8000000
SIM_SCGC6_ADC0_SHIFT     EQU  27
SIM_SCGC6_RTC_MASK       EQU  0x20000000
SIM_SCGC6_RTC_SHIFT      EQU  29
SIM_SCGC6_DAC0_MASK      EQU  0x80000000
SIM_SCGC6_DAC0_SHIFT     EQU  31

; SCGC7 Bit Fields
SIM_SCGC7_FLEXBUS_MASK   EQU  0x1
SIM_SCGC7_FLEXBUS_SHIFT  EQU  0
SIM_SCGC7_DMA_MASK       EQU  0x2
SIM_SCGC7_DMA_SHIFT      EQU  1
SIM_SCGC7_MPU_MASK       EQU  0x4
SIM_SCGC7_MPU_SHIFT      EQU  2

; CLKDIV1 Bit Fields
SIM_CLKDIV1_OUTDIV4_MASK    EQU  0xF0000
SIM_CLKDIV1_OUTDIV4_SHIFT   EQU  16
SIM_CLKDIV1_OUTDIV3_MASK    EQU  0xF00000
SIM_CLKDIV1_OUTDIV3_SHIFT   EQU  20
SIM_CLKDIV1_OUTDIV2_MASK    EQU  0xF000000
SIM_CLKDIV1_OUTDIV2_SHIFT   EQU  24
SIM_CLKDIV1_OUTDIV1_MASK    EQU  0xF0000000
SIM_CLKDIV1_OUTDIV1_SHIFT   EQU  28

; CLKDIV2 Bit Fields
SIM_CLKDIV2_USBFRAC_MASK   EQU  0x1
SIM_CLKDIV2_USBFRAC_SHIFT  EQU  0
SIM_CLKDIV2_USBDIV_MASK    EQU  0xE
SIM_CLKDIV2_USBDIV_SHIFT   EQU  1

; FCFG1 Bit Fields
SIM_FCFG1_FLASHDIS_MASK    EQU  0x1
SIM_FCFG1_FLASHDIS_SHIFT   EQU  0
SIM_FCFG1_FLASHDOZE_MASK   EQU  0x2
SIM_FCFG1_FLASHDOZE_SHIFT  EQU  1
SIM_FCFG1_DEPART_MASK      EQU  0xF00
SIM_FCFG1_DEPART_SHIFT     EQU  8
SIM_FCFG1_EESIZE_MASK      EQU  0xF0000
SIM_FCFG1_EESIZE_SHIFT     EQU  16
SIM_FCFG1_PFSIZE_MASK      EQU  0xF000000
SIM_FCFG1_PFSIZE_SHIFT     EQU  24
SIM_FCFG1_NVMSIZE_MASK     EQU  0xF0000000
SIM_FCFG1_NVMSIZE_SHIFT    EQU  28

; FCFG2 Bit Fields
SIM_FCFG2_MAXADDR1_MASK    EQU  0x7F0000
SIM_FCFG2_MAXADDR1_SHIFT   EQU  16
SIM_FCFG2_PFLSH_MASK       EQU  0x800000
SIM_FCFG2_PFLSH_SHIFT      EQU  23
SIM_FCFG2_MAXADDR0_MASK    EQU  0x7F000000
SIM_FCFG2_MAXADDR0_SHIFT   EQU  24

; UIDH Bit Fields
SIM_UIDH_UID_MASK          EQU  0xFFFFFFFF
SIM_UIDH_UID_SHIFT         EQU  0

; UIDMH Bit Fields
SIM_UIDMH_UID_MASK         EQU  0xFFFFFFFF
SIM_UIDMH_UID_SHIFT        EQU  0

; UIDML Bit Fields
SIM_UIDML_UID_MASK         EQU  0xFFFFFFFF
SIM_UIDML_UID_SHIFT        EQU  0

; UIDL Bit Fields
SIM_UIDL_UID_MASK          EQU  0xFFFFFFFF
SIM_UIDL_UID_SHIFT         EQU  0

; ----------------------------------------------------------------------------
; -- SMC Peripheral Access Layer
; ----------------------------------------------------------------------------

SMC_PMPROT   EQU  0x0   ; Power Mode Protection register, offset: 0x0
SMC_PMCTRL   EQU  0x1   ; Power Mode Control register, offset: 0x1
SMC_VLLSCTRL EQU  0x2   ; VLLS Control register, offset: 0x2
SMC_PMSTAT   EQU  0x3   ; Power Mode Status register, offset: 0x3

; ----------------------------------------------------------------------------
; -- SMC Register Masks
; ----------------------------------------------------------------------------

; PMPROT Bit Fields
SMC_PMPROT_AVLLS_MASK    EQU  0x2
SMC_PMPROT_AVLLS_SHIFT   EQU  1
SMC_PMPROT_ALLS_MASK     EQU  0x8
SMC_PMPROT_ALLS_SHIFT    EQU  3
SMC_PMPROT_AVLP_MASK     EQU  0x20
SMC_PMPROT_AVLP_SHIFT    EQU  5
; PMCTRL Bit Fields
SMC_PMCTRL_STOPM_MASK    EQU  0x7
SMC_PMCTRL_STOPM_SHIFT   EQU  0
SMC_PMCTRL_STOPA_MASK    EQU  0x8
SMC_PMCTRL_STOPA_SHIFT   EQU  3
SMC_PMCTRL_RUNM_MASK     EQU  0x60
SMC_PMCTRL_RUNM_SHIFT    EQU  5
SMC_PMCTRL_LPWUI_MASK    EQU  0x80
SMC_PMCTRL_LPWUI_SHIFT   EQU  7
; VLLSCTRL Bit Fields
SMC_VLLSCTRL_VLLSM_MASK  EQU  0x7
SMC_VLLSCTRL_VLLSM_SHIFT EQU  0
SMC_VLLSCTRL_PORPO_MASK  EQU  0x20
SMC_VLLSCTRL_PORPO_SHIFT EQU  5
; PMSTAT Bit Fields
SMC_PMSTAT_PMSTAT_MASK   EQU  0x7F
SMC_PMSTAT_PMSTAT_SHIFT  EQU  0

; ----------------------------------------------------------------------------
; -- SPI Peripheral Access Layer
; ----------------------------------------------------------------------------

SPI_MCR         EQU  0x00   ; Module Configuration Register, offset: 0x0
SPI_TCR         EQU  0x08   ; Transfer Count Register, offset: 0x8
SPI_CTAR0       EQU  0x0C   ; Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4
SPI_CTAR1       EQU  0x10   ; Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4
SPI_CTAR_SLAVE  EQU  0x0C   ; Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4
SPI_SR          EQU  0x2C   ; Status Register, offset: 0x2C
SPI_RSER        EQU  0x30   ; DMA/Interrupt Request Select and Enable Register, offset: 0x30
SPI_PUSHR       EQU  0x34   ; PUSH TX FIFO Register In Master Mode, offset: 0x34
SPI_PUSHR_SLAVE EQU  0x34   ; PUSH TX FIFO Register In Slave Mode, offset: 0x34

SPI_POPR     EQU  0x38   ; POP RX FIFO Register, offset: 0x38
SPI_TXFR0    EQU  0x3C   ; Transmit FIFO Registers, offset: 0x3C
SPI_TXFR1    EQU  0x40   ; Transmit FIFO Registers, offset: 0x40
SPI_TXFR2    EQU  0x44   ; Transmit FIFO Registers, offset: 0x44
SPI_TXFR3    EQU  0x48   ; Transmit FIFO Registers, offset: 0x48
SPI_RXFR0    EQU  0x7C   ; Receive FIFO Registers, offset: 0x7C
SPI_RXFR1    EQU  0x80   ; Receive FIFO Registers, offset: 0x80
SPI_RXFR2    EQU  0x84   ; Receive FIFO Registers, offset: 0x84
SPI_RXFR3    EQU  0x88   ; Receive FIFO Registers, offset: 0x88

; ----------------------------------------------------------------------------
; -- SPI Register Masks
; ----------------------------------------------------------------------------

; MCR Bit Fields
SPI_MCR_HALT_MASK        EQU  0x1
SPI_MCR_HALT_SHIFT       EQU  0
SPI_MCR_SMPL_PT_MASK     EQU  0x300
SPI_MCR_SMPL_PT_SHIFT    EQU  8
SPI_MCR_CLR_RXF_MASK     EQU  0x400
SPI_MCR_CLR_RXF_SHIFT    EQU  10
SPI_MCR_CLR_TXF_MASK     EQU  0x800
SPI_MCR_CLR_TXF_SHIFT    EQU  11
SPI_MCR_DIS_RXF_MASK     EQU  0x1000
SPI_MCR_DIS_RXF_SHIFT    EQU  12
SPI_MCR_DIS_TXF_MASK     EQU  0x2000
SPI_MCR_DIS_TXF_SHIFT    EQU  13
SPI_MCR_MDIS_MASK        EQU  0x4000
SPI_MCR_MDIS_SHIFT       EQU  14
SPI_MCR_DOZE_MASK        EQU  0x8000
SPI_MCR_DOZE_SHIFT       EQU  15
SPI_MCR_PCSIS_MASK       EQU  0x3F0000
SPI_MCR_PCSIS_SHIFT      EQU  16
SPI_MCR_ROOE_MASK        EQU  0x1000000
SPI_MCR_ROOE_SHIFT       EQU  24
SPI_MCR_PCSSE_MASK       EQU  0x2000000
SPI_MCR_PCSSE_SHIFT      EQU  25
SPI_MCR_MTFE_MASK        EQU  0x4000000
SPI_MCR_MTFE_SHIFT       EQU  26
SPI_MCR_FRZ_MASK         EQU  0x8000000
SPI_MCR_FRZ_SHIFT        EQU  27
SPI_MCR_DCONF_MASK       EQU  0x30000000
SPI_MCR_DCONF_SHIFT      EQU  28
SPI_MCR_CONT_SCKE_MASK   EQU  0x40000000
SPI_MCR_CONT_SCKE_SHIFT  EQU  30
SPI_MCR_MSTR_MASK        EQU  0x80000000
SPI_MCR_MSTR_SHIFT       EQU  31

; TCR Bit Fields
SPI_TCR_SPI_TCNT_MASK    EQU  0xFFFF0000
SPI_TCR_SPI_TCNT_SHIFT   EQU  16

; CTAR Bit Fields
SPI_CTAR_BR_MASK         EQU  0xF
SPI_CTAR_BR_SHIFT        EQU  0
SPI_CTAR_DT_MASK         EQU  0xF0
SPI_CTAR_DT_SHIFT        EQU  4
SPI_CTAR_ASC_MASK        EQU  0xF00
SPI_CTAR_ASC_SHIFT       EQU  8
SPI_CTAR_CSSCK_MASK      EQU  0xF000
SPI_CTAR_CSSCK_SHIFT     EQU  12
SPI_CTAR_PBR_MASK        EQU  0x30000
SPI_CTAR_PBR_SHIFT       EQU  16
SPI_CTAR_PDT_MASK        EQU  0xC0000
SPI_CTAR_PDT_SHIFT       EQU  18
SPI_CTAR_PASC_MASK       EQU  0x300000
SPI_CTAR_PASC_SHIFT      EQU  20
SPI_CTAR_PCSSCK_MASK     EQU  0xC00000
SPI_CTAR_PCSSCK_SHIFT    EQU  22
SPI_CTAR_LSBFE_MASK      EQU  0x1000000
SPI_CTAR_LSBFE_SHIFT     EQU  24
SPI_CTAR_CPHA_MASK       EQU  0x2000000
SPI_CTAR_CPHA_SHIFT      EQU  25
SPI_CTAR_CPOL_MASK       EQU  0x4000000
SPI_CTAR_CPOL_SHIFT      EQU  26
SPI_CTAR_FMSZ_MASK       EQU  0x78000000
SPI_CTAR_FMSZ_SHIFT      EQU  27
SPI_CTAR_DBR_MASK        EQU  0x80000000
SPI_CTAR_DBR_SHIFT       EQU  31

; CTAR_SLAVE Bit Fields
SPI_CTAR_SLAVE_CPHA_MASK      EQU  0x2000000
SPI_CTAR_SLAVE_CPHA_SHIFT     EQU  25
SPI_CTAR_SLAVE_CPOL_MASK      EQU  0x4000000
SPI_CTAR_SLAVE_CPOL_SHIFT     EQU  26
SPI_CTAR_SLAVE_FMSZ_MASK      EQU  0xF8000000
SPI_CTAR_SLAVE_FMSZ_SHIFT     EQU  27

; SR Bit Fields
SPI_SR_POPNXTPTR_MASK    EQU  0xF
SPI_SR_POPNXTPTR_SHIFT   EQU  0
SPI_SR_RXCTR_MASK        EQU  0xF0
SPI_SR_RXCTR_SHIFT       EQU  4
SPI_SR_TXNXTPTR_MASK     EQU  0xF00
SPI_SR_TXNXTPTR_SHIFT    EQU  8
SPI_SR_TXCTR_MASK        EQU  0xF000
SPI_SR_TXCTR_SHIFT       EQU  12
SPI_SR_RFDF_MASK         EQU  0x20000
SPI_SR_RFDF_SHIFT        EQU  17
SPI_SR_RFOF_MASK         EQU  0x80000
SPI_SR_RFOF_SHIFT        EQU  19
SPI_SR_TFFF_MASK         EQU  0x2000000
SPI_SR_TFFF_SHIFT        EQU  25
SPI_SR_TFUF_MASK         EQU  0x8000000
SPI_SR_TFUF_SHIFT        EQU  27
SPI_SR_EOQF_MASK         EQU  0x10000000
SPI_SR_EOQF_SHIFT        EQU  28
SPI_SR_TXRXS_MASK        EQU  0x40000000
SPI_SR_TXRXS_SHIFT       EQU  30
SPI_SR_TCF_MASK          EQU  0x80000000
SPI_SR_TCF_SHIFT         EQU  31

; RSER Bit Fields
SPI_RSER_RFDF_DIRS_MASK  EQU  0x10000
SPI_RSER_RFDF_DIRS_SHIFT EQU  16
SPI_RSER_RFDF_RE_MASK    EQU  0x20000
SPI_RSER_RFDF_RE_SHIFT   EQU  17
SPI_RSER_RFOF_RE_MASK    EQU  0x80000
SPI_RSER_RFOF_RE_SHIFT   EQU  19
SPI_RSER_TFFF_DIRS_MASK  EQU  0x1000000
SPI_RSER_TFFF_DIRS_SHIFT EQU  24
SPI_RSER_TFFF_RE_MASK    EQU  0x2000000
SPI_RSER_TFFF_RE_SHIFT   EQU  25
SPI_RSER_TFUF_RE_MASK    EQU  0x8000000
SPI_RSER_TFUF_RE_SHIFT   EQU  27
SPI_RSER_EOQF_RE_MASK    EQU  0x10000000
SPI_RSER_EOQF_RE_SHIFT   EQU  28
SPI_RSER_TCF_RE_MASK     EQU  0x80000000
SPI_RSER_TCF_RE_SHIFT    EQU  31

; PUSHR Bit Fields
SPI_PUSHR_TXDATA_MASK    EQU  0xFFFF
SPI_PUSHR_TXDATA_SHIFT   EQU  0
SPI_PUSHR_PCS_MASK       EQU  0x3F0000
SPI_PUSHR_PCS_SHIFT      EQU  16
SPI_PUSHR_CTCNT_MASK     EQU  0x4000000
SPI_PUSHR_CTCNT_SHIFT    EQU  26
SPI_PUSHR_EOQ_MASK       EQU  0x8000000
SPI_PUSHR_EOQ_SHIFT      EQU  27
SPI_PUSHR_CTAS_MASK      EQU  0x70000000
SPI_PUSHR_CTAS_SHIFT     EQU  28
SPI_PUSHR_CONT_MASK      EQU  0x80000000
SPI_PUSHR_CONT_SHIFT     EQU  31

; PUSHR_SLAVE Bit Fields
SPI_PUSHR_SLAVE_TXDATA_MASK   EQU  0xFFFFFFFF
SPI_PUSHR_SLAVE_TXDATA_SHIFT  EQU  0

; POPR Bit Fields
SPI_POPR_RXDATA_MASK          EQU  0xFFFFFFFF
SPI_POPR_RXDATA_SHIFT         EQU  0

; TXFR0 Bit Fields
SPI_TXFR0_TXDATA_MASK         EQU  0xFFFF
SPI_TXFR0_TXDATA_SHIFT        EQU  0
SPI_TXFR0_TXCMD_TXDATA_MASK   EQU  0xFFFF0000
SPI_TXFR0_TXCMD_TXDATA_SHIFT  EQU  16

; TXFR1 Bit Fields
SPI_TXFR1_TXDATA_MASK         EQU  0xFFFF
SPI_TXFR1_TXDATA_SHIFT        EQU  0
SPI_TXFR1_TXCMD_TXDATA_MASK   EQU  0xFFFF0000
SPI_TXFR1_TXCMD_TXDATA_SHIFT  EQU  16

; TXFR2 Bit Fields
SPI_TXFR2_TXDATA_MASK         EQU  0xFFFF
SPI_TXFR2_TXDATA_SHIFT        EQU  0
SPI_TXFR2_TXCMD_TXDATA_MASK   EQU  0xFFFF0000
SPI_TXFR2_TXCMD_TXDATA_SHIFT  EQU  16

; TXFR3 Bit Fields
SPI_TXFR3_TXDATA_MASK         EQU  0xFFFF
SPI_TXFR3_TXDATA_SHIFT        EQU  0
SPI_TXFR3_TXCMD_TXDATA_MASK   EQU  0xFFFF0000
SPI_TXFR3_TXCMD_TXDATA_SHIFT  EQU  16

; RXFR0 Bit Fields
SPI_RXFR0_RXDATA_MASK         EQU  0xFFFFFFFF
SPI_RXFR0_RXDATA_SHIFT        EQU  0

; RXFR1 Bit Fields
SPI_RXFR1_RXDATA_MASK         EQU  0xFFFFFFFF
SPI_RXFR1_RXDATA_SHIFT        EQU  0

; RXFR2 Bit Fields
SPI_RXFR2_RXDATA_MASK         EQU  0xFFFFFFFF
SPI_RXFR2_RXDATA_SHIFT        EQU  0

; RXFR3 Bit Fields
SPI_RXFR3_RXDATA_MASK         EQU  0xFFFFFFFF
SPI_RXFR3_RXDATA_SHIFT        EQU  0

; ----------------------------------------------------------------------------
; -- UART Peripheral Access Layer
; ----------------------------------------------------------------------------

UART_BDH      EQU  0x00   ; UART Baud Rate Registers: High, offset: 0x0
UART_BDL      EQU  0x01   ; UART Baud Rate Registers: Low, offset: 0x1
UART_C1       EQU  0x02   ; UART Control Register 1, offset: 0x2
UART_C2       EQU  0x03   ; UART Control Register 2, offset: 0x3
UART_S1       EQU  0x04   ; UART Status Register 1, offset: 0x4
UART_S2       EQU  0x05   ; UART Status Register 2, offset: 0x5
UART_C3       EQU  0x06   ; UART Control Register 3, offset: 0x6
UART_D        EQU  0x07   ; UART Data Register, offset: 0x7
UART_MA1      EQU  0x08   ; UART Match Address Registers 1, offset: 0x8
UART_MA2      EQU  0x09   ; UART Match Address Registers 2, offset: 0x9
UART_C4       EQU  0x0A   ; UART Control Register 4, offset: 0xA
UART_C5       EQU  0x0B   ; UART Control Register 5, offset: 0xB
UART_ED       EQU  0x0C   ; UART Extended Data Register, offset: 0xC
UART_MODEM    EQU  0x0D   ; UART Modem Register, offset: 0xD
UART_IR       EQU  0x0E   ; UART Infrared Register, offset: 0xE
UART_PFIFO    EQU  0x10   ; UART FIFO Parameters, offset: 0x10
UART_CFIFO    EQU  0x11   ; UART FIFO Control Register, offset: 0x11
UART_SFIFO    EQU  0x12   ; UART FIFO Status Register, offset: 0x12
UART_TWFIFO   EQU  0x13   ; UART FIFO Transmit Watermark, offset: 0x13
UART_TCFIFO   EQU  0x14   ; UART FIFO Transmit Count, offset: 0x14
UART_RWFIFO   EQU  0x15   ; UART FIFO Receive Watermark, offset: 0x15
UART_RCFIFO   EQU  0x16   ; UART FIFO Receive Count, offset: 0x16
UART_C7816    EQU  0x18   ; UART 7816 Control Register, offset: 0x18
UART_IE7816   EQU  0x19   ; UART 7816 Interrupt Enable Register, offset: 0x19
UART_IS7816   EQU  0x1A   ; UART 7816 Interrupt Status Register, offset: 0x1A

UART_WP7816_T_TYPE0    EQU  0x1B   ; UART 7816 Wait Parameter Register, offset: 0x1B
UART_WP7816_T_TYPE1    EQU  0x1C   ; UART 7816 Wait Parameter Register, offset: 0x1B

UART_WN7816   EQU  0x1C   ; UART 7816 Wait N Register, offset: 0x1C
UART_WF7816   EQU  0x1D   ; UART 7816 Wait FD Register, offset: 0x1D
UART_ET7816   EQU  0x1E   ; UART 7816 Error Threshold Register, offset: 0x1E
UART_TL7816   EQU  0x1F   ; UART 7816 Transmit Length Register, offset: 0x1F

; ----------------------------------------------------------------------------
; -- UART Register Masks
; ----------------------------------------------------------------------------

; BDH Bit Fields
UART_BDH_SBR_MASK        EQU  0x1F
UART_BDH_SBR_SHIFT       EQU  0
UART_BDH_SBNS_MASK       EQU  0x20
UART_BDH_SBNS_SHIFT      EQU  5
UART_BDH_RXEDGIE_MASK    EQU  0x40
UART_BDH_RXEDGIE_SHIFT   EQU  6
UART_BDH_LBKDIE_MASK     EQU  0x80
UART_BDH_LBKDIE_SHIFT    EQU  7

; BDL Bit Fields
UART_BDL_SBR_MASK        EQU  0xFF
UART_BDL_SBR_SHIFT       EQU  0

; C1 Bit Fields
UART_C1_PT_MASK          EQU  0x1
UART_C1_PT_SHIFT         EQU  0
UART_C1_PE_MASK          EQU  0x2
UART_C1_PE_SHIFT         EQU  1
UART_C1_ILT_MASK         EQU  0x4
UART_C1_ILT_SHIFT        EQU  2
UART_C1_WAKE_MASK        EQU  0x8
UART_C1_WAKE_SHIFT       EQU  3
UART_C1_M_MASK           EQU  0x10
UART_C1_M_SHIFT          EQU  4
UART_C1_RSRC_MASK        EQU  0x20
UART_C1_RSRC_SHIFT       EQU  5
UART_C1_UARTSWAI_MASK    EQU  0x40
UART_C1_UARTSWAI_SHIFT   EQU  6
UART_C1_LOOPS_MASK       EQU  0x80
UART_C1_LOOPS_SHIFT      EQU  7

; C2 Bit Fields
UART_C2_SBK_MASK         EQU  0x1
UART_C2_SBK_SHIFT        EQU  0
UART_C2_RWU_MASK         EQU  0x2
UART_C2_RWU_SHIFT        EQU  1
UART_C2_RE_MASK          EQU  0x4
UART_C2_RE_SHIFT         EQU  2
UART_C2_TE_MASK          EQU  0x8
UART_C2_TE_SHIFT         EQU  3
UART_C2_ILIE_MASK        EQU  0x10
UART_C2_ILIE_SHIFT       EQU  4
UART_C2_RIE_MASK         EQU  0x20
UART_C2_RIE_SHIFT        EQU  5
UART_C2_TCIE_MASK        EQU  0x40
UART_C2_TCIE_SHIFT       EQU  6
UART_C2_TIE_MASK         EQU  0x80
UART_C2_TIE_SHIFT        EQU  7

; S1 Bit Fields
UART_S1_PF_MASK          EQU  0x1
UART_S1_PF_SHIFT         EQU  0
UART_S1_FE_MASK          EQU  0x2
UART_S1_FE_SHIFT         EQU  1
UART_S1_NF_MASK          EQU  0x4
UART_S1_NF_SHIFT         EQU  2
UART_S1_OR_MASK          EQU  0x8
UART_S1_OR_SHIFT         EQU  3
UART_S1_IDLE_MASK        EQU  0x10
UART_S1_IDLE_SHIFT       EQU  4
UART_S1_RDRF_MASK        EQU  0x20
UART_S1_RDRF_SHIFT       EQU  5
UART_S1_TC_MASK          EQU  0x40
UART_S1_TC_SHIFT         EQU  6
UART_S1_TDRE_MASK        EQU  0x80
UART_S1_TDRE_SHIFT       EQU  7

; S2 Bit Fields
UART_S2_RAF_MASK         EQU  0x1
UART_S2_RAF_SHIFT        EQU  0
UART_S2_LBKDE_MASK       EQU  0x2
UART_S2_LBKDE_SHIFT      EQU  1
UART_S2_BRK13_MASK       EQU  0x4
UART_S2_BRK13_SHIFT      EQU  2
UART_S2_RWUID_MASK       EQU  0x8
UART_S2_RWUID_SHIFT      EQU  3
UART_S2_RXINV_MASK       EQU  0x10
UART_S2_RXINV_SHIFT      EQU  4
UART_S2_MSBF_MASK        EQU  0x20
UART_S2_MSBF_SHIFT       EQU  5
UART_S2_RXEDGIF_MASK     EQU  0x40
UART_S2_RXEDGIF_SHIFT    EQU  6
UART_S2_LBKDIF_MASK      EQU  0x80
UART_S2_LBKDIF_SHIFT     EQU  7

; C3 Bit Fields
UART_C3_PEIE_MASK        EQU  0x1
UART_C3_PEIE_SHIFT       EQU  0
UART_C3_FEIE_MASK        EQU  0x2
UART_C3_FEIE_SHIFT       EQU  1
UART_C3_NEIE_MASK        EQU  0x4
UART_C3_NEIE_SHIFT       EQU  2
UART_C3_ORIE_MASK        EQU  0x8
UART_C3_ORIE_SHIFT       EQU  3
UART_C3_TXINV_MASK       EQU  0x10
UART_C3_TXINV_SHIFT      EQU  4
UART_C3_TXDIR_MASK       EQU  0x20
UART_C3_TXDIR_SHIFT      EQU  5
UART_C3_T8_MASK          EQU  0x40
UART_C3_T8_SHIFT         EQU  6
UART_C3_R8_MASK          EQU  0x80
UART_C3_R8_SHIFT         EQU  7

; D Bit Fields
UART_D_RT_MASK           EQU  0xFF
UART_D_RT_SHIFT          EQU  0

; MA1 Bit Fields
UART_MA1_MA_MASK         EQU  0xFF
UART_MA1_MA_SHIFT        EQU  0

; MA2 Bit Fields
UART_MA2_MA_MASK         EQU  0xFF
UART_MA2_MA_SHIFT        EQU  0

; C4 Bit Fields
UART_C4_BRFA_MASK        EQU  0x1F
UART_C4_BRFA_SHIFT       EQU  0
UART_C4_M10_MASK         EQU  0x20
UART_C4_M10_SHIFT        EQU  5
UART_C4_MAEN2_MASK       EQU  0x40
UART_C4_MAEN2_SHIFT      EQU  6
UART_C4_MAEN1_MASK       EQU  0x80
UART_C4_MAEN1_SHIFT      EQU  7

; C5 Bit Fields
UART_C5_LBKDDMAS_MASK    EQU  0x8
UART_C5_LBKDDMAS_SHIFT   EQU  3
UART_C5_ILDMAS_MASK      EQU  0x10
UART_C5_ILDMAS_SHIFT     EQU  4
UART_C5_RDMAS_MASK       EQU  0x20
UART_C5_RDMAS_SHIFT      EQU  5
UART_C5_TCDMAS_MASK      EQU  0x40
UART_C5_TCDMAS_SHIFT     EQU  6
UART_C5_TDMAS_MASK       EQU  0x80
UART_C5_TDMAS_SHIFT      EQU  7

; ED Bit Fields
UART_ED_PARITYE_MASK     EQU  0x40
UART_ED_PARITYE_SHIFT    EQU  6
UART_ED_NOISY_MASK       EQU  0x80
UART_ED_NOISY_SHIFT      EQU  7

; MODEM Bit Fields
UART_MODEM_TXCTSE_MASK    EQU  0x1
UART_MODEM_TXCTSE_SHIFT   EQU  0
UART_MODEM_TXRTSE_MASK    EQU  0x2
UART_MODEM_TXRTSE_SHIFT   EQU  1
UART_MODEM_TXRTSPOL_MASK  EQU  0x4
UART_MODEM_TXRTSPOL_SHIFT EQU  2
UART_MODEM_RXRTSE_MASK    EQU  0x8
UART_MODEM_RXRTSE_SHIFT   EQU  3

; IR Bit Fields
UART_IR_TNP_MASK         EQU  0x3
UART_IR_TNP_SHIFT        EQU  0
UART_IR_IREN_MASK        EQU  0x4
UART_IR_IREN_SHIFT       EQU  2

; PFIFO Bit Fields
UART_PFIFO_RXFIFOSIZE_MASK      EQU  0x7
UART_PFIFO_RXFIFOSIZE_SHIFT     EQU  0
UART_PFIFO_RXFE_MASK            EQU  0x8
UART_PFIFO_RXFE_SHIFT           EQU  3
UART_PFIFO_TXFIFOSIZE_MASK      EQU  0x70
UART_PFIFO_TXFIFOSIZE_SHIFT     EQU  4
UART_PFIFO_TXFE_MASK            EQU  0x80
UART_PFIFO_TXFE_SHIFT           EQU  7

; CFIFO Bit Fields
UART_CFIFO_RXUFE_MASK           EQU  0x1
UART_CFIFO_RXUFE_SHIFT          EQU  0
UART_CFIFO_TXOFE_MASK           EQU  0x2
UART_CFIFO_TXOFE_SHIFT          EQU  1
UART_CFIFO_RXOFE_MASK           EQU  0x4
UART_CFIFO_RXOFE_SHIFT          EQU  2
UART_CFIFO_RXFLUSH_MASK         EQU  0x40
UART_CFIFO_RXFLUSH_SHIFT        EQU  6
UART_CFIFO_TXFLUSH_MASK         EQU  0x80
UART_CFIFO_TXFLUSH_SHIFT        EQU  7

; SFIFO Bit Fields
UART_SFIFO_RXUF_MASK            EQU  0x1
UART_SFIFO_RXUF_SHIFT           EQU  0
UART_SFIFO_TXOF_MASK            EQU  0x2
UART_SFIFO_TXOF_SHIFT           EQU  1
UART_SFIFO_RXOF_MASK            EQU  0x4
UART_SFIFO_RXOF_SHIFT           EQU  2
UART_SFIFO_RXEMPT_MASK          EQU  0x40
UART_SFIFO_RXEMPT_SHIFT         EQU  6
UART_SFIFO_TXEMPT_MASK          EQU  0x80
UART_SFIFO_TXEMPT_SHIFT         EQU  7

; TWFIFO Bit Fields
UART_TWFIFO_TXWATER_MASK        EQU  0xFF
UART_TWFIFO_TXWATER_SHIFT       EQU  0

; TCFIFO Bit Fields
UART_TCFIFO_TXCOUNT_MASK        EQU  0xFF
UART_TCFIFO_TXCOUNT_SHIFT       EQU  0

; RWFIFO Bit Fields
UART_RWFIFO_RXWATER_MASK        EQU  0xFF
UART_RWFIFO_RXWATER_SHIFT       EQU  0

; RCFIFO Bit Fields
UART_RCFIFO_RXCOUNT_MASK        EQU  0xFF
UART_RCFIFO_RXCOUNT_SHIFT       EQU  0

; C7816 Bit Fields
UART_C7816_ISO_7816E_MASK       EQU  0x1
UART_C7816_ISO_7816E_SHIFT      EQU  0
UART_C7816_TTYPE_MASK           EQU  0x2
UART_C7816_TTYPE_SHIFT          EQU  1
UART_C7816_INIT_MASK            EQU  0x4
UART_C7816_INIT_SHIFT           EQU  2
UART_C7816_ANACK_MASK           EQU  0x8
UART_C7816_ANACK_SHIFT          EQU  3
UART_C7816_ONACK_MASK           EQU  0x10
UART_C7816_ONACK_SHIFT          EQU  4

; IE7816 Bit Fields
UART_IE7816_RXTE_MASK           EQU  0x1
UART_IE7816_RXTE_SHIFT          EQU  0
UART_IE7816_TXTE_MASK           EQU  0x2
UART_IE7816_TXTE_SHIFT          EQU  1
UART_IE7816_GTVE_MASK           EQU  0x4
UART_IE7816_GTVE_SHIFT          EQU  2
UART_IE7816_INITDE_MASK         EQU  0x10
UART_IE7816_INITDE_SHIFT        EQU  4
UART_IE7816_BWTE_MASK           EQU  0x20
UART_IE7816_BWTE_SHIFT          EQU  5
UART_IE7816_CWTE_MASK           EQU  0x40
UART_IE7816_CWTE_SHIFT          EQU  6
UART_IE7816_WTE_MASK            EQU  0x80
UART_IE7816_WTE_SHIFT           EQU  7

; IS7816 Bit Fields
UART_IS7816_RXT_MASK            EQU  0x1
UART_IS7816_RXT_SHIFT           EQU  0
UART_IS7816_TXT_MASK            EQU  0x2
UART_IS7816_TXT_SHIFT           EQU  1
UART_IS7816_GTV_MASK            EQU  0x4
UART_IS7816_GTV_SHIFT           EQU  2
UART_IS7816_INITD_MASK          EQU  0x10
UART_IS7816_INITD_SHIFT         EQU  4
UART_IS7816_BWT_MASK            EQU  0x20
UART_IS7816_BWT_SHIFT           EQU  5
UART_IS7816_CWT_MASK            EQU  0x40
UART_IS7816_CWT_SHIFT           EQU  6
UART_IS7816_WT_MASK             EQU  0x80
UART_IS7816_WT_SHIFT            EQU  7

; WP7816_T_TYPE0 Bit Fields
UART_WP7816_T_TYPE0_WI_MASK     EQU  0xFF
UART_WP7816_T_TYPE0_WI_SHIFT    EQU  0

; WP7816_T_TYPE1 Bit Fields
UART_WP7816_T_TYPE1_BWI_MASK    EQU  0xF
UART_WP7816_T_TYPE1_BWI_SHIFT   EQU  0
UART_WP7816_T_TYPE1_CWI_MASK    EQU  0xF0
UART_WP7816_T_TYPE1_CWI_SHIFT   EQU  4

; WN7816 Bit Fields
UART_WN7816_GTN_MASK            EQU  0xFF
UART_WN7816_GTN_SHIFT           EQU  0

; WF7816 Bit Fields
UART_WF7816_GTFD_MASK           EQU  0xFF
UART_WF7816_GTFD_SHIFT          EQU  0

; ET7816 Bit Fields
UART_ET7816_RXTHRESHOLD_MASK    EQU  0xF
UART_ET7816_RXTHRESHOLD_SHIFT   EQU  0
UART_ET7816_TXTHRESHOLD_MASK    EQU  0xF0
UART_ET7816_TXTHRESHOLD_SHIFT   EQU  4

; 
;TL7816 Bit Fields
UART_TL7816_TLEN_MASK           EQU  0xFF
UART_TL7816_TLEN_SHIFT          EQU  0

; ----------------------------------------------------------------------------
; -- USB Peripheral Access Layer
; ----------------------------------------------------------------------------

USB_PERID             EQU  0x000   ; Peripheral ID register, offset: 0x0
USB_IDCOMP            EQU  0x004   ; Peripheral ID Complement register, offset: 0x4
USB_REV               EQU  0x008   ; Peripheral Revision register, offset: 0x8
USB_ADDINFO           EQU  0x00C   ; Peripheral Additional Info register, offset: 0xC
USB_OTGISTAT          EQU  0x010   ; OTG Interrupt Status register, offset: 0x10
USB_OTGICR            EQU  0x014   ; OTG Interrupt Control register, offset: 0x14
USB_OTGSTAT           EQU  0x018   ; OTG Status register, offset: 0x18
USB_OTGCTL            EQU  0x01C   ; OTG Control register, offset: 0x1C
USB_ISTAT             EQU  0x080   ; Interrupt Status register, offset: 0x80
USB_INTEN             EQU  0x084   ; Interrupt Enable register, offset: 0x84
USB_ERRSTAT           EQU  0x088   ; Error Interrupt Status register, offset: 0x88
USB_ERREN             EQU  0x08C   ; Error Interrupt Enable register, offset: 0x8C
USB_STAT              EQU  0x090   ; Status register, offset: 0x90
USB_CTL               EQU  0x094   ; Control register, offset: 0x94
USB_ADDR              EQU  0x098   ; Address register, offset: 0x98
USB_BDTPAGE1          EQU  0x09C   ; BDT Page register 1, offset: 0x9C
USB_FRMNUML           EQU  0x0A0   ; Frame Number register Low, offset: 0xA0
USB_FRMNUMH           EQU  0x0A4   ; Frame Number register High, offset: 0xA4
USB_TOKEN             EQU  0x0A8   ; Token register, offset: 0xA8
USB_SOFTHLD           EQU  0x0AC   ; SOF Threshold register, offset: 0xAC
USB_BDTPAGE2          EQU  0x0B0   ; BDT Page Register 2, offset: 0xB0
USB_BDTPAGE3          EQU  0x0B4   ; BDT Page Register 3, offset: 0xB4

USB_ENDPOINT0_ENDPT   EQU  0x0C0   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT1_ENDPT   EQU  0x0C4   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT2_ENDPT   EQU  0x0C8   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT3_ENDPT   EQU  0x0CC   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT4_ENDPT   EQU  0x0D0   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT5_ENDPT   EQU  0x0D4   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT6_ENDPT   EQU  0x0D8   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT7_ENDPT   EQU  0x0DC   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT8_ENDPT   EQU  0x0E0   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT9_ENDPT   EQU  0x0E4   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT10_ENDPT  EQU  0x0E8   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT11_ENDPT  EQU  0x0EC   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT12_ENDPT  EQU  0x0F0   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT13_ENDPT  EQU  0x0F4   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT14_ENDPT  EQU  0x0F8   ; endpoint Control register, array offset: 0xC0, array step: 0x4
USB_ENDPOINT15_ENDPT  EQU  0x0FC   ; endpoint Control register, array offset: 0xC0, array step: 0x4

USB_USBCTRL           EQU  0x100   ; USB Control register, offset: 0x100
USB_OBSERVE           EQU  0x104   ; USB OTG Observe register, offset: 0x104
USB_CONTROL           EQU  0x108   ; USB OTG Control register, offset: 0x108
USB_USBTRC0           EQU  0x10C   ; USB Transceiver Control register 0, offset: 0x10C

USB_USBFRMADJUST             EQU  0x114   ; Frame Adjust Register, offset: 0x114
USB_CLK_RECOVER_CTRL         EQU  0x140   ; USB Clock recovery control, offset: 0x140
USB_CLK_RECOVER_IRC_EN       EQU  0x144   ; IRC48M oscillator enable register, offset: 0x144
USB_CLK_RECOVER_INT_STATUS   EQU  0x15C    ; Clock recovery separated interrupt status, offset: 0x15C

; ----------------------------------------------------------------------------
; -- USB Register Masks
; ----------------------------------------------------------------------------

; PERID Bit Fields
USB_PERID_ID_MASK        EQU  0x3F
USB_PERID_ID_SHIFT       EQU  0

; IDCOMP Bit Fields
USB_IDCOMP_NID_MASK      EQU  0x3F
USB_IDCOMP_NID_SHIFT     EQU  0

; REV Bit Fields
USB_REV_REV_MASK         EQU  0xFF
USB_REV_REV_SHIFT        EQU  0

; ADDINFO Bit Fields
USB_ADDINFO_IEHOST_MASK  EQU  0x1
USB_ADDINFO_IEHOST_SHIFT EQU  0
USB_ADDINFO_IRQNUM_MASK  EQU  0xF8
USB_ADDINFO_IRQNUM_SHIFT EQU  3

; OTGISTAT Bit Fields
USB_OTGISTAT_AVBUSCHG_MASK               EQU  0x1
USB_OTGISTAT_AVBUSCHG_SHIFT              EQU  0
USB_OTGISTAT_B_SESS_CHG_MASK             EQU  0x4
USB_OTGISTAT_B_SESS_CHG_SHIFT            EQU  2
USB_OTGISTAT_SESSVLDCHG_MASK             EQU  0x8
USB_OTGISTAT_SESSVLDCHG_SHIFT            EQU  3
USB_OTGISTAT_LINE_STATE_CHG_MASK         EQU  0x20
USB_OTGISTAT_LINE_STATE_CHG_SHIFT        EQU  5
USB_OTGISTAT_ONEMSEC_MASK                EQU  0x40
USB_OTGISTAT_ONEMSEC_SHIFT               EQU  6
USB_OTGISTAT_IDCHG_MASK                  EQU  0x80
USB_OTGISTAT_IDCHG_SHIFT                 EQU  7

; OTGICR Bit Fields
USB_OTGICR_AVBUSEN_MASK                  EQU  0x1
USB_OTGICR_AVBUSEN_SHIFT                 EQU  0
USB_OTGICR_BSESSEN_MASK                  EQU  0x4
USB_OTGICR_BSESSEN_SHIFT                 EQU  2
USB_OTGICR_SESSVLDEN_MASK                EQU  0x8
USB_OTGICR_SESSVLDEN_SHIFT               EQU  3
USB_OTGICR_LINESTATEEN_MASK              EQU  0x20
USB_OTGICR_LINESTATEEN_SHIFT             EQU  5
USB_OTGICR_ONEMSECEN_MASK                EQU  0x40
USB_OTGICR_ONEMSECEN_SHIFT               EQU  6
USB_OTGICR_IDEN_MASK                     EQU  0x80
USB_OTGICR_IDEN_SHIFT                    EQU  7

; OTGSTAT Bit Fields
USB_OTGSTAT_AVBUSVLD_MASK                EQU  0x1
USB_OTGSTAT_AVBUSVLD_SHIFT               EQU  0
USB_OTGSTAT_BSESSEND_MASK                EQU  0x4
USB_OTGSTAT_BSESSEND_SHIFT               EQU  2
USB_OTGSTAT_SESS_VLD_MASK                EQU  0x8
USB_OTGSTAT_SESS_VLD_SHIFT               EQU  3
USB_OTGSTAT_LINESTATESTABLE_MASK         EQU  0x20
USB_OTGSTAT_LINESTATESTABLE_SHIFT        EQU  5
USB_OTGSTAT_ONEMSECEN_MASK               EQU  0x40
USB_OTGSTAT_ONEMSECEN_SHIFT              EQU  6
USB_OTGSTAT_ID_MASK                      EQU  0x80
USB_OTGSTAT_ID_SHIFT                     EQU  7

; OTGCTL Bit Fields
USB_OTGCTL_OTGEN_MASK    EQU  0x4
USB_OTGCTL_OTGEN_SHIFT   EQU  2
USB_OTGCTL_DMLOW_MASK    EQU  0x10
USB_OTGCTL_DMLOW_SHIFT   EQU  4
USB_OTGCTL_DPLOW_MASK    EQU  0x20
USB_OTGCTL_DPLOW_SHIFT   EQU  5
USB_OTGCTL_DPHIGH_MASK   EQU  0x80
USB_OTGCTL_DPHIGH_SHIFT  EQU  7

; ISTAT Bit Fields
USB_ISTAT_USBRST_MASK    EQU  0x1
USB_ISTAT_USBRST_SHIFT   EQU  0
USB_ISTAT_ERROR_MASK     EQU  0x2
USB_ISTAT_ERROR_SHIFT    EQU  1
USB_ISTAT_SOFTOK_MASK    EQU  0x4
USB_ISTAT_SOFTOK_SHIFT   EQU  2
USB_ISTAT_TOKDNE_MASK    EQU  0x8
USB_ISTAT_TOKDNE_SHIFT   EQU  3
USB_ISTAT_SLEEP_MASK     EQU  0x10
USB_ISTAT_SLEEP_SHIFT    EQU  4
USB_ISTAT_RESUME_MASK    EQU  0x20
USB_ISTAT_RESUME_SHIFT   EQU  5
USB_ISTAT_ATTACH_MASK    EQU  0x40
USB_ISTAT_ATTACH_SHIFT   EQU  6
USB_ISTAT_STALL_MASK     EQU  0x80
USB_ISTAT_STALL_SHIFT    EQU  7

; INTEN Bit Fields
USB_INTEN_USBRSTEN_MASK  EQU  0x1
USB_INTEN_USBRSTEN_SHIFT EQU  0
USB_INTEN_ERROREN_MASK   EQU  0x2
USB_INTEN_ERROREN_SHIFT  EQU  1
USB_INTEN_SOFTOKEN_MASK  EQU  0x4
USB_INTEN_SOFTOKEN_SHIFT EQU  2
USB_INTEN_TOKDNEEN_MASK  EQU  0x8
USB_INTEN_TOKDNEEN_SHIFT EQU  3
USB_INTEN_SLEEPEN_MASK   EQU  0x10
USB_INTEN_SLEEPEN_SHIFT  EQU  4
USB_INTEN_RESUMEEN_MASK  EQU  0x20
USB_INTEN_RESUMEEN_SHIFT EQU  5
USB_INTEN_ATTACHEN_MASK  EQU  0x40
USB_INTEN_ATTACHEN_SHIFT EQU  6
USB_INTEN_STALLEN_MASK   EQU  0x80
USB_INTEN_STALLEN_SHIFT  EQU  7

; ERRSTAT Bit Fields
USB_ERRSTAT_PIDERR_MASK  EQU  0x1
USB_ERRSTAT_PIDERR_SHIFT EQU  0
USB_ERRSTAT_CRC5EOF_MASK EQU  0x2
USB_ERRSTAT_CRC5EOF_SHIFT EQU  1
USB_ERRSTAT_CRC16_MASK   EQU  0x4
USB_ERRSTAT_CRC16_SHIFT  EQU  2
USB_ERRSTAT_DFN8_MASK    EQU  0x8
USB_ERRSTAT_DFN8_SHIFT   EQU  3
USB_ERRSTAT_BTOERR_MASK  EQU  0x10
USB_ERRSTAT_BTOERR_SHIFT EQU  4
USB_ERRSTAT_DMAERR_MASK  EQU  0x20
USB_ERRSTAT_DMAERR_SHIFT EQU  5
USB_ERRSTAT_BTSERR_MASK  EQU  0x80
USB_ERRSTAT_BTSERR_SHIFT EQU  7

; ERREN Bit Fields
USB_ERREN_PIDERREN_MASK  EQU  0x1
USB_ERREN_PIDERREN_SHIFT EQU  0
USB_ERREN_CRC5EOFEN_MASK EQU  0x2
USB_ERREN_CRC5EOFEN_SHIFT EQU  1
USB_ERREN_CRC16EN_MASK   EQU  0x4
USB_ERREN_CRC16EN_SHIFT  EQU  2
USB_ERREN_DFN8EN_MASK    EQU  0x8
USB_ERREN_DFN8EN_SHIFT   EQU  3
USB_ERREN_BTOERREN_MASK  EQU  0x10
USB_ERREN_BTOERREN_SHIFT EQU  4
USB_ERREN_DMAERREN_MASK  EQU  0x20
USB_ERREN_DMAERREN_SHIFT EQU  5
USB_ERREN_BTSERREN_MASK  EQU  0x80
USB_ERREN_BTSERREN_SHIFT EQU  7

; STAT Bit Fields
USB_STAT_ODD_MASK        EQU  0x4
USB_STAT_ODD_SHIFT       EQU  2
USB_STAT_TX_MASK         EQU  0x8
USB_STAT_TX_SHIFT        EQU  3
USB_STAT_ENDP_MASK       EQU  0xF0
USB_STAT_ENDP_SHIFT      EQU  4

; CTL Bit Fields
USB_CTL_USBENSOFEN_MASK  EQU  0x1
USB_CTL_USBENSOFEN_SHIFT EQU  0
USB_CTL_ODDRST_MASK      EQU  0x2
USB_CTL_ODDRST_SHIFT     EQU  1
USB_CTL_RESUME_MASK      EQU  0x4
USB_CTL_RESUME_SHIFT     EQU  2
USB_CTL_HOSTMODEEN_MASK  EQU  0x8
USB_CTL_HOSTMODEEN_SHIFT EQU  3
USB_CTL_RESET_MASK       EQU  0x10
USB_CTL_RESET_SHIFT      EQU  4
USB_CTL_TXSUSPENDTOKENBUSY_MASK EQU  0x20
USB_CTL_TXSUSPENDTOKENBUSY_SHIFT         EQU  5
USB_CTL_SE0_MASK         EQU  0x40
USB_CTL_SE0_SHIFT        EQU  6
USB_CTL_JSTATE_MASK      EQU  0x80
USB_CTL_JSTATE_SHIFT     EQU  7

; ADDR Bit Fields
USB_ADDR_ADDR_MASK       EQU  0x7F
USB_ADDR_ADDR_SHIFT      EQU  0
USB_ADDR_LSEN_MASK       EQU  0x80
USB_ADDR_LSEN_SHIFT      EQU  7

; BDTPAGE1 Bit Fields
USB_BDTPAGE1_BDTBA_MASK  EQU  0xFE
USB_BDTPAGE1_BDTBA_SHIFT EQU  1

; FRMNUML Bit Fields
USB_FRMNUML_FRM_MASK     EQU  0xFF
USB_FRMNUML_FRM_SHIFT    EQU  0

; FRMNUMH Bit Fields
USB_FRMNUMH_FRM_MASK     EQU  0x7
USB_FRMNUMH_FRM_SHIFT    EQU  0

; TOKEN Bit Fields
USB_TOKEN_TOKENENDPT_MASK  EQU  0xF
USB_TOKEN_TOKENENDPT_SHIFT EQU  0
USB_TOKEN_TOKENPID_MASK    EQU  0xF0
USB_TOKEN_TOKENPID_SHIFT   EQU  4

; SOFTHLD Bit Fields
USB_SOFTHLD_CNT_MASK     EQU  0xFF
USB_SOFTHLD_CNT_SHIFT    EQU  0

; BDTPAGE2 Bit Fields
USB_BDTPAGE2_BDTBA_MASK  EQU  0xFF
USB_BDTPAGE2_BDTBA_SHIFT EQU  0

; BDTPAGE3 Bit Fields
USB_BDTPAGE3_BDTBA_MASK  EQU  0xFF
USB_BDTPAGE3_BDTBA_SHIFT EQU  0

; ENDPT Bit Fields
USB_ENDPT_EPHSHK_MASK    EQU  0x1
USB_ENDPT_EPHSHK_SHIFT   EQU  0
USB_ENDPT_EPSTALL_MASK   EQU  0x2
USB_ENDPT_EPSTALL_SHIFT  EQU  1
USB_ENDPT_EPTXEN_MASK    EQU  0x4
USB_ENDPT_EPTXEN_SHIFT   EQU  2
USB_ENDPT_EPRXEN_MASK    EQU  0x8
USB_ENDPT_EPRXEN_SHIFT   EQU  3
USB_ENDPT_EPCTLDIS_MASK  EQU  0x10
USB_ENDPT_EPCTLDIS_SHIFT EQU  4
USB_ENDPT_RETRYDIS_MASK  EQU  0x40
USB_ENDPT_RETRYDIS_SHIFT EQU  6
USB_ENDPT_HOSTWOHUB_MASK EQU  0x80
USB_ENDPT_HOSTWOHUB_SHIFT EQU  7

; USBCTRL Bit Fields
USB_USBCTRL_PDE_MASK     EQU  0x40
USB_USBCTRL_PDE_SHIFT    EQU  6
USB_USBCTRL_SUSP_MASK    EQU  0x80
USB_USBCTRL_SUSP_SHIFT   EQU  7

; OBSERVE Bit Fields
USB_OBSERVE_DMPD_MASK    EQU  0x10
USB_OBSERVE_DMPD_SHIFT   EQU  4
USB_OBSERVE_DPPD_MASK    EQU  0x40
USB_OBSERVE_DPPD_SHIFT   EQU  6
USB_OBSERVE_DPPU_MASK    EQU  0x80
USB_OBSERVE_DPPU_SHIFT   EQU  7

; CONTROL Bit Fields
USB_CONTROL_DPPULLUPNONOTG_MASK          EQU  0x10
USB_CONTROL_DPPULLUPNONOTG_SHIFT         EQU  4

; USBTRC0 Bit Fields
USB_USBTRC0_USB_RESUME_INT_MASK          EQU  0x1
USB_USBTRC0_USB_RESUME_INT_SHIFT         EQU  0

USB_USBTRC0_SYNC_DET_MASK EQU  0x2
USB_USBTRC0_SYNC_DET_SHIFT               EQU  1
USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK    EQU  0x4
USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT   EQU  2
USB_USBTRC0_USBRESMEN_MASK               EQU  0x20
USB_USBTRC0_USBRESMEN_SHIFT              EQU  5
USB_USBTRC0_USBRESET_MASK EQU  0x80
USB_USBTRC0_USBRESET_SHIFT               EQU  7

; USBFRMADJUST Bit Fields
USB_USBFRMADJUST_ADJ_MASK EQU  0xFF
USB_USBFRMADJUST_ADJ_SHIFT               EQU  0

; CLK_RECOVER_CTRL Bit Fields
USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK     EQU  0x20
USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT    EQU  5
USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK  EQU  0x40
USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT EQU  6
USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK       EQU  0x80
USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT      EQU  7

; CLK_RECOVER_IRC_EN Bit Fields
USB_CLK_RECOVER_IRC_EN_REG_EN_MASK       EQU  0x1
USB_CLK_RECOVER_IRC_EN_REG_EN_SHIFT      EQU  0
USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK       EQU  0x2
USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT      EQU  1

; CLK_RECOVER_INT_STATUS Bit Fields
USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK  EQU  0x10
USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT EQU  4

; ----------------------------------------------------------------------------
; -- USBDCD Peripheral Access Layer
; ----------------------------------------------------------------------------

USBDCD_CONTROL      EQU  0x00   ; Control register, offset: 0x0
USBDCD_CLOCK        EQU  0x04   ; Clock register, offset: 0x4
USBDCD_STATUS       EQU  0x08   ; Status register, offset: 0x8
USBDCD_TIMER0       EQU  0x10   ; TIMER0 register, offset: 0x10
USBDCD_TIMER1       EQU  0x14   ; TIMER1 register, offset: 0x14
USBDCD_TIMER2_BC11  EQU  0x18   ; TIMER2_BC11 register, offset: 0x18
USBDCD_TIMER2_BC12  EQU  0x18   ; TIMER2_BC12 register, offset: 0x18

; ----------------------------------------------------------------------------
; -- USBDCD Register Masks
; ----------------------------------------------------------------------------

; CONTROL Bit Fields
USBDCD_CONTROL_IACK_MASK   EQU  0x1
USBDCD_CONTROL_IACK_SHIFT  EQU  0
USBDCD_CONTROL_IF_MASK     EQU  0x100
USBDCD_CONTROL_IF_SHIFT    EQU  8
USBDCD_CONTROL_IE_MASK     EQU  0x10000
USBDCD_CONTROL_IE_SHIFT    EQU  16
USBDCD_CONTROL_BC12_MASK   EQU  0x20000
USBDCD_CONTROL_BC12_SHIFT  EQU  17
USBDCD_CONTROL_START_MASK  EQU  0x1000000
USBDCD_CONTROL_START_SHIFT EQU  24
USBDCD_CONTROL_SR_MASK     EQU  0x2000000
USBDCD_CONTROL_SR_SHIFT    EQU  25

; CLOCK Bit Fields
USBDCD_CLOCK_CLOCK_UNIT_MASK             EQU  0x1
USBDCD_CLOCK_CLOCK_UNIT_SHIFT            EQU  0
USBDCD_CLOCK_CLOCK_SPEED_MASK            EQU  0xFFC
USBDCD_CLOCK_CLOCK_SPEED_SHIFT           EQU  2

; STATUS Bit Fields
USBDCD_STATUS_SEQ_RES_MASK               EQU  0x30000
USBDCD_STATUS_SEQ_RES_SHIFT              EQU  16
USBDCD_STATUS_SEQ_STAT_MASK              EQU  0xC0000
USBDCD_STATUS_SEQ_STAT_SHIFT             EQU  18
USBDCD_STATUS_ERR_MASK                   EQU  0x100000
USBDCD_STATUS_ERR_SHIFT                  EQU  20
USBDCD_STATUS_TO_MASK                    EQU  0x200000
USBDCD_STATUS_TO_SHIFT                   EQU  21
USBDCD_STATUS_ACTIVE_MASK                EQU  0x400000
USBDCD_STATUS_ACTIVE_SHIFT               EQU  22

; TIMER0 Bit Fields
USBDCD_TIMER0_TUNITCON_MASK              EQU  0xFFF
USBDCD_TIMER0_TUNITCON_SHIFT             EQU  0
USBDCD_TIMER0_TSEQ_INIT_MASK             EQU  0x3FF0000
USBDCD_TIMER0_TSEQ_INIT_SHIFT            EQU  16

; TIMER1 Bit Fields
USBDCD_TIMER1_TVDPSRC_ON_MASK            EQU  0x3FF
USBDCD_TIMER1_TVDPSRC_ON_SHIFT           EQU  0
USBDCD_TIMER1_TDCD_DBNC_MASK             EQU  0x3FF0000
USBDCD_TIMER1_TDCD_DBNC_SHIFT            EQU  16

; TIMER2_BC11 Bit Fields
USBDCD_TIMER2_BC11_CHECK_DM_MASK         EQU  0xF
USBDCD_TIMER2_BC11_CHECK_DM_SHIFT        EQU  0
USBDCD_TIMER2_BC11_TVDPSRC_CON_MASK      EQU  0x3FF0000
USBDCD_TIMER2_BC11_TVDPSRC_CON_SHIFT     EQU  16

; TIMER2_BC12 Bit Fields
USBDCD_TIMER2_BC12_TVDMSRC_ON_MASK       EQU  0x3FF
USBDCD_TIMER2_BC12_TVDMSRC_ON_SHIFT      EQU  0
USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_MASK  EQU  0x3FF0000
USBDCD_TIMER2_BC12_TWAIT_AFTER_PRD_SHIFT EQU  16

; ----------------------------------------------------------------------------
; -- VREF Peripheral Access Layer
; ----------------------------------------------------------------------------

TRM               EQU  0x000   ; VREF Trim Register, offset: 0x0
SC                EQU  0x000   ; VREF Status and Control Register, offset: 0x1

; ----------------------------------------------------------------------------
; -- VREF Register Masks
; ----------------------------------------------------------------------------

; TRM Bit Fields
VREF_TRM_TRIM_MASK       EQU  0x3F
VREF_TRM_TRIM_SHIFT      EQU  0
VREF_TRM_CHOPEN_MASK     EQU  0x40
VREF_TRM_CHOPEN_SHIFT    EQU  6

; SC Bit Fields
VREF_SC_MODE_LV_MASK     EQU  0x3
VREF_SC_MODE_LV_SHIFT    EQU  0
VREF_SC_VREFST_MASK      EQU  0x4
VREF_SC_VREFST_SHIFT     EQU  2
VREF_SC_ICOMPEN_MASK     EQU  0x20
VREF_SC_ICOMPEN_SHIFT    EQU  5
VREF_SC_REGEN_MASK       EQU  0x40
VREF_SC_REGEN_SHIFT      EQU  6
VREF_SC_VREFEN_MASK      EQU  0x80
VREF_SC_VREFEN_SHIFT     EQU  7

; ----------------------------------------------------------------------------
; -- WDOG Peripheral Access Layer
; ----------------------------------------------------------------------------

STCTRLH EQU  0x000   ; Watchdog Status and Control Register High, offset: 0x0
STCTRLL EQU  0x000   ; Watchdog Status and Control Register Low, offset: 0x2
TOVALH  EQU  0x000   ; Watchdog Time-out Value Register High, offset: 0x4
TOVALL  EQU  0x000   ; Watchdog Time-out Value Register Low, offset: 0x6
WINH    EQU  0x000   ; Watchdog Window Register High, offset: 0x8
WINL    EQU  0x000   ; Watchdog Window Register Low, offset: 0xA
REFRESH EQU  0x000   ; Watchdog Refresh register, offset: 0xC
UNLOCK  EQU  0x000   ; Watchdog Unlock register, offset: 0xE
TMROUTH EQU  0x000   ; Watchdog Timer Output Register High, offset: 0x10
TMROUTL EQU  0x000   ; Watchdog Timer Output Register Low, offset: 0x12
RSTCNT  EQU  0x000   ; Watchdog Reset Count register, offset: 0x14
PRESC   EQU  0x000   ; Watchdog Prescaler register, offset: 0x16

; ----------------------------------------------------------------------------
; -- WDOG Register Masks
; ----------------------------------------------------------------------------

; STCTRLH Bit Fields
WDOG_STCTRLH_WDOGEN_MASK                 EQU  0x1
WDOG_STCTRLH_WDOGEN_SHIFT                EQU  0
WDOG_STCTRLH_CLKSRC_MASK                 EQU  0x2
WDOG_STCTRLH_CLKSRC_SHIFT                EQU  1
WDOG_STCTRLH_IRQRSTEN_MASK               EQU  0x4
WDOG_STCTRLH_IRQRSTEN_SHIFT              EQU  2
WDOG_STCTRLH_WINEN_MASK                  EQU  0x8
WDOG_STCTRLH_WINEN_SHIFT                 EQU  3
WDOG_STCTRLH_ALLOWUPDATE_MASK            EQU  0x10
WDOG_STCTRLH_ALLOWUPDATE_SHIFT           EQU  4
WDOG_STCTRLH_DBGEN_MASK                  EQU  0x20
WDOG_STCTRLH_DBGEN_SHIFT                 EQU  5
WDOG_STCTRLH_STOPEN_MASK                 EQU  0x40
WDOG_STCTRLH_STOPEN_SHIFT                EQU  6
WDOG_STCTRLH_WAITEN_MASK                 EQU  0x80
WDOG_STCTRLH_WAITEN_SHIFT                EQU  7
WDOG_STCTRLH_TESTWDOG_MASK               EQU  0x400
WDOG_STCTRLH_TESTWDOG_SHIFT              EQU  10
WDOG_STCTRLH_TESTSEL_MASK                EQU  0x800
WDOG_STCTRLH_TESTSEL_SHIFT               EQU  11
WDOG_STCTRLH_BYTESEL_MASK                EQU  0x3000
WDOG_STCTRLH_BYTESEL_SHIFT               EQU  12
WDOG_STCTRLH_DISTESTWDOG_MASK            EQU  0x4000
WDOG_STCTRLH_DISTESTWDOG_SHIFT           EQU  14

; STCTRLL Bit Fields
WDOG_STCTRLL_INTFLG_MASK                 EQU  0x8000
WDOG_STCTRLL_INTFLG_SHIFT                EQU  15

; TOVALH Bit Fields
WDOG_TOVALH_TOVALHIGH_MASK               EQU  0xFFFF
WDOG_TOVALH_TOVALHIGH_SHIFT              EQU  0

; TOVALL Bit Fields
WDOG_TOVALL_TOVALLOW_MASK                EQU  0xFFFF
WDOG_TOVALL_TOVALLOW_SHIFT               EQU  0

; WINH Bit Fields
WDOG_WINH_WINHIGH_MASK                   EQU  0xFFFF
WDOG_WINH_WINHIGH_SHIFT                  EQU  0

; WINL Bit Fields
WDOG_WINL_WINLOW_MASK                    EQU  0xFFFF
WDOG_WINL_WINLOW_SHIFT                   EQU  0

; REFRESH Bit Fields
WDOG_REFRESH_WDOGREFRESH_MASK            EQU  0xFFFF
WDOG_REFRESH_WDOGREFRESH_SHIFT           EQU  0

; UNLOCK Bit Fields
WDOG_UNLOCK_WDOGUNLOCK_MASK              EQU  0xFFFF
WDOG_UNLOCK_WDOGUNLOCK_SHIFT             EQU  0

; TMROUTH Bit Fields
WDOG_TMROUTH_TIMEROUTHIGH_MASK           EQU  0xFFFF
WDOG_TMROUTH_TIMEROUTHIGH_SHIFT          EQU  0

; TMROUTL Bit Fields
WDOG_TMROUTL_TIMEROUTLOW_MASK            EQU  0xFFFF
WDOG_TMROUTL_TIMEROUTLOW_SHIFT           EQU  0

; RSTCNT Bit Fields
WDOG_RSTCNT_RSTCNT_MASK                  EQU  0xFFFF
WDOG_RSTCNT_RSTCNT_SHIFT                 EQU  0

; PRESC Bit Fields
WDOG_PRESC_PRESCVAL_MASK                 EQU  0x700
WDOG_PRESC_PRESCVAL_SHIFT                EQU  8

	END
