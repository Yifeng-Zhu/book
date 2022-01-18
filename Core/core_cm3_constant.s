;******************** (C) Yifeng ZHU ******************************************************************
; @file    core_cm3_constant.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    December-22-2012
; @note    Modifed from core_cm3.h (C) 2010 STMicroelectronics
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

; ******************************************************************************
; *                 Register Abstraction
; *****************************************************************************

; NVIC_ISER       EQU    0x000       ; Offset: 0x000  Interrupt Set Enable Register           
; NVIC_ICER       EQU    0x080       ; Offset: 0x080  Interrupt Clear Enable Register         
; NVIC_ISPR       EQU    0x100       ; Offset: 0x100  Interrupt Set Pending Register          
; NVIC_ICPR       EQU    0x180       ; Offset: 0x180  Interrupt Clear Pending Register        
; NVIC_IABR       EQU    0x200       ; Offset: 0x200  Interrupt Active bit Register           
; NVIC_IP         EQU    0x300       ; Offset: 0x300  Interrupt Priority Register (8Bit wide) 
; NVIC_STIR       EQU    0xE00       ; Offset: 0xE00  Software Trigger Interrupt Register     


; SCB_CPUID       EQU    0x00        ; Offset: 0x00  CPU ID Base Register                                  
; SCB_ICSR        EQU    0x04        ; Offset: 0x04  Interrupt Control State Register                      
; SCB_VTOR        EQU    0x08        ; Offset: 0x08  Vector Table Offset Register                          
; SCB_AIRCR       EQU    0x0C        ; Offset: 0x0C  Application Interrupt / Reset Control Register        
; SCB_SCR         EQU    0x10        ; Offset: 0x10  System Control Register                               
; SCB_CCR         EQU    0x14        ; Offset: 0x14  Configuration Control Register                        
; SCB_SHP         EQU    0x18        ; Offset: 0x18  System Handlers Priority Registers (4-7, 8-11, 12-15) 
; SCB_SHCSR       EQU    0x24        ; Offset: 0x24  System Handler Control and State Register             
; SCB_CFSR        EQU    0x28        ; Offset: 0x28  Configurable Fault Status Register                    
; SCB_HFSR        EQU    0x2C        ; Offset: 0x2C  Hard Fault Status Register                            
; SCB_DFSR        EQU    0x30        ; Offset: 0x30  Debug Fault Status Register                           
; SCB_MMFAR       EQU    0x34        ; Offset: 0x34  Mem Manage Address Register                           
; SCB_BFAR        EQU    0x38        ; Offset: 0x38  Bus Fault Address Register                            
; SCB_AFSR        EQU    0x3C        ; Offset: 0x3C  Auxiliary Fault Status Register                       
; SCB_PFR         EQU    0x40        ; Offset: 0x40  Processor Feature Register                            
; SCB_DFR         EQU    0x48        ; Offset: 0x48  Debug Feature Register                                
; SCB_ADR         EQU    0x4C        ; Offset: 0x4C  Auxiliary Feature Register                            
; SCB_MMFR        EQU    0x50        ; Offset: 0x50  Memory Model Feature Register                         
; SCB_ISAR        EQU    0x60        ; Offset: 0x60  ISA Feature Register                                  

; SCB CPUID Register Definitions 
SCB_CPUID_IMPLEMENTER_Pos          EQU        24                                   ; SCB CPUID: IMPLEMENTER Position 
SCB_CPUID_IMPLEMENTER_Msk          EQU        (0xFF << SCB_CPUID_IMPLEMENTER_Pos)  ; SCB CPUID: IMPLEMENTER Mask 

SCB_CPUID_VARIANT_Pos              EQU        20                                   ; SCB CPUID: VARIANT Position 
SCB_CPUID_VARIANT_Msk              EQU        (0xF << SCB_CPUID_VARIANT_Pos)       ; SCB CPUID: VARIANT Mask 

SCB_CPUID_PARTNO_Pos               EQU         4                                   ; SCB CPUID: PARTNO Position 
SCB_CPUID_PARTNO_Msk               EQU        (0xFFF << SCB_CPUID_PARTNO_Pos)      ; SCB CPUID: PARTNO Mask 

SCB_CPUID_REVISION_Pos             EQU         0                                   ; SCB CPUID: REVISION Position 
SCB_CPUID_REVISION_Msk             EQU        (0xF << SCB_CPUID_REVISION_Pos)      ; SCB CPUID: REVISION Mask 

; SCB Interrupt Control State Register Definitions 
SCB_ICSR_NMIPENDSET_Pos            EQU        31                                   ; SCB ICSR: NMIPENDSET Position 
SCB_ICSR_NMIPENDSET_Msk            EQU        (1 << SCB_ICSR_NMIPENDSET_Pos)       ; SCB ICSR: NMIPENDSET Mask 

SCB_ICSR_PENDSVSET_Pos             EQU        28                                   ; SCB ICSR: PENDSVSET Position 
SCB_ICSR_PENDSVSET_Msk             EQU        (1 << SCB_ICSR_PENDSVSET_Pos)        ; SCB ICSR: PENDSVSET Mask 

SCB_ICSR_PENDSVCLR_Pos             EQU        27                                   ; SCB ICSR: PENDSVCLR Position 
SCB_ICSR_PENDSVCLR_Msk             EQU        (1 << SCB_ICSR_PENDSVCLR_Pos)        ; SCB ICSR: PENDSVCLR Mask 

SCB_ICSR_PENDSTSET_Pos             EQU        26                                   ; SCB ICSR: PENDSTSET Position 
SCB_ICSR_PENDSTSET_Msk             EQU        (1 << SCB_ICSR_PENDSTSET_Pos)        ; SCB ICSR: PENDSTSET Mask 

SCB_ICSR_PENDSTCLR_Pos             EQU        25                                   ; SCB ICSR: PENDSTCLR Position 
SCB_ICSR_PENDSTCLR_Msk             EQU        (1 << SCB_ICSR_PENDSTCLR_Pos)        ; SCB ICSR: PENDSTCLR Mask 

SCB_ICSR_ISRPREEMPT_Pos            EQU        23                                   ; SCB ICSR: ISRPREEMPT Position 
SCB_ICSR_ISRPREEMPT_Msk            EQU        (1 << SCB_ICSR_ISRPREEMPT_Pos)       ; SCB ICSR: ISRPREEMPT Mask 

SCB_ICSR_ISRPENDING_Pos            EQU        22                                   ; SCB ICSR: ISRPENDING Position 
SCB_ICSR_ISRPENDING_Msk            EQU        (1 << SCB_ICSR_ISRPENDING_Pos)       ; SCB ICSR: ISRPENDING Mask 

SCB_ICSR_VECTPENDING_Pos           EQU        12                                   ; SCB ICSR: VECTPENDING Position 
SCB_ICSR_VECTPENDING_Msk           EQU        (0x1FF << SCB_ICSR_VECTPENDING_Pos)  ; SCB ICSR: VECTPENDING Mask 

SCB_ICSR_RETTOBASE_Pos             EQU        11                                   ; SCB ICSR: RETTOBASE Position 
SCB_ICSR_RETTOBASE_Msk             EQU        (1 << SCB_ICSR_RETTOBASE_Pos)        ; SCB ICSR: RETTOBASE Mask 

SCB_ICSR_VECTACTIVE_Pos            EQU         0                                   ; SCB ICSR: VECTACTIVE Position 
SCB_ICSR_VECTACTIVE_Msk            EQU        (0x1FF << SCB_ICSR_VECTACTIVE_Pos)   ; SCB ICSR: VECTACTIVE Mask 

; SCB Interrupt Control State Register Definitions 
SCB_VTOR_TBLBASE_Pos               EQU        29                                   ; SCB VTOR: TBLBASE Position 
SCB_VTOR_TBLBASE_Msk               EQU        (0x1FF << SCB_VTOR_TBLBASE_Pos)      ; SCB VTOR: TBLBASE Mask 

SCB_VTOR_TBLOFF_Pos                EQU        7                                    ; SCB VTOR: TBLOFF Position 
SCB_VTOR_TBLOFF_Msk                EQU        (0x3FFFFF << SCB_VTOR_TBLOFF_Pos)    ; SCB VTOR: TBLOFF Mask 

; SCB Application Interrupt and Reset Control Register Definitions 
SCB_AIRCR_VECTKEY_Pos              EQU        16                                   ; SCB AIRCR: VECTKEY Position 
SCB_AIRCR_VECTKEY_Msk              EQU        (0xFFFF << SCB_AIRCR_VECTKEY_Pos)    ; SCB AIRCR: VECTKEY Mask 

SCB_AIRCR_VECTKEYSTAT_Pos          EQU        16                                   ; SCB AIRCR: VECTKEYSTAT Position 
SCB_AIRCR_VECTKEYSTAT_Msk          EQU        (0xFFFF << SCB_AIRCR_VECTKEYSTAT_Pos); SCB AIRCR: VECTKEYSTAT Mask 

SCB_AIRCR_ENDIANESS_Pos            EQU        15                                   ; SCB AIRCR: ENDIANESS Position 
SCB_AIRCR_ENDIANESS_Msk            EQU        (1 << SCB_AIRCR_ENDIANESS_Pos)       ; SCB AIRCR: ENDIANESS Mask 

SCB_AIRCR_PRIGROUP_Pos             EQU         8                                   ; SCB AIRCR: PRIGROUP Position 
SCB_AIRCR_PRIGROUP_Msk             EQU        (7 << SCB_AIRCR_PRIGROUP_Pos)        ; SCB AIRCR: PRIGROUP Mask 

SCB_AIRCR_SYSRESETREQ_Pos          EQU         2                                   ; SCB AIRCR: SYSRESETREQ Position 
SCB_AIRCR_SYSRESETREQ_Msk          EQU        (1 << SCB_AIRCR_SYSRESETREQ_Pos)     ; SCB AIRCR: SYSRESETREQ Mask 

SCB_AIRCR_VECTCLRACTIVE_Pos        EQU         1                                   ; SCB AIRCR: VECTCLRACTIVE Position 
SCB_AIRCR_VECTCLRACTIVE_Msk        EQU        (1 << SCB_AIRCR_VECTCLRACTIVE_Pos)   ; SCB AIRCR: VECTCLRACTIVE Mask 

SCB_AIRCR_VECTRESET_Pos            EQU         0                                     ; SCB AIRCR: VECTRESET Position 
SCB_AIRCR_VECTRESET_Msk            EQU        (1 << SCB_AIRCR_VECTRESET_Pos)       ; SCB AIRCR: VECTRESET Mask 

; SCB System Control Register Definitions 
SCB_SCR_SEVONPEND_Pos              EQU         4                                   ; SCB SCR: SEVONPEND Position 
SCB_SCR_SEVONPEND_Msk              EQU        (1 << SCB_SCR_SEVONPEND_Pos)         ; SCB SCR: SEVONPEND Mask 

SCB_SCR_SLEEPDEEP_Pos              EQU         2                                   ; SCB SCR: SLEEPDEEP Position 
SCB_SCR_SLEEPDEEP_Msk              EQU        (1 << SCB_SCR_SLEEPDEEP_Pos)         ; SCB SCR: SLEEPDEEP Mask 

SCB_SCR_SLEEPONEXIT_Pos            EQU         1                                   ; SCB SCR: SLEEPONEXIT Position 
SCB_SCR_SLEEPONEXIT_Msk            EQU        (1 << SCB_SCR_SLEEPONEXIT_Pos)       ; SCB SCR: SLEEPONEXIT Mask 

; SCB Configuration Control Register Definitions 
SCB_CCR_STKALIGN_Pos               EQU         9                                   ; SCB CCR: STKALIGN Position 
SCB_CCR_STKALIGN_Msk               EQU        (1 << SCB_CCR_STKALIGN_Pos)          ; SCB CCR: STKALIGN Mask 

SCB_CCR_BFHFNMIGN_Pos              EQU         8                                   ; SCB CCR: BFHFNMIGN Position 
SCB_CCR_BFHFNMIGN_Msk              EQU        (1 << SCB_CCR_BFHFNMIGN_Pos)         ; SCB CCR: BFHFNMIGN Mask 

SCB_CCR_DIV_0_TRP_Pos              EQU         4                                   ; SCB CCR: DIV_0_TRP Position 
SCB_CCR_DIV_0_TRP_Msk              EQU        (1 << SCB_CCR_DIV_0_TRP_Pos)         ; SCB CCR: DIV_0_TRP Mask 

SCB_CCR_UNALIGN_TRP_Pos            EQU         3                                   ; SCB CCR: UNALIGN_TRP Position 
SCB_CCR_UNALIGN_TRP_Msk            EQU        (1 << SCB_CCR_UNALIGN_TRP_Pos)       ; SCB CCR: UNALIGN_TRP Mask 

SCB_CCR_USERSETMPEND_Pos           EQU         1                                   ; SCB CCR: USERSETMPEND Position 
SCB_CCR_USERSETMPEND_Msk           EQU        (1 << SCB_CCR_USERSETMPEND_Pos)      ; SCB CCR: USERSETMPEND Mask 

SCB_CCR_NONBASETHRDENA_Pos         EQU         0                                   ; SCB CCR: NONBASETHRDENA Position 
SCB_CCR_NONBASETHRDENA_Msk         EQU        (1 << SCB_CCR_NONBASETHRDENA_Pos)    ; SCB CCR: NONBASETHRDENA Mask 

; SCB System Handler Control and State Register Definitions 
SCB_SHCSR_USGFAULTENA_Pos          EQU        18                                   ; SCB SHCSR: USGFAULTENA Position 
SCB_SHCSR_USGFAULTENA_Msk          EQU        (1 << SCB_SHCSR_USGFAULTENA_Pos)     ; SCB SHCSR: USGFAULTENA Mask 

SCB_SHCSR_BUSFAULTENA_Pos          EQU        17                                   ; SCB SHCSR: BUSFAULTENA Position 
SCB_SHCSR_BUSFAULTENA_Msk          EQU        (1 << SCB_SHCSR_BUSFAULTENA_Pos)     ; SCB SHCSR: BUSFAULTENA Mask 

SCB_SHCSR_MEMFAULTENA_Pos          EQU        16                                   ; SCB SHCSR: MEMFAULTENA Position 
SCB_SHCSR_MEMFAULTENA_Msk          EQU        (1 << SCB_SHCSR_MEMFAULTENA_Pos)     ; SCB SHCSR: MEMFAULTENA Mask 

SCB_SHCSR_SVCALLPENDED_Pos         EQU        15                                   ; SCB SHCSR: SVCALLPENDED Position 
SCB_SHCSR_SVCALLPENDED_Msk         EQU        (1 << SCB_SHCSR_SVCALLPENDED_Pos)    ; SCB SHCSR: SVCALLPENDED Mask 

SCB_SHCSR_BUSFAULTPENDED_Pos       EQU        14                                   ; SCB SHCSR: BUSFAULTPENDED Position 
SCB_SHCSR_BUSFAULTPENDED_Msk       EQU        (1 << SCB_SHCSR_BUSFAULTPENDED_Pos)  ; SCB SHCSR: BUSFAULTPENDED Mask 

SCB_SHCSR_MEMFAULTPENDED_Pos       EQU        13                                   ; SCB SHCSR: MEMFAULTPENDED Position 
SCB_SHCSR_MEMFAULTPENDED_Msk       EQU        (1 << SCB_SHCSR_MEMFAULTPENDED_Pos)  ; SCB SHCSR: MEMFAULTPENDED Mask 

SCB_SHCSR_USGFAULTPENDED_Pos       EQU        12                                   ; SCB SHCSR: USGFAULTPENDED Position 
SCB_SHCSR_USGFAULTPENDED_Msk       EQU        (1 << SCB_SHCSR_USGFAULTPENDED_Pos)  ; SCB SHCSR: USGFAULTPENDED Mask 

SCB_SHCSR_SYSTICKACT_Pos           EQU        11                                   ; SCB SHCSR: SYSTICKACT Position 
SCB_SHCSR_SYSTICKACT_Msk           EQU        (1 << SCB_SHCSR_SYSTICKACT_Pos)      ; SCB SHCSR: SYSTICKACT Mask 

SCB_SHCSR_PENDSVACT_Pos            EQU        10                                   ; SCB SHCSR: PENDSVACT Position 
SCB_SHCSR_PENDSVACT_Msk            EQU        (1 << SCB_SHCSR_PENDSVACT_Pos)       ; SCB SHCSR: PENDSVACT Mask 

SCB_SHCSR_MONITORACT_Pos           EQU         8                                   ; SCB SHCSR: MONITORACT Position 
SCB_SHCSR_MONITORACT_Msk           EQU        (1 << SCB_SHCSR_MONITORACT_Pos)      ; SCB SHCSR: MONITORACT Mask 

SCB_SHCSR_SVCALLACT_Pos            EQU         7                                   ; SCB SHCSR: SVCALLACT Position 
SCB_SHCSR_SVCALLACT_Msk            EQU        (1 << SCB_SHCSR_SVCALLACT_Pos)       ; SCB SHCSR: SVCALLACT Mask 
                                     
SCB_SHCSR_USGFAULTACT_Pos          EQU         3                                   ; SCB SHCSR: USGFAULTACT Position 
SCB_SHCSR_USGFAULTACT_Msk          EQU        (1 << SCB_SHCSR_USGFAULTACT_Pos)     ; SCB SHCSR: USGFAULTACT Mask 

SCB_SHCSR_BUSFAULTACT_Pos          EQU         1                                   ; SCB SHCSR: BUSFAULTACT Position 
SCB_SHCSR_BUSFAULTACT_Msk          EQU        (1 << SCB_SHCSR_BUSFAULTACT_Pos)     ; SCB SHCSR: BUSFAULTACT Mask 

SCB_SHCSR_MEMFAULTACT_Pos          EQU         0                                   ; SCB SHCSR: MEMFAULTACT Position 
SCB_SHCSR_MEMFAULTACT_Msk          EQU        (1 << SCB_SHCSR_MEMFAULTACT_Pos)     ; SCB SHCSR: MEMFAULTACT Mask 

; SCB Configurable Fat Status Registers Definitions 
SCB_CFSR_USGFAULTSR_Pos            EQU        16                                   ; SCB CFSR: Usage Fault Status Register Position 
SCB_CFSR_USGFAULTSR_Msk            EQU        (0xFFFF << SCB_CFSR_USGFAULTSR_Pos)  ; SCB CFSR: Usage Fault Status Register Mask 

SCB_CFSR_BUSFAULTSR_Pos            EQU         8                                   ; SCB CFSR: Bus Fault Status Register Position 
SCB_CFSR_BUSFAULTSR_Msk            EQU        (0xFF << SCB_CFSR_BUSFAULTSR_Pos)    ; SCB CFSR: Bus Fault Status Register Mask 

SCB_CFSR_MEMFAULTSR_Pos            EQU         0                                   ; SCB CFSR: Memory Manage Fault Status Register Position 
SCB_CFSR_MEMFAULTSR_Msk            EQU        (0xFF << SCB_CFSR_MEMFAULTSR_Pos)    ; SCB CFSR: Memory Manage Fault Status Register Mask 

; SCB Hard Fault Status Registers Definitions 
SCB_HFSR_DEBUGEVT_Pos              EQU        31                                   ; SCB HFSR: DEBUGEVT Position 
SCB_HFSR_DEBUGEVT_Msk              EQU        (1 << SCB_HFSR_DEBUGEVT_Pos)         ; SCB HFSR: DEBUGEVT Mask 

SCB_HFSR_FORCED_Pos                EQU        30                                   ; SCB HFSR: FORCED Position 
SCB_HFSR_FORCED_Msk                EQU        (1 << SCB_HFSR_FORCED_Pos)           ; SCB HFSR: FORCED Mask 

SCB_HFSR_VECTTBL_Pos               EQU         1                                   ; SCB HFSR: VECTTBL Position 
SCB_HFSR_VECTTBL_Msk               EQU        (1 << SCB_HFSR_VECTTBL_Pos)          ; SCB HFSR: VECTTBL Mask 

; SCB Debug Fault Status Register Definitions 
SCB_DFSR_EXTERNAL_Pos              EQU         4                                   ; SCB DFSR: EXTERNAL Position 
SCB_DFSR_EXTERNAL_Msk              EQU        (1 << SCB_DFSR_EXTERNAL_Pos)         ; SCB DFSR: EXTERNAL Mask 

SCB_DFSR_VCATCH_Pos                EQU         3                                   ; SCB DFSR: VCATCH Position 
SCB_DFSR_VCATCH_Msk                EQU        (1 << SCB_DFSR_VCATCH_Pos)           ; SCB DFSR: VCATCH Mask 

SCB_DFSR_DWTTRAP_Pos               EQU         2                                   ; SCB DFSR: DWTTRAP Position 
SCB_DFSR_DWTTRAP_Msk               EQU        (1 << SCB_DFSR_DWTTRAP_Pos)          ; SCB DFSR: DWTTRAP Mask 

SCB_DFSR_BKPT_Pos                  EQU         1                                   ; SCB DFSR: BKPT Position 
SCB_DFSR_BKPT_Msk                  EQU        (1 << SCB_DFSR_BKPT_Pos)             ; SCB DFSR: BKPT Mask 

SCB_DFSR_HALTED_Pos                EQU         0                                   ; SCB DFSR: HALTED Position 
SCB_DFSR_HALTED_Msk                EQU        (1 << SCB_DFSR_HALTED_Pos)           ; SCB DFSR: HALTED Mask 
;  end of group CMSIS_CM3_SCB 


; addtogroup CMSIS_CM3_SysTick CMSIS CM3 SysTick
; SysTick_CTRL    EQU 0x00            ; Offset: 0x00  SysTick Control and Status Register 
; SysTick_LOAD    EQU 0x04            ; Offset: 0x04  SysTick Reload Value Register       
; SysTick_VAL        EQU 0x08            ; Offset: 0x08  SysTick Current Value Register      
; SysTick_CALIB    EQU 0x0C            ; Offset: 0x0C  SysTick Calibration Register        

;  SysTick Control / Status Register Definitions 
SysTick_CTRL_COUNTFLAG_Pos         EQU        16                                       ; SysTick CTRL: COUNTFLAG Position 
SysTick_CTRL_COUNTFLAG_Msk         EQU        (1 << SysTick_CTRL_COUNTFLAG_Pos)        ; SysTick CTRL: COUNTFLAG Mask 

SysTick_CTRL_CLKSOURCE_Pos         EQU         2                                       ; SysTick CTRL: CLKSOURCE Position 
SysTick_CTRL_CLKSOURCE_Msk         EQU        (1 << SysTick_CTRL_CLKSOURCE_Pos)        ; SysTick CTRL: CLKSOURCE Mask 

SysTick_CTRL_TICKINT_Pos           EQU         1                                       ; SysTick CTRL: TICKINT Position 
SysTick_CTRL_TICKINT_Msk           EQU        (1 << SysTick_CTRL_TICKINT_Pos)          ; SysTick CTRL: TICKINT Mask 

SysTick_CTRL_ENABLE_Pos            EQU         0                                       ; SysTick CTRL: ENABLE Position 
SysTick_CTRL_ENABLE_Msk            EQU        (1 << SysTick_CTRL_ENABLE_Pos)           ; SysTick CTRL: ENABLE Mask 

;  SysTick Reload Register Definitions 
SysTick_LOAD_RELOAD_Pos            EQU         0                                       ; SysTick LOAD: RELOAD Position 
SysTick_LOAD_RELOAD_Msk            EQU        (0xFFFFFF << SysTick_LOAD_RELOAD_Pos)    ; SysTick LOAD: RELOAD Mask 

;  SysTick Current Register Definitions 
SysTick_VAL_CURRENT_Pos            EQU         0                                       ; SysTick VAL: CURRENT Position 
SysTick_VAL_CURRENT_Msk            EQU        (0xFFFFFF << SysTick_VAL_CURRENT_Pos)    ; SysTick VAL: CURRENT Mask 

;  SysTick Calibration Register Definitions 
SysTick_CALIB_NOREF_Pos            EQU         31                                      ; SysTick CALIB: NOREF Position 
SysTick_CALIB_NOREF_Msk            EQU        (1 << SysTick_CALIB_NOREF_Pos)           ; SysTick CALIB: NOREF Mask 

SysTick_CALIB_SKEW_Pos             EQU        30                                       ; SysTick CALIB: SKEW Position 
SysTick_CALIB_SKEW_Msk             EQU        (1 << SysTick_CALIB_SKEW_Pos)            ; SysTick CALIB: SKEW Mask 

SysTick_CALIB_TENMS_Pos            EQU         0                                       ; SysTick CALIB: TENMS Position 
SysTick_CALIB_TENMS_Msk            EQU        (0xFFFFFF << SysTick_VAL_CURRENT_Pos)    ; SysTick CALIB: TENMS Mask 
;  end of group CMSIS_CM3_SysTick 


; addtogroup CMSIS_CM3_ITM CMSIS CM3 ITM
;  memory mapped structure for Instrumentation Trace Macrocell (ITM)
;
;  __O  union  
;  {
;    __O  uint8_t    u8;                   ; Offset:       ITM Stimulus Port 8-bit                   
;    __O  uint16_t   u16;                  ; Offset:       ITM Stimulus Port 16-bit                  
;    __O  uint32_t   u32;                  ; Offset:       ITM Stimulus Port 32-bit                  
;  }  PORT [32];                           ; Offset: 0x00  ITM Stimulus Port Registers               
;       uint32_t RESERVED0[864];                                 
;  __IO uint32_t TER;                      ; Offset:       ITM Trace Enable Register                 
;       uint32_t RESERVED1[15];                                  
;  __IO uint32_t TPR;                      ; Offset:       ITM Trace Privilege Register              
;       uint32_t RESERVED2[15];                                  
;  __IO uint32_t TCR;                      ; Offset:       ITM Trace Control Register                
;       uint32_t RESERVED3[29];                                  
;  __IO uint32_t IWR;                      ; Offset:       ITM Integration Write Register            
;  __IO uint32_t IRR;                      ; Offset:       ITM Integration Read Register             
;  __IO uint32_t IMCR;                     ; Offset:       ITM Integration Mode Control Register     
;       uint32_t RESERVED4[43];                                  
;  __IO uint32_t LAR;                      ; Offset:       ITM Lock Access Register                  
;  __IO uint32_t LSR;                      ; Offset:       ITM Lock Status Register                  
;       uint32_t RESERVED5[6];                                   
;  __I  uint32_t PID4;                     ; Offset:       ITM Peripheral Identification Register #4 
;  __I  uint32_t PID5;                     ; Offset:       ITM Peripheral Identification Register #5 
;  __I  uint32_t PID6;                     ; Offset:       ITM Peripheral Identification Register #6 
;  __I  uint32_t PID7;                     ; Offset:       ITM Peripheral Identification Register #7 
;  __I  uint32_t PID0;                     ; Offset:       ITM Peripheral Identification Register #0 
;  __I  uint32_t PID1;                     ; Offset:       ITM Peripheral Identification Register #1 
;  __I  uint32_t PID2;                     ; Offset:       ITM Peripheral Identification Register #2 
;  __I  uint32_t PID3;                     ; Offset:       ITM Peripheral Identification Register #3 
;  __I  uint32_t CID0;                     ; Offset:       ITM Component  Identification Register #0 
;  __I  uint32_t CID1;                     ; Offset:       ITM Component  Identification Register #1 
;  __I  uint32_t CID2;                     ; Offset:       ITM Component  Identification Register #2 
;  __I  uint32_t CID3;                     ; Offset:       ITM Component  Identification Register #3 
; } ITM_Type;     
                                           

;  ITM Trace Privilege Register Definitions 
ITM_TPR_PRIVMASK_Pos               EQU         0                                       ; ITM TPR: PRIVMASK Position 
ITM_TPR_PRIVMASK_Msk               EQU        (0xF << ITM_TPR_PRIVMASK_Pos)            ; ITM TPR: PRIVMASK Mask 

;  ITM Trace Control Register Definitions 
ITM_TCR_BUSY_Pos                   EQU        23                                       ; ITM TCR: BUSY Position 
ITM_TCR_BUSY_Msk                   EQU        (1 << ITM_TCR_BUSY_Pos)                  ; ITM TCR: BUSY Mask 

ITM_TCR_ATBID_Pos                  EQU        16                                       ; ITM TCR: ATBID Position 
ITM_TCR_ATBID_Msk                  EQU        (0x7F << ITM_TCR_ATBID_Pos)              ; ITM TCR: ATBID Mask 

ITM_TCR_TSPrescale_Pos             EQU         8                                       ; ITM TCR: TSPrescale Position 
ITM_TCR_TSPrescale_Msk             EQU        (3 << ITM_TCR_TSPrescale_Pos)            ; ITM TCR: TSPrescale Mask 

ITM_TCR_SWOENA_Pos                 EQU         4                                       ; ITM TCR: SWOENA Position 
ITM_TCR_SWOENA_Msk                 EQU        (1 << ITM_TCR_SWOENA_Pos)                ; ITM TCR: SWOENA Mask 

ITM_TCR_DWTENA_Pos                 EQU         3                                       ; ITM TCR: DWTENA Position 
ITM_TCR_DWTENA_Msk                 EQU        (1 << ITM_TCR_DWTENA_Pos)                ; ITM TCR: DWTENA Mask 

ITM_TCR_SYNCENA_Pos                EQU         2                                       ; ITM TCR: SYNCENA Position 
ITM_TCR_SYNCENA_Msk                EQU        (1 << ITM_TCR_SYNCENA_Pos)               ; ITM TCR: SYNCENA Mask 

ITM_TCR_TSENA_Pos                  EQU         1                                       ; ITM TCR: TSENA Position 
ITM_TCR_TSENA_Msk                  EQU        (1 << ITM_TCR_TSENA_Pos)                 ; ITM TCR: TSENA Mask 

ITM_TCR_ITMENA_Pos                 EQU         0                                       ; ITM TCR: ITM Enable bit Position 
ITM_TCR_ITMENA_Msk                 EQU        (1 << ITM_TCR_ITMENA_Pos)                ; ITM TCR: ITM Enable bit Mask 

;  ITM Integration Write Register Definitions 
ITM_IWR_ATVALIDM_Pos               EQU         0                                       ; ITM IWR: ATVALIDM Position 
ITM_IWR_ATVALIDM_Msk               EQU        (1 << ITM_IWR_ATVALIDM_Pos)              ; ITM IWR: ATVALIDM Mask 

;  ITM Integration Read Register Definitions 
ITM_IRR_ATREADYM_Pos               EQU         0                                       ; ITM IRR: ATREADYM Position 
ITM_IRR_ATREADYM_Msk               EQU        (1 << ITM_IRR_ATREADYM_Pos)              ; ITM IRR: ATREADYM Mask 

;  ITM Integration Mode Control Register Definitions 
ITM_IMCR_INTEGRATION_Pos           EQU         0                                       ; ITM IMCR: INTEGRATION Position 
ITM_IMCR_INTEGRATION_Msk           EQU        (1 << ITM_IMCR_INTEGRATION_Pos)          ; ITM IMCR: INTEGRATION Mask 

;  ITM Lock Status Register Definitions 
ITM_LSR_ByteAcc_Pos                EQU         2                                       ; ITM LSR: ByteAcc Position 
ITM_LSR_ByteAcc_Msk                EQU        (1 << ITM_LSR_ByteAcc_Pos)               ; ITM LSR: ByteAcc Mask 

ITM_LSR_Access_Pos                 EQU         1                                       ; ITM LSR: Access Position 
ITM_LSR_Access_Msk                 EQU        (1 << ITM_LSR_Access_Pos)                ; ITM LSR: Access Mask 

ITM_LSR_Present_Pos                EQU         0                                       ; ITM LSR: Present Position 
ITM_LSR_Present_Msk                EQU        (1 << ITM_LSR_Present_Pos)               ; ITM LSR: Present Mask 


; addtogroup CMSIS_CM3_InterruptType CMSIS CM3 Interrupt Type
InterruptType_ICTR                 EQU        0x04        ; Offset: 0x04  Interrupt Control Type Register 
InterruptType_ACTLR                EQU        0x08        ; Offset: 0x08  Auxiliary Control Register      


;  Interrupt Controller Type Register Definitions 
InterruptType_ICTR_INTLINESNUM_Pos    EQU         0                                         ; InterruptType ICTR: INTLINESNUM Position 
InterruptType_ICTR_INTLINESNUM_Msk    EQU        (0x1F << InterruptType_ICTR_INTLINESNUM_Pos) ; InterruptType ICTR: INTLINESNUM Mask 

;  Auxiliary Control Register Definitions 
InterruptType_ACTLR_DISFOLD_Pos        EQU        2                                         ; InterruptType ACTLR: DISFOLD Position 
InterruptType_ACTLR_DISFOLD_Msk        EQU        (1 << InterruptType_ACTLR_DISFOLD_Pos)    ; InterruptType ACTLR: DISFOLD Mask 

InterruptType_ACTLR_DISDEFWBUF_Pos     EQU        1                                         ; InterruptType ACTLR: DISDEFWBUF Position 
InterruptType_ACTLR_DISDEFWBUF_Msk     EQU        (1 << InterruptType_ACTLR_DISDEFWBUF_Pos) ; InterruptType ACTLR: DISDEFWBUF Mask 

InterruptType_ACTLR_DISMCYCINT_Pos     EQU        0                                         ; InterruptType ACTLR: DISMCYCINT Position 
InterruptType_ACTLR_DISMCYCINT_Msk     EQU        (1 << InterruptType_ACTLR_DISMCYCINT_Pos) ; InterruptType ACTLR: DISMCYCINT Mask 
;  end of group CMSIS_CM3_InterruptType 


MPUTYPE         EQU     0x00 ; Offset: 0x00  MPU Type Register                              
MPU_CTRL        EQU     0x04 ; Offset: 0x04  MPU Control Register                           
MPU_RNR         EQU     0x08 ; Offset: 0x08  MPU Region RNRber Register                     
MPU_RBAR        EQU     0x0C ; Offset: 0x0C  MPU Region Base Address Register               
MPU_RASR        EQU     0x10 ; Offset: 0x10  MPU Region Attribute and Size Register         
MPU_RBAR_A1     EQU     0x14 ; Offset: 0x14  MPU Alias 1 Region Base Address Register       
MPU_RASR_A1     EQU     0x18 ; Offset: 0x18  MPU Alias 1 Region Attribute and Size Register 
MPU_RBAR_A2     EQU     0x1C ; Offset: 0x1C  MPU Alias 2 Region Base Address Register       
MPU_RASR_A2     EQU     0x20 ; Offset: 0x20  MPU Alias 2 Region Attribute and Size Register 
MPU_RBAR_A3     EQU     0x24 ; Offset: 0x24  MPU Alias 3 Region Base Address Register       
MPU_RASR_A3     EQU     0x28 ; Offset: 0x28  MPU Alias 3 Region Attribute and Size Register 

; MPU Type Register 
MPU_TYPE_IREGION_Pos              EQU         16                                       ; MPU TYPE: IREGION Position 
MPU_TYPE_IREGION_Msk              EQU         (0xFF << MPU_TYPE_IREGION_Pos)           ; MPU TYPE: IREGION Mask 

MPU_TYPE_DREGION_Pos              EQU          8                                       ; MPU TYPE: DREGION Position 
MPU_TYPE_DREGION_Msk              EQU         (0xFF << MPU_TYPE_DREGION_Pos)           ; MPU TYPE: DREGION Mask 

MPU_TYPE_SEPARATE_Pos             EQU          0                                       ; MPU TYPE: SEPARATE Position 
MPU_TYPE_SEPARATE_Msk             EQU         (1 << MPU_TYPE_SEPARATE_Pos)             ; MPU TYPE: SEPARATE Mask 

; MPU Control Register 
MPU_CTRL_PRIVDEFENA_Pos           EQU          2                                       ; MPU CTRL: PRIVDEFENA Position 
MPU_CTRL_PRIVDEFENA_Msk           EQU         (1 << MPU_CTRL_PRIVDEFENA_Pos)           ; MPU CTRL: PRIVDEFENA Mask 

MPU_CTRL_HFNMIENA_Pos             EQU         1                                        ; MPU CTRL: HFNMIENA Position 
MPU_CTRL_HFNMIENA_Msk             EQU         (1 << MPU_CTRL_HFNMIENA_Pos)             ; MPU CTRL: HFNMIENA Mask 

MPU_CTRL_ENABLE_Pos               EQU          0                                       ; MPU CTRL: ENABLE Position 
MPU_CTRL_ENABLE_Msk               EQU         (1 << MPU_CTRL_ENABLE_Pos)               ; MPU CTRL: ENABLE Mask 

; MPU Region Number Register 
MPU_RNR_REGION_Pos                EQU          0                                       ; MPU RNR: REGION Position 
MPU_RNR_REGION_Msk                EQU         (0xFF << MPU_RNR_REGION_Pos)             ; MPU RNR: REGION Mask 

; MPU Region Base Address Register 
MPU_RBAR_ADDR_Pos                 EQU          5                                       ; MPU RBAR: ADDR Position 
MPU_RBAR_ADDR_Msk                 EQU         (0x7FFFFFF << MPU_RBAR_ADDR_Pos)         ; MPU RBAR: ADDR Mask 

MPU_RBAR_VALID_Pos                EQU          4                                       ; MPU RBAR: VALID Position 
MPU_RBAR_VALID_Msk                EQU         (1 << MPU_RBAR_VALID_Pos)                ; MPU RBAR: VALID Mask 

MPU_RBAR_REGION_Pos               EQU          0                                       ; MPU RBAR: REGION Position 
MPU_RBAR_REGION_Msk               EQU         (0xF << MPU_RBAR_REGION_Pos)             ; MPU RBAR: REGION Mask 

; MPU Region Attribute and Size Register 
MPU_RASR_XN_Pos                   EQU         28                                       ; MPU RASR: XN Position 
MPU_RASR_XN_Msk                   EQU         (1 << MPU_RASR_XN_Pos)                   ; MPU RASR: XN Mask 

MPU_RASR_AP_Pos                   EQU         24                                       ; MPU RASR: AP Position 
MPU_RASR_AP_Msk                   EQU         (7 << MPU_RASR_AP_Pos)                   ; MPU RASR: AP Mask 

MPU_RASR_TEX_Pos                  EQU         19                                       ; MPU RASR: TEX Position 
MPU_RASR_TEX_Msk                  EQU         (7 << MPU_RASR_TEX_Pos)                  ; MPU RASR: TEX Mask 

MPU_RASR_S_Pos                    EQU         18                                       ; MPU RASR: Shareable bit Position 
MPU_RASR_S_Msk                    EQU         (1 << MPU_RASR_S_Pos)                    ; MPU RASR: Shareable bit Mask 

MPU_RASR_C_Pos                    EQU         17                                       ; MPU RASR: Cacheable bit Position 
MPU_RASR_C_Msk                    EQU         (1 << MPU_RASR_C_Pos)                    ; MPU RASR: Cacheable bit Mask 

MPU_RASR_B_Pos                    EQU         16                                       ; MPU RASR: Bufferable bit Position 
MPU_RASR_B_Msk                    EQU         (1 << MPU_RASR_B_Pos)                    ; MPU RASR: Bufferable bit Mask 

MPU_RASR_SRD_Pos                  EQU          8                                       ; MPU RASR: Sub-Region Disable Position 
MPU_RASR_SRD_Msk                  EQU         (0xFF << MPU_RASR_SRD_Pos)               ; MPU RASR: Sub-Region Disable Mask 

MPU_RASR_SIZE_Pos                 EQU          1                                       ; MPU RASR: Region Size Field Position 
MPU_RASR_SIZE_Msk                 EQU         (0x1F << MPU_RASR_SIZE_Pos)              ; MPU RASR: Region Size Field Mask 

MPU_RASR_ENA_Pos                  EQU           0                                      ; MPU RASR: Region enable bit Position 
MPU_RASR_ENA_Msk                  EQU          (0x1F << MPU_RASR_ENA_Pos)              ; MPU RASR: Region enable bit Disable Mask 


; addtogroup CMSIS_CM3_CoreDebug CMSIS CM3 Core Debug
CoreDebug_DHCSR        EQU    0x00    ; Offset: 0x00  Debug Halting Control and Status Register    
CoreDebug_DCRSR        EQU    0x04    ; Offset: 0x04  Debug Core Register Selector Register        
CoreDebug_DCRDR        EQU    0x08    ; Offset: 0x08  Debug Core Register Data Register            
CoreDebug_DEMCR        EQU    0x0C    ; Offset: 0x0C  Debug Exception and Monitor Control Register 

;  Debug Halting Control and Status Register 
CoreDebug_DHCSR_DBGKEY_Pos         EQU        16                                       ; CoreDebug DHCSR: DBGKEY Position 
CoreDebug_DHCSR_DBGKEY_Msk         EQU        (0xFFFF << CoreDebug_DHCSR_DBGKEY_Pos)   ; CoreDebug DHCSR: DBGKEY Mask 

CoreDebug_DHCSR_S_RESET_ST_Pos     EQU        25                                       ; CoreDebug DHCSR: S_RESET_ST Position 
CoreDebug_DHCSR_S_RESET_ST_Msk     EQU        (1 << CoreDebug_DHCSR_S_RESET_ST_Pos)    ; CoreDebug DHCSR: S_RESET_ST Mask 

CoreDebug_DHCSR_S_RETIRE_ST_Pos    EQU        24                                       ; CoreDebug DHCSR: S_RETIRE_ST Position 
CoreDebug_DHCSR_S_RETIRE_ST_Msk    EQU        (1 << CoreDebug_DHCSR_S_RETIRE_ST_Pos)   ; CoreDebug DHCSR: S_RETIRE_ST Mask 

CoreDebug_DHCSR_S_LOCKUP_Pos       EQU        19                                       ; CoreDebug DHCSR: S_LOCKUP Position 
CoreDebug_DHCSR_S_LOCKUP_Msk       EQU        (1 << CoreDebug_DHCSR_S_LOCKUP_Pos)      ; CoreDebug DHCSR: S_LOCKUP Mask 

CoreDebug_DHCSR_S_SLEEP_Pos        EQU        18                                       ; CoreDebug DHCSR: S_SLEEP Position 
CoreDebug_DHCSR_S_SLEEP_Msk        EQU        (1 << CoreDebug_DHCSR_S_SLEEP_Pos)       ; CoreDebug DHCSR: S_SLEEP Mask 

CoreDebug_DHCSR_S_HALT_Pos         EQU        17                                       ; CoreDebug DHCSR: S_HALT Position 
CoreDebug_DHCSR_S_HALT_Msk         EQU        (1 << CoreDebug_DHCSR_S_HALT_Pos)        ; CoreDebug DHCSR: S_HALT Mask 

CoreDebug_DHCSR_S_REGRDY_Pos       EQU        16                                       ; CoreDebug DHCSR: S_REGRDY Position 
CoreDebug_DHCSR_S_REGRDY_Msk       EQU        (1 << CoreDebug_DHCSR_S_REGRDY_Pos)      ; CoreDebug DHCSR: S_REGRDY Mask 

CoreDebug_DHCSR_C_SNAPSTALL_Pos    EQU         5                                       ; CoreDebug DHCSR: C_SNAPSTALL Position 
CoreDebug_DHCSR_C_SNAPSTALL_Msk    EQU        (1 << CoreDebug_DHCSR_C_SNAPSTALL_Pos)   ; CoreDebug DHCSR: C_SNAPSTALL Mask 

CoreDebug_DHCSR_C_MASKINTS_Pos     EQU         3                                       ; CoreDebug DHCSR: C_MASKINTS Position 
CoreDebug_DHCSR_C_MASKINTS_Msk     EQU        (1 << CoreDebug_DHCSR_C_MASKINTS_Pos)    ; CoreDebug DHCSR: C_MASKINTS Mask 

CoreDebug_DHCSR_C_STEP_Pos         EQU         2                                       ; CoreDebug DHCSR: C_STEP Position 
CoreDebug_DHCSR_C_STEP_Msk         EQU        (1 << CoreDebug_DHCSR_C_STEP_Pos)        ; CoreDebug DHCSR: C_STEP Mask 

CoreDebug_DHCSR_C_HALT_Pos         EQU         1                                       ; CoreDebug DHCSR: C_HALT Position 
CoreDebug_DHCSR_C_HALT_Msk         EQU        (1 << CoreDebug_DHCSR_C_HALT_Pos)        ; CoreDebug DHCSR: C_HALT Mask 

CoreDebug_DHCSR_C_DEBUGEN_Pos      EQU         0                                       ; CoreDebug DHCSR: C_DEBUGEN Position 
CoreDebug_DHCSR_C_DEBUGEN_Msk      EQU        (1 << CoreDebug_DHCSR_C_DEBUGEN_Pos)     ; CoreDebug DHCSR: C_DEBUGEN Mask 

;  Debug Core Register Selector Register 
CoreDebug_DCRSR_REGWnR_Pos         EQU        16                                       ; CoreDebug DCRSR: REGWnR Position 
CoreDebug_DCRSR_REGWnR_Msk         EQU        (1 << CoreDebug_DCRSR_REGWnR_Pos)        ; CoreDebug DCRSR: REGWnR Mask 

CoreDebug_DCRSR_REGSEL_Pos         EQU         0                                       ; CoreDebug DCRSR: REGSEL Position 
CoreDebug_DCRSR_REGSEL_Msk         EQU        (0x1F << CoreDebug_DCRSR_REGSEL_Pos)     ; CoreDebug DCRSR: REGSEL Mask 

;  Debug Exception and Monitor Control Register 
CoreDebug_DEMCR_TRCENA_Pos         EQU        24                                       ; CoreDebug DEMCR: TRCENA Position 
CoreDebug_DEMCR_TRCENA_Msk         EQU        (1 << CoreDebug_DEMCR_TRCENA_Pos)        ; CoreDebug DEMCR: TRCENA Mask 

CoreDebug_DEMCR_MON_REQ_Pos        EQU        19                                       ; CoreDebug DEMCR: MON_REQ Position 
CoreDebug_DEMCR_MON_REQ_Msk        EQU        (1 << CoreDebug_DEMCR_MON_REQ_Pos)       ; CoreDebug DEMCR: MON_REQ Mask 

CoreDebug_DEMCR_MON_STEP_Pos       EQU        18                                       ; CoreDebug DEMCR: MON_STEP Position 
CoreDebug_DEMCR_MON_STEP_Msk       EQU        (1 << CoreDebug_DEMCR_MON_STEP_Pos)      ; CoreDebug DEMCR: MON_STEP Mask 

CoreDebug_DEMCR_MON_PEND_Pos       EQU        17                                       ; CoreDebug DEMCR: MON_PEND Position 
CoreDebug_DEMCR_MON_PEND_Msk       EQU        (1 << CoreDebug_DEMCR_MON_PEND_Pos)      ; CoreDebug DEMCR: MON_PEND Mask 

CoreDebug_DEMCR_MON_EN_Pos         EQU        16                                       ; CoreDebug DEMCR: MON_EN Position 
CoreDebug_DEMCR_MON_EN_Msk         EQU        (1 << CoreDebug_DEMCR_MON_EN_Pos)        ; CoreDebug DEMCR: MON_EN Mask 

CoreDebug_DEMCR_VC_HARDERR_Pos     EQU        10                                       ; CoreDebug DEMCR: VC_HARDERR Position 
CoreDebug_DEMCR_VC_HARDERR_Msk     EQU        (1 << CoreDebug_DEMCR_VC_HARDERR_Pos)    ; CoreDebug DEMCR: VC_HARDERR Mask 

CoreDebug_DEMCR_VC_INTERR_Pos      EQU         9                                       ; CoreDebug DEMCR: VC_INTERR Position 
CoreDebug_DEMCR_VC_INTERR_Msk      EQU        (1 << CoreDebug_DEMCR_VC_INTERR_Pos)     ; CoreDebug DEMCR: VC_INTERR Mask 

CoreDebug_DEMCR_VC_BUSERR_Pos      EQU         8                                       ; CoreDebug DEMCR: VC_BUSERR Position 
CoreDebug_DEMCR_VC_BUSERR_Msk      EQU        (1 << CoreDebug_DEMCR_VC_BUSERR_Pos)     ; CoreDebug DEMCR: VC_BUSERR Mask 

CoreDebug_DEMCR_VC_STATERR_Pos     EQU         7                                       ; CoreDebug DEMCR: VC_STATERR Position 
CoreDebug_DEMCR_VC_STATERR_Msk     EQU        (1 << CoreDebug_DEMCR_VC_STATERR_Pos)    ; CoreDebug DEMCR: VC_STATERR Mask 

CoreDebug_DEMCR_VC_CHKERR_Pos      EQU         6                                       ; CoreDebug DEMCR: VC_CHKERR Position 
CoreDebug_DEMCR_VC_CHKERR_Msk      EQU        (1 << CoreDebug_DEMCR_VC_CHKERR_Pos)     ; CoreDebug DEMCR: VC_CHKERR Mask 

CoreDebug_DEMCR_VC_NOCPERR_Pos     EQU         5                                       ; CoreDebug DEMCR: VC_NOCPERR Position 
CoreDebug_DEMCR_VC_NOCPERR_Msk     EQU        (1 << CoreDebug_DEMCR_VC_NOCPERR_Pos)    ; CoreDebug DEMCR: VC_NOCPERR Mask 

CoreDebug_DEMCR_VC_MMERR_Pos       EQU         4                                       ; CoreDebug DEMCR: VC_MMERR Position 
CoreDebug_DEMCR_VC_MMERR_Msk       EQU        (1 << CoreDebug_DEMCR_VC_MMERR_Pos)      ; CoreDebug DEMCR: VC_MMERR Mask 

CoreDebug_DEMCR_VC_CORERESET_Pos   EQU         0                                       ; CoreDebug DEMCR: VC_CORERESET Position 
CoreDebug_DEMCR_VC_CORERESET_Msk   EQU        (1 << CoreDebug_DEMCR_VC_CORERESET_Pos)  ; CoreDebug DEMCR: VC_CORERESET Mask 


;  Memory mapping of Cortex-M3 Hardware 
; SCS_BASE            EQU        (0xE000E000)                          ; System Control Space Base Address 
; ITM_BASE            EQU        (0xE0000000)                          ; ITM Base Address                  
; CoreDebug_BASE      EQU        (0xE000EDF0)                          ; Core Debug Base Address           
; SysTick_BASE        EQU        (SCS_BASE +  0x0010)                  ; SysTick Base Address              
; NVIC_BASE           EQU        (SCS_BASE +  0x0100)                  ; NVIC Base Address                 
; SCB_BASE            EQU        (SCS_BASE +  0x0D00)                  ; System Control Block Base Address 

; InterruptType       EQU        (SCS_BASE)     ; Interrupt Type Register           
; SCB                 EQU        (SCB_BASE)     ; SCB configuration struct          
; SysTick             EQU        (SysTick_BASE) ; SysTick configuration struct      
; NVIC                EQU        (NVIC_BASE)    ; NVIC configuration struct         
; ITM                 EQU        (ITM_BASE)     ; ITM configuration struct          
; CoreDebug           EQU        (CoreDebug_BASE)   ; Core Debug configuration struct   

; MPU_BASE            EQU        (SCS_BASE +  0x0D90)                  ; Memory Protection Unit            
; MPU                    EQU        (MPU_BASE)     ; Memory Protection Unit            



                    END