;******************** (C) Yifeng ZHU ******************************************************************
; @file    stm32l1xx_tim_constants.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    April-22-2012
; @note    Modifed from stm32l1xx_tim.h (C) 2010 STMicroelectronics
; @brief   Assembly version of TIM firmware library
;******************************************************************************************************




; TIM Time Base Init structure definition
; This structure is used with all TIMx except for TIM6 and TIM7.    

TIM_TimeBase_Prescaler      EQU   0x00          ; Specifies the prescaler value used to divide the TIM clock.
                                                ; This parameter can be a number between 0x0000 and 0xFFFF 
TIM_TimeBase_CounterMode    EQU   0x02          ; Specifies the counter mode.
                                                ; This parameter can be a value of @ref TIM_Counter_Mode 
TIM_TimeBase_Period         EQU   0x04          ; Specifies the period value to be loaded into the active
                                                ; Auto-Reload Register at the next update event.
                                                ; This parameter must be a number between 0x0000 and 0xFFFF.   
TIM_TimeBase_ClockDivision  EQU   0x06          ; Specifies the clock division.
                                                ; This parameter can be a value of @ref TIM_Clock_Division_CKD 


; TIM Output Compare Init structure definition  
TIM_OC_OCMode               EQU   0x00          ; Specifies the TIM mode.
                                                ; This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes 
TIM_OC_OutputState          EQU   0x02          ; Specifies the TIM Output Compare state.
                                                ; This parameter can be a value of @ref TIM_Output_Compare_state 
TIM_OC_Pulse                EQU   0x04          ; Specifies the pulse value to be loaded into the Capture Compare Register. 
                                                ; This parameter can be a number between 0x0000 and 0xFFFF 
TIM_OC_OCPolarity           EQU   0x06          ; Specifies the output polarity.
                                                ; This parameter can be a value of @ref TIM_Output_Compare_Polarity 


; TIM Input Capture Init structure definition  
TIM_IC_Channel              EQU   0x00          ; Specifies the TIM channel.
                                                ; This parameter can be a value of @ref TIM_Channel 
TIM_IC_ICPolarity           EQU   0x02          ; Specifies the active edge of the input signal.
                                                ; This parameter can be a value of @ref TIM_Input_Capture_Polarity 
TIM_IC_ICSelection          EQU   0x04          ; Specifies the input.
                                                ; This parameter can be a value of @ref TIM_Input_Capture_Selection 
TIM_IC_ICPrescaler          EQU   0x06          ; Specifies the Input Capture Prescaler.
                                                ; This parameter can be a value of @ref TIM_Input_Capture_Prescaler 
TIM_IC_ICFilte              EQU   0x08          ; Specifies the input capture filter.
                                                ; This parameter can be a number between 0x0 and 0xF 

; TIM_Output_Compare_and_PWM_modes 
TIM_OCMode_Timing                  EQU    (0x0000)
TIM_OCMode_Active                  EQU    (0x0010)
TIM_OCMode_Inactive                EQU    (0x0020)
TIM_OCMode_Toggle                  EQU    (0x0030)
TIM_OCMode_PWM1                    EQU    (0x0060)
TIM_OCMode_PWM2                    EQU    (0x0070)


; TIM_One_Pulse_Mode 
TIM_OPMode_Single                  EQU    (0x0008)
TIM_OPMode_Repetitive              EQU    (0x0000)


; TIM_Channel 
TIM_Channel_1                      EQU    (0x0000)
TIM_Channel_2                      EQU    (0x0004)
TIM_Channel_3                      EQU    (0x0008)
TIM_Channel_4                      EQU    (0x000C)


; TIM_Clock_Division_CKD 
TIM_CKD_DIV1                       EQU    (0x0000)
TIM_CKD_DIV2                       EQU    (0x0100)
TIM_CKD_DIV4                       EQU    (0x0200)

; TIM_Counter_Mode 
TIM_CounterMode_Up                 EQU    (0x0000)
TIM_CounterMode_Down               EQU    (0x0010)
TIM_CounterMode_CenterAligned1     EQU    (0x0020)
TIM_CounterMode_CenterAligned2     EQU    (0x0040)
TIM_CounterMode_CenterAligned3     EQU    (0x0060)


; TIM_Output_Compare_Polarity 
TIM_OCPolarity_High                EQU    (0x0000)
TIM_OCPolarity_Low                 EQU    (0x0002)

; TIM_Output_Compare_state
TIM_OutputState_Disable            EQU    (0x0000)
TIM_OutputState_Enable             EQU    (0x0001)

; TIM_Capture_Compare_state 
TIM_CCx_Enable                      EQU    (0x0001)
TIM_CCx_Disable                     EQU    (0x0000)

; TIM_Input_Capture_Polarity 
TIM_ICPolarity_Rising             EQU    (0x0000)
TIM_ICPolarity_Falling            EQU    (0x0002)
TIM_ICPolarity_BothEdge           EQU    (0x000A)


; TIM_Input_Capture_Selection 
TIM_ICSelection_DirectTI           EQU    (0x0001) ; TIM Input 1, 2, 3 or 4 is selected to be 
                                                   ;   connected to IC1, IC2, IC3 or IC4, respectively 
TIM_ICSelection_IndirectTI         EQU    (0x0002) ; TIM Input 1, 2, 3 or 4 is selected to be
                                                   ;   connected to IC2, IC1, IC4 or IC3, respectively. 
TIM_ICSelection_TRC                EQU    (0x0003) ; TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. 

; TIM_Input_Capture_Prescaler 
TIM_ICPSC_DIV1                     EQU    (0x0000) ; Capture performed each time an edge is detected on the capture input. 
TIM_ICPSC_DIV2                     EQU    (0x0004) ; Capture performed once every 2 events. 
TIM_ICPSC_DIV4                     EQU    (0x0008) ; Capture performed once every 4 events. 
TIM_ICPSC_DIV8                     EQU    (0x000C) ; Capture performed once every 8 events. 


; TIM_interrupt_sources 
TIM_IT_Update                      EQU    (0x0001)
TIM_IT_CC1                         EQU    (0x0002)
TIM_IT_CC2                         EQU    (0x0004)
TIM_IT_CC3                         EQU    (0x0008)
TIM_IT_CC4                         EQU    (0x0010)
TIM_IT_Trigger                     EQU    (0x0040)

; TIM_DMA_Base_address 
TIM_DMABase_CR1                    EQU    (0x0000)
TIM_DMABase_CR2                    EQU    (0x0001)
TIM_DMABase_SMCR                   EQU    (0x0002)
TIM_DMABase_DIER                   EQU    (0x0003)
TIM_DMABase_SR                     EQU    (0x0004)
TIM_DMABase_EGR                    EQU    (0x0005)
TIM_DMABase_CCMR1                  EQU    (0x0006)
TIM_DMABase_CCMR2                  EQU    (0x0007)
TIM_DMABase_CCER                   EQU    (0x0008)
TIM_DMABase_CNT                    EQU    (0x0009)
TIM_DMABase_PSC                    EQU    (0x000A)
TIM_DMABase_ARR                    EQU    (0x000B)
TIM_DMABase_RCR                    EQU    (0x000C)
TIM_DMABase_CCR1                   EQU    (0x000D)
TIM_DMABase_CCR2                   EQU    (0x000E)
TIM_DMABase_CCR3                   EQU    (0x000F)
TIM_DMABase_CCR4                   EQU    (0x0010)
TIM_DMABase_DCR                    EQU    (0x0012)

; TIM_DMA_Burst_Length 
TIM_DMABurstLength_1Byte           EQU    (0x0000)
TIM_DMABurstLength_2Bytes          EQU    (0x0100)
TIM_DMABurstLength_3Bytes          EQU    (0x0200)
TIM_DMABurstLength_4Bytes          EQU    (0x0300)
TIM_DMABurstLength_5Bytes          EQU    (0x0400)
TIM_DMABurstLength_6Bytes          EQU    (0x0500)
TIM_DMABurstLength_7Bytes          EQU    (0x0600)
TIM_DMABurstLength_8Bytes          EQU    (0x0700)
TIM_DMABurstLength_9Bytes          EQU    (0x0800)
TIM_DMABurstLength_10Bytes         EQU    (0x0900)
TIM_DMABurstLength_11Bytes         EQU    (0x0A00)
TIM_DMABurstLength_12Bytes         EQU    (0x0B00)
TIM_DMABurstLength_13Bytes         EQU    (0x0C00)
TIM_DMABurstLength_14Bytes         EQU    (0x0D00)
TIM_DMABurstLength_15Bytes         EQU    (0x0E00)
TIM_DMABurstLength_16Bytes         EQU    (0x0F00)
TIM_DMABurstLength_17Bytes         EQU    (0x1000)
TIM_DMABurstLength_18Bytes         EQU    (0x1100)

; TIM_DMA_sources 
TIM_DMA_Update                     EQU    (0x0100)
TIM_DMA_CC1                        EQU    (0x0200)
TIM_DMA_CC2                        EQU    (0x0400)
TIM_DMA_CC3                        EQU    (0x0800)
TIM_DMA_CC4                        EQU    (0x1000)
TIM_DMA_Trigger                    EQU    (0x4000)

; TIM_External_Trigger_Prescaler 
TIM_ExtTRGPSC_OFF                  EQU    (0x0000)
TIM_ExtTRGPSC_DIV2                 EQU    (0x1000)
TIM_ExtTRGPSC_DIV4                 EQU    (0x2000)
TIM_ExtTRGPSC_DIV8                 EQU    (0x3000)

; TIM_Internal_Trigger_Selection 
TIM_TS_ITR0                        EQU    (0x0000)
TIM_TS_ITR1                        EQU    (0x0010)
TIM_TS_ITR2                        EQU    (0x0020)
TIM_TS_ITR3                        EQU    (0x0030)
TIM_TS_TI1F_ED                     EQU    (0x0040)
TIM_TS_TI1FP1                      EQU    (0x0050)
TIM_TS_TI2FP2                      EQU    (0x0060)
TIM_TS_ETRF                        EQU    (0x0070)

; TIM_TIx_External_Clock_Source 
TIM_TIxExternalCLK1Source_TI1      EQU    (0x0050)
TIM_TIxExternalCLK1Source_TI2      EQU    (0x0060)
TIM_TIxExternalCLK1Source_TI1ED    EQU    (0x0040)


; TIM_External_Trigger_Polarity 
TIM_ExtTRGPolarity_Inverted        EQU    (0x8000)
TIM_ExtTRGPolarity_NonInverted     EQU    (0x0000)

; TIM_Prescaler_Reload_Mode 
TIM_PSCReloadMode_Update           EQU    (0x0000)
TIM_PSCReloadMode_Immediate        EQU    (0x0001)

; TIM_Forced_Action 
TIM_ForcedAction_Active            EQU    (0x0050)
TIM_ForcedAction_InActive          EQU    (0x0040)

; TIM_Encoder_Mode 
TIM_EncoderMode_TI1                EQU    (0x0001)
TIM_EncoderMode_TI2                EQU    (0x0002)
TIM_EncoderMode_TI12               EQU    (0x0003)

; TIM_Event_Source 
TIM_EventSource_Update             EQU    (0x0001)
TIM_EventSource_CC1                EQU    (0x0002)
TIM_EventSource_CC2                EQU    (0x0004)
TIM_EventSource_CC3                EQU    (0x0008)
TIM_EventSource_CC4                EQU    (0x0010)
TIM_EventSource_Trigger            EQU    (0x0040)

; TIM_Update_Source 
TIM_UpdateSource_Global            EQU    (0x0000) ; Source of update is the counter overflow/underflow
                                                   ; or the setting of UG bit, or an update generation
                                                   ; through the slave mode controller. 
TIM_UpdateSource_Regular           EQU    (0x0001) ; Source of update is counter overflow/underflow. 

; TIM_Output_Compare_Preload_State 
TIM_OCPreload_Enable               EQU    (0x0008)
TIM_OCPreload_Disable              EQU    (0x0000)

; TIM_Output_Compare_Fast_State 
TIM_OCFast_Enable                  EQU    (0x0004)
TIM_OCFast_Disable                 EQU    (0x0000)

; TIM_Output_Compare_Clear_State 
TIM_OCClear_Enable                 EQU    (0x0080)
TIM_OCClear_Disable                EQU    (0x0000)

; TIM_Trigger_Output_Source 
TIM_TRGOSource_Reset               EQU    (0x0000)
TIM_TRGOSource_Enable              EQU    (0x0010)
TIM_TRGOSource_Update              EQU    (0x0020)
TIM_TRGOSource_OC1                 EQU    (0x0030)
TIM_TRGOSource_OC1Ref              EQU    (0x0040)
TIM_TRGOSource_OC2Ref              EQU    (0x0050)
TIM_TRGOSource_OC3Ref              EQU    (0x0060)
TIM_TRGOSource_OC4Ref              EQU    (0x0070)

; TIM_Slave_Mode 
TIM_SlaveMode_Reset                EQU    (0x0004)
TIM_SlaveMode_Gated                EQU    (0x0005)
TIM_SlaveMode_Trigger              EQU    (0x0006)
TIM_SlaveMode_External1            EQU    (0x0007)

; TIM_Master_Slave_Mode 
TIM_MasterSlaveMode_Enable         EQU    (0x0080)
TIM_MasterSlaveMode_Disable        EQU    (0x0000)
  
; TIM_Flags 
TIM_FLAG_Update                    EQU    (0x0001)
TIM_FLAG_CC1                       EQU    (0x0002)
TIM_FLAG_CC2                       EQU    (0x0004)
TIM_FLAG_CC3                       EQU    (0x0008)
TIM_FLAG_CC4                       EQU    (0x0010)
TIM_FLAG_Trigger                   EQU    (0x0040)
TIM_FLAG_CC1OF                     EQU    (0x0200)
TIM_FLAG_CC2OF                     EQU    (0x0400)
TIM_FLAG_CC3OF                     EQU    (0x0800)
TIM_FLAG_CC4OF                     EQU    (0x1000)


; TIM_OCReferenceClear 
TIM_OCReferenceClear_ETRF          EQU    (0x0008)
TIM_OCReferenceClear_OCREFCLR      EQU    (0x0000)

; TIM_Remap 
TIM9_GPIO                          EQU    (0x0000)
TIM9_LSE                           EQU    (0x0001)

TIM10_GPIO                         EQU    (0x0000)
TIM10_LSI                          EQU    (0x0001)
TIM10_LSE                          EQU    (0x0002)
TIM10_RTC                          EQU    (0x0003)

TIM11_GPIO                         EQU    (0x0000)
TIM11_MSI                          EQU    (0x0001)
TIM11_HSE_RTC                      EQU    (0x0002)


; This following is added to remove the compiler warning.
    AREA    __DEFINES_STM32L1xx_TIM_DUMMY, CODE, READONLY
    END