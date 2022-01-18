;******************** (C) Yifeng ZHU **********************************************************************
; @file    main.s
; @author  Yifeng Zhu
; @version V1.0.0
; @date    May-17-2015
; @note    
; @brief   Assembly code for Tiva C Series LaunchPad Evaluation Kit
; @note
;          This code is for the book "Embedded Systems with ARM Cortex-M 
;          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;          ISBN-10: 0982692633.
; @attension
;          This code is provided for education purpose. The author shall not be 
;          held liable for any direct, indirect or consequential damages, for any 
;          reason whatever. More information can be found from book website: 
;          http://www.eece.maine.edu/~zhu/book
;**********************************************************************************************************


; Tiva C Series LaunchPad Evaluation Kit (EK-TM4C123GXL)
;   Processor:  TM4C123GH6PM
;   PF1 <---> RGB LED (Red)
;   PF2 <---> RGB LED (Blue)
;   PF3 <---> RGB LED (Green)
;   PF0 <---> User switch 1
;   PF4 <---> User switch 2


;**********************************************************************************************************
;  Demo code of lighting up the RGB LED
;**********************************************************************************************************
	INCLUDE tm4c123gh6pm_constants.s 


	AREA    main, CODE, READONLY
	EXPORT	__main			; make __main visible to linker
	ENTRY						

__main	PROC
	
	; Enable the clock of GPIO Port F
	LDR r0, =SYSCTL_RCGC2_R
	LDR r1, [r0]
	ORR r1, r1, #SYSCTL_RCGC2_GPIOF
	STR r1, [r0]				
	
	; GPIO direction: 0 = Input, 1 = Output
	LDR r0, =GPIO_PORTF_DIR_R
	LDR r1, [r0]
	ORR r1, r1, #0xE  ; Pin 1, 2, and 3 as output
	STR r1, [r0]
				
	; GPIO alternate function select
	; 0 = Regular GPIO, 1 = Alternate hardware function
	LDR r0, =GPIO_PORTF_AFSEL_R
	LDR r1, [r0]
	BIC r1, r1, #0xE  ; Pin 1, 2, and 3 as regular GPIO
	STR r1, [r0]
				
	; GPIO digital enable: 0 = Disabled, 1 = Enabled
	LDR r0, =GPIO_PORTF_DEN_R
	LDR r1, [r0]
	ORR r1, r1, #0xE  ; Enable Pin 1, 2, and 3
	STR r1, [r0]
				
	; Turn on Green LED
	LDR r0, =GPIO_PORTF_DATA_R
	LDR r1, [r0]
	ORR r1, r1, #(1<<3)  ; Output of Pin 3 = 1
	STR r1, [r0]			
				
stop 	B 		stop     	; dead loop & program hangs here

	ENDP
					
	ALIGN			

	AREA    myData, DATA, READWRITE
	ALIGN
array	DCD   1, 2, 3, 4
	END
