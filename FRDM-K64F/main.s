;******************** (C) Yifeng ZHU ********************
; @file    main.s
; @author  
; V1.0.0
; @date    Feb-07-2014
; @brief   Assembly code for FRDM-K64F Freescale Freedom Development Platform
; @note
;          This code is for the book "Embedded Systems with ARM Cortex-M 
;          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;          ISBN-10: 0982692633.
; @attension
;          This code is provided for education purpose. The author shall not be 
;          held liable for any direct, indirect or consequential damages, for any 
;          reason whatever. More information can be found from book website: 
;          http:;www.eece.maine.edu/~zhu/book
;********************************************************


;************************************************************************************ 
; Install USB driver
;    https:;developer.mbed.org/handbook/CMSIS-DAP-MDK
;
; FRDM-K64F: Freescale Freedom Development Platform
;    Processor: MK64FN1M0VLL12 
;    PTB 21 <------>  Blue LED
;    PTB 22 <------>  Red LED    
;    PTE 26 <------>  Green LED
;    PTC 6  <------>  User Switch 2
;    PTA 4  <------>  User Switch 3
;************************************************************************************


;************************************************************************************
; Simple Demo of light up Red LED
;************************************************************************************

	INCLUDE MK64F12_constants.s		; Load Constant Definitions
	INCLUDE core_cm4_constants.s

	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY
				
__main	PROC
 
	; System Clock Gating Control Register (SCGC)
	; Enable Clock of GPIO Port B and E
	LDR r1, =SIM_BASE    ; Base address
	LDR r2, =SIM_SCGC5   ; Offset
	LDR r0, [r1, r2]
	ORR r0, r0,  #SIM_SCGC5_PORTB_MASK
	ORR r0, r0,  #SIM_SCGC5_PORTE_MASK
	STR r0, [r1, r2]
		
	; Pin Control Register (PCR)
	; 000 = Pin disabled (analog)
	; 001 = Alternative 1 (GPIO)
	; 010 = Alternative 2
	; 011 = Alternative 3
	; 100 = Alternative 4
	; 101 = Alternative 5
	; 110 = Alternative 6
	; 111 = Alternative 7
		
	; Set PTB 22 as General GPIO pin  
	LDR r1, =PORTB_BASE
	LDR r0, [r1, #PORT_PCR22]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR22]
		
	; Set PTB 21 as General GPIO pin  
	LDR r0, [r1, #PORT_PCR21]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR21]
		
	; Set PTE 26 as General GPIO pin
	LDR r1, =PORTE_BASE
	LDR r0, [r1, #PORT_PCR26]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR26]     
	
	; Port Data Direction Register (PDDR)
	; 0 = digital input, 1 = digital output
	LDR r1, =PTB_BASE
	LDR r0, [r1, #GPIO_PDDR]
	ORR r0, r0, #(1<<21)    ; PTB 21 as output
	ORR r0,	r0, #(1 <<22)   ; PTB 22 as output
	STR r0, [r1, #GPIO_PDDR]

	LDR r1, =PTE_BASE
	LDR r0, [r1, #GPIO_PDDR]
	ORR r0, r0, #(1<<26) 
	STR r0, [r1, #GPIO_PDDR]
	
	; Port Data Output Register (PDOR) 
	; Output low to light up LED
	LDR r1, =PTB_BASE
	LDR r0, [r1, #GPIO_PDOR]
	ORR r0, r0, #(1<<21)       ; Turn off Blue
	BIC r0, r0, #(1<<22)       ; Turn on Red
	STR r0, [r1, #GPIO_PDOR]

	LDR r1, =PTE_BASE
	LDR r0, [r1, #GPIO_PDOR]
	ORR r0, r0, #(1<<26)       ; Turn off Green
	STR r0, [r1, #GPIO_PDOR]	  

loop	B	loop   ; Dead loop

	ENDP
		
	ALIGN
	AREA    myData, DATA, READWRITE
	ALIGN
array	DCD   1, 2, 3, 4	
	END
