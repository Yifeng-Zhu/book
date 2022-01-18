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

; Register used:
;	r0 = return sinusoidal value in Q31 notation
;	r1 = sin argument (in degrees)
;	r4 = starting address of sine table
;	r5 = temp
;	r6 = copy of arguments

sine	PROC  
	PUSH  {r1,r4,r5,r6,lr}  ; stack used registers
	MOV   r6, r1            ; make a copy
	LDR   r5, =270          ; won't fit into rotation scheme
	LDR   r4, =sin_data     ; load address of sin table
	CMP   r1, #90           ; determine quadrant
	BLS   retvalue          ; first quadrant
	CMP   r1, #180
	RSBLS r1, r1, #180      ; second quadrant
	BLS   retvalue
	CMP   r1, r5
	SUBLE r1, r1, #180      ; third quadrant
	BLS   retvalue
	RSB   r1, r1, #360      ; fourth quadrant
	
retvalue	
	LDR   r0, [r4, r1, LSL #2]  ; get sin value from table
	CMP   r6, #180              ; should we return a negative value?
	RSBGT r0, r0, #4096         ; 4096 - sin(x)
	pop   {r1,r4,r5,r6,pc}
	ENDP

	ALIGN
sin_data     ; DAC has 12 bits.
	DCD	0x800,0x823,0x847,0x86b,0x88e,0x8b2,0x8d6,0x8f9,0x91d,0x940
	DCD	0x963,0x986,0x9a9,0x9cc,0x9ef,0xa12,0xa34,0xa56,0xa78,0xa9a
	DCD	0xabc,0xadd,0xaff,0xb20,0xb40,0xb61,0xb81,0xba1,0xbc1,0xbe0
	DCD	0xc00,0xc1e,0xc3d,0xc5b,0xc79,0xc96,0xcb3,0xcd0,0xcec,0xd08
	DCD	0xd24,0xd3f,0xd5a,0xd74,0xd8e,0xda8,0xdc1,0xdd9,0xdf1,0xe09
	DCD	0xe20,0xe37,0xe4d,0xe63,0xe78,0xe8d,0xea1,0xeb5,0xec8,0xedb
	DCD	0xeed,0xeff,0xf10,0xf20,0xf30,0xf40,0xf4e,0xf5d,0xf6a,0xf77
	DCD	0xf84,0xf90,0xf9b,0xfa6,0xfb0,0xfba,0xfc3,0xfcb,0xfd3,0xfda
	DCD	0xfe0,0xfe6,0xfec,0xff0,0xff4,0xff8,0xffb,0xffd,0xffe,0xfff
	DCD	0xfff
	; sin(90) = 1. However, 1 cannot be represented in Q12 notation
	; thus we set sin(90) = 0xFFF
	END
