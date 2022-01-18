//******************** (C) Yifeng ZHU **********************************************************************
// @file    main.c
// @author  Yifeng Zhu
// @version V1.0.0
// @date    May-17-2015
// @note    
// @brief   Assembly code for Tiva C Series LaunchPad Evaluation Kit
// @note
//          This code is for the book "Embedded Systems with ARM Cortex-M 
//          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
//          ISBN-10: 0982692633.
// @attension
//          This code is provided for education purpose. The author shall not be 
//          held liable for any direct, indirect or consequential damages, for any 
//          reason whatever. More information can be found from book website: 
//          http://www.eece.maine.edu/~zhu/book
//**********************************************************************************************************


// Tiva C Series LaunchPad Evaluation Kit (EK-TM4C123GXL)
//   Processor:  TM4C123GH6PM
//   PF1 <---> RGB LED (Red)
//   PF2 <---> RGB LED (Blue)
//   PF3 <---> RGB LED (Green)
//   PF0 <---> User switch 1
//   PF4 <---> User switch 2


//**********************************************************************************************************
//  Demo code of lighting up the RGB LED
//**********************************************************************************************************
	
#include <stdint.h>
#include "tm4c123gh6pm.h"

int main(void){  volatile unsigned long delay;
	
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // Enable the clock of GPIO Port F	
  GPIO_PORTF_DIR_R |= 0xE;              // GPIO direction: 0 = Input, 1 = Output
  GPIO_PORTF_AFSEL_R &= ~0xE; 		// GPIO alternate function select
	                                // 0 = Regular GPIO, 1 = Alternate hardware function	
  GPIO_PORTF_DEN_R |= 0xE;    		// GPIO digital enable: 0 = Disabled, 1 = Enabled,
  
  GPIO_PORTF_DATA_R |= 0x8;             // Turn on Green LED
	
  while(1);  
}
