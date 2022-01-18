//******************** (C) Yifeng ZHU ***********************************************
// @file    main.c
// @author  Yifeng Zhu
// @version V1.0.0
// @date    May-17-2015
// @note    
// @brief   C code for FRDM-K64F: Freescale Freedom Development Platform
// @note
//          This code is for the book "Embedded Systems with ARM Cortex-M 
//          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
//          ISBN-10: 0982692633.
// @attension
//          This code is provided for education purpose. The author shall not be 
//          held liable for any direct, indirect or consequential damages, for any 
//          reason whatever. More information can be found from book website: 
//          http://www.eece.maine.edu/~zhu/book
//************************************************************************************

//************************************************************************************ 
// Install USB driver
//    https://developer.mbed.org/handbook/CMSIS-DAP-MDK
//
// FRDM-K64F: Freescale Freedom Development Platform
//    Processor: MK64FN1M0VLL12 
//    PTB 21 <------>  Blue LED
//    PTB 22 <------>  Red LED    
//    PTE 26 <------>  Green LED
//    PTC 6  <------>  User Switch 2
//    PTA 4  <------>  User Switch 3
//************************************************************************************

//************************************************************************************
// Simple Demo of light up Red LED
//************************************************************************************

#include "MK64F12.h"

int main (void) {
  
  // System Clock Gating Control Register (SCGC)
  SIM->SCGC5 |= ((1UL << 13) | (1UL << 10));    // Enable Clock of GPIO Port B and E
	
  // Pin Control Register (PCR)
  // 000 = Pin disabled (analog)
  // 001 = Alternative 1 (GPIO)
  // 010 = Alternative 2
  // 011 = Alternative 3
  // 100 = Alternative 4
  // 101 = Alternative 5
  // 110 = Alternative 6
  // 111 = Alternative 7
  PORTB->PCR[22]  = (1UL << 8);    // Set PTB 22 as General GPIO pin     
  PORTB->PCR[21]  = (1UL << 8);    // Set PTB 21 as General GPIO pin     
  PORTE->PCR[26]  = (1UL << 8);    // Set PTE 26 as General GPIO pin  
	
  // Port Data Direction Register (PDDR)
  PTB->PDDR |= ((1UL << 21) | (1UL << 22));  // 0 = digital input, 1 = digital output
  PTE->PDDR |= (1UL << 26);
	
  // Port Data Output Register (PDOR) 
  // Output low to light up LED
  PTB->PDOR |= (1UL << 21);  // Turn off Blue 
  PTB->PDOR &= ~(1UL << 22); // Turn on Red
  PTE->PDOR |= (1UL << 26);  // Turn off Green
	
  while(1);
}
