//******************** (C) Yifeng ZHU ***********************************************
// @file    main.c
// @author  Yifeng Zhu
// @version V1.0.0
// @date    May-17-2015
// @note    
// @brief   Assembly code for STM32F4 Discovery Kit
// @note
//          This code is for the book "Embedded Systems with ARM Cortex-M3 
//          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
//          ISBN-10: 0982692625.
// @attension
//          This code is provided for education purpose. The author shall not be 
//          held liable for any direct, indirect or consequential damages, for any 
//          reason whatever. More information can be found from book website: 
//          http://www.eece.maine.edu/~zhu/book
//************************************************************************************

				
//************************************************************************************
// Discovery kit for STM32F407/417 lines
// LEDs:
//  LD3 ORANGE <---> PD.13 
//  LD4 GREEN  <---> PD.12
//  LD5 RED    <---> PD.14
//  LD6 BLUE   <---> PD.15
//  LD7 GREEN LED indicates when VBUS is present on CN5 (pin PA.9)
//  LD8 RED LED indicates an overcurrent from VBUS of CN5 (pin PD.5)
// 
// Pushbutton
//  B1 <---> PA.0
//  B2 <---> Reset
//************************************************************************************


//************************************************************************************
//  Demo code of lighting up the Orange LED
//************************************************************************************


#include <stm32f407xx.h>

int main(void){
	
  // Enable Clock of GPIO Port D
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; 				
								
  // Set Pin 13 I/O direction as Digital Output
  GPIOD->MODER &= ~(GPIO_MODER_MODER13);   // Clear mode
  GPIOD->MODER |= (GPIO_MODER_MODER13_0);  // Input(00, reset), Output(01), Alternate Func(10), Analog(11)

  // Set Pin 13 the push-pull mode for the output type
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_13);   // Output Push-Pull(0, reset), Output Open-Drain(1)
    			
  // Set I/O output speed value as high speed
  GPIOD->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13); // Low speed (00), Medium speed (01), Fast speed(01), High speed (11)
		
  // Set I/O as no pull-up pull-down  
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR13); // No PUPD(00, reset), Pull up(01), Pull down (10), Reserved (11)

  // Output 1
  GPIOD->ODR |= GPIO_ODR_ODR_13; 
				
  while(1);
}


