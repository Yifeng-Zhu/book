#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>

char RxComByte = 0;
uint8_t buffer[BufferSize];

void demo_of_printf_scanf(){
	char rxByte;
	printf("Give Red LED control input (Y = On, N = off):\r\n");
	scanf ("%c", &rxByte);
	if (rxByte == 'N' || rxByte == 'n'){
		LED_Off();
		printf("LED is Off\r\n\r\n");
	}
	else if (rxByte == 'Y' || rxByte == 'y'){
		LED_On();
		printf("LED is On\r\n\r\n");
	}
}

void demo_of_UART_print(int a){
	int n, i;
	float b;
	b = (float)a/100;
	n = sprintf((char *)buffer, "a = %d\t", a);
	n += sprintf((char *)buffer + n, "b = %f\r\n", b);
	USART_Write(USART2, buffer, n);		
	for (i = 0; i < 8000000; i++); // Delay
	LED_Toggle();
}
	
int main(void){
	int a;
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	UART2_Init();
	
	a = 0;
	while (1){
		//demo_of_printf_scanf();
		demo_of_UART_print(a);
		a++;
	}
}

