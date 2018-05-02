// UARTTestMain.c
// Runs on LM4F120/TM4C123
// Used to test the UART.c driver
// Daniel Valvano
// May 30, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1
#include <stdint.h>
#include "PLL.h"
#include "UART.h"
#include "ST7735.h"

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none

//debug code
extern uint8_t allvals[4];
int main(void){
	
  char ch;
  char string[20];  // global to assist in debugging
  uint32_t n;
	ST7735_InitR();
  PLL_Init(Bus80MHz);       // 50  MHz
  UART_Init();              // initialize UART
	int idx=0;
	for(idx=0;idx<20;idx++){
		string[idx]='a';
	}
	string[19]= '\0';
	
while(1){
	for (idx=0;idx<4;idx++){
		ST7735_Message(0,idx,"Value: ",allvals[idx]);
}
}
}	


