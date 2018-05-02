// UART.c
// Runs on LM3S811, LM3S1968, LM3S8962, LM4F120, TM4C123
// Simple device driver for the UART.
// Daniel Valvano
// May 30, 2014
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Program 4.12, Section 4.9.4, Figures 4.26 and 4.40

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
#include "UART.h"
#include "tm4c123gh6pm.h"
#include "stdlib.h"


#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable



//------------UART_Init------------
// Initialize the UART for 115,200 baud rate (assuming 50 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none


//U2RX - PD6 
//U3RX - PC6
//U4RX - PC4
//U7RX - PE0
//U6RX - PD4  TODO NO LONGER USED

uint8_t allvals[4];

char UART2_InChar(void);
char UART3_InChar(void);
char UART4_InChar(void);
char UART7_InChar(void);
//char UART6_InChar(void);

void UART2_Handler(void){
  if(UART2_RIS_R&UART_RIS_RXRIS){       // hardware RX FIFO >= 2 items
    UART2_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    allvals[0] = UART2_DR_R;
  }
}


void UART3_Handler(void){
  if(UART3_RIS_R&UART_RIS_RXRIS){       // hardware RX FIFO >= 2 items
    UART3_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    allvals[1] = UART3_DR_R;
  }
}


void UART4_Handler(void){
  if(UART4_RIS_R&UART_RIS_RXRIS){       // hardware RX FIFO >= 2 items
    UART4_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    allvals[2] = UART4_DR_R;
  }
}

void UART7_Handler(void){
  if(UART7_RIS_R&UART_RIS_RXRIS){       // hardware RX FIFO >= 2 items
    UART7_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    allvals[3] = UART7_DR_R;
  }
}


void UART_Init(void){
  SYSCTL_RCGCUART_R |= 0x9C;             // activate UART2,3,4,7
  SYSCTL_RCGCGPIO_R |= 0x1C;               // activate port C,D,E
  while((SYSCTL_PRGPIO_R&0x01) == 0){};
		
  UART2_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART2_IBRD_R = 520;                    // IBRD = int(80,000,000 / (16 * 9600)) = int(520.8333)
  UART2_FBRD_R = 53;                     // FBRD = int(0.8333 * 64 + 0.5) = 53
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART2_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART2_CTL_R |= 0x301;                 // enable UART
	UART2_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART2_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART2_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART2_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);	
		
	UART3_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART3_IBRD_R = 520;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART3_FBRD_R = 53;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART3_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART3_CTL_R |= 0x301;                 // enable UART	
	UART3_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART3_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART3_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART3_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);	
		
		
	UART4_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART4_IBRD_R = 520;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART4_FBRD_R = 53;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART4_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART4_CTL_R |= 0x301;                 // enable UART
	UART4_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART4_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART4_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART4_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);
	
	
	
	UART7_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART7_IBRD_R = 520;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART7_FBRD_R = 53;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART7_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART7_CTL_R |= 0x301;                 // enable UART
	UART7_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART7_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART7_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART7_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);
	
	
	
	GPIO_PORTE_AFSEL_R |= 0x01; //0b00000001;         
  GPIO_PORTE_DEN_R |= 0x01;   //0b00000001;         
	
  GPIO_PORTD_AFSEL_R |= 0x10; //0b00010000;          
  GPIO_PORTD_DEN_R |= 0x10;   //0b00010000;         
	
	GPIO_PORTC_AFSEL_R |= 0x50; //0b01010000;           
  GPIO_PORTC_DEN_R |= 0x50;   //0b01010000;         
	
                                                    // configure PA1-0 as UART
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFFFFFF0)+0x00000001;
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x01000000;
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xF0F0FFFF)+0x01010000;
	
	GPIO_PORTE_AMSEL_R &= ~0x01;    //00000001    // disable analog functionality on PE0
  GPIO_PORTD_AMSEL_R &= ~0x10;    //01000000    // disable analog functionality on PD6
	GPIO_PORTC_AMSEL_R &= ~0x50; 		//01010000		// disable analog functionality on PC6,4
}

//------------UART_InChar------------
// Wait for new serial port input
// Input: none


