/***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT displayun
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// ST7735.c
// Runs on LM4F120/TM4C123
// Low level drivers for the ST7735 160x128 LCD based off of
// the file described above.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano
// March 30, 2015
// Augmented 7/17/2014 to have a simple graphics facility
// Tested with LaunchPadDLL.dll simulator 9/2/2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
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

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// THIS CODE IS MODIFIED. 
// ST7735.c was copied from VALVANOWARE. Unnecessary functions and definitions were deleted. 
// Everything else was directly copied except for the ST7735_Message() function, which is original.
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#include <stdio.h>
#include <stdint.h>
#include "ST7735.h"
#include "string.h"
#include "tm4c123gh6pm.h"

// 16 rows (0 to 15) and 21 characters (0 to 20)
// Requires (11 + size*size*6*8) bytes of transmission for each character
uint32_t StX=0; // position along the horizonal axis 0 to 20
uint32_t StY=0; // position along the vertical axis 0 to 15
uint16_t StTextColor = ST7735_YELLOW;

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define TFT_CS                  (*((volatile uint32_t *)0x40004020))
#define TFT_CS_LOW              0           // CS normally controlled by hardware
#define TFT_CS_HIGH             0x08
#define DC                      (*((volatile uint32_t *)0x40004100))
#define DC_COMMAND              0
#define DC_DATA                 0x40
#define RESET                   (*((volatile uint32_t *)0x40004200))
#define RESET_LOW               0
#define RESET_HIGH              0x80

#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // Either the system clock (if the
                                            // PLL bypass is in effect) or the
                                            // PLL output (default)
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control
#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// standard ascii 5x7 font
// originally from glcdfont.c from Adafruit project
static const uint8_t Font[] = {
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
  0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
  0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
  0x18, 0x3C, 0x7E, 0x3C, 0x18,
  0x1C, 0x57, 0x7D, 0x57, 0x1C,
  0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
  0x00, 0x18, 0x3C, 0x18, 0x00,
  0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
  0x00, 0x18, 0x24, 0x18, 0x00,
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
  0x30, 0x48, 0x3A, 0x06, 0x0E,
  0x26, 0x29, 0x79, 0x29, 0x26,
  0x40, 0x7F, 0x05, 0x05, 0x07,
  0x40, 0x7F, 0x05, 0x25, 0x3F,
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
  0x7F, 0x3E, 0x1C, 0x1C, 0x08,
  0x08, 0x1C, 0x1C, 0x3E, 0x7F,
  0x14, 0x22, 0x7F, 0x22, 0x14,
  0x5F, 0x5F, 0x00, 0x5F, 0x5F,
  0x06, 0x09, 0x7F, 0x01, 0x7F,
  0x00, 0x66, 0x89, 0x95, 0x6A,
  0x60, 0x60, 0x60, 0x60, 0x60,
  0x94, 0xA2, 0xFF, 0xA2, 0x94,
  0x08, 0x04, 0x7E, 0x04, 0x08,
  0x10, 0x20, 0x7E, 0x20, 0x10,
  0x08, 0x08, 0x2A, 0x1C, 0x08,
  0x08, 0x1C, 0x2A, 0x08, 0x08,
  0x1E, 0x10, 0x10, 0x10, 0x10,
  0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
  0x30, 0x38, 0x3E, 0x38, 0x30,
  0x06, 0x0E, 0x3E, 0x0E, 0x06,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x5F, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14,
  0x24, 0x2A, 0x7F, 0x2A, 0x12,
  0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x56, 0x20, 0x50,
  0x00, 0x08, 0x07, 0x03, 0x00,
  0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00,
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
  0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x80, 0x70, 0x30, 0x00,
  0x08, 0x08, 0x08, 0x08, 0x08,
  0x00, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02,
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
  0x00, 0x42, 0x7F, 0x40, 0x00, // 1
  0x72, 0x49, 0x49, 0x49, 0x46, // 2
  0x21, 0x41, 0x49, 0x4D, 0x33, // 3
  0x18, 0x14, 0x12, 0x7F, 0x10, // 4
  0x27, 0x45, 0x45, 0x45, 0x39, // 5
  0x3C, 0x4A, 0x49, 0x49, 0x31, // 6
  0x41, 0x21, 0x11, 0x09, 0x07, // 7
  0x36, 0x49, 0x49, 0x49, 0x36, // 8
  0x46, 0x49, 0x49, 0x29, 0x1E, // 9
  0x00, 0x00, 0x14, 0x00, 0x00,
  0x00, 0x40, 0x34, 0x00, 0x00,
  0x00, 0x08, 0x14, 0x22, 0x41,
  0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08,
  0x02, 0x01, 0x59, 0x09, 0x06,
  0x3E, 0x41, 0x5D, 0x59, 0x4E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, // A
  0x7F, 0x49, 0x49, 0x49, 0x36, // B
  0x3E, 0x41, 0x41, 0x41, 0x22, // C
  0x7F, 0x41, 0x41, 0x41, 0x3E, // D
  0x7F, 0x49, 0x49, 0x49, 0x41, // E
  0x7F, 0x09, 0x09, 0x09, 0x01, // F
  0x3E, 0x41, 0x41, 0x51, 0x73, // G
  0x7F, 0x08, 0x08, 0x08, 0x7F, // H
  0x00, 0x41, 0x7F, 0x41, 0x00, // I
  0x20, 0x40, 0x41, 0x3F, 0x01, // J
  0x7F, 0x08, 0x14, 0x22, 0x41, // K
  0x7F, 0x40, 0x40, 0x40, 0x40, // L
  0x7F, 0x02, 0x1C, 0x02, 0x7F, // M
  0x7F, 0x04, 0x08, 0x10, 0x7F, // N
  0x3E, 0x41, 0x41, 0x41, 0x3E, // O
  0x7F, 0x09, 0x09, 0x09, 0x06, // P
  0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
  0x7F, 0x09, 0x19, 0x29, 0x46, // R
  0x26, 0x49, 0x49, 0x49, 0x32, // S
  0x03, 0x01, 0x7F, 0x01, 0x03, // T
  0x3F, 0x40, 0x40, 0x40, 0x3F, // U
  0x1F, 0x20, 0x40, 0x20, 0x1F, // V
  0x3F, 0x40, 0x38, 0x40, 0x3F, // W
  0x63, 0x14, 0x08, 0x14, 0x63, // X
  0x03, 0x04, 0x78, 0x04, 0x03, // Y
  0x61, 0x59, 0x49, 0x4D, 0x43, // Z
  0x00, 0x7F, 0x41, 0x41, 0x41,
  0x02, 0x04, 0x08, 0x10, 0x20,
  0x00, 0x41, 0x41, 0x41, 0x7F,
  0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40,
  0x00, 0x03, 0x07, 0x08, 0x00,
  0x20, 0x54, 0x54, 0x78, 0x40, // a
  0x7F, 0x28, 0x44, 0x44, 0x38, // b
  0x38, 0x44, 0x44, 0x44, 0x28, // c
  0x38, 0x44, 0x44, 0x28, 0x7F, // d
  0x38, 0x54, 0x54, 0x54, 0x18, // e
  0x00, 0x08, 0x7E, 0x09, 0x02, // f
  0x18, 0xA4, 0xA4, 0x9C, 0x78, // g
  0x7F, 0x08, 0x04, 0x04, 0x78, // h
  0x00, 0x44, 0x7D, 0x40, 0x00, // i
  0x20, 0x40, 0x40, 0x3D, 0x00, // j
  0x7F, 0x10, 0x28, 0x44, 0x00, // k
  0x00, 0x41, 0x7F, 0x40, 0x00, // l
  0x7C, 0x04, 0x78, 0x04, 0x78, // m
  0x7C, 0x08, 0x04, 0x04, 0x78, // n
  0x38, 0x44, 0x44, 0x44, 0x38, // o
  0xFC, 0x18, 0x24, 0x24, 0x18, // p
  0x18, 0x24, 0x24, 0x18, 0xFC, // q
  0x7C, 0x08, 0x04, 0x04, 0x08, // r
  0x48, 0x54, 0x54, 0x54, 0x24, // s
  0x04, 0x04, 0x3F, 0x44, 0x24, // t
  0x3C, 0x40, 0x40, 0x20, 0x7C, // u
  0x1C, 0x20, 0x40, 0x20, 0x1C, // v
  0x3C, 0x40, 0x30, 0x40, 0x3C, // w
  0x44, 0x28, 0x10, 0x28, 0x44, // x
  0x4C, 0x90, 0x90, 0x90, 0x7C, // y
  0x44, 0x64, 0x54, 0x4C, 0x44, // z
  0x00, 0x08, 0x36, 0x41, 0x00,
  0x00, 0x00, 0x77, 0x00, 0x00,
  0x00, 0x41, 0x36, 0x08, 0x00,
  0x02, 0x01, 0x02, 0x04, 0x02,
  0x3C, 0x26, 0x23, 0x26, 0x3C,
  0x1E, 0xA1, 0xA1, 0x61, 0x12,
  0x3A, 0x40, 0x40, 0x20, 0x7A,
  0x38, 0x54, 0x54, 0x55, 0x59,
  0x21, 0x55, 0x55, 0x79, 0x41,
  0x21, 0x54, 0x54, 0x78, 0x41,
  0x21, 0x55, 0x54, 0x78, 0x40,
  0x20, 0x54, 0x55, 0x79, 0x40,
  0x0C, 0x1E, 0x52, 0x72, 0x12,
  0x39, 0x55, 0x55, 0x55, 0x59,
  0x39, 0x54, 0x54, 0x54, 0x59,
  0x39, 0x55, 0x54, 0x54, 0x58,
  0x00, 0x00, 0x45, 0x7C, 0x41,
  0x00, 0x02, 0x45, 0x7D, 0x42,
  0x00, 0x01, 0x45, 0x7C, 0x40,
  0xF0, 0x29, 0x24, 0x29, 0xF0,
  0xF0, 0x28, 0x25, 0x28, 0xF0,
  0x7C, 0x54, 0x55, 0x45, 0x00,
  0x20, 0x54, 0x54, 0x7C, 0x54,
  0x7C, 0x0A, 0x09, 0x7F, 0x49,
  0x32, 0x49, 0x49, 0x49, 0x32,
  0x32, 0x48, 0x48, 0x48, 0x32,
  0x32, 0x4A, 0x48, 0x48, 0x30,
  0x3A, 0x41, 0x41, 0x21, 0x7A,
  0x3A, 0x42, 0x40, 0x20, 0x78,
  0x00, 0x9D, 0xA0, 0xA0, 0x7D,
  0x39, 0x44, 0x44, 0x44, 0x39,
  0x3D, 0x40, 0x40, 0x40, 0x3D,
  0x3C, 0x24, 0xFF, 0x24, 0x24,
  0x48, 0x7E, 0x49, 0x43, 0x66,
  0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
  0xFF, 0x09, 0x29, 0xF6, 0x20,
  0xC0, 0x88, 0x7E, 0x09, 0x03,
  0x20, 0x54, 0x54, 0x79, 0x41,
  0x00, 0x00, 0x44, 0x7D, 0x41,
  0x30, 0x48, 0x48, 0x4A, 0x32,
  0x38, 0x40, 0x40, 0x22, 0x7A,
  0x00, 0x7A, 0x0A, 0x0A, 0x72,
  0x7D, 0x0D, 0x19, 0x31, 0x7D,
  0x26, 0x29, 0x29, 0x2F, 0x28,
  0x26, 0x29, 0x29, 0x29, 0x26,
  0x30, 0x48, 0x4D, 0x40, 0x20,
  0x38, 0x08, 0x08, 0x08, 0x08,
  0x08, 0x08, 0x08, 0x08, 0x38,
  0x2F, 0x10, 0xC8, 0xAC, 0xBA,
  0x2F, 0x10, 0x28, 0x34, 0xFA,
  0x00, 0x00, 0x7B, 0x00, 0x00,
  0x08, 0x14, 0x2A, 0x14, 0x22,
  0x22, 0x14, 0x2A, 0x14, 0x08,
  0xAA, 0x00, 0x55, 0x00, 0xAA,
  0xAA, 0x55, 0xAA, 0x55, 0xAA,
  0x00, 0x00, 0x00, 0xFF, 0x00,
  0x10, 0x10, 0x10, 0xFF, 0x00,
  0x14, 0x14, 0x14, 0xFF, 0x00,
  0x10, 0x10, 0xFF, 0x00, 0xFF,
  0x10, 0x10, 0xF0, 0x10, 0xF0,
  0x14, 0x14, 0x14, 0xFC, 0x00,
  0x14, 0x14, 0xF7, 0x00, 0xFF,
  0x00, 0x00, 0xFF, 0x00, 0xFF,
  0x14, 0x14, 0xF4, 0x04, 0xFC,
  0x14, 0x14, 0x17, 0x10, 0x1F,
  0x10, 0x10, 0x1F, 0x10, 0x1F,
  0x14, 0x14, 0x14, 0x1F, 0x00,
  0x10, 0x10, 0x10, 0xF0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0x10,
  0x10, 0x10, 0x10, 0x1F, 0x10,
  0x10, 0x10, 0x10, 0xF0, 0x10,
  0x00, 0x00, 0x00, 0xFF, 0x10,
  0x10, 0x10, 0x10, 0x10, 0x10,
  0x10, 0x10, 0x10, 0xFF, 0x10,
  0x00, 0x00, 0x00, 0xFF, 0x14,
  0x00, 0x00, 0xFF, 0x00, 0xFF,
  0x00, 0x00, 0x1F, 0x10, 0x17,
  0x00, 0x00, 0xFC, 0x04, 0xF4,
  0x14, 0x14, 0x17, 0x10, 0x17,
  0x14, 0x14, 0xF4, 0x04, 0xF4,
  0x00, 0x00, 0xFF, 0x00, 0xF7,
  0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0xF7, 0x00, 0xF7,
  0x14, 0x14, 0x14, 0x17, 0x14,
  0x10, 0x10, 0x1F, 0x10, 0x1F,
  0x14, 0x14, 0x14, 0xF4, 0x14,
  0x10, 0x10, 0xF0, 0x10, 0xF0,
  0x00, 0x00, 0x1F, 0x10, 0x1F,
  0x00, 0x00, 0x00, 0x1F, 0x14,
  0x00, 0x00, 0x00, 0xFC, 0x14,
  0x00, 0x00, 0xF0, 0x10, 0xF0,
  0x10, 0x10, 0xFF, 0x10, 0xFF,
  0x14, 0x14, 0x14, 0xFF, 0x14,
  0x10, 0x10, 0x10, 0x1F, 0x00,
  0x00, 0x00, 0x00, 0xF0, 0x10,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
  0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xFF, 0xFF,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x38, 0x44, 0x44, 0x38, 0x44,
  0x7C, 0x2A, 0x2A, 0x3E, 0x14,
  0x7E, 0x02, 0x02, 0x06, 0x06,
  0x02, 0x7E, 0x02, 0x7E, 0x02,
  0x63, 0x55, 0x49, 0x41, 0x63,
  0x38, 0x44, 0x44, 0x3C, 0x04,
  0x40, 0x7E, 0x20, 0x1E, 0x20,
  0x06, 0x02, 0x7E, 0x02, 0x02,
  0x99, 0xA5, 0xE7, 0xA5, 0x99,
  0x1C, 0x2A, 0x49, 0x2A, 0x1C,
  0x4C, 0x72, 0x01, 0x72, 0x4C,
  0x30, 0x4A, 0x4D, 0x4D, 0x30,
  0x30, 0x48, 0x78, 0x48, 0x30,
  0xBC, 0x62, 0x5A, 0x46, 0x3D,
  0x3E, 0x49, 0x49, 0x49, 0x00,
  0x7E, 0x01, 0x01, 0x01, 0x7E,
  0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
  0x44, 0x44, 0x5F, 0x44, 0x44,
  0x40, 0x51, 0x4A, 0x44, 0x40,
  0x40, 0x44, 0x4A, 0x51, 0x40,
  0x00, 0x00, 0xFF, 0x01, 0x03,
  0xE0, 0x80, 0xFF, 0x00, 0x00,
  0x08, 0x08, 0x6B, 0x6B, 0x08,
  0x36, 0x12, 0x36, 0x24, 0x36,
  0x06, 0x0F, 0x09, 0x0F, 0x06,
  0x00, 0x00, 0x18, 0x18, 0x00,
  0x00, 0x00, 0x10, 0x10, 0x00,
  0x30, 0x40, 0xFF, 0x01, 0x01,
  0x00, 0x1F, 0x01, 0x01, 0x1E,
  0x00, 0x19, 0x1D, 0x17, 0x12,
  0x00, 0x3C, 0x3C, 0x3C, 0x3C,
  0x00, 0x00, 0x00, 0x00, 0x00,
};


static uint8_t ColStart, RowStart; // some displays need this changed
static int16_t _width = ST7735_TFTWIDTH;   // this could probably be a constant, except it is used in Adafruit_GFX and depends on image rotation
static int16_t _height = ST7735_TFTHEIGHT;




// The Data/Command pin must be valid when the eighth bit is
// sent.  The SSI module has hardware input and output FIFOs
// that are 8 locations deep.  Based on the observation that
// the LCD interface tends to send a few commands and then a
// lot of data, the FIFOs are not used when writing
// commands, and they are used when writing data.  This
// ensures that the Data/Command pin status matches the byte
// that is actually being transmitted.
// The write command operation waits until all data has been
// sent, configures the Data/Command pin for commands, sends
// the command, and then waits for the transmission to
// finish.
// The write data operation waits until there is room in the
// transmit FIFO, configures the Data/Command pin for data,
// and then adds the data to the transmit FIFO.
// NOTE: These functions will crash or stall indefinitely if
// the SSI0 module is not initialized and enabled.
void static writecommand(uint8_t c) {
                                        // wait until SSI0 not busy/transmit FIFO empty
  while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
  DC = DC_COMMAND;
  SSI0_DR_R = c;                        // data out
                                        // wait until SSI0 not busy/transmit FIFO empty
  while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
}


void static writedata(uint8_t c) {
  while((SSI0_SR_R&SSI_SR_TNF)==0){};   // wait until transmit FIFO not full
  DC = DC_DATA;
  SSI0_DR_R = c;                        // data out
}
// Subroutine to wait 1 msec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay1ms(uint32_t n){uint32_t volatile time;
  while(n){
    time = 72724*2/91;  // 1msec, tuned at 80 MHz
    while(time){
      time--;
    }
    n--;
  }
}

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in ROM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80

static const uint8_t
  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 };                 //     16-bit color
static const uint8_t
  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 };      //     XEND = 159
static const uint8_t
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F };           //     XEND = 159
static const uint8_t
  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in ROM byte array.
void static commandList(const uint8_t *addr) {

  uint8_t numCommands, numArgs;
  uint16_t ms;

  numCommands = *(addr++);               // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(*(addr++));             //   Read, issue command
    numArgs  = *(addr++);                //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(*(addr++));              //     Read, issue argument
    }

    if(ms) {
      ms = *(addr++);             // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      Delay1ms(ms);
    }
  }
}


// Initialization code common to both 'B' and 'R' type displays
void static commonInit(const uint8_t *cmdList) {
  volatile uint32_t delay;
  ColStart  = RowStart = 0; // May be overridden in init func

  SYSCTL_RCGCSSI_R |= 0x01;  // activate SSI0
  SYSCTL_RCGCGPIO_R |= 0x01; // activate port A
  while((SYSCTL_PRGPIO_R&0x01)==0){}; // allow time for clock to start

  // toggle RST low to reset; CS low so it'll listen to us
  // SSI0Fss is temporarily used as GPIO
  GPIO_PORTA_DIR_R |= 0xC8;             // make PA3,6,7 out
  GPIO_PORTA_AFSEL_R &= ~0xC8;          // disable alt funct on PA3,6,7
  GPIO_PORTA_DEN_R |= 0xC8;             // enable digital I/O on PA3,6,7
                                        // configure PA3,6,7 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FF0FFF)+0x00000000;
  GPIO_PORTA_AMSEL_R &= ~0xC8;          // disable analog functionality on PA3,6,7
  TFT_CS = TFT_CS_LOW;
  RESET = RESET_HIGH;
  Delay1ms(500);
  RESET = RESET_LOW;
  Delay1ms(500);
  RESET = RESET_HIGH;
  Delay1ms(500);

  // initialize SSI0
  GPIO_PORTA_AFSEL_R |= 0x2C;           // enable alt funct on PA2,3,5
  GPIO_PORTA_DEN_R |= 0x2C;             // enable digital I/O on PA2,3,5
                                        // configure PA2,3,5 as SSI
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF)+0x00202200;
  GPIO_PORTA_AMSEL_R &= ~0x2C;          // disable analog functionality on PA2,3,5
  SSI0_CR1_R &= ~SSI_CR1_SSE;           // disable SSI
  SSI0_CR1_R &= ~SSI_CR1_MS;            // master mode
                                        // configure for system clock/PLL baud clock source
  SSI0_CC_R = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
//                                        // clock divider for 3.125 MHz SSIClk (50 MHz PIOSC/16)
//  SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+16;
                                        // clock divider for 8 MHz SSIClk (80 MHz PLL/24)
                                        // SysClk/(CPSDVSR*(1+SCR))
                                        // 80/(10*(1+0)) = 8 MHz (slower than 4 MHz)
  SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+10; // must be even number
  SSI0_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (8 Mbps data rate)
                  SSI_CR0_SPH |         // SPH = 0
                  SSI_CR0_SPO);         // SPO = 0
                                        // FRF = Freescale format
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
                                        // DSS = 8-bit data
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;
  SSI0_CR1_R |= SSI_CR1_SSE;            // enable SSI

  if(cmdList) commandList(cmdList);
}

//********ST7735_SetCursor*****************
// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=20)
//         newY  new Y-position of the cursor (0<=newY<=15)
// outputs: none
void ST7735_SetCursor(uint32_t newX, uint32_t newY){
  if((newX > 20) || (newY > 15)){       // bad input
    return;                             // do nothing
  }
  StX = newX;
  StY = newY;
}




// Set the region of the screen RAM to be modified
// Pixel colors are sent left to right, top to bottom
// (same as Font table is encoded; different from regular bitmap)
// Requires 11 bytes of transmission
void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

  writecommand(ST7735_CASET); // Column addr set
  writedata(0x00);
  writedata(x0+ColStart);     // XSTART
  writedata(0x00);
  writedata(x1+ColStart);     // XEND

  writecommand(ST7735_RASET); // Row addr set
  writedata(0x00);
  writedata(y0+RowStart);     // YSTART
  writedata(0x00);
  writedata(y1+RowStart);     // YEND

  writecommand(ST7735_RAMWR); // write to RAM
}


// Send two bytes of data, most significant byte first
// Requires 2 bytes of transmission
void static pushColor(uint16_t color) {
  writedata((uint8_t)(color >> 8));
  writedata((uint8_t)color);
}


//------------ST7735_DrawPixel------------
// Color the pixel at the given coordinates with the given color.
// Requires 13 bytes of transmission
// Input: x     horizontal position of the pixel, columns from the left edge
//               must be less than 128
//               0 is on the left, 126 is near the right
//        y     vertical position of the pixel, rows from the top edge
//               must be less than 160
//               159 is near the wires, 0 is the side opposite the wires
//        color 16-bit color, which can be produced by ST7735_Color565()
// Output: none
void ST7735_DrawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

//  setAddrWindow(x,y,x+1,y+1); // original code, bug???
  setAddrWindow(x,y,x,y);

  pushColor(color);
}



//------------ST7735_DrawFastHLine------------
// Draw a horizontal line at the given coordinates with the given width and color.
// A horizontal line is parallel to the shorter side of the rectangular display
// Requires (11 + 2*w) bytes of transmission (assuming image fully on screen)
// Input: x     horizontal position of the start of the line, columns from the left edge
//        y     vertical position of the start of the line, rows from the top edge
//        w     horizontal width of the line
//        color 16-bit color, which can be produced by ST7735_Color565()
// Output: none
void ST7735_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  uint8_t hi = color >> 8, lo = color;

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  while (w--) {
    writedata(hi);
    writedata(lo);
  }
}

//------------ST7735_FillRect------------
// Draw a filled rectangle at the given coordinates with the given width, height, and color.
// Requires (11 + 2*w*h) bytes of transmission (assuming image fully on screen)
// Input: x     horizontal position of the top left corner of the rectangle, columns from the left edge
//        y     vertical position of the top left corner of the rectangle, rows from the top edge
//        w     horizontal width of the rectangle
//        h     vertical height of the rectangle
//        color 16-bit color, which can be produced by ST7735_Color565()
// Output: none
void ST7735_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  uint8_t hi = color >> 8, lo = color;

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      writedata(hi);
      writedata(lo);
    }
  }
}

//------------ST7735_FillScreen------------
// Fill the screen with the given color.
// Requires 40,971 bytes of transmission
// Input: color 16-bit color, which can be produced by ST7735_Color565()
// Output: none
void ST7735_FillScreen(uint16_t color) {
  ST7735_FillRect(0, 0, _width, _height, color);  // original
//  screen is actually 129 by 161 pixels, x 0 to 128, y goes from 0 to 160
}


//------------ST7735_InitR------------
// Initialization for ST7735R screens (green or red tabs).
// Input: option one of the enumerated options depending on tabs
// Output: none
void ST7735_InitR(enum initRFlags option) {
  commonInit(Rcmd1);
  if(option == INITR_GREENTAB) {
    commandList(Rcmd2green);
    ColStart = 2;
    RowStart = 1;
  } else {
    // colstart, rowstart left at default '0' values
    commandList(Rcmd2red);
  }
  commandList(Rcmd3);

  // if black, change MADCTL color filter
  if (option == INITR_BLACKTAB) {
    writecommand(ST7735_MADCTL);
    writedata(0xC0);
  }
  ST7735_SetCursor(0,0);
  StTextColor = ST7735_YELLOW;
  ST7735_FillScreen(0);                 // set screen to black
}

//------------ST7735_Color565------------
// Pass 8-bit (each) R,G,B and get back 16-bit packed color.
// Input: r red value
//        g green value
//        b blue value
// Output: 16-bit color
uint16_t ST7735_Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
}


//------------ST7735_DrawCharS------------
// Simple character draw function.  This is the same function from
// Adafruit_GFX.c but adapted for this processor.  However, each call
// to ST7735_DrawPixel() calls setAddrWindow(), which needs to send
// many extra data and commands.  If the background color is the same
// as the text color, no background will be printed, and text can be
// drawn right over existing images without covering them with a box.
// Requires (11 + 2*size*size)*6*8 (image fully on screen; textcolor != bgColor)
// Input: x         horizontal position of the top left corner of the character, columns from the left edge
//        y         vertical position of the top left corner of the character, rows from the top edge
//        c         character to be printed
//        textColor 16-bit color of the character
//        bgColor   16-bit color of the background
//        size      number of pixels per character pixel (e.g. size==2 prints each pixel of font as 2x2 square)
// Output: none
void ST7735_DrawCharS(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; // vertical column of pixels of character in font
  int32_t i, j;
  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + 5 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  for (i=0; i<6; i++ ) {
    if (i == 5)
      line = 0x0;
    else
      line = Font[(c*5)+i];
    for (j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          ST7735_DrawPixel(x+i, y+j, textColor);
        else {  // big size
          ST7735_FillRect(x+(i*size), y+(j*size), size, size, textColor);
        }
      } else if (bgColor != textColor) {
        if (size == 1) // default size
          ST7735_DrawPixel(x+i, y+j, bgColor);
        else {  // big size
          ST7735_FillRect(x+i*size, y+j*size, size, size, bgColor);
        }
      }
      line >>= 1;
    }
  }
}


//------------ST7735_DrawChar------------
// Advanced character draw function.  This is similar to the function
// from Adafruit_GFX.c but adapted for this processor.  However, this
// function only uses one call to setAddrWindow(), which allows it to
// run at least twice as fast.
// Requires (11 + size*size*6*8) bytes of transmission (assuming image fully on screen)
// Input: x         horizontal position of the top left corner of the character, columns from the left edge
//        y         vertical position of the top left corner of the character, rows from the top edge
//        c         character to be printed
//        textColor 16-bit color of the character
//        bgColor   16-bit color of the background
//        size      number of pixels per character pixel (e.g. size==2 prints each pixel of font as 2x2 square)
// Output: none
void ST7735_DrawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; // horizontal row of pixels of character
  int32_t col, row, i, j;// loop indices
  if(((x + 5*size - 1) >= _width)  || // Clip right
     ((y + 8*size - 1) >= _height) || // Clip bottom
     ((x + 5*size - 1) < 0)        || // Clip left
     ((y + 8*size - 1) < 0)){         // Clip top
    return;
  }

  setAddrWindow(x, y, x+6*size-1, y+8*size-1);

  line = 0x01;        // print the top row first
  // print the rows, starting at the top
  for(row=0; row<8; row=row+1){
    for(i=0; i<size; i=i+1){
      // print the columns, starting on the left
      for(col=0; col<5; col=col+1){
        if(Font[(c*5)+col]&line){
          // bit is set in Font, print pixel(s) in text color
          for(j=0; j<size; j=j+1){
            pushColor(textColor);
          }
        } else{
          // bit is cleared in Font, print pixel(s) in background color
          for(j=0; j<size; j=j+1){
            pushColor(bgColor);
          }
        }
      }
      // print blank column(s) to the right of character
      for(j=0; j<size; j=j+1){
        pushColor(bgColor);
      }
    }
    line = line<<1;   // move up to the next row
  }
}
//------------ST7735_DrawString------------
// String draw function.
// 16 rows (0 to 15) and 21 characters (0 to 20)
// Requires (11 + size*size*6*8) bytes of transmission for each character
// Input: x         columns from the left edge (0 to 20)
//        y         rows from the top edge (0 to 15)
//        pt        pointer to a null terminated string to be printed
//        textColor 16-bit color of the characters
// bgColor is Black and size is 1
// Output: number of characters printed
uint32_t ST7735_DrawString(uint16_t x, uint16_t y, char *pt, int16_t textColor){
  uint32_t count = 0;
  if(y>15) return 0;
  while(*pt){
    ST7735_DrawCharS(x*6, y*10, *pt, textColor, ST7735_BLACK, 1);
    pt++;
    x = x+1;
    if(x>20) return count;  // number of characters printed
    count++;
  }
  return count;  // number of characters printed
}

//-----------------------fillmessage-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
char Message[12];
uint32_t Messageindex;

void fillmessage(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    fillmessage(n/10);
    n = n%10;
  }
  Message[Messageindex] = (n+'0'); /* n is between 0 and 9 */
  if(Messageindex<11)Messageindex++;
}

//-----------------------ST7735_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Position determined by ST7735_SetCursor command
// Color set by ST7735_SetTextColor
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void ST7735_OutUDec(uint32_t n){
  Messageindex = 0;
  fillmessage(n);
  Message[Messageindex] = 0; // terminate
  ST7735_DrawString(StX,StY,Message,StTextColor);
  StX = StX+Messageindex;
  if(StX>20){
    StX = 20;
    ST7735_DrawCharS(StX*6,StY*10,'*',ST7735_RED,ST7735_BLACK, 1);
  }
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04


// *************** ST7735_OutChar ********************
// Output one character to the LCD
// Position determined by ST7735_SetCursor command
// Color set by ST7735_SetTextColor
// Inputs: 8-bit ASCII character
// Outputs: none
void ST7735_OutChar(char ch){
  if((ch == 10) || (ch == 13) || (ch == 27)){
    StY++; StX=0;
    if(StY>15){
      StY = 0;
    }
    ST7735_DrawString(0,StY,"                     ",StTextColor);
    return;
  }
  ST7735_DrawCharS(StX*6,StY*10,ch,ST7735_YELLOW,ST7735_BLACK, 1);
  StX++;
  if(StX>20){
    StX = 20;
    ST7735_DrawCharS(StX*6,StY*10,'*',ST7735_RED,ST7735_BLACK, 1);
  }
  return;
}
//********ST7735_OutString*****************
// Print a string of characters to the ST7735 LCD.
// Position determined by ST7735_SetCursor command
// Color set by ST7735_SetTextColor
// The string will not automatically wrap.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
void ST7735_OutString(char *ptr){
  while(*ptr){
    ST7735_OutChar(*ptr);
    ptr = ptr + 1;
  }
}
// ************** ST7735_SetTextColor ************************
// Sets the color in which the characters will be printed
// Background color is fixed at black
// Input:  16-bit packed color
// Output: none
// ********************************************************
void ST7735_SetTextColor(uint16_t color){
  StTextColor = color;
}
// Print a character to ST7735 LCD.
int fputc(int ch, FILE *f){
  ST7735_OutChar(ch);
  return 1;
}
// No input from Nokia, always return data.
int fgetc (FILE *f){
  return 0;
}
// Function called when file error occurs.
int ferror(FILE *f){
  /* Your implementation of ferror */
  return EOF;
}
// Abstraction of general output device
// Volume 2 section 3.4.5


// *************** Output_Init ********************
// Standard device driver initialization function for printf
// Initialize ST7735 LCD
// Inputs: none
// Outputs: none
void Output_Init(void){
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0);                 // set screen to black
	
}

// Clear display
void Output_Clear(void){ // Clears the display
  ST7735_FillScreen(0);    // set screen to black
}
// Turn off display (low power)
void ST7735_Off(void){   // Turns off the display
  Output_Clear();  // not implemented
}
// Turn on display
void ST7735_On(void){ // Turns on the display
  Output_Init();      // reinitialize
}
// set the color for future output
// Background color is fixed at black
// Input:  16-bit packed color
// Output: none
void Output_Color(uint32_t newColor){ // Set color of future output
  ST7735_SetTextColor(newColor);
}


//------------ST7735_Message------------
// Output message to ST7735 LCD
// LCD screen is split into top and bottom halves
// Each half has 4 lines
// Input: device   zero  top half, nonzero bottom half
//        line     0-3 which line to print on
//        string   pointer to a null terminated string to be printed
//        value    value to be printed 
// Output: none
void ST7735_Message (int device, int line, char *string, int32_t value){
	
	line = line * 2;	                                                        // make a line space
	ST7735_DrawFastHLine(0, 76, 126, ST7735_WHITE);														// draw line to seperate two devices
  if(device){																																// device 1 = bottom half of screen
		if(line > 7){																														// prevents writes past the vertical boundary
			return;
		}
		else{
			if(strlen(string) < 20){																							// prevents writes past the horizontal boundary
				ST7735_DrawString(0, 8 + line, string, ST7735_YELLOW);              // Output string
				char str[5];                                                        // Convert value into a string
        sprintf(str, "%d", value);
				ST7735_DrawString(14, 8 + line, str, ST7735_YELLOW);                // Output value
			}
		}
	}
	else{																																			// device 0 = top half of screen 
		if(line > 7){                                                           // rest is behavior of device 1
			return;
		}
		else{
			if(strlen(string) < 20){
				ST7735_DrawString(0, line, string, ST7735_YELLOW);
				char str[5];
        sprintf(str, "%d", value);
				ST7735_DrawString(14, line, str, ST7735_YELLOW);
			}
		}
	}
	
}
