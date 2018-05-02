/***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
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

// ST7735.h
// Runs on LM4F120/TM4C123
// Low level drivers for the ST7735 160x128 LCD based off of
// the file described above.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano, March 30, 2015
// Augmented 7/17/2014 to have a simple graphics facility
// Tested with LaunchPadDLL.dll simulator 9/2/2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

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
// ST7735.h was copied from VALVANOWARE. Unnecessary functions and definitions deleted. Only function 
// prototypes that are meant to be public and their respective definitions remain. 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef _ST7735H_
#define _ST7735H_

// some flags for ST7735_InitR()
enum initRFlags{
  none,
  INITR_GREENTAB,
  INITR_REDTAB,
  INITR_BLACKTAB
};

#define ST7735_TFTWIDTH  128
#define ST7735_TFTHEIGHT 160


// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0xF800
#define ST7735_RED     0x001F
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0xFFE0
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0x07FF
#define ST7735_WHITE   0xFFFF


// Turn off display (low power)
void ST7735_Off(void);
void ST7735_InitR();

// Turn on display
void ST7735_On(void);

//------------ST7735_Message------------
// Output message to ST7735 LCD
// LCD screen is split into top and bottom halves
// Each half has 4 lines
// Input: device   zero  top half, nonzer0 bottom half
//        line     0-3 which line to print on
//        string   pointer to a null terminated string to be printed
//        value    value to be printed 
// Output: none
void ST7735_Message(int device, int line, char *string, int32_t value);


#endif
