//  ==========================================================================================
//  main.c for STM32F030-CMSIS-I2C-LCD-lib.c
//  ------------------------------------------------------------------------------------------
//  A sample project to demostrate driving an I2C-driven LCD display using the
//  STM32F030-CMSIS-I2C-LCD-lib.c library.
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-I2C-LCD-lib
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//
//    Version 0.9   26 Aug 2023   Inital port from STM32F103-CMSIS-I2C-LCD-lib
//  ------------------------------------------------------------------------------------------
//  Target Devices:
//    STM32F030Fxxx running at 8 MHz internal clock
//    I2C-LCD Driver module based on the PCF8574 I/O Expander for I2C Bus
//    16x2 (or other) LCD module
//  ------------------------------------------------------------------------------------------
//  HARDWARE SETUP:
//    While a 5V 16x2 LCD module can be driven mostly with 3.3 logic levels, and while the I2C
//    display driver module can operate in a 3.3V system, the combination probably requires
//    that, if using a 3.3V microcontroller, the I2C LCD driver module itself must be powered
//    by 5 V in order to properly drive a 5V LCD module.
//
//    The 16 pins of the I2C LCD driver module are connected to the corresponding pins on the
//    16x2 LCD module. The I2C data lines (SCA and SCL) lines can be directly connected to
//    the I2C lines on the microcontroller as PA9 and PA10 (I2C SCL and SDA lines) are 5V
//    tolerant.
//
//    Pullup resistors are built into the I2C LCD driver module, so additional pullup
//    resistors on the PA9 and PA10 I2C lines on the STM32F030 are not needed.
//
//    If the display seems too bright with the backlight jumper installed, then the
//    jumper can be replaced with a resistor of hundreds of ohms or higher to dim
//    the display. Technically the backlight on the LCD could be driven via PWM, but
//    as it is being controlled via I2C on the I2C LCD driver module, this is not
//    practical to implement.
//
//                  STM32F030Fxxx
//                   ,---. .---,                             PC8574-based
//             BOOT0 |1o  V  20| PA14 / SWCLK           I2C LCD Driver Module
//            OSC_IN |2      19| PA13 / SWDIO              ,-------------, 
//           OSC_OUT |3      18| PA10 / I2C1_SDA --------- | SDA         |
//              NRST |4      17| PA9  / I2C1_SCL --------- | SCL         |
//              VDDA |5      16| VCC               GND --- | GND         |
//               PA0 |6      15| GND               VCC --- | VIN         |
//               PA1 |7      14| PB1                       |             |
//               PA2 |8      13| PA7          ,-- [1K] --- | LED Jumper  |
//               PA3 |9      12| PA6          '----------- | LED Jumper  |
//               PA4 |10     11| PA5                       '-------------'
//                   '---------'
//                                     ** Pullups on SDA/SCL not required as they are
//                                        built into the I2C driver module
//
//             I2C LCD Driver Module
//     ======================================
//     1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//     ======================================
//                16x2 LCD Module
//
//  ==========================================================================================

#include "stm32f030x6.h"                  // Primary CMSIS header file
#include "STM32F030-Delay-lib.c"
I2C_TypeDef *LCD_I2C;                     // Needed for below I2C LCD driver library
#include "STM32F030-CMSIS-I2C-LCD-lib.c"  // I2C LCD driver library


// ============================================================================
// main
// ============================================================================
int
main()
{
  char myString[] = "Hello World!";   // Define "string"

  I2C_LCD_init( I2C1, 100e3);         // Set the LCD interface to I2C2 at 100 kHz
  I2C_LCD_cmd( LCD_4B_58F_2L );       // Set LCD to 4-bit, 5x8 character set, 2 lines
  I2C_LCD_cmd( LCD_CLEAR );           // Clear LCD
  I2C_LCD_cmd( LCD_HOME );            // Move LCD to home position
  I2C_LCD_cmd( LCD_ON_NO_CURSOR );    // Turn on LCD

  I2C_LCD_puts( myString );           // Display string
  I2C_LCD_cmd( LCD_1ST_LINE + 13 );   // Adjust display position
  I2C_LCD_putc( '1' );                // Display individual characters
  I2C_LCD_putc( '2' );
  I2C_LCD_putc( '3' );
  for(uint32_t x=0; x<615e3; x++ ) ;  // 1-second delay (1 for-loop iteration = 13 clock cycles)

  I2C_LCD_cmd( LCD_2ND_LINE + 1 );    // Move to 2nd postion on 2nd line
  I2C_LCD_puts( "I2C-LCD-lib.c" );    // Display string constant
  I2C_LCD_cmd( LCD_8B_58F_2L );       // Return to 8-bit mode
  return 1;
}  
