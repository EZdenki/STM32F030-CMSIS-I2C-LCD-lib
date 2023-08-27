# STM32F030-CMSIS-I2C-LCD-lib
A simple library that gives basic initialization and write commands to an LCD connected to a PCF8574-based I2C
LCD driver module.

## The following routines are supported:
+ **```void  I2C_LCD_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed )```**<br>
Initialize the LCD display module and the associated I2C peripheral. Note that the STM32F030F4 only
supports I2C1. I2CSpeed supports speeds from 10 kHz to approx 400 kHz.<br>
**Note**: Commands are sent via 4-bit mode even though all 8 IO lines on the LCD are connected. This
is because the code was ported from the non-I2C code which used 4-bit mode.

+ **```void  I2C_LCD_cmd( uint8_t data )```**<br>
Send command (not character) to LCD display. Commands are sent like data but with the RS pin
set LOW. Will tack on extra delay when given clear or home commands as outlined in the
datasheet.<br>
The following command codes are supported:<br>
```LCD_CLEAR```               : Clear display<br>
```LCD_HOME```                 : Move to home position<br>
```LCD_OFF```                  : Turn off LCD<br>
```LCD_ON_NO_CURSOR```         : Turn on LCD, no cursor<br>
```LCD_ON_BLINK_CURSOR```      : Turn on LCD, blinking block cursor<br>
```LCD_ON_LINE_CURSOR```       : Turn on LCD, underline cursor<br>
```LCD_ON_LINE_BLINK_CURSOR``` : Turn on LCD, blinking underline cursor<br>
```I2C_LCD_4B```               : 4-bit mode (for initialization)<br>
```LCD_4B_58F_2L```            : 4-bit, 5x8 character, 2 lines<br>
```LCD_8B_58F_2L```            : 8-bit, 5x8 character, 2 lines<br>
```LCD_1ST_LINE```             : Position cursor at beginning of 1st line<br>
```LCD_2ND_LINE```             : Position cursor at beginning of 2nd line

+ **```void  I2C_LCD_putc( char data )```**<br>
Writes a single character to the current position on the LCD display

+ **```void  I2C_LCD_puts( char *data )```**<br>
Takes a pointer to a null-terminated string and displays that string from the current LCD 
cursor position. Does not check for LCD line/string overflow.

+ **```void  i100toa( int16_t realV, char *thisString )```**<br>
i100toa takes a number with 2 decimal places multiplied by 100 and a pointer to a string,
and returns a string of the original decimal number rounded to 1 decimal place. For example,
if the number in question is 12.36, then 1236 is passed via realV. The resulting string is 12.4,
because 12.36 rounds up to 12.4. Negative numbers and more complex rounding work as
expected. For example, -2.35, passed as -235, returns "-2.4".
