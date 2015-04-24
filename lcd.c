/*
 * File:   lcd.c
 * Authors:
 *
 * Created on March 23, 2015, 11:36 AM
 */

#include "p24FJ64GA002.h"
#include "lcd.h"
#include "timer.h"

#define LCD_DATA   LATB
#define LCD_RS  LATBbits.LATB7
#define LCD_E   LATBbits.LATB6
#define LCD_WRITE_DATA    1
#define LCD_WRITE_CONTROL 0

#define TRIS_D7  TRISBbits.TRISB15
#define TRIS_D6  TRISBbits.TRISB14
#define TRIS_D5  TRISBbits.TRISB13
#define TRIS_D4  TRISBbits.TRISB12
#define TRIS_RS  TRISBbits.TRISB7
#define TRIS_E   TRISBbits.TRISB6

#define UPPER 0
#define LOWER 1

/* This function should take in a two-byte word and writes either the lower or upper
 * byte to the last four bits of LATB. Additionally, according to the LCD data sheet
 * It should set LCD_RS and LCD_E with the appropriate values and delays.
 * After these commands are issued, there should be a delay.
 * The command type is a simplification. From the data sheet, the RS is '1'
 * when you are simply writing a character. Otherwise, RS is '0'.
 */
void writeFourBits(unsigned char word, unsigned int commandType, unsigned int delayAfter, unsigned int lower)
{
    /*This will take the bottom 4 bits of char word and store them to the top 4 bits of LATB
    * i.e. LATB15, LATB14, LATB13, LATB12
    */
    if(lower)
    {
        LATB = (word << 12);
    }
    else
    {
    /*This will take the bottom 4 bits of char word and store them to the top 4 bits of LATB
     * i.e. LATB15, LATB14, LATB13, LATB12
     */
        LATB = (word >> 4) << 12;
    }
    LCD_RS = commandType; delayUs(1);
    LCD_E = 1;  delayUs(1); //minimum 230 ns
    LCD_E = 0;  delayUs(1);
    delayUs(delayAfter);
}

/* Using writeFourBits, this function should write the two bytes of a character
 * to the LCD.
 */
void writeLCD(unsigned char word, unsigned int commandType, unsigned int delayAfter)
{
    writeFourBits(word, commandType, delayAfter, UPPER);
    writeFourBits(word, commandType, delayAfter, LOWER);
}

/* Given a character, write it to the LCD. RS should be set to the appropriate value.
 */
void printCharLCD(char c)
{
    writeLCD(c, LCD_WRITE_DATA, 46);
}

/*Initialize the LCD
 */
void initLCD(void)
{
    // Setup D, RS, and E to be outputs (0).
    TRIS_RS = 0;    //Sets RS to be output
    TRIS_E = 0;     //Sets Enable to be output
    TRIS_D7 = 0;
    TRIS_D6 = 0;
    TRIS_D5 = 0;
    TRIS_D4 = 0;

    // Initilization sequence utilizes specific LCD commands before the general configuration
    // commands can be utilized. The first few initilition commands cannot be done using the
    // WriteLCD function. Additionally, the specific sequence and timing is very important.
    delayUs(1500);

    writeFourBits(0x03, LCD_WRITE_CONTROL, 4100, LOWER);

    writeFourBits(0x03, LCD_WRITE_CONTROL, 100, LOWER);

    writeLCD(0x32,LCD_WRITE_CONTROL, 40);
    // 4-bit mode initialization is complete. We can now configure the various LCD
    // options to control how the LCD will function.

    writeLCD(0x28,LCD_WRITE_CONTROL, 40);

    
        // Turn Display (D) Off
    writeLCD(0x08,LCD_WRITE_CONTROL, 40);

    
    //You really need to have the clear display delay here.
    writeLCD(0x01,LCD_WRITE_CONTROL, 1640);

    
        // Set Increment Display, No Shift (i.e. cursor move)
    writeLCD(0x06,LCD_WRITE_CONTROL, 40);


  // Turn Display (D) On, Cursor (C) Off, and Blink(B) Off
    writeLCD(0x0C,LCD_WRITE_CONTROL, 40);
}


/*
 * You can use printCharLCD here. Note that every time you write a character
 * the cursor increments its position automatically.
 * Since a string is just a character array, try to be clever with your use of pointers.
 */
void printStringLCD(const char* s)
{
    /*This function prints the current array element of char* s and prints it to the lcd using printCharLCD.
     It then increments the array element and prints it.
     */
    int i = 0;

    while( s[i] != '\0')
    {
       printCharLCD(s[i]);
       i++;
    }
}

/*
 * Clear the display.
 */
void clearLCD()
{
    writeLCD(0x01,LCD_WRITE_CONTROL, 1640);
}
/*
 Use the command for changing the DD RAM address to put the cursor somewhere.
 */
void moveCursorLCD(unsigned char x, unsigned char y)
{
int ddAddress = 0x80;
ddAddress = ddAddress + y;
/*This adds 0x80 to the*/
if( x == 1)
{
    ddAddress = ddAddress + 0x40;
}
   writeLCD(ddAddress,LCD_WRITE_CONTROL, 40);
// writeLCD(ddAddress,LCD_WRITE_CONTROL, 40);
/*NOTE: If the above IF statement does not work, the commented section below will work for where to set the cursor.*/
    writeLCD(0xC0,LCD_WRITE_CONTROL, 40); //This will move the cursor to the second row for the Timer display.
}

void clearPrintStringLCD(const char* s){
    clearLCD();
    int i = 0;
    while( s[i] != '\0')
    {
        if (i != 8){
            printCharLCD(s[i]);
            i++;
        }
        else{
            moveCursorLCD(1,0);
            printCharLCD(s[i]);
            i++;
        }
    }
}
