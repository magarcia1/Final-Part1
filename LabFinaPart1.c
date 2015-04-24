/* 
 * File:   LabFinaPart1.c
 * Author: Rubi Ballesteros
 *
 * Created on April 8, 2015, 12:40 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "p24FJ64GA002.h"
#include "lcd.h"
#include "timer.h"
#include "adc.h"
#include "pwm.h"
#include "initSW.h"
#include <stdio.h>

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_ON & COE_OFF & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

#define true 1   // define true to use with bool data type.
#define false 0  // define false for bool data type.
#define bool int // defines the boolean data type.

#define PRESSED 0 //DOUBLE CHECK THIS it is for switch rb15
#define RELEASED 1 // Define the released state of the switch.

#define black 0
#define white 1

#define ON 0
#define OFF 1

//Use defines for pin settings to make your code
#define LED4 LATBbits.LATB15
#define LED5 LATBbits.LATB14
#define LED6 LATBbits.LATB13

#define changeThreshold 300 //The threshold at which the black line is detected
#define RightChangeThreshold  300

//Pins used for the sensors:
//left 23, middle 24, right 25
//Rb 12-14

//FSM STATES--------------------
typedef enum stateTypeEnum
{
        forward,
        turnLeftState,
        turnRightState,
        //TODO: Make the robot turn around 180 degrees
        //turnAround, 
        wait,

} stateType;


//Waiting until the switch is pressed.
volatile stateType currState;

volatile int sensorLeftReading;
volatile int sensorMiddleReading;
volatile int sensorRightReading;

volatile int sensorLeft;
volatile int sensorMiddle;
volatile int sensorRight;



volatile int linesDetected = 0 ;


int main(void) {
    //Initialize components
    //char v[3];
    initLEDs();
    initPWMLeft();
    initPWMRight();
    initADC();
   // initLCD();
    initSW1();
   // clearLCD();

//   sensorLeft = 1;
//   sensorMiddle = 0;
//   sensorRight= 1;

    currState = wait;

    while (1){
       //Make the LCD display the reading from all three sensors.
//        clearLCD();
//        sprintf(v, "%d %d %d" , sensorLeft, sensorMiddle, sensorRight);
//        printStringLCD(v);
       
//       delayMs(10);
          sensorRightReading = rightSensorADC();
          sensorMiddleReading = middleSensorADC();
          sensorLeftReading = leftSensorADC();
          
          

       //Case Statement: FSM
       switch (currState){
            case wait:
                idleFunction();
                break;
            case forward:
               spinForward();
                //spinBackward();
                break;
            case turnRightState:
                turnRight(); //change the function
                break;
            case turnLeftState:
                turnLeft();  //change function
                break;
//            case turnAround:
//                turnAround(); //change function
//                break;
       }

    }
}

//Interrupt to get out of the Wait state
void _ISR _CNInterrupt(void) {

    IFS1bits.CNIF = 0; //put the flag down
//    delayMs(10);

     if(_RB5 == PRESSED){

         if (currState != wait){
             currState = wait;
         }

         else if (currState == wait){
            currState = forward;
         }

     }
     
}

/*
 This interrupts is the one that "listens" the infrared sensors and decides which way to turn
 depending on which sensors are picking up signals.
 */
void _ISR _ADC1Interrupt(void){
    IFS0bits.AD1IF = 0; //Put the interrupt flag down
//    delayMs(500);
 //Set threshold to turn readings from ADC buffer into a 0 or 1-------------

    if(sensorRightReading < RightChangeThreshold){ //Buffer of pin 25
        sensorRight = black;
        turnOnLED(4);
    }
    else {
        sensorRight = white;
    }

    if(sensorMiddleReading < changeThreshold){ //Buffer of pin 24
        sensorMiddle = black;
        turnOnLED(5);
    }
    else {
        sensorMiddle = white;
    }

    if(sensorLeftReading < changeThreshold){ //Buffer of pin 23
        sensorLeft = black;
        turnOnLED(6);
    }
    else {
        sensorLeft = white;
    }

//-------------------------------------------------------------------------

//    sensorRight = ADC1BUFA;
//    sensorMiddle = ADC1BUFB;
//    sensorLeft = ADC1BUFC;

//Decide on state based on sensor reading. Right sensor has priority.---------
    if(currState != wait){

        //If the sensors are exactly on the line and outside the line.

        if (sensorLeft == white && sensorMiddle == black && sensorRight == white){
            currState = forward;    //keep moving forward
        }

        //BBB = WWW
        //if there is no line keep turning right until the line is found.
        else if (sensorLeft == white && sensorRight == white && sensorMiddle == white){
            currState = turnRightState;
        }

        //If the sensor on the right detects a curve.
        else if (sensorLeft == white && sensorMiddle == white && sensorRight == black){
            currState = turnRightState;
        }
        else if (sensorLeft == white && sensorMiddle == black && sensorRight == black){
            currState = turnRightState;
        }


        //If the sensor on the left detects a curve.
        else if (sensorLeft == black && sensorMiddle == black && sensorRight == white){
            currState = turnLeftState;
        }
        else if (sensorLeft == black && sensorMiddle == white && sensorRight == white){
            currState = turnLeftState;
        }

   }
//----------------------------------------------------------------------------

}


void initLEDs(){
	TRISBbits.TRISB15 = 0;
	TRISBbits.TRISB14 = 0;
	TRISBbits.TRISB13 = 0;


	LED4 = OFF;
	LED5 = OFF;
	LED6 = OFF;

}

void turnOnLED(int led){
//	if (led == 4){
//		LED4 = ON;
//
//	}
//	else if (led == 5){
//		LED5 = ON;
//
//	}
//	else if (led == 6){
//		LED6 = ON;
//	}

        if (sensorRight == black){
            LED4 = ON;
        }
        else{
            LED4 = OFF;
        }

        if (sensorMiddle == black){
            LED5 = ON;
        }
        else{
            LED5 = OFF;
        }

        if (sensorLeft == black){
            LED6 = ON;
        }
        else{
            LED6 = OFF;
        }
	
	
}