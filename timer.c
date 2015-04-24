/*
 * File:   timer.c
 * Authors: Phillip Showers, Miguel Garcia, Angel Lopez
 *
 * Created on March 23, 2015, 11:36 AM
 */

#include "p24FJ64GA002.h"
#include "timer.h"

    void delayUs(unsigned int delay)
{
    TMR2 = 0; //According to the datasheet this should be reset automatically, but it does not sometimes.
    PR2 = delay*ONE_MICROSECOND; //Period register value calculated from lecture
    T2CONbits.TCKPS = 0; //prescalar 1
    IFS0bits.T2IF = 0; // Timer 2 interrupt flag down
    T2CONbits.TON = 1; // Turn timer 2 on

    // Wait until the timer 1 interrupt flag goes up. This is done in hardware.
    while(IFS0bits.T2IF == 0);

    IFS0bits.T2IF = 0; // Put the flag down afterwards.
    T2CONbits.TON = 0; // Turn the timer off so it does not keep counting.
}

/* Delay 1 millisecond * the input (delay)
 */
void delayMs(unsigned int delay)
{
    TMR1 = 0; //According to the datasheet this should be reset automatically, but it does not sometimes.
    PR1 = delay*ONE_MILLISECOND; //Period register value calculated from lecture
    T1CONbits.TCKPS = 0b11; //prescalar 256
    IFS0bits.T1IF = 0; // Timer 1 interrupt flag down
    T1CONbits.TON = 1; // Turn timer 1 on

    // Wait until the timer 1 interrupt flag goes up. This is done in hardware.
    while(IFS0bits.T1IF == 0);
    IFS0bits.T1IF = 0; // Put the flag down afterwards.
    T1CONbits.TON = 0; // Turn timer 1 off

}

/*****************************************************************
 * This function initializes Timer 1 and enables the ISR for that
 * timer. It also sets the PR value for that ISR.
 * NOTE: Changing the TIME_DELAY will change the value at which
 * the counter will need to increment to to reach 2 seconds.
 * Calculating that value for counter is: 2/TIME_DELAY
*****************************************************************/

void initTimer1()
{
    unsigned int prVal = (FCY*TIME_DELAY)/PRE_SCALAR - 1; //Calcs PR val
    PR1 = prVal; //Sets PR1 to the calculated value based on a given time delay

    T1CONbits.TCKPS = 0b11; //prescalar 256
    IEC0bits.T1IE = 1; // Enable the interrupt
    IFS0bits.T1IF = 0; // Put the interrupt flag down
    T1CONbits.TON = 1; // Turns Timer 1 on

}
