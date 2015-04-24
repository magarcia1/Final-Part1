#include "p24FJ64GA002.h"

#define DIGITAL 1;
#define ANALOG 0;

/*****************************************************************
 * This function initializes SW1 to be an input pin and enables
 * the change notification for the switch.
*****************************************************************/
void initSW1()
{

    TRISBbits.TRISB5 = 1;  //Pin for SW1

    //Put the overall CN Interrupt flag down (set it to 0)
    IFS1bits.CNIF = 0;

    //Enable the overall change notification interrupt
    IEC1bits.CNIE = 1;

    /* Enable the change notification interrupt for the switch 1 connected to
     * the development board. This corresponds to CN27 according to the PIC24F
     * datasheet. The internal pull-up should not be used for the switch
     * on the development board because one is already there.
     */
    CNEN2bits.CN27IE = 1;
}
