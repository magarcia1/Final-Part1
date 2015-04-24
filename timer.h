/*
 * File:   timer.h
 * Author: MiguelAnguel
 *
 * Created on March 23, 2015, 11:42 AM
 */

#ifndef TIMER_H
#define	TIMER_H

#define FCY 14745600    //Defines the Frequency Cycle (half of reg cycle)
#define ONE_MILLISECOND 57
#define ONE_MICROSECOND 14
#define TIME_DELAY .01  //Time delay for PR calculation
#define PRE_SCALAR 256  //Sets pre scalar value for the PR calculation
#define PRE_SCALAR1 1  //Sets pre scalar value for the PR calculation

void delayUs(unsigned int delay);
void delayMs(unsigned int delay);
void initTimer1();

#endif	/* TIMER_H */