/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "openmote-cc2538.h"

#include "Callback.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void timer0_callback(void);
static void timer1_callback(void);
static void timer2_callback(void);
static void timer3_callback(void);

static PlainCallback timerCallback0(timer0_callback);
static PlainCallback timerCallback1(timer1_callback);
static PlainCallback timerCallback2(timer2_callback);
static PlainCallback timerCallback3(timer3_callback);

/*=============================== variables =================================*/

/*================================= public ==================================*/

int main (void)
{
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Initialize Timer0
    timer0.init(800000);
    timer0.setCallback(&timerCallback0);
    timer0.enableInterrupts();

    // Initialize Timer1
    timer1.init(1600000);
    timer1.setCallback(&timerCallback1);
    timer1.enableInterrupts();

    // Initialize Timer2
    timer2.init(3200000);
    timer2.setCallback(&timerCallback2);
    timer2.enableInterrupts();

    // Initialize Timer3
    timer3.init(6400000);
    timer3.setCallback(&timerCallback3);
    timer3.enableInterrupts();

    // Start Timer0, 1, 2 and 3
    timer0.start();
    timer1.start();
    timer2.start();
    timer3.start();
    
    // Enable interrupts
    board.enableInterrupts();

    // Forever
    while(true)
    {
        // Sleep
        board.sleep();
    }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void timer0_callback(void)
{
    // Toggle green LED and AD0 debug pin
    led_green.toggle();
}

static void timer1_callback(void)
{
    // Toggle yellow LED and AD1 debug pin
    led_yellow.toggle();
}

static void timer2_callback(void)
{
    // Toggle orange LED and AD2 debug pin
    led_orange.toggle();
}

static void timer3_callback(void)
{
    // Toggle red LED and AD3 debug pin
    led_red.toggle();
}

