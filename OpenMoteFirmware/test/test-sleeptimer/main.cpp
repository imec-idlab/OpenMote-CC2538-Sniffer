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

static void sleep_timer_callback(void);

static PlainCallback sleepTimerCallback(sleep_timer_callback);

/*=============================== variables =================================*/

/*================================= public ==================================*/

int main (void)
{
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Initialize Rtc
    sleepTimer.setCallback(&sleepTimerCallback);
    sleepTimer.enableInterrupts();

    // Start Rtc
    sleepTimer.start(32768);

    // Enable interrupts
    board.enableInterrupts();

    // Forever
    while (true)
    {
        // Sleep
        board.setSleepMode(SleepMode::SleepMode_2);
        board.sleep();
    }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void sleep_timer_callback(void)
{
    static bool led_status = false;

    if (led_status)
    {
        led_status = false;
        led_green.off();
        sleepTimer.start(32768);
    }
    else
    {
        led_status = true;
        led_green.on();
        sleepTimer.start(327);
    }
}
