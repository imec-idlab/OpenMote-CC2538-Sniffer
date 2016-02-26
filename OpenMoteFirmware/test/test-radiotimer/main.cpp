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
static void radio_timer_period_callback(void);
static void radio_timer_compare_callback(void);

static PlainCallback sleepTimerCallback(sleep_timer_callback);
static PlainCallback radioTimerPeriodCallback(radio_timer_period_callback);
static PlainCallback radioTimerCompareCallback(radio_timer_compare_callback);

/*=============================== variables =================================*/

/*================================= public ==================================*/

int main (void)
{
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Initialize sleepTimer
    sleepTimer.setCallback(&sleepTimerCallback);
    sleepTimer.enableInterrupts();

    // Initialize radioTimer
    radioTimer.setPeriodCallback(&radioTimerPeriodCallback);
    radioTimer.setCompareCallback(&radioTimerCompareCallback);
    radioTimer.enableInterrupts();

    // Start sleepTimer
    sleepTimer.start(32768);

    // Start radioTimer
    radioTimer.setPeriod(8192);
    radioTimer.setCompare(4196);
    radioTimer.start();

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
        led_green.off();
        sleepTimer.start(32768);
    }
    else
    {
        led_green.on();
        sleepTimer.start(327);
    }

    led_status = !led_status;
}

static void radio_timer_period_callback(void)
{
    static bool led_status = false;

    if (led_status)
    {
        led_orange.off();
    }
    else
    {
        led_orange.on();
    }

    led_status = !led_status;
}

static void radio_timer_compare_callback(void)
{
    static bool led_status = false;

    if (led_status)
    {
        led_yellow.off();
    }
    else
    {
        led_yellow.on();
    }
    led_status = !led_status;
}
