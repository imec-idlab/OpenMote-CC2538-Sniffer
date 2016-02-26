/**
 * @file       Board.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "Board.h"
#include "Gpio.h"

#include "cc2538_include.h"
#include "cc2538_types.h"

/*================================ define ===================================*/

// Defines for the EUI64 addresses
#define CC2538_EUI64_ADDRESS_HI_H               ( 0x0028002F )
#define CC2538_EUI64_ADDRESS_HI_L               ( 0x0028002C )
#define CC2538_EUI64_ADDRESS_LO_H               ( 0x0028002B )
#define CC2538_EUI64_ADDRESS_LO_L               ( 0x00280028 )

// Defines the Flash CCA address
#define CC2538_FLASH_CCA_ADDRESS                ( 0x0027F800 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern GpioInPow button_user;

const uint32_t Board::BOARD_TICKS_PER_US = 31;

const uint32_t BOARD_BUTTON_DELAY_US    = 100000;
const uint32_t BOARD_BUTTON_DELAY_TICKS = (BOARD_BUTTON_DELAY_US / Board::BOARD_TICKS_PER_US);

/*=============================== prototypes ================================*/

static void flashEraseCallback(void);

/*================================= public ==================================*/

Board::Board():
    sleepMode_(SleepMode_None), \
    flashEraseCallback_(&flashEraseCallback)
{
}

void Board::reset(void)
{
    // Reset the board
    SysCtrlReset();
}

void Board::setSleepMode(SleepMode sleepMode)
{
     sleepMode_ = sleepMode;
}

void Board::sleep(void)
{
    if (sleepMode_ == SleepMode_None)
    {
        SysCtrlSleep();
    }
    else
    {
        SysCtrlPowerModeSet(sleepMode_);
        SysCtrlDeepSleep();
    }
}

void Board::wakeup(void)
{
    if (sleepMode_ != SleepMode_None)
    {
        // Wake-up peripherals
    }
}

void Board::enableInterrupts(void)
{
    // Enable global interrupts
    IntMasterEnable();
}

void Board::disableInterrupts(void)
{
    // Disable global interrupts
    IntMasterDisable();
}

uint32_t Board::getCurrentTicks(void)
{
    // Get the current number of ticks
    return SleepModeTimerCountGet();
}

bool Board::isExpiredTicks(uint32_t futureTicks)
{
    uint32_t currentTicks;
    int32_t remainingTicks;

    // Get the current number of ticks
    currentTicks = SleepModeTimerCountGet();

    // Calculte the number of remaining ticks
    remainingTicks = (int32_t) (futureTicks - currentTicks);

    // Returns true if it has expired
    return (remainingTicks < 0);
}

void Board::enableFlashErase(void)
{
    uint32_t delayTicks = BOARD_BUTTON_DELAY_TICKS;

    // Calculate timeout
    delayTicks += getCurrentTicks();

    // Wait until timeout
    while (!isExpiredTicks(delayTicks));

    // Set the callback and enable interrupt
    button_user.setCallback(&flashEraseCallback_);
    button_user.enableInterrupts();
}

void Board::getEUI48(uint8_t* address)
{
    uint8_t temp[8];

    getEUI64(temp);

    memcpy(&address[0], &temp[0], 3);
    memcpy(&address[3], &temp[5], 3);
}

void Board::getEUI64(uint8_t* address)
{
    uint8_t* eui64_flash;

    eui64_flash = (uint8_t*) CC2538_EUI64_ADDRESS_LO_H;
    while (eui64_flash >= (uint8_t*) CC2538_EUI64_ADDRESS_LO_L)
    {
        *address++ = *eui64_flash--;
    }

    eui64_flash = (uint8_t*) CC2538_EUI64_ADDRESS_HI_H;
    while (eui64_flash >= (uint8_t*) CC2538_EUI64_ADDRESS_HI_L)
    {
        *address++ = *eui64_flash--;
    }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void flashEraseCallback(void)
{
    // Disable global interrupts
    IntMasterDisable();

    // Erase Flash CCA page
    FlashMainPageErase(CC2538_FLASH_CCA_ADDRESS);

    // Reset the board
    SysCtrlReset();
}
