/**
 * @file       cc2538_types.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef CC2538_TYPES_H_
#define CC2538_TYPES_H_

/*================================ include ==================================*/

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/**
 * SleepMode_None:   ~2 mA,   0 + 0 us,   wake-up from any interrupt source (e.g. UART)
 * SleepMode_1:    ~600 uA, 0.5 + 4 us,   wake-up from Gpio, Sleep timer, USB resume
 * SleepMode_2:    ~1.5 uA, 136 + 136 us, wake-up from Gpio, Sleep timer
 * SleepMode_3:    ~0.8 uA, 136 + 136 us, wake-up from Gpio
 */
enum SleepMode : uint8_t {
    SleepMode_None = SYS_CTRL_PM_NOACTION,
    SleepMode_1    = SYS_CTRL_PM_1,
    SleepMode_2    = SYS_CTRL_PM_2,
    SleepMode_3    = SYS_CTRL_PM_3
};

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif  /* CC2538_TYPES_H_ */
