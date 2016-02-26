/**
 * @file       SemaphoreBinary.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "CriticalSection.h"

/*================================ define ===================================*/

#define enterCriticalSection(lock)          \
  do {                                      \
    asm (                                   \
        "mrs r0, PRIMASK\n\t"               \
        "cpsid I\n\t"                       \
        "strb r0, %[output]"                \
        : [output] "=m" (lock) :: "r0");    \
  } while(0)

#define exitCriticalSection(lock)           \
  do {                                      \
    asm (                                   \
        "ldrb r0, %[input]\n\t"             \
        "msr PRIMASK,r0;\n\t"               \
        ::[input] "m" (lock) : "r0");       \
  } while(0)

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

CriticalSection::CriticalSection(void)
{
    enterCriticalSection(lock);
}

CriticalSection::~CriticalSection(void)
{
    exitCriticalSection(lock);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

