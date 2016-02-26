/**
 * @file       cc2538_startup.c
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdint.h>

#include "cc2538_include.h"

/*================================ define ===================================*/

#define DEFAULT_STACK_SIZE              ( 1024 )
#ifndef STACK_SIZE
#define STACK_SIZE                      ( DEFAULT_STACK_SIZE )
#endif

#define FLASH_START_ADDR                ( 0x00200000 )
#define BOOTLOADER_BACKDOOR_ENABLED     ( 0xF6FFFFFF ) // ENABLED: PORT A, PIN 6, LOW
#define BOOTLOADER_BACKDOOR_DISABLED    ( 0xEFFFFFFF ) // DISABLED

#ifndef HWREG
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x)))
#endif

/*================================ typedef ==================================*/

typedef struct
{
    uint32_t bootloader_config;
    uint32_t image_is_valid;
    uint32_t image_vector_address;
    uint8_t  lock_page[32];
} lock_page_cca_t;

/*=============================== prototypes ================================*/

static void system_init(void);
static void system_exit(void);

void reset_handler(void);
void nmi_handler(void);
void hardfault_handler(void);
void default_handler(void);

extern int main (void);

extern void __libc_init_array(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/*=============================== variables =================================*/

extern uint32_t _text_start;
extern uint32_t _text_end;
extern uint32_t _data_start;
extern uint32_t _data_end;
extern uint32_t _bss_start;
extern uint32_t _bss_end;

__attribute__ ((section(".stack")))
static uint8_t _stack[STACK_SIZE];

__attribute__ ((section(".flashcca"), used))
const lock_page_cca_t lock_page_cca =
{
  BOOTLOADER_BACKDOOR_ENABLED,  // Bootloader backdoor enabled
  0,                            // Image valid bytes
  FLASH_START_ADDR,             // Vector table located at flash start address
  {0xFF, 0xFF, 0xFF, 0xFF,      // Unlock all pages and debug
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF,
   0xFF, 0xFF, 0xFF, 0xFF}
};

__attribute__ ((section(".vectors"), used))
void (* const interrupt_vector[])(void) =
{
   (void (*)(void))((uint32_t)&_stack + sizeof(_stack)), // Stack pointer
   reset_handler,                                        // Reset handler
   nmi_handler,                                          // The NMI handler
   hardfault_handler,                                    // The hard fault handler
   default_handler,                                      // 4 The MPU fault handler
   default_handler,                                      // 5 The bus fault handler
   default_handler,                                      // 6 The usage fault handler
   0, 0, 0, 0,                                           // 7-10 Reserved
   vPortSVCHandler,                                      // 11 SVCall handler
   default_handler,                                      // 12 Debug monitor handler
   0,                                                    // 13 Reserved
   xPortPendSVHandler,                                   // 14 The PendSV handler
   xPortSysTickHandler,                                  // 15 The SysTick handler
   default_handler,                                      // 16 GPIO Port A
   default_handler,                                      // 17 GPIO Port B
   default_handler,                                      // 18 GPIO Port C
   default_handler,                                      // 19 GPIO Port D
   0,                                                    // 20 Reserved
   default_handler,                                      // 21 UART0 Rx and Tx
   default_handler,                                      // 22 UART1 Rx and Tx
   default_handler,                                      // 23 SSI0 Rx and Tx
   default_handler,                                      // 24 I2C Master and Slave
   0, 0, 0, 0, 0,                                        // 25-29 Reserved
   default_handler,                                      // 30 ADC Sequence 0
   0, 0, 0,                                              // 31-33 Reserved
   default_handler,                                      // 34 Watchdog timer, timer 0
   default_handler,                                      // 35 Timer 0 subtimer A
   default_handler,                                      // 36 Timer 0 subtimer B
   default_handler,                                      // 37 Timer 1 subtimer A
   default_handler,                                      // 38 Timer 1 subtimer B
   default_handler,                                      // 39 Timer 2 subtimer A
   default_handler,                                      // 40 Timer 2 subtimer B
   default_handler,                                      // 41 Analog Comparator 0
   default_handler,                                      // 42 RFCore Rx/Tx
   default_handler,                                      // 43 RFCore Error
   default_handler,                                      // 44 IcePick
   default_handler,                                      // 45 FLASH Control
   default_handler,                                      // 46 AES
   default_handler,                                      // 47 PKA
   default_handler,                                      // 48 Sleep Timer
   default_handler,                                      // 49 MacTimer
   default_handler,                                      // 50 SSI1 Rx and Tx
   default_handler,                                      // 51 Timer 3 subtimer A
   default_handler,                                      // 52 Timer 3 subtimer B
   0, 0, 0, 0, 0, 0, 0,                                  // 53-59 Reserved
   default_handler,                                      // 60 USB
   0,                                                    // 61 Reserved
   default_handler,                                      // 62 uDMA
   default_handler,                                      // 63 uDMA Error
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 64-73 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 74-83 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 84-93 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 94-103 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 104-113 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 114-123 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 124-133 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 134-143 Reserved
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 144-143 Reserved
   0, 0,                                                 // 154-155 Reserved
   default_handler,                                      // 156 USB
   default_handler,                                      // 157 RFCORE RX/TX
   default_handler,                                      // 158 RFCORE Error
   default_handler,                                      // 159 AES
   default_handler,                                      // 160 PKA
   default_handler,                                      // 161 SMTimer
   default_handler                                       // 162 MACTimer
};

/*================================= public ==================================*/

void
nmi_handler(void)
{
    reset_handler();
    while(1)
    {
        __asm("nop");
    }
}

void
hardfault_handler(void)
{
    while(1)
    {
        __asm("nop");
    }
}

void
default_handler(void)
{
    while(1)
    {
        __asm("nop");
    }
}

void
reset_handler(void)
{
    volatile uint32_t *src, *dst;

    /* Workaround for PM debug issue */
    HWREG(SYS_CTRL_EMUOVR) = 0xFF;

    /* Workaround for J-Link debug issue */
    HWREG(NVIC_VTABLE) = (uint32_t)interrupt_vector;

    /* Copy the data segment initializers from flash to SRAM */
    for (src = &_text_end, dst = &_data_start; dst < &_data_end; )
    {
        *dst++ = *src++;
    }

    /* Initialize the BSS section to zero */
    for (dst = &_bss_start; dst < &_bss_end; *dst++)
    {
        *dst = 0;
    }

    /* Initialize the system */
    system_init();

    /* Initialize standard C library */
    __libc_init_array();

   /* Call the application's entry point */
   main();

   /* End here if return from main() */
   system_exit();
}

/*================================ private ==================================*/

static void system_init(void)
{
    /**
     * Configure the 32 kHz pins, PD6 and PD7, for crystal operation
     * By default they are configured as GPIOs
     */
    GPIODirModeSet(GPIO_D_BASE, 0x40, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_D_BASE, 0x80, GPIO_DIR_MODE_IN);
    IOCPadConfigSet(GPIO_D_BASE, 0x40, IOC_OVERRIDE_ANA);
    IOCPadConfigSet(GPIO_D_BASE, 0x80, IOC_OVERRIDE_ANA);

    /**
     * Set the real-time clock to use the 32.768 kHz external crystal
     * Set the system clock to use the 32 MHz external crystal
     */
    SysCtrlClockSet(true, false, SYS_CTRL_SYSDIV_32MHZ);

    /**
     * Set the IO clock to operate at 16 MHz
     * This way peripherals can run while the system clock is gated
     */
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_16MHZ);

    /**
     * Wait until the 32 MHz oscillator becomes stable
     */
    while (!((HWREG(SYS_CTRL_CLOCK_STA)) & (SYS_CTRL_CLOCK_STA_XOSC_STB)));
}

static void system_exit(void)
{
    while(true)
    {
        /**
         * Put the board in deep sleep
         */
        SysCtrlDeepSleep();
    }
}
