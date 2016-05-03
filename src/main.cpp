////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_serial.hpp"
#include "sniffer_radio.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
    // Enable erasing the flash with the user button
    board.enableFlashErase();

    // Initialize uDMA, radio and UART
    Sniffer::Radio::initialize();
    Sniffer::Serial::initialize();

    // Create the sole task which will send the received packets to the pc (receiving on radio is done through interrupts)
    xTaskCreate(Sniffer::Serial::serialTask, "Serial", 128, NULL, tskIDLE_PRIORITY+1, NULL);

    // Set up interrupts and call our serial task
    portDISABLE_INTERRUPTS();
    xPortStartScheduler();
}
