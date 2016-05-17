////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_global.hpp"

namespace Sniffer
{
    uint8_t  buffer[BUFFER_LEN];
    volatile uint16_t bufferIndexRadio = 0;
    uint16_t bufferIndexSerialSend = 0;
    uint16_t bufferIndexAcked = 0;
    uint16_t seqNr = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void reset()
    {
        // Disable radio interrupts and clear the radio buffer
        IntDisable(INT_RFCORERTX);
        CC2538_RF_CSP_ISFLUSHRX();

        // Turn off all leds
        led_green.off();
        led_yellow.off();
        led_orange.off();
        led_red.off();

        // Empty buffer and reset sequence number
        bufferIndexRadio = 0;
        bufferIndexSerialSend = 0;
        bufferIndexAcked = 0;
        seqNr = 0;
    }
}
