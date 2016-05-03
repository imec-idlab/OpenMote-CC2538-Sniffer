////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SNIFFER_RADIO_HPP
#define SNIFFER_RADIO_HPP

#include "sniffer_global.hpp"

namespace Sniffer
{
    class Radio
    {
    public:
        // Set up radio interrupts and DMA
        static void initialize();

        // Function called when a radio interrupt (SFD or RXPKTDONE) occurs
        static void radioInterruptHandler();

    private:
        // Flush the radio receive buffer, called when something went wrong (e.g. received bytes do not match with PHY header)
        static void flushRadioRX();

        // Handles the received packet when RXPKTDONE interrupt occured
        static void packetReceived(uint8_t packetLength);
    };
}

#endif // SNIFFER_RADIO_HPP
