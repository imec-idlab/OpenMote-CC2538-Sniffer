////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SNIFFER_SERIAL_SEND_HPP
#define SNIFFER_SERIAL_SEND_HPP

#include "sniffer_global.hpp"

namespace Sniffer
{
    class SerialSend
    {
    public:
        // Set the first two bytes of the TX buffer which are always the same
        static void initialize();

        // Send one packet to the host
        static void send();

        // Signal to the host that a reset has happened (either the host requested this or the program was just started)
        static void sendReadyPacket();

    private:
        // Put the packet in an HDLC frame
        static void hdlcEncode();

        // Escape the byte when needed (if it equals the start/end delimiter or the escape octet)
        static void addByteToHdlc(uint8_t byte);

        // Calculate the serial CRC of the data
        static uint16_t calculateCRC(uint8_t* beginAddress, uint8_t length);
    };
}

#endif // SNIFFER_SERIAL_SEND_HPP
