////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SNIFFER_SERIAL_RECEIVE_HPP
#define SNIFFER_SERIAL_RECEIVE_HPP

#include "sniffer_global.hpp"

namespace Sniffer
{
    class SerialReceive
    {
    public:
        // Enable UART interrupt
        static void initialize();

        // Interrupt handler for UART
        static void uartByteReceived();

        // Check if there are bytes the the UART RX buffer and process them
        static void receive();

    private:
        static void processByte(uint8_t byte);
        static void receivedStartByte();
        static void receivedEndByte();
        static void addByteToBuffer(uint8_t byte);
        static bool decodeReceivedMessage();
        static void receivedACK();
        static void receivedNACK();
        static void receivedRESET();
        static void receivedSTOP();
        static void receivedInvalidMessage();
        static bool checkReceivedIndexAndSeqNr(uint16_t receivedIndex, uint16_t receivedSeqNr);
    };
}

#endif // SNIFFER_SERIAL_RECEIVE_HPP
