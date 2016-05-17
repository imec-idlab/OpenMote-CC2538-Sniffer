////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SNIFFER_SERIAL_HPP
#define SNIFFER_SERIAL_HPP

#include "sniffer_global.hpp"

namespace Sniffer
{
    class Serial
    {
    public:
        // Enable UART interrupt and send READY message to host
        static void initialize();

        // Task which handles sending and receiving over UART, which is run the whole time when no interrupts are being executed
        static void serialTask(void*);
    };
}

#endif // SNIFFER_SERIAL_HPP
