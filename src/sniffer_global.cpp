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
}
