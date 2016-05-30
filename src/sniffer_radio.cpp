////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_radio.hpp"

namespace Sniffer
{
    // uDMA Channel Control Table must be 1024-bytes aligned, we thus place it at the beginnging of the memory
    volatile tDMAControlTable uDMAChannelControlTable __attribute__((section(".udma_channel_control_table")));

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Radio::initialize()
    {
        radio.enable();
        radio.setChannel(DEFAULT_RADIO_PORT);

        // Set up radio interrupts but don't enable them yet until pc is connected
        HWREG(RFCORE_XREG_RFIRQM0) = (1 << 6) | (1 << 2) | (1 << 1); // RXPKTDONE, SFD and FIFOP interrupts
        IntRegister(INT_RFCORERTX, Radio::radioInterruptHandler);
        IntPrioritySet(INT_RFCORERTX, (6 << 5)); // More important than UART interrupt, which has the default (7 << 5)

        // Initialize uDMA
        uDMAEnable();
        uDMAControlBaseSet((void*)&uDMAChannelControlTable);
        uDMAChannelAttributeEnable(0, UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
        uDMAChannelControlSet(0, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_128);
        uDMAChannelControlTable.pvSrcEndAddr = (void*)RFCORE_SFR_RFDATA;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Radio::radioInterruptHandler()
    {
        // Read RFCORE_STATUS
        uint32_t irq_status0 = HWREG(RFCORE_SFR_RFIRQF0);

        // Clear pending interrupt and RFCORE_STATUS
        IntPendClear(INT_RFCORERTX);
        HWREG(RFCORE_SFR_RFIRQF0) = 0;

        // Check for end of frame interrupt
        // We have to check for this one first in case both SFD and RXPKTDONE interrupts occur at once
        if ((irq_status0 & RFCORE_SFR_RFIRQF0_RXPKTDONE) == RFCORE_SFR_RFIRQF0_RXPKTDONE)
        {
            // Make sure the packet length is valid
            uint8_t packetLength = HWREG(RFCORE_SFR_RFDATA);
            if ((packetLength > CC2538_RF_MAX_PACKET_LEN)
             || (packetLength < CC2538_RF_MIN_PACKET_LEN)
             || (packetLength != HWREG(RFCORE_XREG_RXFIFOCNT)))
            {
                flushRadioRX();
                return;
            }

            packetReceived(packetLength);
        }

        // Check for start of frame interrupt
        else if ((irq_status0 & RFCORE_SFR_RFIRQF0_SFD) == RFCORE_SFR_RFIRQF0_SFD)
        {
            led_yellow.on();
        }
        else // This should not happen (could be RFCORE_SFR_RFIRQF0_FIFOP which means packet can't be valid)
        {
            flushRadioRX();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void Radio::flushRadioRX()
    {
        CC2538_RF_CSP_ISFLUSHRX();
        CC2538_RF_CSP_ISRXON();
        led_yellow.off();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void Radio::packetReceived(uint8_t packetLength)
    {
        // Full length is the packet including FCS plus 5 extra bytes that we store (length byte + 2 byte index + 2 byte seq nr)
        const uint8_t fullPacketLength = packetLength + BUFFER_EXTRA_BYTES;

        // The radio index must never pass the ack index
        if (((bufferIndexRadio + fullPacketLength >= sizeof(buffer))
          && ((bufferIndexAcked > bufferIndexRadio) || (bufferIndexAcked <= fullPacketLength)))
         || ((bufferIndexRadio + fullPacketLength < sizeof(buffer))
          && (bufferIndexAcked > bufferIndexRadio)
          && (bufferIndexAcked <= bufferIndexRadio + fullPacketLength)))
        {
            // Indicate that we are no longer lossless and discard this packet
            led_red.on();
            flushRadioRX();
            return;
        }

        // Check if there is no more space behind the last packet
        if (bufferIndexRadio + fullPacketLength >= sizeof(buffer))
        {
            // Mark that the last part of the buffer as unused and start at the beginning
            // When the serial task reads this byte it will know that the next packet is found at the beginning of the buffer
            buffer[bufferIndexRadio] = END_OF_BUFFER_BYTE;
            bufferIndexRadio = 0;
        }

        // Copy the RX buffer to our buffer with Direct Memory Access
        uDMAChannelControlTable.pvDstEndAddr = (void*)&buffer[bufferIndexRadio + fullPacketLength - 1];
        uDMAChannelControlTable.ui32Control &= ~(UDMACHCTL_CHCTL_XFERSIZE_M | UDMACHCTL_CHCTL_XFERMODE_M);
        uDMAChannelControlTable.ui32Control |= UDMA_MODE_AUTO | ((packetLength - 1) << 4);
        HWREG(UDMA_ENASET) = 1; // uDMAChannelEnable
        HWREG(UDMA_SWREQ) = 1; // uDMAChannelRequest

        // Put the amount of bytes that we will use (including this length byte) in the buffer
        buffer[bufferIndexRadio] = fullPacketLength;

        // The first two bytes are the index in the buffer right after this packet
        writeUint16(buffer, bufferIndexRadio + BUFFER_INDEX_OFFSET, bufferIndexRadio);

        // The next two bytes form the sequence number (for verifying if correct packet is still in buffer)
        writeUint16(buffer, bufferIndexRadio + BUFFER_SEQNR_OFFSET, seqNr);
        seqNr++;

        // Wait for ongoing DMA to complete
        // It is fast enough to wait inside this interrupt, so there is no reason to use DMA interrupts
        while (HWREG(UDMA_ENASET) & 1)
            ;

        // Correct the RSSI byte (which is stored in the first of the 2 FCS bytes)
        uint8_t& rssi = buffer[bufferIndexRadio + fullPacketLength - 2];
        rssi = ((int8_t)rssi) - CC2538_RF_RSSI_OFFSET;

        // Move the radio index forward
        bufferIndexRadio += fullPacketLength;

        // Ready for next packet
        CC2538_RF_CSP_ISRXON();
        led_yellow.off();
    }
}
