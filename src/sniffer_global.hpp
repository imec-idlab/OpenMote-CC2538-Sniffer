////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SNIFFER_GLOBAL_HPP
#define SNIFFER_GLOBAL_HPP

#include "Board.h"
#include "Radio.h"
#include "Uart.h"
#include "hw_ints.h"
#include "hw_rfcore_sfr.h"
#include "hw_rfcore_xreg.h"
#include "hw_udma.h"
#include "hw_udmachctl.h"
#include "libcc2538_udma.h"
#include "libcc2538_uart.h"
#include "libcc2538_interrupt.h"

#include "sniffer_precompiled_crc16_table.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BUFFER_LEN                  24000   // Size of buffer in which packets are stored that have been received on the radio
#define RETRANSMIT_THRESHOLD        10000   // After how many unacknowledged bytes we will retransmit the buffer contents to the pc
#define SERIAL_RX_BUFFER_LEN        256     // How big is the buffer for incoming serial messages
#define SERIAL_RX_MAX_MESSAGE_LEN   8       // The maximum length of an incoming serial message
#define BAUDRATE                    921600  // The baudrate for the UART to communicate with the pc

////////////////////////////////////////////////////////////////////////////////////////////////////////

#define HDLC_FLAG           0x7E
#define HDLC_ESCAPE         0x7D
#define HDLC_ESCAPE_MASK    0x20

#define CC2538_RF_MIN_PACKET_LEN    3
#define CC2538_RF_MAX_PACKET_LEN    127
#define CC2538_RF_RSSI_OFFSET       73
#define CC2538_RF_CSP_OP_ISRXON     0xE3
#define CC2538_RF_CSP_OP_ISFLUSHRX  0xED

#define CC2538_RF_CSP_ISRXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON; } while(0)

#define CC2538_RF_CSP_ISFLUSHRX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
} while(0)


#define CRC_INIT                0xffff
#define END_OF_BUFFER_BYTE      0xff
#define DEFAULT_RADIO_PORT      26

#define ACK_MESSAGE_LENGTH      6   // Length = 2 bytes index + 2 bytes sequence number + 2 bytes crc
#define ACK_INDEX_OFFSET        2
#define ACK_SEQNR_OFFSET        4

#define NACK_MESSAGE_LENGTH     6   // Length = 2 bytes index + 2 bytes sequence number + 2 bytes crc
#define NACK_INDEX_OFFSET       2
#define NACK_SEQNR_OFFSET       4

#define RESET_MESSAGE_LENGTH    3   // Length = radio channel + 2 bytes crc
#define RESET_CHANNEL_OFFSET    2

#define STOP_MESSAGE_LENGTH     2   // Length = 2 bytes crc

// There are 5 extra bytes stored in front of the radio packet: 1 byte length, 2 byte index and 2 byte sequence number
#define BUFFER_EXTRA_BYTES      5
#define BUFFER_INDEX_OFFSET     1
#define BUFFER_SEQNR_OFFSET     3

// The size of the TX buffer is defined by what is needed to pass the largest possible radio packet.
// Together with the radio packet, 4 extra bytes (2 byte index and 2 byte sequence number) are send.
// There are 2 bytes in front of this which are the type and length bytes. At the end a 2 bytes serial CRC is added.
// This whole thing has to be multiplied by 2 as we need twice as much space in case each character would have to be escaped.
// Finally one start and one end byte is added around this data.
#define SERIAL_TX_BUFFER_EXTRA_BYTES   4
#define SERIAL_TX_BUFFER_SIZE          (1 + ((2 + (CC2538_RF_MAX_PACKET_LEN + SERIAL_TX_BUFFER_EXTRA_BYTES) + 2) * 2) + 1)

////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Sniffer
{
    extern uint8_t  buffer[BUFFER_LEN];
    extern volatile uint16_t bufferIndexRadio;
    extern uint16_t bufferIndexSerialSend;
    extern uint16_t bufferIndexAcked;
    extern uint16_t seqNr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    namespace SerialDataType
    {
        enum SerialDataTypes
        {
            Packet = 1,
            Ack    = 2,
            Nack   = 3,
            Reset  = 4,
            Ready  = 5,
            Stop   = 6
        };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // Disable radio interrupts, reset global variables and turn off all leds
    void reset();

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline uint16_t crcCalculationStep(uint8_t byte, uint16_t crc)
    {
        return crc16_table[byte ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline uint16_t readUint16(uint8_t buf[], uint16_t index)
    {
        return (buf[index] << 8) + buf[index+1];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void writeUint16(uint8_t buf[], uint16_t index, uint16_t value)
    {
        buf[index] = (value >> 8) & 0xff;
        buf[index+1] = value & 0xff;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

extern Board board;
extern Uart uart;
extern Radio radio;
extern GpioOut led_green;
extern GpioOut led_orange;
extern GpioOut led_red;
extern GpioOut led_yellow;

#endif // SNIFFER_GLOBAL_HPP
