////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_global.hpp"
#include "sniffer_serial.hpp"
#include "sniffer_radio.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Sniffer
{
    // Global variables
    uint8_t  buffer[BUFFER_LEN];
    volatile uint16_t bufferIndexRadio = 0;
    uint16_t bufferIndexSerialSend = 0;
    uint16_t bufferIndexAcked = 0;
    uint16_t seqNr = 0;

    // Radio specific variables
    volatile tDMAControlTable Radio::uDMAChannelControlTable __attribute__((section(".udma_channel_control_table")));

    // Serial specific variables
    volatile uint8_t Serial::uartRxBuffer[SERIAL_RX_BUFFER_LEN];
    volatile uint8_t Serial::uartRxBufferIndexWrite = 0;
    PlainCallback Serial::uartRxCallback(&Serial::Receive::uartByteReceived);

    uint16_t Serial::Receive::crc = CRC_INIT;
    bool Serial::Receive::receivingStatus = false;
    bool Serial::Receive::escaping = false;
    uint8_t Serial::Receive::message[SERIAL_RX_MAX_MESSAGE_LEN];
    uint8_t Serial::Receive::messageLen = 0;
    uint16_t Serial::Receive::previousReceivedIndex = INVALID_RECEIVED_INDEX;

    uint8_t  Serial::Send::uartTxBuffer[SERIAL_TX_BUFFER_SIZE];
    uint16_t Serial::Send::uartTxBufferLen = 0;
}

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
