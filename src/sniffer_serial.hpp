////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Sniffer
{
    class Serial
    {
    public:
        static void initialize();
        static void serialTask(void*);

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////

        class Receive
        {
        public:
            static void uartByteReceived();
            static void serialReceive(uint8_t byte);

        private:
            static void receivedStartByte();
            static void receivedEndByte();
            static void addByteToBuffer(uint8_t byte);
            static bool decodeReceivedMessage();
            static void receivedACK();
            static void receivedNACK();
            static void receivedRESET();
            static void receivedSTOP();
            static bool checkReceivedIndexAndSeqNr(uint16_t receivedIndex, uint16_t receivedSeqNr);

            static uint16_t crc;
            static bool receivingStatus;
            static bool escaping;
            static uint8_t message[SERIAL_RX_MAX_MESSAGE_LEN];
            static uint8_t messageLen;
            static uint16_t previousReceivedIndex;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////

        class Send
        {
        public:
            static void initialize();
            static void serialSend();
            static void sendReadyPacket();

        private:
            static void hdlcEncode();
            static void addByteToHdlc(uint8_t byte);
            static uint16_t calculateCRC(uint8_t* beginAddress, uint8_t length);

            static uint8_t uartTxBuffer[SERIAL_TX_BUFFER_SIZE];
            static uint16_t uartTxBufferLen;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////

        static volatile uint8_t uartRxBuffer[SERIAL_RX_BUFFER_LEN];
        static volatile uint8_t uartRxBufferIndexWrite;

        static PlainCallback uartRxCallback;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void Serial::initialize()
    {
        Send::initialize();

        // Enable the UART
        uart.enable(BAUDRATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE, UART_TXINT_MODE_EOT);

        // Send the READY message
        Send::sendReadyPacket();

        // Enable the receive interrupt
        uart.setRxCallback(&uartRxCallback);
        uart.enableInterrupts();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Serial::serialTask(void*)
    {
        while (true)
        {
            uint8_t uartRxBufferIndexRead = 0;

            // Check if there are bytes the the UART RX buffer
            while (uartRxBufferIndexRead != uartRxBufferIndexWrite)
            {
                Receive::serialReceive(uartRxBuffer[uartRxBufferIndexRead]);

                uartRxBufferIndexRead++;
                if (uartRxBufferIndexRead == sizeof(uartRxBuffer))
                    uartRxBufferIndexRead = 0;
            }

            // Check if there is a packet in the buffer that still has to be send to the pc
            if ((bufferIndexSerialSend != bufferIndexRadio) && !UARTBusy(uart.getBase()))
            {
                Send::serialSend();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    #include "sniffer_serial_receive.hpp"
    #include "sniffer_serial_send.hpp"
}
