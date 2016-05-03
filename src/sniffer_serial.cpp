////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_serial.hpp"
#include "sniffer_serial_send.hpp"
#include "sniffer_serial_receive.hpp"

namespace Sniffer
{
    PlainCallback Serial::uartRxCallback(&SerialReceive::uartByteReceived);

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Serial::initialize()
    {
        // Enable the UART
        uart.enable(BAUDRATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE, UART_TXINT_MODE_EOT);

        // Send the READY message to the host
        SerialSend::initialize();
        SerialSend::sendReadyPacket();

        // Enable the receive interrupt
        uart.setRxCallback(&uartRxCallback);
        uart.enableInterrupts();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Serial::serialTask(void*)
    {
        while (true)
        {
            // Check if there are bytes the the UART RX buffer and process them
            SerialReceive::receive();

            // Check if there is a packet in the buffer that still has to be send to the pc
            if ((bufferIndexSerialSend != bufferIndexRadio) && !UARTBusy(uart.getBase()))
            {
                SerialSend::send();
            }
        }
    }
}
