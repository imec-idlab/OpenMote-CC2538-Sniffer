/**
 * @file       Radio.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Radio.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

// Defines for the transmit power
#define CC2538_RF_TX_POWER_DEFAULT              ( 0xD5 )

// Defines for the channel
#define CC2538_RF_CHANNEL_MIN                   ( 11 )
#define CC2538_RF_CHANNEL_MAX                   ( 26 )
#define CC2538_RF_CHANNEL_DEFAULT               ( 17 )
#define CC2538_RF_CHANNEL_SPACING               ( 5 )

// Defines for the RSSI
#define CC2538_RF_RSSI_OFFSET                   ( 73 )

// Defines for the CRC and LQI
#define CC2538_RF_CRC_BITMASK                   ( 0x80 )
#define CC2538_RF_LQI_BITMASK                   ( 0x7F )

// Defines for the packet
#define CC2538_RF_MAX_PACKET_LEN                ( 127 )
#define CC2538_RF_MIN_PACKET_LEN                ( 3 )

// Defines for the CCA (Clear Channel Assessment)
#define CC2538_RF_CCA_CLEAR                     ( 0x01 )
#define CC2538_RF_CCA_BUSY                      ( 0x00 )
#define CC2538_RF_CCA_THRESHOLD                 ( 0xF8 )

// Defines for the CSP (Command Strobe Processor)
#define CC2538_RF_CSP_OP_ISRXON                 ( 0xE3 )
#define CC2538_RF_CSP_OP_ISTXON                 ( 0xE9 )
#define CC2538_RF_CSP_OP_ISTXONCCA              ( 0xEA )
#define CC2538_RF_CSP_OP_ISRFOFF                ( 0xEF )
#define CC2538_RF_CSP_OP_ISFLUSHRX              ( 0xED )
#define CC2538_RF_CSP_OP_ISFLUSHTX              ( 0xEE )

// Send an RX ON command strobe to the CSP
#define CC2538_RF_CSP_ISRXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON; } while(0)

// Send a TX ON command strobe to the CSP
#define CC2538_RF_CSP_ISTXON()    \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON; } while(0)

// Send a RF OFF command strobe to the CSP
#define CC2538_RF_CSP_ISRFOFF()   \
  do { HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRFOFF; } while(0)

// Flush the RX FIFO
#define CC2538_RF_CSP_ISFLUSHRX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHRX; \
} while(0)

// Flush the TX FIFO
#define CC2538_RF_CSP_ISFLUSHTX()  do { \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
  HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX; \
} while(0)

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Radio::Radio():
    radioState_(RadioState_Off), \
    rxInit_(nullptr), rxDone_(nullptr), \
    txInit_(nullptr), txDone_(nullptr)
{
}

void Radio::enable(void)
{
    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
    SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_RFC);
    SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_RFC);

    /* Adjust for optimal radio performance */
    HWREG(RFCORE_XREG_MDMCTRL1)    = 0x14;
    HWREG(RFCORE_XREG_RXCTRL)      = 0x3F;

    /* Adjust current in synthesizer */
    HWREG(RFCORE_XREG_FSCTRL)      = 0x55;

    /* Tune sync word detection by requiring two zero symbols before the sync word */
    HWREG(RFCORE_XREG_MDMCTRL0)    = 0x85;

    /* Adjust current in VCO */
    HWREG(RFCORE_XREG_FSCAL1)      = 0x01;

    /* Adjust target value for AGC control loop */
    HWREG(RFCORE_XREG_AGCCTRL1)    = 0x15;

    /* Tune ADC performance */
    HWREG(RFCORE_XREG_ADCTEST0)    = 0x10;
    HWREG(RFCORE_XREG_ADCTEST1)    = 0x0E;
    HWREG(RFCORE_XREG_ADCTEST2)    = 0x03;

    /* Update CCA register to -81 dB */
    HWREG(RFCORE_XREG_CCACTRL0)    = 0xF8;

    /* Set transmit anti-aliasing filter bandwidth */
    HWREG(RFCORE_XREG_TXFILTCFG)   = 0x09;

    /* Set AGC target value */
    HWREG(RFCORE_XREG_AGCCTRL1)    = 0x15;

    /* Set bias currents */
    HWREG(ANA_REGS_O_IVCTRL)       = 0x0B;

    /* Disable the CSPT register compare function */
    HWREG(RFCORE_XREG_CSPT)        = 0xFFUL;

    /* Enable automatic CRC calculation and RSSI append */
    HWREG(RFCORE_XREG_FRMCTRL0)    = RFCORE_XREG_FRMCTRL0_AUTOCRC;

    /* Disable frame filtering */
    HWREG(RFCORE_XREG_FRMFILT0)   &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;

    /* Disable source address matching and autopend */
    HWREG(RFCORE_XREG_SRCMATCH)    = 0;

    /* Set maximum FIFOP threshold */
    HWREG(RFCORE_XREG_FIFOPCTRL)   = CC2538_RF_MAX_PACKET_LEN;

    /* Flush transmit and receive */
    CC2538_RF_CSP_ISFLUSHRX();
    CC2538_RF_CSP_ISFLUSHTX();

    /* Set default transmit power and channel */
    HWREG(RFCORE_XREG_TXPOWER)     = CC2538_RF_TX_POWER_DEFAULT;
    HWREG(RFCORE_XREG_FREQCTRL)    = CC2538_RF_CHANNEL_MIN;
}

void Radio::sleep(void)
{
    off();
}

void Radio::wakeup(void)
{
    enable();
}

void Radio::on(void)
{
    /* Set the radio state to idle */
    radioState_ = RadioState_Idle;

    /* Turn on the radio */
    CC2538_RF_CSP_ISRXON();
}

void Radio::off(void)
{
    /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
    while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
        ;

    /* Set the radio state to off */
    radioState_ = RadioState_Off;

    /* Don't turn off if we are off as this will trigger a Strobe Error */
    if (HWREG(RFCORE_XREG_RXENABLE) != 0)
    {
        /* Turn off the radio */
        CC2538_RF_CSP_ISRFOFF();

        /* Clear FIFO interrupt flags */
        HWREG(RFCORE_SFR_RFIRQF0) = ~(RFCORE_SFR_RFIRQF0_FIFOP|RFCORE_SFR_RFIRQF0_RXPKTDONE);
    }
}

void Radio::reset(void)
{
    /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
    while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

    /* Set the radio state to off */
    radioState_ = RadioState_Off;

    /* Flush the RX and TX buffers */
    CC2538_RF_CSP_ISFLUSHRX();
    CC2538_RF_CSP_ISFLUSHTX();

    /* Don't turn off if we are off since this will trigger a Strobe Error */
    if (HWREG(RFCORE_XREG_RXENABLE) != 0)
    {
        /* Turn off the radio */
        CC2538_RF_CSP_ISRFOFF();
    }
}

void Radio::setRxCallbacks(Callback* rxInit, Callback* rxDone)
{
    /* Store the receive init and done callbacks */
    rxInit_ = rxInit;
    rxDone_ = rxDone;
}

void Radio::setTxCallbacks(Callback* txInit, Callback* txDone)
{
    /* Store the transmit init and done callbacks */
    txInit_ = txInit;
    txDone_ = txDone;
}

void Radio::enableInterrupts(void)
{
    /* Register the receive interrupt handlers */
    InterruptHandler::getInstance().setInterruptHandler(this);

    /* Enable RF interrupts 0, RXPKTDONE, SFD and FIFOP only -- see page 751  */
    HWREG(RFCORE_XREG_RFIRQM0) |= ((0x06 | 0x02 | 0x01) << RFCORE_XREG_RFIRQM0_RFIRQM_S) & RFCORE_XREG_RFIRQM0_RFIRQM_M;

    /* Enable RF interrupts 1, TXDONE only */
    HWREG(RFCORE_XREG_RFIRQM1) |= ((0x02) << RFCORE_XREG_RFIRQM1_RFIRQM_S) & RFCORE_XREG_RFIRQM1_RFIRQM_M;

    /* Enable RF error interrupts */
    HWREG(RFCORE_XREG_RFERRM) = RFCORE_XREG_RFERRM_RFERRM_M;

    // Set the UART interrupt priority
    IntPrioritySet(INT_RFCORERTX, (7 << 5));
    // IntPrioritySet(INT_RFCOREERR, (7 << 5));

    /* Enable radio interrupts */
    IntEnable(INT_RFCORERTX);
    // IntEnable(INT_RFCOREERR);
}

void Radio::disableInterrupts(void)
{
    /* Disable RF interrupts 0, RXPKTDONE, SFD and FIFOP only -- see page 751  */
    HWREG(RFCORE_XREG_RFIRQM0) = 0;

    /* Disable RF interrupts 1, TXDONE only */
    HWREG(RFCORE_XREG_RFIRQM1) = 0;

    /* Disable the radio interrupts */
    IntDisable(INT_RFCORERTX);
    // IntDisable(INT_RFCOREERR);
}

void Radio::setChannel(uint8_t channel)
{
    /* Check that the channel is within bounds */
    if (!(channel < CC2538_RF_CHANNEL_MIN) || !(channel > CC2538_RF_CHANNEL_MAX))
    {
        /* Changes to FREQCTRL take effect after the next recalibration */
        HWREG(RFCORE_XREG_FREQCTRL) = (CC2538_RF_CHANNEL_MIN +
                                      (channel - CC2538_RF_CHANNEL_MIN) * CC2538_RF_CHANNEL_SPACING);
    }
}

void Radio::setPower(uint8_t power)
{
    /* Set the radio transmit power */
    HWREG(RFCORE_XREG_TXPOWER) = power;
}

void Radio::transmit(void)
{
    /* Make sure we are not transmitting already */
    while(HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
        ;

    /* Set the radio state to transmit */
    radioState_ = RadioState_TransmitInit;

    /* Enable transmit mode */
    CC2538_RF_CSP_ISTXON();

    /* Busy-wait until radio really transmitting */
    while(!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)))
        ;
}

void Radio::receive(void)
{
    /* Flush the RX buffer */
    CC2538_RF_CSP_ISFLUSHRX();

    /* Set the radio state to receive */
    radioState_ = RadioState_ReceiveInit;

    /* Enable receive mode */
    CC2538_RF_CSP_ISRXON();

    /* Busy-wait until radio really listening */
    while(!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE)))
        ;
}

/**
 * When loading a packet to the RX buffer, the following is expected:
 * - *[1B]      Length (not required)
 * -  [0-125B]  Payload (exluding CRC)
 * - *[2B]      CRC (not required)
 */
RadioResult Radio::loadPacket(uint8_t* data, uint8_t length)
{
    uint8_t packetLength;

    /* Make sure previous transmission is not still in progress */
    while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
        ;

    /* Check if the radio state is correct */
    if (radioState_ != RadioState_Idle)
    {
        /* Return error */
        return RadioResult_Error;
    }

    /* Account for the CRC bytes */
    packetLength = length + 2;

    /* Check if packet is too long */
    if ((packetLength >  CC2538_RF_MAX_PACKET_LEN) ||
        (packetLength <= CC2538_RF_MIN_PACKET_LEN))
    {
        /* Return error */
        return RadioResult_Error;
    }

    /* Flush the TX buffer */
    CC2538_RF_CSP_ISFLUSHTX();

    /* Append the PHY length to the TX buffer */
    HWREG(RFCORE_SFR_RFDATA) = packetLength;

    /* Append the packet payload to the TX buffer */
    for (uint8_t i = 0; i < length; i++)
    {
        HWREG(RFCORE_SFR_RFDATA) = data[i];
    }

    /* Return success */
    return RadioResult_Success;
}

/**
 * When reading the packet from the RX buffer, you get the following:
 * - *[1B]      Length  (excluding itself)
 * -  [0-125B]  Payload (excluding CRC)
 * - *[2B]      CRC
 * - Notice that the CRC field gets replaced by:
 * - *[1B]      RSSI (signed 2s complement)
 * - *[1B]      CRC_OK (1 bit) + LQI (7 bits)
 */
RadioResult Radio::getPacket(uint8_t* buffer, uint8_t* length, int8_t* rssi, uint8_t* lqi, uint8_t* crc)
{
    uint8_t packetLength;
    uint8_t scratch;

    /* Check if the radio state is correct */
    if (radioState_ != RadioState_ReceiveDone)
    {
        /* Return error */
        return RadioResult_Error;
    }

    /* Check the packet length (first byte) */
    packetLength = HWREG(RFCORE_SFR_RFDATA);

    /* Check if packet is too long or too short */
    if ((packetLength > CC2538_RF_MAX_PACKET_LEN) ||
        (packetLength <= CC2538_RF_MIN_PACKET_LEN))
    {
        /* Flush the RX buffer */
        CC2538_RF_CSP_ISFLUSHRX();

        /* Return error */
        return RadioResult_Error;
    }

    /* Account for the CRC bytes */
    packetLength -= 2;

    /* Check if the packet fits in the buffer */
    if (packetLength > *length)
    {
        /* Flush the RX buffer */
        CC2538_RF_CSP_ISFLUSHRX();

        /* Return error */
        return RadioResult_Error;
    }

    /* Copy the RX buffer to the buffer (except for the CRC) */
    for (uint8_t i = 0; i < packetLength; i++)
    {
        buffer[i] = HWREG(RFCORE_SFR_RFDATA);
    }

    /* Update the packet length, RSSI and CRC */
    *length    = packetLength;
    *rssi      = ((int8_t) (HWREG(RFCORE_SFR_RFDATA)) - CC2538_RF_RSSI_OFFSET);
    scratch    = HWREG(RFCORE_SFR_RFDATA);
    *crc       = scratch & CC2538_RF_CRC_BITMASK;
    *lqi       = scratch & CC2538_RF_LQI_BITMASK;

    /* Flush the RX buffer */
    CC2538_RF_CSP_ISFLUSHRX();

    /* Set the radio state to receive */
    radioState_ = RadioState_Idle;

    return RadioResult_Success;
}

/*=============================== protected =================================*/

void Radio::interruptHandler(void)
{
    uint32_t irq_status0, irq_status1;

    /* Read RFCORE_STATUS */
    irq_status0 = HWREG(RFCORE_SFR_RFIRQF0);
    irq_status1 = HWREG(RFCORE_SFR_RFIRQF1);

    /* Clear pending interrupt */
    IntPendClear(INT_RFCORERTX);

    /* Clear RFCORE_STATUS */
    HWREG(RFCORE_SFR_RFIRQF0) = 0;
    HWREG(RFCORE_SFR_RFIRQF1) = 0;

    /* STATUS0 Register: Start of frame event */
    if ((irq_status0 & RFCORE_SFR_RFIRQF0_SFD) == RFCORE_SFR_RFIRQF0_SFD)
    {
        if (radioState_ == RadioState_ReceiveInit &&
            rxInit_ != nullptr)
        {
            radioState_ = RadioState_Receiving;
            rxInit_->execute();
        }
        else if (radioState_ == RadioState_TransmitInit &&
                 txInit_ != nullptr)
        {
            radioState_ = RadioState_Transmitting;
            txInit_->execute();
        }
        else
        {
            // ToDo: Handle otherwise
        }
    }

    /* STATUS0 Register: End of frame event */
    if (((irq_status0 & RFCORE_SFR_RFIRQF0_RXPKTDONE) ==  RFCORE_SFR_RFIRQF0_RXPKTDONE))
    {
        if (radioState_ == RadioState_Receiving &&
            rxDone_ != nullptr)
        {
            radioState_ = RadioState_ReceiveDone;
            rxDone_->execute();
        }
        else
        {
            // ToDo: Handle otherwise
        }
    }

    /* STATUS0 Register: FIFO is full event */
    if (((irq_status0 & RFCORE_SFR_RFIRQF0_FIFOP) ==  RFCORE_SFR_RFIRQF0_FIFOP))
    {
        // ToDo: Handle otherwise
    }

    /* STATUS1 Register: End of frame event */
    if (((irq_status1 & RFCORE_SFR_RFIRQF1_TXDONE) == RFCORE_SFR_RFIRQF1_TXDONE))
    {
        if (radioState_ == RadioState_Transmitting &&
            txDone_ != nullptr)
        {
            radioState_ = RadioState_TransmitDone;
            txDone_->execute();
        }
        else
        {
            // ToDo: Handle otherwise
        }
    }
}

void Radio::errorHandler(void)
{
    uint32_t irq_error;

    /* Read RFERR_STATUS */
    irq_error = HWREG(RFCORE_XREG_RFERRM);

    /* Clear pending interrupt */
    IntPendClear(INT_RFCOREERR);

    /* Check the error interrupt */
    if ((HWREG(RFCORE_XREG_RFERRM) & (((0x02) << RFCORE_XREG_RFERRM_RFERRM_S) & RFCORE_XREG_RFERRM_RFERRM_M)) & irq_error)
    {
        /* Clear error interrupt */
        HWREG(RFCORE_XREG_RFERRM) = ~(((0x02)<<RFCORE_XREG_RFERRM_RFERRM_S)&RFCORE_XREG_RFERRM_RFERRM_M);

        // ToDo: Handle error
    }
    else
    {
        // ToDo: Handle otherwise
    }
}

/*================================ private ==================================*/
