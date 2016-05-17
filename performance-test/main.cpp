////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
#define MAX_PERFORMANCE
#define FIXED_PACKET_SIZE   125
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "openmote-cc2538.h"

#define CC2538_RF_CSP_OP_ISTXON     0xE9
#define CC2538_RF_CSP_OP_ISRXON     0xE3
#define CC2538_RF_CSP_OP_ISRFOFF    0xEF

enum Phase
{
    MinLength,
    MediumLength,
    MaxLength,
    RapidMinMaxChange,
    Increasing,
    Decreasing,
    SmallRandom,
    MediumRandom,
    LargeRandom,
    FullyRandom1,
    FullyRandom2,
    FullyRandom3,
    PhaseCount // # of phases described in this enum
};

static volatile tDMAControlTable uDMAChannelControlTable __attribute__((section(".udma_channel_control_table")));
static uint8_t radio_buffer[125];

////////////////////////////////////////////////////////////////////////////////////////////////////////

void seedRandom()
{
    // Make sure the RNG is on (ADCCON1[3:2] = 00)
    HWREG(SOC_ADC_ADCCON1) &= ~0x0000000C;

    // Enable clock for the RF Core
    HWREG(SYS_CTRL_RCGCRFC) = 1;
    while (HWREG(SYS_CTRL_RCGCRFC) != 1)
        ;

    // Place the radio in infinit RX state (FRMCTRL0[3:2] = 10)
    HWREG(RFCORE_XREG_FRMCTRL0) = 0x00000008;

    // Turn radio on
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRXON;

    // Wait until the chip has been in RX long enough for the transients to have died out
    while (!(HWREG(RFCORE_XREG_RSSISTAT) & RFCORE_XREG_RSSISTAT_RSSI_VALID))
        ;

    // Form the seed by concatenating bits from IF_ADC in the RF receive path.
    // Keep sampling until we have read at least 16 bits AND the seed is valid
    // Invalid seeds are 0x0000 and 0x8003 and should not be used
    unsigned short seed = 0x0000;
    while (seed == 0x0000 || seed == 0x8003)
    {
        for (unsigned int i = 0; i < 16; ++i)
        {
            seed |= (HWREG(RFCORE_XREG_RFRND) & RFCORE_XREG_RFRND_IRND);
            seed <<= 1;
       }
    }

    HWREG(SOC_ADC_RNDL) = (seed >> 8) & 0xFF;
    HWREG(SOC_ADC_RNDL) = seed & 0xFF;

    // Turn radio off
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISRFOFF;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t rnd()
{
    // Clock the RNG LSFR once
    HWREG(SOC_ADC_ADCCON1) |= 0x00000004;

    uint32_t retVal = (HWREG(SOC_ADC_RNDL) | (HWREG(SOC_ADC_RNDH) << 8)) & 0x7F;

    if (retVal < 2)
        return 2;
    else if (retVal > 125)
        return 125;
    else
        return retVal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

static void prvRadioSendTask(void *pvParameters)
{
    seedRandom();

    uDMAEnable();
    uDMAControlBaseSet((void*)&uDMAChannelControlTable);
    uDMAChannelAttributeEnable(0, UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
    uDMAChannelControlSet(0, UDMA_SIZE_8 | UDMA_DST_INC_NONE | UDMA_SRC_INC_8 | UDMA_ARB_128);
    uDMAChannelControlTable.pvDstEndAddr = (void*)RFCORE_SFR_RFDATA;

    radio.enable();
    radio.setChannel(26);

    uint16_t packetLen = rnd();
    uint16_t count = 0;
    uint32_t byteCountInPhase = 0;
    uint32_t maxBytesInPhase = 100000;
    uint8_t phase = 0;
    while (true)
    {
#ifdef MAX_PERFORMANCE
    #ifdef FIXED_PACKET_SIZE
        packetLen = FIXED_PACKET_SIZE;
    #else
        packetLen = 1;
    #endif
#else
    if (++count > 50000)
        count = 1;

    radio_buffer[0] = (count >> 8) & 0xff;
    radio_buffer[1] = count & 0xff;

    #ifdef FIXED_PACKET_SIZE
        packetLen = FIXED_PACKET_SIZE;
    #else
        // Determine the packet length based on the phase
        switch (phase)
        {
            case MinLength:
                packetLen = 2;
                break;
            case MediumLength:
                packetLen = 60;
                break;
            case MaxLength:
                packetLen = 125;
                break;
            case RapidMinMaxChange:
            {
                uint8_t multiplier = (10 * (byteCountInPhase / (float)maxBytesInPhase)) + 1;
                if (count % (2 * multiplier) < multiplier)
                    packetLen = 125;
                else
                    packetLen = 2;
                break;
            }
            case Increasing:
                packetLen = (count % 124) + 2;
                break;
            case Decreasing:
                packetLen = 125 - (count % 124);
                break;
            case SmallRandom:
            {
                uint8_t rand = rnd();
                while (rand > 30)
                    rand -= 29;
                packetLen = rand;
                break;
            }
            case MediumRandom:
            {
                uint8_t rand = rnd();
                if (rand < 38)
                    rand += 36;
                if (rand > 83)
                    rand -= 42;
                packetLen = rand;
                break;
            }
            case LargeRandom:
            {
                uint8_t rand = rnd();
                while (rand < 95)
                    rand += 30;
                packetLen = rand;
                break;
            }
            default:
            {
                packetLen = rnd();
                break;
            }
        };

        // Go to the next phase every now and then
        byteCountInPhase += packetLen;
        if (byteCountInPhase >= maxBytesInPhase)
        {
            phase = (phase + 1) % PhaseCount;
            byteCountInPhase = 0;

            if (phase == MinLength)
                maxBytesInPhase = 100000;
            else if (phase == MediumLength)
                maxBytesInPhase = 600000;
            else
                maxBytesInPhase = 1000000;
        }
    #endif
#endif

        // Wait for ongoing TX to complete
        while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
            ;

        led_red.off();

        // Append the PHY length to the TX buffer
        HWREG(RFCORE_SFR_RFDATA) = packetLen+2;

        // Append the packet payload to the TX buffer
        uDMAChannelControlTable.pvSrcEndAddr = (void*)&radio_buffer[packetLen-1];
        uDMAChannelControlTable.ui32Control &= ~(UDMACHCTL_CHCTL_XFERSIZE_M | UDMACHCTL_CHCTL_XFERMODE_M);
        uDMAChannelControlTable.ui32Control |= UDMA_MODE_AUTO | ((packetLen-1) << 4);
        HWREG(UDMA_ENASET) = 1;
        HWREG(UDMA_SWREQ) = 1;
        while (HWREG(UDMA_ENASET))
            ;

        led_red.on();

        // Enable transmit mode
#ifdef MAX_PERFORMANCE
        while (true)
        {
            while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
                ;

            HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON;
        }
#else
        HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON;
#endif
    }
}

int main (void)
{
    // Fill the buffer with numbers larger than the possible length byte on the sniffer
    // If a pointer on the sniffer would point to the wrong byte it should be detected immediately
    for (unsigned int i = 0; i < 60; ++i)
        radio_buffer[i] = 140+i;
    for (unsigned int i = 0; i < 65; ++i)
        radio_buffer[60+i] = 140+i;

    board.enableFlashErase();

    xTaskCreate(prvRadioSendTask, (const char *) "RadioSend", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    portDISABLE_INTERRUPTS();
    xPortStartScheduler();
}
