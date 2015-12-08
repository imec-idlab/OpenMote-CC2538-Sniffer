/*================================ include ==================================*/

#include "openmote-cc2538.h"
#include "Crc16.h"

/*================================ define ===================================*/

#define CC2538_RF_CSP_OP_ISTXON                 ( 0xE9 )
#define CC2538_RF_CSP_OP_ISFLUSHTX              ( 0xEE )

/*=============================== variables =================================*/

static uint8_t radio_buffer[124];

const uint8_t packetSize = 2;

/*================================= public ==================================*/

int main (void)
{
    for (unsigned int i = 0; i < 26; ++i)
        radio_buffer[i] = 'a' + i;
    for (unsigned int i = 0; i < 26; ++i)
        radio_buffer[26+i] = 'A' + i;
    for (unsigned int i = 0; i < 10; ++i)
        radio_buffer[52+i] = '0' + i;
    for (unsigned int i = 0; i < 62; ++i)
        radio_buffer[62+i] = radio_buffer[i];

    // Turn on a led to show that the program is running
    led_green.on();

    // Enable the IEEE 802.15.4 radio
    radio.enable();
    radio.setChannel(26);

    // Disable the auto CRC calculation
    HWREG(RFCORE_XREG_FRMCTRL0) &= ~RFCORE_XREG_FRMCTRL0_AUTOCRC;

    // Flush the TX buffer
    HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX;

    // Append the PHY length to the TX buffer
    HWREG(RFCORE_SFR_RFDATA) = packetSize+2;

    // Append the packet payload to the TX buffer
    for (uint8_t i = 0; i < packetSize; i++)
        HWREG(RFCORE_SFR_RFDATA) = radio_buffer[i];

    // Calculate the CRC of the packet that we will send
    Crc16 crcCalculator;
    crcCalculator.init();
    for (uint8_t i = 0; i < packetSize; ++i)
        crcCalculator.set(radio_buffer[i]);
    uint16_t crc = crcCalculator.get();

    // Append the CRC to the TX buffer
    HWREG(RFCORE_SFR_RFDATA) = (crc >> 8) & 0xff;
    HWREG(RFCORE_SFR_RFDATA) = (crc >> 0) & 0xff;

    while (true)
    {
        // Enable transmit mode
        HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON;

        // Busy-wait until radio is really transmitting
        while (!((HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)))
            ;

        // Wait for ongoing TX to complete
        while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
            ;
    }
}
