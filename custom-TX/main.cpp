/*================================ include ==================================*/

#include "openmote-cc2538.h"

#include "Callback.h"

/*================================ define ===================================*/

#define CC2538_RF_CSP_OP_ISTXON                 ( 0xE9 )
#define CC2538_RF_CSP_OP_ISFLUSHTX              ( 0xEE )
#define CC2538_RF_CSP_OP_ISRXON                 ( 0xE3 )
#define CC2538_RF_CSP_OP_ISRFOFF                ( 0xEF )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvRadioSendTask(void *pvParameters);

/*=============================== variables =================================*/

static uint8_t radio_buffer[125];

/*================================= public ==================================*/

int main (void)
{
    for (unsigned int i = 0; i < 26; ++i)
        radio_buffer[i] = 'A' + i;
    for (unsigned int i = 0; i < 26; ++i)
        radio_buffer[26+i] = 'a' + i;
    for (unsigned int i = 0; i < 10; ++i)
        radio_buffer[52+i] = '0' + i;
    for (unsigned int i = 0; i < 62; ++i)
        radio_buffer[62+i] = radio_buffer[i];
    radio_buffer[124] = '!';

    board.enableFlashErase();
    board.enableInterrupts();

    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvRadioSendTask, (const char *) "RadioSend", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

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

uint8_t rnd()
{
    // Clock the RNG LSFR once
    HWREG(SOC_ADC_ADCCON1) |= 0x00000004;

    uint32_t retVal = (HWREG(SOC_ADC_RNDL) | (HWREG(SOC_ADC_RNDH) << 8)) & 0x80;

    if (retVal < 2)
        return 2;
    else if (retVal > 125)
        return 125;
    else
        return retVal;
}

static void prvRadioSendTask(void *pvParameters)
{
    seedRandom();

    radio.enable();
    radio.setChannel(25);

    uint16_t packetLen = rnd();
    uint16_t count = 0;
    while (true)
    {
        if (++count == 25000)
            count = 1;

        radio_buffer[0] = (count >> 8) & 0xff;
        radio_buffer[1] = count & 0xff;

        packetLen = rnd();
        if (count % 2)
            packetLen = 2;
        else
            packetLen = 125;

        // Wait for ongoing TX to complete
        while (HWREG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE)
            ;

        led_red.off();

        // Flush the TX buffer
        HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISFLUSHTX;

        // Append the PHY length to the TX buffer
        HWREG(RFCORE_SFR_RFDATA) = packetLen+2;

        // Append the packet payload to the TX buffer
        for (uint8_t i = 0; i < packetLen; i++)
            HWREG(RFCORE_SFR_RFDATA) = radio_buffer[i];

        // Enable transmit mode
        HWREG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISTXON;
        led_red.on();
    }
}

static void prvGreenLedTask(void *pvParameters)
{
    while (true)
    {
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}
