/*================================ include ==================================*/

#include "openmote-cc2538.h"

#include "Callback.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvRadioSendTask(void *pvParameters);

static void txInit(void);
static void txDone(void);

/*=============================== variables =================================*/

static PlainCallback txInitCallback(&txInit);
static PlainCallback txDoneCallback(&txDone);

static xSemaphoreHandle txSemaphore;

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

    // Enable erasing the Flash with the user button
    board.enableFlashErase();
    
    txSemaphore = xSemaphoreCreateMutex();
    
    // Enable the IEEE 802.15.4 radio
    radio.setTxCallbacks(&txInitCallback, &txDoneCallback);
    radio.enable();
    radio.enableInterrupts();
    radio.setChannel(25);

    // Enable interrupts
    board.enableInterrupts();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvRadioSendTask, (const char *) "RadioSend", 128, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvRadioSendTask(void *pvParameters)
{
    RadioResult result;

    uint32_t num = 0;
    while (true)
    {
        num++;
        radio_buffer[0] = (num >> 24) & 0xff;
        radio_buffer[1] = (num >> 16) & 0xff;
        radio_buffer[2] = (num >> 8) & 0xff;
        radio_buffer[3] = num & 0xff;

        // Take the txSemaphre, block until available
        if (xSemaphoreTake(txSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Turn on the radio transceiver
            radio.on();

            // Turn the yellow LED on when the packet is being loaded
            led_yellow.on();

            // Load the EUI64 address to the transmit buffer
            result = radio.loadPacket(radio_buffer, sizeof(radio_buffer));

            if (result == RadioResult_Success)
            {
                // Put the radio transceiver in transmit mode
                radio.transmit();

                // Turn the yellow LED off when the packet has beed loaded
                led_yellow.off();
            }
        }
    }
}

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off the green LED and keep it for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on the green LED and keep it for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void txInit(void)
{
    // Turn on the radio LED as the packet is now transmitting
    led_red.on();
}

static void txDone(void)
{
    // Determines if the interrupt triggers a context switch
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Turn off the radio LED as the packet is transmitted
    led_red.off();

    // Turn off the radio until the next packet
    radio.off();

    // Give the transmit semaphore as the packet has been transmitted
    xSemaphoreGiveFromISR(txSemaphore, &xHigherPriorityTaskWoken);

    // Force a context switch after the interrupt if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
