/*================================ include ==================================*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

#include "Callback.h"
#include "Serial.h"
#include "Uart.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY  ( tskIDLE_PRIORITY + 4 )
#define SNIFFER_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSnifferTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);

/*=============================== variables =================================*/

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);

static xSemaphoreHandle rxSemaphore;

static uint8_t uart_buffer[125];
static uint8_t uart_len = sizeof(uart_buffer);

static Serial serial(uart);

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();
    
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the IEEE 802.15.4 radio
    radio.setRxCallbacks(&rxInitCallback, &rxDoneCallback);
    radio.enable();
    radio.setChannel(26);
    radio.enableInterrupts();

    // Enable the UART peripheral
    uart.enable(460800, UART_CONFIG, UART_INT_MODE);
    serial.init();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvSnifferTask, (const char *) "SnifferTask", 128, NULL, SNIFFER_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*================================ private ==================================*/

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

static void prvSnifferTask(void *pvParameters)
{
    RadioResult result;
    uint8_t radio_buffer[125];
    uint8_t* radio_ptr = radio_buffer;
    uint8_t radio_len = sizeof(radio_buffer);
    int8_t rssi;
    uint8_t lqi;
    uint8_t crc;

    // Create the receive semaphore
    rxSemaphore = xSemaphoreCreateMutex();

    // Take the receive semaphore so that we block until a packet is received
    xSemaphoreTake(rxSemaphore, portMAX_DELAY);

    // Turn on the radio transceiver
    radio.on();

    // Forever
    while (true)
    {
        // Put the radio transceiver in receive mode
        radio.receive();

        // Turn the yellow LED on when a the radio is receiving
        led_yellow.on();

        // Take the rxSemaphre, block until available
        if (xSemaphoreTake(rxSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Turn the yellow LED off when a packet is received
            led_yellow.off();

            // Get a packet from the radio buffer
            radio_len = sizeof(radio_buffer);
            result = radio.getPacket(radio_buffer, &radio_len, &rssi, &lqi, &crc);

            if (result == RadioResult_Success && crc)
            {
                // Copy the payload to the UART buffer
                memcpy(uart_buffer, radio_ptr, radio_len);
                uart_len = radio_len;

                // Transmit the buffer over the UART
                serial.write(uart_buffer, uart_len);
            }
        }
    }
}

static void rxInit(void)
{
    // Turn on the radio LED as the radio is now receiving a packet
    led_red.on();
}

static void rxDone(void)
{
    // Determines if the interrupt triggers a context switch
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Turn off the radio LED as the packet is now received
    led_red.off();

    // Give the receive semaphore as the packet has been received
    xSemaphoreGiveFromISR(rxSemaphore, &xHigherPriorityTaskWoken);

    // Force a context switch after the interrupt if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
