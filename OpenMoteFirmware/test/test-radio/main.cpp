/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

#include "Callback.h"
#include "Serial.h"

/*================================ define ===================================*/

#define RADIO_MODE_RX                       ( 0 )
#define RADIO_MODE_TX                       ( 1 )
#define RADIO_MODE                          ( RADIO_MODE_TX )
#define RADIO_CHANNEL                       ( 26 )

#define PAYLOAD_LENGTH                      ( 125 )
#define EUI48_LENGTH                        ( 6 )

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 2 )
#define RADIO_RX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )
#define RADIO_TX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

extern "C" void vApplicationTickHook(void);
extern "C" void vApplicationIdleHook(void);

static void prvGreenLedTask(void *pvParameters);
static void prvRadioRxTask(void *pvParameters);
static void prvRadioTxTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);
static void txInit(void);
static void txDone(void);

/*=============================== variables =================================*/

static xSemaphoreHandle rxSemaphore;
static xSemaphoreHandle txSemaphore;

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);
static PlainCallback txInitCallback(&txInit);
static PlainCallback txDoneCallback(&txDone);

static uint8_t radio_buffer[PAYLOAD_LENGTH];
static uint8_t* radio_ptr = radio_buffer;
static uint8_t  radio_len = sizeof(radio_buffer);
static int8_t rssi;
static uint8_t lqi;
static uint8_t crc;

static uint8_t uart_buffer[PAYLOAD_LENGTH];
static uint8_t* uart_ptr = uart_buffer;
static uint8_t  uart_len = sizeof(radio_buffer);

static Serial serial(uart);

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();
    
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the IEEE 802.15.4 radio
    radio.setTxCallbacks(&txInitCallback, &txDoneCallback);
    radio.setRxCallbacks(&rxInitCallback, &rxDoneCallback);
    radio.enable();
    radio.enableInterrupts();
    radio.setChannel(RADIO_CHANNEL);

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);

#if (RADIO_MODE == RADIO_MODE_RX)
    // Enable the UART driver and Serial device
    uart.enable(UART_BAUDRATE, UART_CONFIG, UART_INT_MODE);
    serial.init();

    // Create the radio receive task
    xTaskCreate(prvRadioRxTask, (const char *) "RadioRx", 128, NULL, RADIO_RX_TASK_PRIORITY, NULL);
#elif (RADIO_MODE == RADIO_MODE_TX)
    // Create the radio transmit task
    xTaskCreate(prvRadioTxTask, (const char *) "RadioTx", 128, NULL, RADIO_TX_TASK_PRIORITY, NULL);
#endif

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*================================ private ==================================*/

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

static void prvRadioRxTask(void *pvParameters)
{
    static RadioResult result;

    // Create the receive semaphore
    rxSemaphore = xSemaphoreCreateMutex();

    // Take the receive semaphore so that we block until a packet is received
    xSemaphoreTake(rxSemaphore, (TickType_t) portMAX_DELAY);

    // Forever
    while (true)
    {
        // Turn on the radio transceiver
        radio.on();

        // Put the radio transceiver in receive mode
        radio.receive();

        // Turn the yellow LED on when a the radio is receiving
        led_yellow.on();

        // Take the rxSemaphre, block until available
        if (xSemaphoreTake(rxSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Turn the yellow LED off when a packet is received
            led_yellow.off();

            // Get a packet from the radio buffer
            radio_ptr = radio_buffer;
            radio_len = sizeof(radio_buffer);
            result = radio.getPacket(radio_ptr, &radio_len, &rssi, &lqi, &crc);

            if (result == RadioResult_Success && crc)
            {
                // Restore the UART pointer
                uart_ptr = uart_buffer;

                // Copy the payload to the UART buffer
                memcpy(uart_ptr, radio_ptr, radio_len);
                uart_ptr += radio_len;
                uart_len = radio_len;

                // Copy the RSSI to the UART buffer
                *uart_ptr++ = rssi;
                uart_len += 1;

                // Copy the CRC to the UART buffer
                *uart_ptr++ = crc;
                uart_len += 1;

                // Transmit the buffer over the UART
                serial.write(uart_buffer, uart_len);
            }
            
            // Turn off the radio until the next packet
            radio.off();
        }
    }
}

static void prvRadioTxTask(void *pvParameters)
{
    static RadioResult result;

    // Create the transmit semaphore
    txSemaphore = xSemaphoreCreateMutex();

    // Get the EUI64 address of the board
    board.getEUI48(radio_buffer);

    // Forever
    while (true)
    {
        // Take the txSemaphre, block until available
        if (xSemaphoreTake(txSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Turn on the radio transceiver
            radio.on();

            // Turn the yellow LED on when the packet is being loaded
            led_yellow.on();

            // Load the EUI64 address to the transmit buffer
            radio_ptr = radio_buffer;
            radio_len = EUI48_LENGTH;
            result = radio.loadPacket(radio_ptr, radio_len);

            if (result == RadioResult_Success)
            {
                // Put the radio transceiver in transmit mode
                radio.transmit();

                // Turn the yellow LED off when the packet has beed loaded
                led_yellow.off();
            }

            // Delay the transmission of the next packet 250 ms
            vTaskDelay(250 / portTICK_RATE_MS);
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
