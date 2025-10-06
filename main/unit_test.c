#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "header.h"   // Include your data

#define TXD_PIN (17)
#define RXD_PIN (16)
#define UART_PORT_NUM (UART_NUM_1)
#define UART_BAUD_RATE (115200)
#define TAG "TX_TASK"

static void tx_task(void *arg)
{
    while (1) {
        int bytes = uart_write_bytes(UART_PORT_NUM, (const char *)radar_frame, sizeof(radar_frame));
        ESP_LOGI(TAG, "Sent %d bytes", bytes);
        vTaskDelay(pdMS_TO_TICKS(2000)); // repeat every 2 sec
    }
}

void tx_init(void)
{
    xTaskCreate(tx_task, "tx_task", 2048, NULL, 5, NULL);
}
