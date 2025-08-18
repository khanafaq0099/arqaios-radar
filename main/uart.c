#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Local includes
#include "tlv_parser.h"

#define UART_PORT           UART_NUM_1
#define TX_PIN              (17)
#define RX_PIN              (16)
#define UART_BAUD           921600
#define UART_RING_BUF_SIZE  (24 * 1024)    // 24 KB ring buffer
#define HDR_SIZE            40             // 40 bytes Frame Header
#define MAX_PKT_SIZE        1024*4         // Max frame size //TODO  use dynamic allocation if needed

static const char *TAG = "UART_READER";

radarFrame_t radar_frame; // Global frame structure to hold parsed data

typedef struct {
    uint16_t pkt_len;
    uint8_t  data[MAX_PKT_SIZE];
} frame_t;

static QueueHandle_t parser_queue;

static void uart_reader_task(void *arg)
{
    uint8_t local_buf[2048];
    uint8_t frame_buf[4096];   //TODO Adjust Size is very large
    uint32_t     frame_buf_len = 0;
    uint8_t *ptr = NULL;

    // uint8_t *frame_buf = (uint8_t *)malloc(8*1024);

    while (1){
        // Read any available data from UART ring buffer
        int len = uart_read_bytes(UART_PORT, local_buf, sizeof(local_buf), pdMS_TO_TICKS(20));
        if (len > 0) {
            printf("Read %d bytes from UART\n", len);
            // Append to local accumulation buffer
            memcpy(frame_buf + frame_buf_len, local_buf, len);
            frame_buf_len += len;

            int pos;
            while (find_magic_word(frame_buf, frame_buf_len, &pos)) {
                // Ensure we have at least a full header
                if (frame_buf_len - pos < HDR_SIZE)
                    break; // Wait for more data

                uint32_t pkt_len = ((uint32_t)frame_buf[pos+12]) | 
                                   ((uint32_t)frame_buf[pos+13] << 8) |
                                   ((uint32_t)frame_buf[pos+14] << 16) |
                                   ((uint32_t)frame_buf[pos+15] << 24);

                // ptr = frame_buf + pos + 8; // Skip magic word
                // parse_header(ptr, &radar_frame.header);
                // ptr = NULL;

                printf("Found packet at pos %d, length %ld\n", pos, pkt_len);

                // Wait until we have the full packet
                if (frame_buf_len - pos < pkt_len)
                    break;

                frame_t frame;
                frame.pkt_len = pkt_len - HDR_SIZE;
                memcpy(frame.data, frame_buf + pos + HDR_SIZE, frame.pkt_len);

                // Send to parser
                xQueueSend(parser_queue, &frame, portMAX_DELAY);

                // Remove processed bytes from buffer
                memmove(frame_buf, frame_buf + pos + pkt_len, frame_buf_len - (pos + pkt_len));
                frame_buf_len -= (pos + pkt_len);
            }
        }
    }
}

static void parser_task(void *arg)
{
    frame_t frame;
    uint32_t numTLVs = 0;
    int offset = 0;
    int total_len = 0;
    while (1) {
        if (xQueueReceive(parser_queue, &frame, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Parser got frame len=%d", frame.pkt_len);
            numTLVs = radar_frame.header.numTLVs;
            parse_tlv(frame.data, numTLVs, offset, frame.pkt_len, &radar_frame);
            ESP_LOG_BUFFER_HEX("FRAME", frame.data, frame.pkt_len);
        }
    }
}

void uart_init() 
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, UART_RING_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void app_main(void)
{
    uart_init();

    parser_queue = xQueueCreate(10, sizeof(frame_t));

    xTaskCreate(uart_reader_task, "uart_reader_task", 4096*4, NULL, 10, NULL);
    // xTaskCreate(tx_test_task, "tx_test_task", 4096, NULL, 8, NULL);
    xTaskCreate(parser_task, "parser_task", 4096*3, NULL, 9, NULL);
}
