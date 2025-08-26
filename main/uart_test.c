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
#define MAX_TLVS_PKT_SIZE   1024*4         // Max frame size //TODO  use dynamic allocation if needed
#define FRAME_BUFFER_SIZE   8192

static const char *TAG = "UART_READER";

// radarFrame_t radar_frame; // Global frame structure to hold parsed data

typedef struct {
    uint16_t pkt_len;
    uint8_t  tlv_data[MAX_TLVS_PKT_SIZE];

    mmwHeader header;
} frame_t;

static QueueHandle_t parser_queue;

static void uart_reader_task(void *arg)
{
    uint8_t frame_buf[FRAME_BUFFER_SIZE];   //TODO Adjust Size is very large
    uint32_t frame_buf_len = 0;
    uint8_t *ptr = NULL;

    while (1) {
        int len = uart_read_bytes(UART_PORT, frame_buf + frame_buf_len, FRAME_BUFFER_SIZE - frame_buf_len, pdMS_TO_TICKS(20));
        if (len > 0) {
            printf("Read %d bytes from UART\n", len);

            frame_buf_len += len;

            int magic_idx;
            while (find_magic_word(frame_buf, frame_buf_len, &magic_idx)) {

                if(frame_buf_len - magic_idx < HDR_SIZE)
                    break;

                    
                uint32_t pkt_len = ((uint32_t)frame_buf[magic_idx+12]) | 
                ((uint32_t)frame_buf[magic_idx+13] << 8) |
                ((uint32_t)frame_buf[magic_idx+14] << 16) |
                ((uint32_t)frame_buf[magic_idx+15] << 24);
                    
                radarFrame_t radar_frame;

                ptr = frame_buf + magic_idx + 8; // Skip magic word
                parse_header(ptr, &radar_frame.header);
                ptr = NULL;

                printf("Found packet at magic_idx %d, length %ld\n", magic_idx, pkt_len);

                // Wait until we have the full packet
                if (frame_buf_len - magic_idx < pkt_len)
                    break;

                frame_t frame;
                frame.pkt_len = pkt_len - HDR_SIZE;
                memcpy(frame.tlv_data, frame_buf + magic_idx + HDR_SIZE, frame.pkt_len);
                memcpy(&frame.header, &radar_frame.header, sizeof(mmwHeader));

                // Send to parser
                xQueueSend(parser_queue, &frame, portMAX_DELAY);

                // Remove processed bytes from buffer
                memmove(frame_buf, frame_buf + magic_idx + pkt_len, frame_buf_len - (magic_idx + pkt_len));
                frame_buf_len -= (magic_idx + pkt_len);
            }
            // Reset buffer if it gets too full
            if (frame_buf_len > FRAME_BUFFER_SIZE - 1024)
                frame_buf_len = 0;
            
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
            radarFrame_t radar_frame = {0};
            ESP_LOGI(TAG, "Parser got frame len=%d", frame.pkt_len);
            printf("\n=== Processing Frame %lu ===\n", frame.header.frameNumber);
            numTLVs = frame.header.numTLVs;

            parse_tlv(frame.tlv_data, numTLVs, offset, frame.pkt_len, &radar_frame);
            ESP_LOG_BUFFER_HEX("FRAME", frame.tlv_data, frame.pkt_len);
        } else
            ESP_LOGW(TAG, "Failed to receive frame from queue");
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
