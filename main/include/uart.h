#ifndef UART_H_
#define UART_H_
 
#include "tlv_parser.h"


#define UART_PORT UART_NUM_1
#define TX_PIN (17)
#define RX_PIN (16)
#define UART_BAUD 921600

#define UART_RING_BUF_SIZE (24 * 1024) // 24 KB ring buffer
#define HDR_SIZE 40                    // 40 bytes Frame Header
#define MAX_TLVS_PKT_SIZE 1024 * 4     // Max frame size //TODO  use dynamic allocation if needed

#define UART1_PORT UART_NUM_2
#define UART1_TX_PIN (18)
#define UART1_RX_PIN (19)

#define UART1_RING_BUF_SIZE (16 * 1024) // 16 KB ring buffer

typedef struct
{
    uint16_t pkt_len;
    uint8_t tlv_data[MAX_TLVS_PKT_SIZE];
    mmwHeader header;
} frame_t;

void parser_init();
void uart_read_init();
void uart_init();
void tx_init();

#endif /* UART_H_ */
