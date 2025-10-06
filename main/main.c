#include "uart.h"
#include "wifi.h"
void app_main(void)
{
    Wifi_Init();
    uart_init();
    tx_init();
    uart_read_init();
    parser_init();
}
