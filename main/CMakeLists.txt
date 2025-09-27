idf_component_register(SRCS "uart.c" "tlv_parser.c" "main.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES wifi_manager mqtt_client driver mqtt common)
