idf_component_register(SRCS "uart_test.c" "tlv_parser.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES wifi_manager mqtt_client driver)
