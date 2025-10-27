idf_component_register(SRCS "esp1_host_uart_bridge.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES usb fatfs esp_driver_gpio esp_timer
                    REQUIRES esp_driver_uart
                    REQUIRES esp_driver_uart usb fatfs
                    )
