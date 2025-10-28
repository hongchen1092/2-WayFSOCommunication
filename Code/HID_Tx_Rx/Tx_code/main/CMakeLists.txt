idf_component_register(SRCS "main.c" "hid_host_example.c"
                       INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio
    REQUIRES esp_driver_uart
    REQUIRES usb
    REQUIRES usb_host_hid
                       )
