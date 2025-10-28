idf_component_register(
    SRCS "main.c" "tusb_hid_example_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio
    REQUIRES esp_driver_uart
    )
