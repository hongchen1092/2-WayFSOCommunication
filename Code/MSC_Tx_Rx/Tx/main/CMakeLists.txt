set(priv_requires fatfs console)

if(CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()

idf_component_register(
    SRCS "esp2_device_uart_bridge.c"
    INCLUDE_DIRS .
    PRIV_REQUIRES "${priv_requires}"
    REQUIRES esp_driver_uart
)
